"""
平衡车 PD 参数自动整定 & 离线分析工具

用法:
    python auto_tune_balance.py analyze <csv_path>      离线分析遥测数据
    python auto_tune_balance.py search <csv_path>       模型辨识 + 仿真扫描最优参数
    python auto_tune_balance.py sensitivity <csv_path>  参数敏感度分析
    python auto_tune_balance.py tune [--port COMx]      在线自动搜索最优 PD 参数

在线调参流程:
    1. 粗粒度网格搜索 Kp∈[10,80] Kd∈[1,8]
    2. 每组参数: 设置→稳定2s→施加"虚拟推搡"(target_angle ±3°脉冲)→采集5s→评分
    3. 细粒度局部搜索最优区域
    4. 输出最优参数和性能报告

代价函数:
    J = w1*std(tilt) + w2*overshoot_ratio + w3*settling_time_ratio + w4*control_effort_ratio
"""

import sys, time, struct, csv, os, math
from collections import deque
import serial
import serial.tools.list_ports

BAUD = 230400
TIMEOUT = 0.5
TELEM_CHANNELS = 10
FRAME_BYTES = TELEM_CHANNELS * 4 + 4
FOOTER = b'\x00\x00\x80\x7F'

CH_NAMES = [
    "tilt_angle", "gyro_rate", "balance_out",
    "speed_R", "speed_L", "iq_R", "iq_L",
    "target_angle", "yaw_rate", "target_yaw_rate"
]

# 直接力矩控制参数
BALANCE_DIRECT_GAIN = 0.012   # A/RPM
CURRENT_LIMIT = 2.0            # A


def find_port():
    ports = [p.device for p in serial.tools.list_ports.comports()
             if 'STLink' in (p.description or '') or 'STMicro' in (p.description or '')]
    if ports:
        return ports[0]
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if ports:
        print(f"未检测到 ST-Link，使用第一个串口: {ports[0]}")
        return ports[0]
    print("ERROR: 未找到串口")
    sys.exit(1)


def send_cmd(ser, cmd):
    ser.reset_input_buffer()
    ser.write((cmd + '\r').encode('ascii'))
    buf = b''
    deadline = time.time() + TIMEOUT
    idle_start = time.time()
    while time.time() < deadline:
        n = ser.in_waiting
        if n > 0:
            buf += ser.read(n)
            idle_start = time.time()
        elif buf and time.time() - idle_start > 0.05:
            break
        else:
            time.sleep(0.005)
    return buf.decode('utf-8', errors='replace')


def set_pd(ser, kp, kd):
    """设置平衡 PD 参数并验证"""
    r1 = send_cmd(ser, f'PK ANG={kp:.1f}')
    time.sleep(0.05)
    r2 = send_cmd(ser, f'PK GYR={kd:.1f}')
    time.sleep(0.05)
    # 验证
    r3 = send_cmd(ser, 'PK')
    return r3


def collect_telem(ser, duration_s, perturb_fn=None):
    """采集遥测数据，可选施加扰动"""
    # 确保遥测开启
    resp = send_cmd(ser, 'T')
    if 'ON' in resp:
        send_cmd(ser, 'T')
    send_cmd(ser, 'T')
    time.sleep(0.05)
    ser.reset_input_buffer()

    if perturb_fn:
        time.sleep(1.0)  # 先采集 1s 稳态
        perturb_fn(ser)  # 施加扰动

    raw = b''
    deadline = time.time() + duration_s
    while time.time() < deadline:
        n = ser.in_waiting
        if n > 0:
            raw += ser.read(n)
        else:
            time.sleep(0.002)

    send_cmd(ser, 'T')

    # 解析帧
    frames = []
    pos = 0
    while pos + FRAME_BYTES <= len(raw):
        footer_pos = raw.find(FOOTER, pos)
        if footer_pos == -1:
            break
        data_start = footer_pos - TELEM_CHANNELS * 4
        if data_start >= pos:
            data = raw[data_start:footer_pos]
            if len(data) == TELEM_CHANNELS * 4:
                floats = list(struct.unpack(f'<{TELEM_CHANNELS}f', data))
                frames.append(floats)
        pos = footer_pos + 4

    return frames


def _decode_hex_csv(csv_path):
    """从十六进制编码的 CSV 中提取所有 RX 字节流"""
    all_bytes = bytearray()
    with open(csv_path, 'r', encoding='utf-8', errors='replace') as f:
        csv.field_size_limit(10 * 1024 * 1024)
        reader = csv.reader(f)
        header = next(reader, None)
        if not header:
            return bytes(all_bytes)
        for row in reader:
            if len(row) < 4:
                continue
            direction = row[2].strip() if len(row) > 2 else ''
            hex_data = row[3].strip() if len(row) > 3 else ''
            if direction != 'RX' or not hex_data:
                continue
            hex_parts = hex_data.split()
            for hp in hex_parts:
                try:
                    all_bytes.append(int(hp, 16))
                except ValueError:
                    pass
    return bytes(all_bytes)


def parse_offline_frames(csv_path):
    """从串口日志 CSV 中离线解析遥测帧 (支持 hex 和 raw 格式)"""
    # 先尝试十六进制编码 CSV 格式
    data = _decode_hex_csv(csv_path)
    if len(data) > 100:
        print(f"解析 hex CSV: {len(data)} RX 字节")
    else:
        # 回退: 直接从文件搜索二进制帧
        with open(csv_path, 'rb') as f:
            data = f.read()

    marker = b'TELEMETRY ON'
    idx = data.find(marker)
    if idx < 0:
        print("未找到 TELEMETRY ON 标记")
        return []

    search_data = data[idx + len(marker):]
    footer = b'\x00\x00\x80\x7F'
    frames = []
    pos = 0

    while pos < len(search_data) - 44:
        next_footer = search_data.find(footer, pos + 40, pos + 48)
        if next_footer < 0:
            next_footer = search_data.find(footer, pos, pos + 100)
            if next_footer < 0:
                break

        frame_start = next_footer - 40
        if frame_start < pos:
            pos = next_footer + 4
            continue

        frame_bytes = search_data[frame_start:next_footer]
        if len(frame_bytes) != 40:
            pos = next_footer + 4
            continue

        try:
            floats = struct.unpack('<10f', frame_bytes)
            if all(abs(f) < 1e6 for f in floats):
                frames.append(floats)
        except:
            pass

        pos = next_footer + 4

    return frames


# ============================================================
# 性能指标计算
# ============================================================

def compute_metrics(frames, fs=200.0):
    """从遥测帧序列计算平衡性能指标"""
    if len(frames) < 50:
        return None

    n = len(frames)
    tilts = [f[0] for f in frames]
    gyros = [f[1] for f in frames]
    bal_outs = [f[2] for f in frames]
    iq_R = [f[5] for f in frames]
    iq_L = [f[6] for f in frames]
    targets = [f[7] for f in frames]

    tilt_mean = sum(tilts) / n
    tilt_std = math.sqrt(sum((t - tilt_mean) ** 2 for t in tilts) / n)

    # RMS 电流 (控制代价)
    iq_rms = math.sqrt(sum((iR**2 + iL**2) / 2 for iR, iL in zip(iq_R, iq_L)) / n)

    # 找到扰动后的恢复特性 (target_angle 突变后的响应)
    # 找 target_angle 跳变点
    jump_idx = None
    for i in range(1, n):
        if abs(targets[i] - targets[i - 1]) > 0.5:
            jump_idx = i
            break

    overshoot = 0.0
    settling_time = 0.0
    settling_frames = 0

    if jump_idx is not None and jump_idx < n - 20:
        post_tilts = tilts[jump_idx:]
        post_target = targets[jump_idx]

        # 超调量: max deviation from target after perturbation
        max_dev = max(abs(t - post_target) for t in post_tilts[:100])
        initial_dev = abs(tilts[jump_idx] - targets[jump_idx - 1])
        overshoot = max(0, max_dev - initial_dev)

        # 稳定时间: tilt 回到 ±1° 以内且保持 50 帧
        steady_count = 0
        for i in range(jump_idx, n):
            if abs(tilts[i] - targets[i]) < 1.0:
                steady_count += 1
                if steady_count >= 50:
                    settling_frames = i - jump_idx - 50
                    break
            else:
                steady_count = 0
        settling_time = settling_frames / fs

    # 振荡指数: 频谱峰值与宽带 RMS 之比 (FFT)
    oscillation_index = 1.0
    if n >= 128:
        # 去均值 + Hanning窗
        detrend = [t - tilt_mean for t in tilts]
        window = [0.5 * (1 - math.cos(2 * math.pi * i / (n - 1))) for i in range(n)]
        sig = [detrend[i] * window[i] for i in range(n)]
        # 补零到 2 的幂
        nfft = 1
        while nfft < n:
            nfft *= 2
        sig_padded = sig + [0.0] * (nfft - n)
        # 简单 DFT 计算幅度谱 (只用前半)
        spectrum = []
        for k in range(nfft // 2):
            re = sum(sig_padded[i] * math.cos(2 * math.pi * k * i / nfft) for i in range(nfft))
            im = sum(sig_padded[i] * math.sin(-2 * math.pi * k * i / nfft) for i in range(nfft))
            spectrum.append(math.sqrt(re * re + im * im))

        # 找频谱峰值 (0.5-20Hz 范围)
        peak_mag = 0.0
        total_power = 0.0
        for k in range(1, len(spectrum)):
            freq = k * fs / nfft
            if 0.5 < freq < 20:
                if spectrum[k] > peak_mag:
                    peak_mag = spectrum[k]
                total_power += spectrum[k] ** 2

        avg_power = total_power / max(1, len([k for k in range(1, len(spectrum)) if 0.5 < k * fs / nfft < 20]))
        if avg_power > 1e-10:
            oscillation_index = peak_mag / math.sqrt(avg_power)

    return {
        'n_frames': n,
        'tilt_mean': tilt_mean,
        'tilt_std': tilt_std,
        'iq_rms': iq_rms,
        'overshoot': overshoot,
        'settling_time': settling_time,
        'oscillation_index': oscillation_index,
        'jump_idx': jump_idx,
    }


def cost_function(m, w=(1.0, 2.0, 1.0, 0.5)):
    """代价函数: 越小越好
    w[0]: tilt_std 权重
    w[1]: overshoot 权重
    w[2]: settling_time 权重
    w[3]: control_effort (iq_rms) 权重
    """
    if m is None:
        return float('inf')

    settling_norm = min(m['settling_time'], 5.0) / 5.0  # 归一化到 [0,1]
    overshoot_norm = min(m['overshoot'], 10.0) / 10.0

    return (w[0] * m['tilt_std'] +
            w[1] * overshoot_norm +
            w[2] * settling_norm +
            w[3] * m['iq_rms'])


# ============================================================
# 离线分析
# ============================================================

def cmd_analyze(csv_path):
    """离线分析遥测数据"""
    print(f"\n{'='*60}")
    print(f"离线分析: {csv_path}")
    print(f"{'='*60}")

    frames = parse_offline_frames(csv_path)
    if not frames:
        # 尝试直接读 CSV (tune_telem 格式)
        print("尝试 tune_telem CSV 格式...")
        frames = []
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            header = next(reader, None)
            for row in reader:
                if len(row) >= 10:
                    try:
                        frames.append([float(v) for v in row[:10]])
                    except ValueError:
                        continue
        if not frames:
            print("ERROR: 无法解析遥测数据")
            return

    print(f"帧数: {len(frames)} (约 {len(frames)/200:.1f}s @ 200Hz)")

    metrics = compute_metrics(frames)
    if metrics is None:
        print("ERROR: 帧数不足")
        return

    n = metrics['n_frames']
    tilts = [f[0] for f in frames]
    gyros = [f[1] for f in frames]
    bal_outs = [f[2] for f in frames]
    iq_R = [f[5] for f in frames]
    iq_L = [f[6] for f in frames]
    targets = [f[7] for f in frames]

    print(f"\n--- 基本统计 ---")
    print(f"  倾角: mean={metrics['tilt_mean']:.3f}° std={metrics['tilt_std']:.3f}°")
    print(f"  倾角: min={min(tilts):.2f}° max={max(tilts):.2f}°")
    print(f"  角速度 RMS: {math.sqrt(sum(g*g for g in gyros)/n):.1f} °/s")
    print(f"  平衡输出 RMS: {math.sqrt(sum(b*b for b in bal_outs)/n):.0f} RPM")
    print(f"  电流 RMS: {metrics['iq_rms']:.3f} A")

    # 倾角分布
    tilt_abs = [abs(t) for t in tilts]
    tilt_abs.sort()
    print(f"\n--- 倾角分布 ---")
    for pct in [50, 68, 90, 95, 99]:
        idx = int(n * pct / 100)
        if idx < n:
            print(f"  P{pct}: ≤{tilt_abs[idx]:.3f}°")

    # 频谱分析
    if n >= 256:
        print(f"\n--- 频谱分析 (FFT) ---")
        detrend = [t - metrics['tilt_mean'] for t in tilts]
        window = [0.5 * (1 - math.cos(2 * math.pi * i / (n - 1))) for i in range(n)]
        sig = [detrend[i] * window[i] for i in range(n)]
        nfft = 1
        while nfft < n:
            nfft *= 2
        sig_padded = sig + [0.0] * (nfft - n)

        spectrum = []
        for k in range(nfft // 2):
            re = sum(sig_padded[i] * math.cos(2 * math.pi * k * i / nfft) for i in range(nfft))
            im = sum(sig_padded[i] * math.sin(-2 * math.pi * k * i / nfft) for i in range(nfft))
            spectrum.append(math.sqrt(re * re + im * im))

        # 找前 5 个峰值
        peaks = []
        for k in range(1, len(spectrum) - 1):
            freq = k * 200.0 / nfft
            if 0.3 < freq < 50:
                if spectrum[k] > spectrum[k - 1] and spectrum[k] > spectrum[k + 1]:
                    peaks.append((freq, spectrum[k]))
        peaks.sort(key=lambda x: -x[1])

        print(f"  主要频率成分:")
        for freq, mag in peaks[:5]:
            print(f"    {freq:.2f} Hz  magnitude={mag:.3f}")

        print(f"\n  振荡指数: {metrics['oscillation_index']:.2f} (越低越稳定)")

    # 扰动响应分析
    jump_idx = metrics['jump_idx']
    if jump_idx is not None:
        print(f"\n--- 扰动响应 (target_angle 跳变 @ 帧{jump_idx}) ---")
        print(f"  超调量: {metrics['overshoot']:.2f}°")
        print(f"  稳定时间: {metrics['settling_time']:.2f}s")

    # 代价
    cost = cost_function(metrics)
    print(f"\n--- 综合代价 ---")
    print(f"  J = {cost:.4f} (越小越好)")
    print(f"    tilt_std={metrics['tilt_std']:.3f}° ×1.0 = {metrics['tilt_std']:.3f}")
    ov_n = min(metrics['overshoot'], 10.0) / 10.0
    st_n = min(metrics['settling_time'], 5.0) / 5.0
    print(f"    overshoot_norm={ov_n:.3f} ×2.0 = {2.0*ov_n:.3f}")
    print(f"    settling_norm={st_n:.3f} ×1.0 = {st_n:.3f}")
    print(f"    iq_rms={metrics['iq_rms']:.3f}A ×0.5 = {0.5*metrics['iq_rms']:.3f}")

    return metrics


# ============================================================
# 敏感度分析
# ============================================================

def cmd_sensitivity(csv_path):
    """分析为什么参数变化不敏感"""
    print(f"\n{'='*60}")
    print(f"平衡 PD 参数敏感度分析")
    print(f"{'='*60}")

    # 理论分析: 直接力矩控制架构
    print(f"\n【控制架构】")
    print(f"  balance_out(RPM) = Kp × tilt_err(°) - Kd × gyro_filt(°/s)")
    print(f"  iq_ref(A) = {BALANCE_DIRECT_GAIN} × balance_out, 限幅 ±{CURRENT_LIMIT}A")
    print(f"")

    # 1. 电流饱和分析
    print(f"【1. 电流饱和分析】")
    print(f"  Kp=20 → iq饱和在 tilt_err = {CURRENT_LIMIT/(20*BALANCE_DIRECT_GAIN):.1f}°")
    print(f"  Kp=30 → iq饱和在 tilt_err = {CURRENT_LIMIT/(30*BALANCE_DIRECT_GAIN):.1f}°")
    print(f"  Kp=40 → iq饱和在 tilt_err = {CURRENT_LIMIT/(40*BALANCE_DIRECT_GAIN):.1f}°")
    print(f"  Kp=60 → iq饱和在 tilt_err = {CURRENT_LIMIT/(60*BALANCE_DIRECT_GAIN):.1f}°")
    print(f"  结论: 倾角 >4°时 Kp=20~60 全部饱和, 响应一致")
    print(f"")

    # 2. D 项贡献分析
    print(f"【2. D 项 (角速度阻尼) 贡献分析】")
    print(f"  假设平衡车固有振荡频率 2-5Hz, 振幅 2°")
    for freq in [2.0, 3.0, 5.0]:
        gyro_peak = 2.0 * 2 * math.pi * freq  # 峰值角速度
        d_torque = 3.0 * gyro_peak * BALANCE_DIRECT_GAIN  # Kd=3 时的 D 力矩
        p_torque_20 = 20 * 2.0 * BALANCE_DIRECT_GAIN  # Kp=20 时的 P 力矩
        p_torque_40 = 40 * 2.0 * BALANCE_DIRECT_GAIN  # Kp=40 时的 P 力矩
        print(f"  {freq}Hz: gyro_peak={gyro_peak:.0f}°/s, "
              f"D力矩={d_torque:.2f}A, "
              f"P20力矩={p_torque_20:.2f}A, "
              f"P40力矩={p_torque_40:.2f}A")
    print(f"  结论: 在高频振荡时 D 项力矩 > P 项力矩, P 变化被 D 项掩盖")
    print(f"")

    # 3. 有效刚度对比
    print(f"【3. 等效刚度对比 (小角度线性区)】")
    for kp in [10, 20, 30, 40, 60, 80]:
        effective_gain = kp * BALANCE_DIRECT_GAIN
        print(f"  Kp={kp:3d}: {effective_gain:.3f} A/° → 在 1°误差时 = {effective_gain:.2f}A")
    print(f"  结论: 1°倾角误差时 Kp=20 vs Kp=40 的力矩差仅 {40*BALANCE_DIRECT_GAIN - 20*BALANCE_DIRECT_GAIN:.2f}A")
    print(f"        这个差值对电机-轮子-地面系统的加速度差异很小")
    print(f"")

    # 4. 速度漂移抑制的影响
    print(f"【4. 速度漂移抑制影响】")
    print(f"  SPEED_DRIFT_KP = 0.03 °/RPM")
    print(f"  当车轮平均速度 100RPM 时: target_angle 偏置 = 0.03×100 = 3°")
    print(f"  这相当于给系统加了一个额外的角度偏置来抑制速度漂移")
    print(f"  此机制会平滑速度波动, 进一步减小 Kp 变化的感知差异")
    print(f"")

    # 5. 物理系统限制
    print(f"【5. 物理系统限制】")
    print(f"  电机转矩常数 Kt: 决定电流→转矩的转换效率")
    print(f"  轮地摩擦: 静摩擦可能使小电流变化无法产生有效轮速变化")
    print(f"  电机 dead-zone: 小电流(<0.1A)可能无法克服齿槽转矩")
    print(f"")

    # 如果有离线数据, 做实际敏感度分析
    frames = []
    if csv_path and os.path.exists(csv_path):
        frames = parse_offline_frames(csv_path)
        if not frames:
            try:
                with open(csv_path, 'r') as f:
                    reader = csv.reader(f)
                    next(reader, None)
                    for row in reader:
                        if len(row) >= 10:
                            try:
                                frames.append([float(v) for v in row[:10]])
                            except ValueError:
                                continue
            except:
                pass

    if frames and len(frames) > 100:
        print(f"【6. 实际数据验证 ({len(frames)} 帧)】")
        tilts = [f[0] for f in frames]
        gyros = [f[1] for f in frames]
        bal_outs = [f[2] for f in frames]
        iq_R = [f[5] for f in frames]
        iq_L = [f[6] for f in frames]

        # 计算各分量贡献
        tilt_rms = math.sqrt(sum(t*t for t in tilts)/len(tilts))
        gyro_rms = math.sqrt(sum(g*g for g in gyros)/len(gyros))
        bal_rms = math.sqrt(sum(b*b for b in bal_outs)/len(bal_outs))

        # 估算 P 和 D 分量的相对贡献 (假设当前 Kp=40, Kd=3)
        Kp_assumed = 40
        Kd_assumed = 3
        p_rms = Kp_assumed * tilt_rms
        d_rms = Kd_assumed * gyro_rms

        print(f"  tilt RMS = {tilt_rms:.2f}°")
        print(f"  gyro RMS = {gyro_rms:.1f} °/s")
        print(f"  balance_out RMS = {bal_rms:.0f} RPM")
        print(f"  估算 P 分量 RMS = {p_rms:.0f} RPM ({100*p_rms/(p_rms+d_rms):.0f}%)")
        print(f"  估算 D 分量 RMS = {d_rms:.0f} RPM ({100*d_rms/(p_rms+d_rms):.0f}%)")

        # 电流饱和统计
        iq_abs = [abs(f[5]) + abs(f[6]) for f in frames]
        sat_count = sum(1 for iq in iq_abs if iq > 1.9)
        print(f"  电流饱和帧: {sat_count}/{len(frames)} ({100*sat_count/len(frames):.1f}%)")

        # 有效刚度
        tilt_peak_rms = math.sqrt(sum(t*t for t in tilts if abs(t) > 1.0) /
                                  max(1, sum(1 for t in tilts if abs(t) > 1.0)))
        print(f"  >1°倾角的 RMS = {tilt_peak_rms:.2f}°")
        print(f"  此倾角下 Kp=20 力矩 = {20*tilt_peak_rms*BALANCE_DIRECT_GAIN:.2f}A")
        print(f"  此倾角下 Kp=40 力矩 = {40*tilt_peak_rms*BALANCE_DIRECT_GAIN:.2f}A")

        if sat_count / len(frames) > 0.05:
            print(f"\n  *** 关键发现: {100*sat_count/len(frames):.1f}% 的帧电流饱和!")
            print(f"  电流饱和使不同 Kp 值的响应在高倾角时完全相同")
            print(f"  建议: 增大 CURRENT_LIMIT 或减小 BALANCE_DIRECT_GAIN 以扩大线性区")


# ============================================================
# 系统辨识 + 模型扫描搜索
# ============================================================

def identify_system(frames, gain=BALANCE_DIRECT_GAIN, fs=200.0):
    """从扰动数据中辨识倒立摆模型: θ̈ = a·θ + b·u
    返回 (a, b, r_squared)
    """
    if len(frames) < 100:
        return None, None, 0

    tilts = [f[0] for f in frames]       # θ (°)
    gyros = [f[1] for f in frames]       # θ̇ (°/s)
    iq_R = [f[5] for f in frames]        # 右轮电流 (A)
    iq_L = [f[6] for f in frames]        # 左轮电流 (A)

    dt = 1.0 / fs

    # 计算角加速度 θ̈ ≈ Δgyro/Δt (中心差分)
    X_theta = []  # θ
    X_u = []      # u = iq (A)
    Y_ddtheta = []  # θ̈

    for i in range(2, len(frames) - 2):
        # 中心差分计算角加速度
        ddtheta = (gyros[i+2] - gyros[i-2]) / (4 * dt)
        theta = tilts[i]
        u = (iq_R[i] + iq_L[i]) / 2.0  # 平均电流

        X_theta.append(theta)
        X_u.append(u)
        Y_ddtheta.append(ddtheta)

    if len(X_theta) < 50:
        return None, None, 0

    # 多元线性回归: θ̈ = a·θ + b·u
    n = len(X_theta)
    sum_x1x1 = sum(t*t for t in X_theta)
    sum_x2x2 = sum(u*u for u in X_u)
    sum_x1x2 = sum(X_theta[i]*X_u[i] for i in range(n))
    sum_x1y  = sum(X_theta[i]*Y_ddtheta[i] for i in range(n))
    sum_x2y  = sum(X_u[i]*Y_ddtheta[i] for i in range(n))
    sum_yy   = sum(y*y for y in Y_ddtheta)

    det = sum_x1x1 * sum_x2x2 - sum_x1x2 * sum_x1x2
    if abs(det) < 1e-12:
        return None, None, 0

    a = (sum_x2x2 * sum_x1y - sum_x1x2 * sum_x2y) / det
    b = (sum_x1x1 * sum_x2y - sum_x1x2 * sum_x1y) / det

    # R²
    y_mean = sum(Y_ddtheta) / n
    ss_res = sum((Y_ddtheta[i] - (a*X_theta[i] + b*X_u[i]))**2 for i in range(n))
    ss_tot = sum((y - y_mean)**2 for y in Y_ddtheta)
    r2 = 1 - ss_res / ss_tot if ss_tot > 1e-12 else 0

    return a, b, r2


def simulate_closed_loop(a, b, kp, kd, gain=BALANCE_DIRECT_GAIN,
                         theta0=5.0, dt=0.005, duration=3.0):
    """仿真倒立摆 PD 闭环阶跃响应
    初始条件: θ(0)=theta0, θ̇(0)=0
    返回 (t_arr, theta_arr, u_arr, metrics)
    """
    Kp_eff = gain * kp   # 有效 P 增益: A/°
    Kd_eff = gain * kd   # 有效 D 增益: A/(°/s)

    n = int(duration / dt)
    t = [0.0]
    theta = [theta0]
    gyro = [0.0]
    u = [0.0]

    for i in range(n):
        # PD 控制
        u_i = -Kp_eff * theta[-1] - Kd_eff * gyro[-1]
        # 限幅 ±2A
        if u_i > 2.0:
            u_i = 2.0
        elif u_i < -2.0:
            u_i = -2.0

        # RK4 积分
        def dyn(th, g, u_val):
            dth = g
            dg = a * th + b * u_val
            return dth, dg

        th0, g0 = theta[-1], gyro[-1]
        k1_th, k1_g = dyn(th0, g0, u_i)
        k2_th, k2_g = dyn(th0 + 0.5*dt*k1_th, g0 + 0.5*dt*k1_g, u_i)
        k3_th, k3_g = dyn(th0 + 0.5*dt*k2_th, g0 + 0.5*dt*k2_g, u_i)
        k4_th, k4_g = dyn(th0 + dt*k3_th, g0 + dt*k3_g, u_i)

        next_th = th0 + (dt/6)*(k1_th + 2*k2_th + 2*k3_th + k4_th)
        next_g  = g0  + (dt/6)*(k1_g  + 2*k2_g  + 2*k3_g  + k4_g)

        t.append(t[-1] + dt)
        theta.append(next_th)
        gyro.append(next_g)
        u.append(u_i)

    # 计算性能指标
    theta_abs = [abs(th) for th in theta]

    # 稳定时间: |θ| < 0.5° 且持续
    settling_idx = len(theta) - 1
    steady_count = 0
    for i in range(len(theta)):
        if abs(theta[i]) < 0.5:
            steady_count += 1
            if steady_count >= int(0.1 / dt):  # 持续 100ms
                settling_idx = i - int(0.1 / dt)
                break
        else:
            steady_count = 0

    settling_time = settling_idx * dt

    # 超调量 (第二个峰的幅度)
    overshoot = 0.0
    sign = 1 if theta0 > 0 else -1
    prev_th = theta[0]
    peaks = []
    for i in range(1, len(theta)-1):
        if sign * theta[i] > sign * theta[i-1] and sign * theta[i] > sign * theta[i+1]:
            peaks.append(theta[i])
        elif sign * theta[i] < sign * theta[i-1] and sign * theta[i] < sign * theta[i+1]:
            peaks.append(theta[i])

    # 振荡次数 (穿越零点的次数 / 2)
    zero_crossings = 0
    for i in range(1, len(theta)):
        if theta[i-1] * theta[i] < 0:
            zero_crossings += 1
    oscillation_count = max(0, zero_crossings - 1)  # 减去初始穿越

    # 积分指标
    ise = sum(th*th for th in theta) * dt  # ISE
    u_rms = math.sqrt(sum(ui*ui for ui in u) / len(u))

    return {
        'settling_time': settling_time,
        'overshoot': max(theta_abs) - abs(theta0),
        'oscillation_count': oscillation_count,
        'ISE': ise,
        'u_rms': u_rms,
        'stable': abs(theta[-1]) < 0.5 and abs(gyro[-1]) < 1.0,
    }


def cmd_search(csv_path):
    """基于系统辨识 + 模型仿真的离线参数搜索"""
    print(f"\n{'='*60}")
    print(f"模型驱动 PD 参数搜索")
    print(f"{'='*60}")

    # 1. 加载数据
    frames = None
    if csv_path and os.path.exists(csv_path):
        frames = parse_offline_frames(csv_path)
        if not frames:
            try:
                frames = []
                with open(csv_path, 'r') as f:
                    reader = csv.reader(f)
                    next(reader, None)
                    for row in reader:
                        if len(row) >= 10:
                            try:
                                frames.append([float(v) for v in row[:10]])
                            except ValueError:
                                continue
            except:
                pass

    if not frames or len(frames) < 100:
        print("ERROR: 无法加载数据或帧数不足")
        return

    print(f"数据: {len(frames)} 帧 @ 200Hz")

    # 2. 系统辨识
    print(f"\n--- 系统辨识 ---")
    a, b, r2 = identify_system(frames)

    if a is None:
        print("ERROR: 系统辨识失败 (数据激励不足)")
        return

    print(f"  模型: theta_ddot = a*theta + b*u")
    print(f"  a = {a:.4f}  (重力项, >0 表示不稳定)")
    print(f"  b = {b:.4f}  (控制效率, A per deg/s^2)")
    print(f"  R^2 = {r2:.3f}")

    if a <= 0:
        print(f"  WARNING: a <= 0, 模型可能不准, 使用典型值 a=20")
        a = 20.0
    if b <= 0:
        print(f"  WARNING: b <= 0, 控制方向错误或数据不足, 使用典型值 b=15")
        b = 15.0

    # 稳定性条件: b·GAIN·Kp > a → Kp > a/(b·GAIN)
    kp_min = a / (b * BALANCE_DIRECT_GAIN)
    print(f"\n  稳定性条件: Kp > {kp_min:.1f} (当前 Kp=40)")

    # 3. 参数网格扫描
    print(f"\n--- 参数扫描 (基于模型仿真) ---")
    print(f"  初始条件: theta0 = 5 deg (模拟推搡)")
    kp_vals = list(range(max(5, int(kp_min)-5), 101, 5))
    kd_vals = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 8.0, 10.0]

    results = []
    for kp in kp_vals:
        for kd in kd_vals:
            sim = simulate_closed_loop(a, b, kp, kd, theta0=5.0)
            if sim is None:
                continue

            # 综合代价: 稳定快 + 超调小 + 振荡少 + 控制代价低
            cost = (sim['settling_time'] * 0.5 +
                    max(0, sim['overshoot']) * 2.0 +
                    sim['oscillation_count'] * 3.0 +
                    sim['ISE'] * 0.1 +
                    sim['u_rms'] * 0.5)

            # 不稳定惩罚
            if not sim['stable']:
                cost += 100

            results.append({
                'kp': kp, 'kd': kd,
                'sim': sim, 'cost': cost
            })

    results.sort(key=lambda r: r['cost'])

    # 4. 输出
    print(f"\n{'='*60}")
    print(f"Top 15 参数组合 (5° 初始扰动)")
    print(f"{'='*60}")
    print(f"{'Rank':<5} {'Kp':<6} {'Kd':<6} {'稳定时间':<9} {'超调':<7} {'振荡次数':<9} {'ISE':<8} {'u_rms':<7} {'代价':<7}")
    print("-" * 70)
    for i, r in enumerate(results[:15]):
        s = r['sim']
        stable_mark = '' if s['stable'] else ' ✗'
        print(f"{i+1:<5} {r['kp']:<6.0f} {r['kd']:<6.1f} "
              f"{s['settling_time']:<9.3f} {max(0,s['overshoot']):<7.2f} "
              f"{s['oscillation_count']:<9.0f} {s['ISE']:<8.2f} {s['u_rms']:<7.3f} "
              f"{r['cost']:<7.2f}{stable_mark}")

    best = results[0]
    print(f"\n推荐参数: Kp={best['kp']:.0f}  Kd={best['kd']:.1f}")
    print(f"设置命令: python tune_balance.py set ANG={best['kp']:.0f} GYR={best['kd']:.1f}")

    # 5. 当前参数 vs 最优参数对比
    print(f"\n--- 当前参数 vs 推荐参数 对比 ---")
    for label, kp, kd in [("当前 Kp=40 Kd=3", 40, 3.0),
                            (f"推荐 Kp={best['kp']:.0f} Kd={best['kd']:.1f}", best['kp'], best['kd'])]:
        sim = simulate_closed_loop(a, b, kp, kd, theta0=5.0)
        print(f"  {label}:")
        print(f"    稳定时间: {sim['settling_time']:.3f}s")
        print(f"    超调: {max(0,sim['overshoot']):.2f}°")
        print(f"    振荡次数: {sim['oscillation_count']:.0f}")
        print(f"    ISE: {sim['ISE']:.2f}")
        print(f"    u_rms: {sim['u_rms']:.3f}A")

    # 6. 敏感度热力图数据
    print(f"\n--- 最优区域分析 ---")
    print(f"  围绕最优参数的附近组合:")
    for r in results[:10]:
        s = r['sim']
        if abs(r['kp'] - best['kp']) <= 10 and abs(r['kd'] - best['kd']) <= 2:
            print(f"    Kp={r['kp']:.0f} Kd={r['kd']:.1f} "
                  f"cost={r['cost']:.2f} settle={s['settling_time']:.3f}s")

    return best


# ============================================================
# 在线自动调参
# ============================================================

def test_parameters(ser, kp, kd, duration=5.0, do_perturb=True):
    """测试一组 PD 参数, 返回性能指标"""
    print(f"  测试 Kp={kp:.1f} Kd={kd:.1f} ...", end=' ', flush=True)

    set_pd(ser, kp, kd)
    time.sleep(0.5)

    def perturb(ser):
        """施加 target_angle 脉冲作为虚拟推搡"""
        send_cmd(ser, 'PK ANG0=3.0')
        time.sleep(0.3)
        send_cmd(ser, 'PK ANG0=0.0')

    frames = collect_telem(ser, duration, perturb if do_perturb else None)

    if len(frames) < 50:
        print(f"帧数不足({len(frames)})")
        return None, frames

    metrics = compute_metrics(frames)
    if metrics is None:
        print(f"无法计算指标")
        return None, frames

    cost = cost_function(metrics)
    print(f"tilt_std={metrics['tilt_std']:.3f}° "
          f"iq_rms={metrics['iq_rms']:.3f}A "
          f"osc_idx={metrics['oscillation_index']:.1f} "
          f"cost={cost:.4f}")
    return metrics, frames


def cmd_tune(port=None, duration_per_test=5.0):
    """在线自动搜索最优 PD 参数"""
    if port is None:
        port = find_port()

    print(f"\n{'='*60}")
    print(f"平衡车 PD 参数自动整定")
    print(f"{'='*60}")
    print(f"串口: {port}")
    print(f"每组测试时长: {duration_per_test}s")
    print(f"")

    ser = serial.Serial(port, BAUD, timeout=0.1)

    # 确保平衡已激活
    print("检查平衡状态...")
    resp = send_cmd(ser, 'P')
    print(f"  {resp.strip()[:100]}")

    if 'INACTIVE' in resp:
        print("发送 B 激活平衡控制...")
        send_cmd(ser, 'B')
        time.sleep(1.0)

    all_results = []

    # ===== Phase 1: 粗网格搜索 =====
    print(f"\n--- Phase 1: 粗网格搜索 ---")
    kp_range = [10, 20, 30, 40, 50, 60, 80]
    kd_range = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0]

    print(f"Kp ∈ {kp_range}")
    print(f"Kd ∈ {kd_range}")
    print(f"总计 {len(kp_range) * len(kd_range)} 组, 预计 {len(kp_range) * len(kd_range) * (duration_per_test + 1) / 60:.0f} 分钟")
    print()

    for kp in kp_range:
        for kd in kd_range:
            metrics, frames = test_parameters(ser, kp, kd, duration_per_test)
            if metrics:
                all_results.append({
                    'kp': kp, 'kd': kd,
                    'cost': cost_function(metrics),
                    'metrics': metrics,
                    'frames': frames
                })

    if not all_results:
        print("ERROR: 无有效测试结果")
        ser.close()
        return

    # 排序找最优
    all_results.sort(key=lambda r: r['cost'])

    print(f"\n--- 粗搜索 Top 5 ---")
    for i, r in enumerate(all_results[:5]):
        m = r['metrics']
        print(f"  #{i+1}: Kp={r['kp']:.0f} Kd={r['kd']:.1f} "
              f"cost={r['cost']:.4f} "
              f"tilt_std={m['tilt_std']:.3f}° "
              f"iq_rms={m['iq_rms']:.3f}A "
              f"osc={m['oscillation_index']:.1f}")

    # ===== Phase 2: 局部精细搜索 =====
    best = all_results[0]
    print(f"\n--- Phase 2: 局部精细搜索 (围绕 Kp={best['kp']:.0f} Kd={best['kd']:.1f}) ---")

    fine_kp = []
    for dk in [-15, -10, -5, -2, 0, 2, 5, 10, 15]:
        kp_val = best['kp'] + dk
        if 5 <= kp_val <= 100:
            fine_kp.append(kp_val)
    fine_kp = sorted(set(fine_kp))

    fine_kd = []
    for dd in [-1.5, -1.0, -0.5, -0.2, 0, 0.2, 0.5, 1.0, 1.5]:
        kd_val = best['kd'] + dd
        if 0.5 <= kd_val <= 10:
            fine_kd.append(kd_val)
    fine_kd = sorted(set(fine_kd))

    print(f"Kp ∈ {fine_kp}")
    print(f"Kd ∈ {fine_kd}")

    fine_results = []
    for kp in fine_kp:
        for kd in fine_kd:
            # 跳过粗搜索已测过的点
            already = any(abs(r['kp'] - kp) < 0.1 and abs(r['kd'] - kd) < 0.1 for r in all_results)
            if already:
                continue
            metrics, frames = test_parameters(ser, kp, kd, duration_per_test)
            if metrics:
                fine_results.append({
                    'kp': kp, 'kd': kd,
                    'cost': cost_function(metrics),
                    'metrics': metrics
                })

    all_results.extend(fine_results)
    all_results.sort(key=lambda r: r['cost'])

    print(f"\n{'='*60}")
    print(f"最终结果: Top 10")
    print(f"{'='*60}")
    for i, r in enumerate(all_results[:10]):
        m = r['metrics']
        flag = " ← 最优" if i == 0 else ""
        print(f"  #{i+1}: Kp={r['kp']:.1f} Kd={r['kd']:.1f} "
              f"cost={r['cost']:.4f} "
              f"tilt_std={m['tilt_std']:.3f}° "
              f"iq_rms={m['iq_rms']:.3f}A{flag}")

    best = all_results[0]
    print(f"\n推荐参数: PK ANG={best['kp']:.1f} GYR={best['kd']:.1f}")
    print(f"设置命令: python tune_balance.py set ANG={best['kp']:.1f} GYR={best['kd']:.1f}")

    # 应用最优参数
    print(f"\n正在应用最优参数...")
    set_pd(ser, best['kp'], best['kd'])
    resp = send_cmd(ser, 'PK')
    print(f"  {resp.strip()}")

    # 保存结果
    out_csv = f'data/auto_tune_results_{time.strftime("%Y%m%d_%H%M%S")}.csv'
    with open(out_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['rank', 'kp', 'kd', 'cost', 'tilt_std', 'iq_rms', 'overshoot',
                    'settling_time', 'oscillation_index', 'phase'])
        for i, r in enumerate(all_results[:20]):
            m = r['metrics']
            phase = 'fine' if r in fine_results else 'coarse'
            w.writerow([i+1, r['kp'], r['kd'], r['cost'], m['tilt_std'], m['iq_rms'],
                        m['overshoot'], m['settling_time'], m['oscillation_index'], phase])
    print(f"结果已保存: {out_csv}")

    ser.close()
    return best


# ============================================================
# 主入口
# ============================================================

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    action = sys.argv[1]

    if action == 'analyze':
        path = sys.argv[2] if len(sys.argv) > 2 else None
        if not path:
            # 找最新 CSV
            data_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'data')
            csvs = sorted([f for f in os.listdir(data_dir) if f.endswith('.csv') and 'serial' in f])
            if csvs:
                path = os.path.join(data_dir, csvs[-1])
                print(f"使用最新数据: {path}")
            else:
                print("未找到 CSV 文件")
                sys.exit(1)
        cmd_analyze(path)

    elif action == 'sensitivity':
        path = sys.argv[2] if len(sys.argv) > 2 else None
        if not path:
            data_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'data')
            csvs = sorted([f for f in os.listdir(data_dir) if f.endswith('.csv') and 'serial' in f])
            if csvs:
                path = os.path.join(data_dir, csvs[-1])
        cmd_sensitivity(path)

    elif action == 'search':
        path = sys.argv[2] if len(sys.argv) > 2 else None
        if not path:
            data_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'data')
            csvs = sorted([f for f in os.listdir(data_dir) if f.endswith('.csv') and 'test' in f])
            if csvs:
                path = os.path.join(data_dir, csvs[-1])
                print(f"使用最新数据: {path}")
            else:
                print("未找到数据文件")
                sys.exit(1)
        cmd_search(path)

    elif action == 'tune':
        port = None
        rest = sys.argv[2:]
        for i, a in enumerate(rest):
            if a == '--port' and i + 1 < len(rest):
                port = rest[i + 1]
                break
            elif a.upper().startswith('COM'):
                port = a.upper()
                break
        cmd_tune(port)

    else:
        print(f"未知操作: {action}")
        print(__doc__)


if __name__ == '__main__':
    main()
