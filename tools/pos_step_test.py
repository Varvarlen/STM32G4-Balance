"""
位置环阶跃测试 — 采集遥测 + burst 数据, 分析位置响应

用法:
    python pos_step_test.py                # 自动模式 (M1 0→15° 30° 45° 60°)
    python pos_step_test.py COM5           # 指定串口
    python pos_step_test.py --manual       # 手动输入命令

手动命令:
    RP<deg>      M1: 旋转到绝对角度
    LP<deg>      M2: 旋转到绝对角度
    RP<deg> <deg> M1: 位置阶跃 from→to
    q            退出

分析指标:
    rise (10-90%), overshoot (%), settling ±5%, 稳态误差
"""

import serial, serial.tools.list_ports, struct, csv, sys, time, os, threading
import numpy as np
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

BURST_MAGIC = b'RBUS'
FRAME_FLOATS = 10
FOOTER = b'\x00\x00\x80\x7F'

TELEM_COLS = [
    't_ms',
    'M1_pos_ref', 'M1_pos_est', 'M1_speed_fb', 'M1_iq', 'M1_speed_ref',
    'M2_pos_ref', 'M2_pos_est', 'M2_speed_fb', 'M2_iq', 'M2_speed_ref',
]
BURST_COLS = ['t_ms', 'speed_fb', 'iq', 'speed_ref', 'T_load_est']

# 默认阶跃序列 (°)
AUTO_SEQUENCE = [
    "RP15",
    "RP30",
    "RP45",
    "RP60",
    "RP-15",
]
AUTO_INTERVAL = 6.0  # 每条命令间隔(秒), 位置移动+settle 需更长时间

STOP = False
buf_lock = threading.Lock()
raw_bytes = bytearray()
burst_blocks = []
telem_rows = []
telem_drop = 0
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(SCRIPT_DIR, 'figures', 'pos_step')
os.makedirs(OUT_DIR, exist_ok=True)
DATA_DIR = os.path.join(SCRIPT_DIR, 'data')
os.makedirs(DATA_DIR, exist_ok=True)


def parse_float_frame(buf):
    idx = buf.find(FOOTER)
    if idx < 0 or idx < 40:
        return None, 0
    data = buf[idx - 40:idx]
    try:
        return struct.unpack(f'<{FRAME_FLOATS}f', data), idx + 4
    except struct.error:
        return None, idx + 4


def serial_reader(ser):
    global STOP, raw_bytes
    while not STOP:
        try:
            w = ser.in_waiting
            if w > 0:
                with buf_lock:
                    raw_bytes.extend(ser.read(w))
        except serial.SerialException:
            break
        time.sleep(0.001)


def extract_and_echo():
    global raw_bytes, burst_blocks, telem_rows, telem_drop

    with buf_lock:
        data = bytes(raw_bytes)
        new_start = 0
        last_frame_end = 0

        while new_start < len(data):
            burst_idx = data.find(BURST_MAGIC, new_start)
            sentinel_idx = data.find(FOOTER, new_start)

            if burst_idx >= 0:
                if burst_idx > last_frame_end:
                    echo_mcu_text(data[last_frame_end:burst_idx])

                if len(data) >= burst_idx + 8:
                    count = struct.unpack('<H', data[burst_idx+4:burst_idx+6])[0]
                    fields = data[burst_idx+6]
                    data_bytes = count * fields * 4
                    total = 8 + data_bytes
                    if len(data) >= burst_idx + total:
                        block = data[burst_idx+8:burst_idx+total]
                        rows = []
                        for i in range(count):
                            off = i * fields * 4
                            vals = struct.unpack(f'<{fields}f', block[off:off+fields*4])
                            rows.append(vals)
                        burst_blocks.append((fields, rows))
                        last_frame_end = burst_idx + total
                        new_start = last_frame_end
                        continue
                break

            elif sentinel_idx >= 0 and sentinel_idx >= 40:
                frame_start = sentinel_idx - 40

                if frame_start > last_frame_end:
                    gap = data[last_frame_end:frame_start]
                    echo_mcu_text(gap)

                frame_data = data[frame_start:sentinel_idx]
                try:
                    floats = struct.unpack(f'<{FRAME_FLOATS}f', frame_data)
                    telem_rows.append(floats)
                except struct.error:
                    telem_drop += 1

                last_frame_end = sentinel_idx + 4
                new_start = last_frame_end
                continue

            elif sentinel_idx >= 0 and sentinel_idx < 40:
                last_frame_end = sentinel_idx + 4
                new_start = last_frame_end
                telem_drop += 1
                continue

            else:
                break

        raw_bytes = bytearray(data[new_start:])


def echo_mcu_text(chunk):
    text = chunk.decode('utf-8', errors='replace')
    i = 0
    while i < len(text):
        nl = text.find('\n', i)
        if nl < 0:
            break
        line = text[i:nl].rstrip('\r')
        i = nl + 1
        if len(line) < 3 or len(line) > 200:
            continue
        if '\x00' in line:
            continue
        printable = sum(1 for c in line if 32 <= ord(c) <= 126 or c == '\t')
        if printable / len(line) > 0.85:
            print(f"  [MCU] {line}")


def auto_thread(ser):
    global STOP
    # 确保退出位置/速度模式, 回到电流模式
    ser.write(b'R0\r')
    time.sleep(0.5)
    while ser.in_waiting:
        ser.read(ser.in_waiting)
    # 开遥测
    ser.write(b'T\r')
    time.sleep(0.5)
    while ser.in_waiting:
        ser.read(ser.in_waiting)
    print(f"\n  Auto: {len(AUTO_SEQUENCE)} tests, ~{len(AUTO_SEQUENCE)*AUTO_INTERVAL:.0f}s\n")
    for i, cmd in enumerate(AUTO_SEQUENCE):
        if STOP:
            break
        print(f"  [{i+1}/{len(AUTO_SEQUENCE)}] {cmd} ...", end=' ', flush=True)
        try:
            ser.write((cmd + '\r').encode('ascii'))
        except serial.SerialException:
            break
        time.sleep(AUTO_INTERVAL)
    # 关遥测
    ser.write(b'T\r')
    time.sleep(0.3)


def keyboard_thread(ser):
    global STOP
    print("\n  RP<deg> / RP<from> <to> (M1)   LP<deg> / LP<from> <to> (M2)   q=quit\n")
    while not STOP:
        try:
            line = input()
        except (EOFError, OSError):
            break
        if STOP or not line:
            continue
        if line.lower() == 'q':
            STOP = True
            break
        try:
            ser.write((line + '\r').encode('ascii'))
            print(f"  -> sent: {line}")
        except serial.SerialException:
            break


def find_stlink_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if 'stlink' in (p.description + p.hwid).lower():
            return p.device
    return None


def analyze_position_response(rows, label):
    """分析位置阶跃响应 — 直接使用遥测帧 pos_est + pos_ref"""
    n = len(rows)
    if n < 100:
        return None

    DT = 0.005  # 200Hz
    t = np.arange(n) * DT * 1000  # ms

    # 新遥测帧 M1 字段: 0=pos_ref 1=pos_est 2=speed_fb 3=iq 4=speed_ref
    pos_ref = np.array([r[0] for r in rows])   # 位置环目标 (rad)
    pos_est = np.array([r[1] for r in rows])   # EKF 连续位置 (rad)
    speed_fb = np.array([r[2] for r in rows])
    iq = np.array([r[3] for r in rows])
    speed_ref = np.array([r[4] for r in rows])

    # 阶跃检测: pos_ref 突变
    pos_ref_chg = np.abs(np.diff(pos_ref))
    step_idx = int(np.argmax(pos_ref_chg)) + 1
    if step_idx < 10 or step_idx > n - 20:
        return None

    # 起始位置和目标
    pos_start = float(np.mean(pos_est[max(0, step_idx-20):step_idx]))
    ss_start = min(step_idx + 150, n - 50)
    pos_target = float(np.mean(pos_est[ss_start:]))
    step_rad = pos_target - pos_start
    step_deg = step_rad * 57.29578

    if abs(step_deg) < 0.5:
        return None

    # 转换为度用于分析
    pos_deg = pos_est * 57.29578
    target_deg = pos_target * 57.29578
    post_deg = pos_deg[step_idx:]

    # Rise 10-90%
    lo = pos_start * 57.29578 + 0.1 * step_deg
    hi = pos_start * 57.29578 + 0.9 * step_deg
    if step_deg > 0:
        a10 = int(np.argmax(post_deg >= lo))
        a90 = int(np.argmax(post_deg >= hi))
    else:
        a10 = int(np.argmax(post_deg <= lo))
        a90 = int(np.argmax(post_deg <= hi))
    rise = (a90 - a10) * DT * 1000 if a10 > 0 and a90 > 0 else None

    # Overshoot
    if step_deg > 0:
        overshoot = max(0.0, (float(np.max(post_deg)) - target_deg) / abs(step_deg) * 100)
    else:
        overshoot = max(0.0, (target_deg - float(np.min(post_deg))) / abs(step_deg) * 100)

    # Settling ±5% (容忍 0.5°)
    bound = max(abs(step_deg) * 0.05, 0.5)
    settle_ms = None
    for j in range(step_idx, n - 20):
        if all(abs(pos_deg[j:j+20] - target_deg) <= bound):
            settle_ms = (j - step_idx) * DT * 1000
            break

    # 稳态误差
    ss_err_deg = float(np.mean(pos_deg[-50:] - target_deg))

    # 振荡
    n_cross = 0
    above = post_deg[0] > target_deg
    for v in post_deg[1:]:
        now_above = v > target_deg
        if now_above != above:
            n_cross += 1
            above = now_above

    result = {
        'label': label, 'step_deg': step_deg, 'rise_ms': rise,
        'overshoot': overshoot, 'settle_ms': settle_ms,
        'ss_err_deg': ss_err_deg, 'n_cross': n_cross, 'step_idx': step_idx
    }

    if HAS_MPL:
        fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

        ax = axes[0]
        ax.plot(t, pos_ref * 57.29578, 'g--', lw=1.0, alpha=0.7, label='pos_ref')
        ax.plot(t, pos_deg, 'r', lw=1.5, label='pos_est')
        ax.axhline(target_deg, color='g', ls=':', lw=0.5)
        ax.axvline(t[step_idx], color='blue', lw=0.5, ls=':')
        if settle_ms:
            ax.axvline(t[step_idx] + settle_ms, color='orange', lw=0.8, ls='--',
                       label=f'Settle {settle_ms:.0f}ms')
        ax.set_ylabel('Position (deg)')
        ax.legend(fontsize=7)
        info = f'{label}  {step_deg:.1f}deg step'
        if rise: info += f'  rise={rise:.0f}ms'
        info += f'  OS={overshoot:.1f}%'
        if settle_ms: info += f'  settle={settle_ms:.0f}ms'
        info += f'  SS err={ss_err_deg:.3f}deg'
        ax.set_title(info)
        ax.grid(alpha=0.3)

        ax = axes[1]
        ax.plot(t, speed_ref, 'g--', lw=0.8, alpha=0.5, label='speed_ref (pos out)')
        ax.plot(t, speed_fb, 'b', lw=1.2, label='speed_fb')
        ax.axvline(t[step_idx], color='blue', lw=0.5, ls=':')
        ax.axhline(0, color='k', lw=0.5)
        ax.set_ylabel('Speed (RPM)')
        ax.legend(fontsize=7)
        ax.grid(alpha=0.3)

        ax = axes[2]
        ax.plot(t, iq, 'r', lw=1.0, label='iq')
        ax.axvline(t[step_idx], color='blue', lw=0.5, ls=':')
        ax.axhline(0, color='k', lw=0.5)
        ax.set_ylabel('Current (A)')
        ax.set_xlabel('Time (ms)')
        ax.legend(fontsize=7)
        ax.grid(alpha=0.3)

        plt.tight_layout()
        tag = label.replace(' ', '_')
        ts = time.strftime('%Y%m%d_%H%M%S')
        path = os.path.join(OUT_DIR, f'pos_step_{tag}_{ts}.png')
        plt.savefig(path, dpi=130)
        plt.close()
        result['path'] = path

    return result


def main():
    global STOP, raw_bytes, burst_blocks, telem_rows, telem_drop

    manual_mode = '--manual' in sys.argv

    port = None
    for a in sys.argv[1:]:
        if a not in ('--manual',):
            port = a
            break
    if port is None:
        port = find_stlink_port()
        if port is None:
            print("No STLink port found. Specify: python pos_step_test.py COM5")
            sys.exit(1)

    print(f"Using {port}")
    ser = serial.Serial(port, 230400, timeout=0.5)
    ser.reset_input_buffer()

    reader = threading.Thread(target=serial_reader, args=(ser,), daemon=True)
    reader.start()

    print("Waiting for MCU...")
    time.sleep(3.0)
    extract_and_echo()

    if manual_mode:
        cmd_thread = threading.Thread(target=keyboard_thread, args=(ser,), daemon=True)
    else:
        cmd_thread = threading.Thread(target=auto_thread, args=(ser,), daemon=True)
    cmd_thread.start()

    try:
        while not STOP:
            extract_and_echo()
            time.sleep(0.02)
            if not manual_mode and not cmd_thread.is_alive():
                time.sleep(3.0)
                STOP = True
    except KeyboardInterrupt:
        STOP = True

    time.sleep(0.5)
    ser.close()
    reader.join(timeout=1)
    extract_and_echo()

    ts = time.strftime('%Y%m%d_%H%M%S')

    if telem_rows:
        fname = f"pos_step_telem_{ts}.csv"
        path = os.path.join(DATA_DIR, fname)
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(TELEM_COLS)
            for i, row in enumerate(telem_rows):
                w.writerow([f"{i*5:.1f}"] + [f"{v:.6f}" for v in row])
        print(f"Telemetry: {path}  ({len(telem_rows)} frames @ 200Hz)")

    for bi, (fields, rows) in enumerate(burst_blocks):
        fname = f"pos_step_burst_{bi}_{ts}.csv"
        path = os.path.join(DATA_DIR, fname)
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(BURST_COLS[:fields+1])
            for i, row in enumerate(rows):
                w.writerow([f"{i*1.0:.1f}"] + [f"{v:.6f}" for v in row])
        print(f"Burst[{bi}]: {path}  ({len(rows)} samples @ 1kHz)")

    # 分析位置响应 — 多阶跃检测
    if telem_rows and not manual_mode:
        # 新遥测帧: 0=pos_ref 1=pos_est 2=speed_fb 3=iq 4=speed_ref
        pos_ref = np.array([r[0] for r in telem_rows])  # 位置环目标 (rad)
        pos_ref_chg = np.abs(np.diff(pos_ref))
        # 找所有显著速度变化 (>10 RPM)
        threshold = 0.05  # rad, 最小 ~3° 阶跃检测
        peaks = []
        for i in range(1, len(pos_ref_chg) - 1):
            if pos_ref_chg[i] > threshold and pos_ref_chg[i] >= pos_ref_chg[i-1] and pos_ref_chg[i] >= pos_ref_chg[i+1]:
                # 合并邻近峰 (100 帧 = 500ms 内只取最大)
                if not peaks or (i - peaks[-1]) > 100:
                    peaks.append(i + 1)  # step_idx (after diff)

        if not peaks:
            print(f"No pos step detected (max pos_ref_chg={np.max(pos_ref_chg):.3f} rad, threshold={threshold})")
        else:
            print(f"\n=== 位置响应分析 ({len(peaks)} 个阶跃) ===")
            for pi, pk in enumerate(peaks):
                # 分段: 从上一个阶跃间中点 到 下一个阶跃间中点
                seg_start = max(0, pk - 150) if pi == 0 else max(0, (peaks[pi-1] + pk) // 2)
                seg_end = min(len(telem_rows), pk + 300) if pi == len(peaks)-1 else (pk + peaks[pi+1]) // 2
                seg = telem_rows[seg_start:seg_end]
                cmd = AUTO_SEQUENCE[pi] if pi < len(AUTO_SEQUENCE) else f"step{pi+1}"
                result = analyze_position_response(seg, f"M1 {cmd}")
                if result:
                    r = result
                    r_str = f"{r['rise_ms']:.0f}ms" if r['rise_ms'] else '---'
                    s_str = f"{r['settle_ms']:.0f}ms" if r['settle_ms'] else '---'
                    print(f"  [{pi+1}] {cmd}: step={r['step_deg']:.1f}° "
                          f"rise={r_str} OS={r['overshoot']:.1f}% "
                          f"settle={s_str} SSerr={r['ss_err_deg']:.2f}°")
                else:
                    print(f"  [{pi+1}] {cmd}: 分析失败")

    if not telem_rows and not burst_blocks:
        print("No data captured.")
    else:
        print("Done.")


if __name__ == '__main__':
    main()
