"""
位置环 Kp 自动扫描 — 扫描多档 Kp, 每档跑 RP30 阶跃, 分析响应

用法: python pos_kp_scan.py [COM5]

输出: 对比表 (Kp, BW, rise, OS, settle, SSerr), 推荐最优 Kp
"""

import serial, serial.tools.list_ports, struct, time, sys, os
import numpy as np
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

BAUD = 230400
FRAME_FLOATS = 10
FOOTER = b'\x00\x00\x80\x7F'
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(SCRIPT_DIR, 'figures', 'kp_scan')
os.makedirs(OUT_DIR, exist_ok=True)

# 扫描序列
KP_VALUES = [60, 90, 120, 150, 180, 210, 240]
TEST_CMD = "RP30"
SETTLE_TIME = 4.0  # 阶跃后等待时间


def send_cmd(ser, cmd):
    """发送 ASCII 命令, 读取回显"""
    ser.reset_input_buffer()
    ser.write((cmd + '\r').encode('ascii'))
    time.sleep(0.3)
    buf = b''
    deadline = time.time() + 0.5
    while time.time() < deadline:
        n = ser.in_waiting
        if n > 0:
            buf += ser.read(n)
        time.sleep(0.01)
    return buf.decode('utf-8', errors='replace')


def collect_telem(ser, duration):
    """采集遥测帧"""
    data = bytearray()
    deadline = time.time() + duration
    while time.time() < deadline:
        n = ser.in_waiting
        if n > 0:
            data.extend(ser.read(n))
        time.sleep(0.01)
    return bytes(data)


def parse_telem(data):
    """解析 10-float 遥测帧"""
    frames = []
    pos = 0
    while pos < len(data) - 44:
        idx = data.find(FOOTER, pos)
        if idx < 0:
            break
        if idx >= 40:
            frame_data = data[idx - 40:idx]
            try:
                vals = struct.unpack(f'<{FRAME_FLOATS}f', frame_data)
                # 基本校验
                if abs(vals[2]) < 5000 and abs(vals[3]) < 10:
                    frames.append(vals)
            except struct.error:
                pass
        pos = idx + 4
    return frames


def analyze_step(frames, kp):
    """分析 RP30 阶跃: speed_fb 突变检测 → pos_est 响应分析"""
    n = len(frames)
    if n < 50:
        return None

    DT = 0.005
    t = np.arange(n) * DT * 1000  # ms

    pos_ref = np.array([f[0] for f in frames])
    pos_est = np.array([f[1] for f in frames])
    speed_fb = np.array([f[2] for f in frames])
    iq = np.array([f[3] for f in frames])
    speed_ref = np.array([f[4] for f in frames])

    # 阶跃检测: pos_ref 可能在遥测首帧前已跳变 → 用 speed_fb 突变检测
    spd_chg = np.abs(np.diff(speed_fb))
    step_idx = int(np.argmax(spd_chg)) + 1
    if step_idx < 10 or step_idx > n - 30:
        return None

    pos_start = float(np.mean(pos_est[max(0, step_idx-20):step_idx]))
    ss_start = min(step_idx + 120, n - 30)
    pos_target = float(np.mean(pos_est[ss_start:]))
    step_deg = (pos_target - pos_start) * 57.29578

    if abs(step_deg) < 0.5:
        return None

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
    rise_ms = (a90 - a10) * DT * 1000 if a10 > 0 and a90 > 0 else None

    # Overshoot
    if step_deg > 0:
        os_pct = max(0.0, (float(np.max(post_deg)) - target_deg) / abs(step_deg) * 100)
    else:
        os_pct = max(0.0, (target_deg - float(np.min(post_deg))) / abs(step_deg) * 100)

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

    # 实测 BW ≈ 0.35 / rise (s)
    bw_hz = 350.0 / rise_ms if rise_ms and rise_ms > 0 else None

    return {
        'kp': kp, 'step_deg': step_deg, 'rise_ms': rise_ms, 'os_pct': os_pct,
        'settle_ms': settle_ms, 'ss_err_deg': ss_err_deg, 'n_cross': n_cross,
        'bw_hz': bw_hz,
        'pos_deg': pos_deg, 'pos_ref_deg': pos_ref * 57.29578,
        'speed_fb': speed_fb, 'speed_ref': speed_ref, 'iq': iq,
        'step_idx': step_idx, 't': t
    }


def plot_scan(results):
    """汇总图: 各 Kp 的 pos_est 重叠 + BW vs Kp 曲线"""
    if not HAS_MPL:
        return

    valid = [r for r in results if r is not None]
    if not valid:
        return

    fig, axes = plt.subplots(2, 2, figsize=(16, 10))

    # 左上: 各 Kp 位置响应重叠
    ax = axes[0, 0]
    colors = plt.cm.viridis(np.linspace(0, 1, len(KP_VALUES)))
    for r, c in zip(valid, colors):
        t_rel = r['t'] - r['t'][r['step_idx']]
        ax.plot(t_rel, r['pos_deg'] - r['pos_deg'][r['step_idx']],
                color=c, lw=1.0, alpha=0.8, label=f"Kp={r['kp']}")
    ax.axhline(15, color='gray', ls=':', lw=0.5)  # target 15°
    ax.set_xlabel('Time from step (ms)')
    ax.set_ylabel('Position change (deg)')
    ax.set_title('Position Step Response vs Kp')
    ax.legend(fontsize=7, ncol=2)
    ax.grid(alpha=0.3)
    ax.set_xlim(-50, 800)

    # 右上: BW vs Kp
    ax = axes[0, 1]
    kp_vals = [r['kp'] for r in valid]
    bw_vals = [r['bw_hz'] for r in valid if r['bw_hz']]
    # 理论 BW = Kp/60
    kp_th = np.array(KP_VALUES)
    bw_th = kp_th / 60.0
    ax.plot(kp_th, bw_th, 'k--', lw=0.8, label='Theory (BW=Kp/60)')
    ax.plot(kp_vals[:len(bw_vals)], bw_vals, 'ro-', lw=1.5, label='Measured')
    ax.set_xlabel('Kp (RPM/rad)')
    ax.set_ylabel('BW (Hz)')
    ax.set_title('Bandwidth vs Kp')
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)

    # 左下: Rise + Settle vs Kp
    ax = axes[1, 0]
    rise_vals = [r['rise_ms'] for r in valid if r['rise_ms']]
    settle_vals = [r['settle_ms'] for r in valid if r['settle_ms']]
    kp_r = [r['kp'] for r in valid if r['rise_ms']]
    kp_s = [r['kp'] for r in valid if r['settle_ms']]
    ax.plot(kp_r, rise_vals, 'bo-', lw=1.5, label='Rise 10-90%')
    ax.plot(kp_s, settle_vals, 'ro-', lw=1.5, label='Settle ±5%')
    ax.set_xlabel('Kp (RPM/rad)')
    ax.set_ylabel('Time (ms)')
    ax.set_title('Rise & Settle vs Kp')
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)

    # 右下: OS + SSerr vs Kp
    ax = axes[1, 1]
    os_vals = [r['os_pct'] for r in valid]
    ss_vals = [abs(r['ss_err_deg']) for r in valid]
    ax.plot(kp_vals, os_vals, 'mo-', lw=1.5, label='Overshoot (%)')
    ax2 = ax.twinx()
    ax2.plot(kp_vals, ss_vals, 'co-', lw=1.5, label='|SS Error| (deg)')
    ax.set_xlabel('Kp (RPM/rad)')
    ax.set_ylabel('Overshoot (%)', color='m')
    ax2.set_ylabel('|SS Error| (deg)', color='c')
    ax.set_title('Overshoot & Steady-state Error vs Kp')
    ax.grid(alpha=0.3)

    plt.tight_layout()
    ts = time.strftime('%Y%m%d_%H%M%S')
    path = os.path.join(OUT_DIR, f'kp_scan_{ts}.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"\nSummary plot: {path}")


def find_stlink_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if 'stlink' in (p.description + p.hwid).lower():
            return p.device
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_stlink_port()
    if port is None:
        print("No STLink port found. Specify: python pos_kp_scan.py COM5")
        sys.exit(1)

    print(f"Port: {port}")
    ser = serial.Serial(port, BAUD, timeout=0.5)

    # Wait for MCU
    print("Waiting for MCU...")
    time.sleep(3.0)
    ser.reset_input_buffer()
    time.sleep(0.3)

    # Record original Kp
    resp = send_cmd(ser, 'PP')
    print(f"Default Kp: {resp.strip()}")

    results = []

    for kp in KP_VALUES:
        print(f"\n--- Kp={kp} ---")

        # Set Kp (PRP/PLP/PP 统一格式)
        send_cmd(ser, f'PP P={kp}')
        time.sleep(0.2)

        # Pre-position: RP0→RP45 产生稳定 45° 累计绝对阶跃
        print(f"  Pre: RP0")
        ser.write(b'RP0\r')
        time.sleep(2.5)
        while ser.in_waiting:
            ser.read(ser.in_waiting)

        # Enable telemetry + 基线采集 (0.5s, ~100 帧)
        ser.write(b'T\r')
        time.sleep(0.5)

        # Position step (0° → 45° = 45°, 累计绝对位置)
        print(f"  TX: RP45")
        ser.write(b'RP45\r')

        # Collect telemetry
        raw = collect_telem(ser, SETTLE_TIME)
        frames = parse_telem(raw)

        # Disable telemetry
        ser.write(b'T\r')
        time.sleep(0.3)
        while ser.in_waiting:
            ser.read(ser.in_waiting)

        # Stop motor
        send_cmd(ser, 'R0')
        time.sleep(0.5)

        if len(frames) < 50:
            print(f"  No data ({len(frames)} frames)")
            results.append(None)
            continue

        print(f"  Collected {len(frames)} frames")

        # Analyze
        result = analyze_step(frames, kp)
        if result:
            r = result
            r_str = f"{r['rise_ms']:.0f}ms" if r['rise_ms'] else '---'
            s_str = f"{r['settle_ms']:.0f}ms" if r['settle_ms'] else '---'
            bw_str = f"{r['bw_hz']:.1f}Hz" if r['bw_hz'] else '---'
            print(f"  Step={r['step_deg']:.1f}deg rise={r_str} OS={r['os_pct']:.1f}% "
                  f"settle={s_str} SSerr={r['ss_err_deg']:.2f}deg BW={bw_str} "
                  f"n_cross={r['n_cross']}")
            results.append(result)
        else:
            # 诊断失败原因
            speed_fb_a = np.array([f[2] for f in frames])
            spd_chg = np.abs(np.diff(speed_fb_a))
            max_chg = np.max(spd_chg) if len(spd_chg) > 0 else 0
            step_idx = int(np.argmax(spd_chg)) + 1 if len(spd_chg) > 0 else 0
            reason = f"n={len(frames)}"
            if len(frames) < 50:
                reason += " <50"
            elif step_idx < 10 or step_idx > len(frames) - 30:
                reason += f" step_idx={step_idx} OOB"
            else:
                pos_est_a = np.array([f[1] for f in frames])
                ps = float(np.mean(pos_est_a[max(0,step_idx-20):step_idx]))
                pt = float(np.mean(pos_est_a[min(step_idx+120, len(frames)-30):]))
                step_deg = (pt - ps) * 57.3
                reason += f" step_deg={step_deg:.1f}deg"
            print(f"  Analysis failed ({reason})")
            results.append(None)

        time.sleep(0.5)

    # Restore default
    send_cmd(ser, 'PP P=60')
    ser.close()

    # ===== Summary =====
    valid = [r for r in results if r is not None]
    print(f"\n{'='*80}")
    print(f"{'Kp':>6} {'BW(exp)':>8} {'BW(th)':>8} {'Rise':>8} {'OS':>7} {'Settle':>8} {'SSerr':>8} {'n_cross':>8}")
    print(f"{'RPM/rad':>6} {'Hz':>8} {'Hz':>8} {'ms':>8} {'%':>7} {'ms':>8} {'deg':>8} {'':>8}")
    print(f"{'-'*80}")

    for r in valid:
        bw_exp = f"{r['bw_hz']:.1f}" if r['bw_hz'] else '---'
        bw_th = f"{r['kp']/60:.1f}"
        rise = f"{r['rise_ms']:.0f}" if r['rise_ms'] else '---'
        settle = f"{r['settle_ms']:.0f}" if r['settle_ms'] else '---'
        print(f"{r['kp']:>6} {bw_exp:>8} {bw_th:>8} {rise:>8} {r['os_pct']:>6.1f}% "
              f"{settle:>8} {r['ss_err_deg']:>8.2f} {r['n_cross']:>8}")

    # 推荐最优 Kp: settle 最小且 OS<5%
    good = [r for r in valid if r['os_pct'] < 5.0 and r['settle_ms']]
    if good:
        best = min(good, key=lambda r: r['settle_ms'])
        print(f"\n  Recommended Kp={best['kp']} (settle={best['settle_ms']:.0f}ms, OS={best['os_pct']:.1f}%, BW~{best['kp']/60:.1f}Hz)")
    else:
        # Fallback: fastest rise with acceptable OS
        acceptable = [r for r in valid if r['os_pct'] < 10.0]
        if acceptable:
            best = min(acceptable, key=lambda r: r['rise_ms'] if r['rise_ms'] else 9999)
            print(f"\n  Recommended Kp={best['kp']} (fastest rise={best['rise_ms']:.0f}ms, OS={best['os_pct']:.1f}%)")

    # 绘图
    plot_scan(results)

    print("Done.")


if __name__ == '__main__':
    main()
