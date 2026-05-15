"""
电流环阶跃测试 — 测试模式下发送电流阶跃, 采集 id/iq 响应

用法: python current_step_test.py COM5 [电流值A]

MCU 需先进入测试模式 (上电时按 s)
默认阶跃 0.3A, 可指定其他值如 0.5
"""

import serial, struct, time, sys, os, csv, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BAUD = 230400
FRAME_FLOATS = 3
FOOTER = b'\x00\x00\x80\x7F'
TOTAL_FRAMES = 500
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'figures', 'current_step')
os.makedirs(OUT_DIR, exist_ok=True)


def parse_float_frames(data):
    """从串口数据中提取所有 3-float 帧"""
    frames = []
    pos = 0
    while pos < len(data) - 16:
        idx = data.find(FOOTER, pos)
        if idx < 0:
            break
        if idx >= FRAME_FLOATS * 4:
            frame_data = data[idx - FRAME_FLOATS * 4:idx]
            try:
                vals = struct.unpack(f'<{FRAME_FLOATS}f', frame_data)
                frames.append(vals)
            except struct.error:
                pass
        pos = idx + 4
    return frames


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM5'
    step = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3

    ser = serial.Serial(port, BAUD, timeout=0.5)
    ser.reset_input_buffer()

    # 排空
    time.sleep(0.5)
    while ser.in_waiting:
        ser.read(ser.in_waiting)
        time.sleep(0.05)

    print(f"发送 M1 电流阶跃 {step}A ...")
    ser.write(f'R{step}\r'.encode())

    # 采集 3 秒
    data = bytearray()
    deadline = time.time() + 3.0
    while time.time() < deadline:
        n = ser.in_waiting
        if n > 0:
            data.extend(ser.read(n))
        time.sleep(0.01)

    ser.close()

    frames = parse_float_frames(bytes(data))
    print(f"采集到 {len(frames)} 帧 (期望 {TOTAL_FRAMES})")

    if len(frames) < 100:
        print("数据不足, 退出")
        return

    # 转为 numpy
    rows = np.array(frames)
    t = np.arange(len(rows)) * 0.05  # 20kHz ADC触发 → 0.05ms 间隔
    id_cur = rows[:, 0]
    iq = rows[:, 1]
    step_ref = rows[:, 2]

    # 找阶跃点: step_ref 从 0 跳变
    step_chg = np.abs(np.diff(step_ref))
    step_idx = int(np.argmax(step_chg)) + 1
    step_t = t[step_idx]
    step_amp = step_ref[-1]
    print(f"阶跃点: t={step_t:.1f}ms, 幅值={step_amp:.3f}A")

    # iq 响应分析
    post_iq = iq[step_idx:]
    pre_iq = iq[:step_idx]
    iq_base = float(np.mean(pre_iq[-20:])) if len(pre_iq) >= 20 else 0
    iq_peak = float(np.max(post_iq)) if step_amp > 0 else float(np.min(post_iq))
    iq_overshoot = (iq_peak - step_amp) / abs(step_amp) * 100 if abs(step_amp) > 0.01 else 0

    # Rise 10-90%
    lo = iq_base + 0.1 * step_amp
    hi = iq_base + 0.9 * step_amp
    cond = post_iq >= lo if step_amp > 0 else post_iq <= lo
    a10 = int(np.argmax(cond)) if np.any(cond) else 0
    cond2 = post_iq >= hi if step_amp > 0 else post_iq <= hi
    a90 = int(np.argmax(cond2)) if np.any(cond2) else 0
    rise = (a90 - a10) * 0.05 if a10 > 0 and a90 > 0 else None

    # Settling ±5%
    bound = abs(step_amp) * 0.05
    settle = None
    for j in range(step_idx, len(rows) - 20):
        if all(abs(iq[j:j+20] - step_amp) <= bound):
            settle = (j - step_idx) * 0.05
            break

    # 稳态
    ss = iq[-100:]
    ss_std = float(np.std(ss))
    ss_mean = float(np.mean(ss))

    # 振荡分析
    post = iq[step_idx:]
    n_cross = 0
    above = post[0] > step_amp
    for v in post[1:]:
        now_above = v > step_amp
        if now_above != above:
            n_cross += 1
            above = now_above

    print(f"\niq 响应:")
    print(f"  Rise 10-90%: {rise:.2f}ms" if rise else "  Rise: ---")
    print(f"  Overshoot:   {iq_overshoot:.1f}%")
    print(f"  Settle ±5%:  {settle:.1f}ms" if settle else "  Settle: ---")
    print(f"  Peak:        {iq_peak:.4f}A")
    print(f"  SS mean:     {ss_mean:.4f}A  σ={ss_std*1000:.1f}mA")
    print(f"  n_cross:     {n_cross}")

    # ===== 绘图 =====
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    ax = axes[0]
    ax.plot(t, iq, 'r', lw=1.0, label='iq')
    ax.plot(t, step_ref, 'g--', lw=1.0, label='iq_ref')
    ax.axvline(step_t, color='blue', lw=0.5, ls=':')
    if settle:
        ax.axvline(t[step_idx] + settle, color='orange', lw=0.8, ls='--',
                   label=f'Settle {settle:.1f}ms')
    ax.set_ylabel('Current (A)')
    ax.legend(fontsize=7)
    ax.set_title(f'Current Step {step_amp:.3f}A  '
                 f'rise={"%.2f" % rise + "ms" if rise else "---"}  '
                 f'OS={iq_overshoot:.1f}%  settle={"%.1f" % settle + "ms" if settle else "---"}  '
                 f'ncross={n_cross}')
    ax.grid(alpha=0.3)
    ax.set_ylim(iq_base - 0.05, step_amp * 1.4 + 0.05)

    ax = axes[1]
    ax.plot(t, id_cur, 'b', lw=1.0, label='id')
    ax.axvline(step_t, color='blue', lw=0.5, ls=':')
    ax.axhline(0, color='k', lw=0.5)
    ax.set_ylabel('id (A)')
    ax.set_xlabel('Time (ms)')
    ax.legend(fontsize=7)
    ax.set_title(f'd-axis Current  (mean={np.mean(id_cur[-100:]):.4f}A)')
    ax.grid(alpha=0.3)

    plt.tight_layout()
    ts = time.strftime('%Y%m%d_%H%M%S')
    path = os.path.join(OUT_DIR, f'current_step_{step_amp:.2f}A_{ts}.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f"\n→ {path}")

    # 保存 CSV
    csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data',
                            f'current_step_{step_amp:.2f}A_{ts}.csv')
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['t_ms', 'id', 'iq', 'iq_ref'])
        for i, row in enumerate(frames):
            w.writerow([f"{i*0.05:.2f}"] + [f"{v:.6f}" for v in row])
    print(f"→ {csv_path}")


if __name__ == '__main__':
    main()
