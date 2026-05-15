"""
批量阶跃响应分析 — 绘制全部 burst 数据

用法:
    python analyze_step_batch.py                              # 自动找最新数据
    python analyze_step_batch.py data/step_burst_*_170832.csv  # 指定文件
"""

import csv, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os, glob

# 阶跃测试序列 (与 run_step_test.py AUTO_SEQUENCE 对应)
TESTS = [
    ("M1",  0,  30),
    ("M1",  0, 100),
    ("M1",  0, 200),
    ("M1",  0, 300),
    ("M1", 50, -50),
    ("M2",  0,  30),
    ("M2",  0, 200),
    ("M2",  0, 300),
    ("M2", 50, -50),
]

# 路径相对于脚本所在目录
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(SCRIPT_DIR, 'data')
OUT = os.path.join(SCRIPT_DIR, 'figures', 'step_test')
os.makedirs(OUT, exist_ok=True)


def find_latest_files():
    """自动找最新的 burst CSV 文件组"""
    all_files = sorted(glob.glob(os.path.join(DATA_DIR, 'step_burst_*.csv')))
    if not all_files:
        print("No burst CSV files found.")
        sys.exit(1)
    # 按时间戳分组
    groups = {}
    for f in all_files:
        base = os.path.basename(f)
        ts = base.split('_')[-1].replace('.csv', '')
        groups.setdefault(ts, []).append(f)
    latest_ts = sorted(groups.keys())[-1]
    files = sorted(groups[latest_ts])
    print(f"Found {len(files)} files from {latest_ts}")
    return files


def analyze_one(csv_path, label, step_from, step_to):
    """分析单个阶跃, 返回 metrics + 绘图"""
    with open(csv_path) as f:
        r = list(csv.reader(f))
    rows = np.array([[float(x) for x in row] for row in r[1:]])
    n = len(rows)
    t = np.arange(n)  # ms, 1kHz

    fb  = rows[:, 1]
    iq  = rows[:, 2]
    ref = rows[:, 3]
    tl  = rows[:, 4] if rows.shape[1] > 4 else np.zeros(n)

    step_size = step_to - step_from

    # 阶跃检测: fb 跨过 step_from + 10% 步长
    if abs(step_size) > 0.1:
        thr = step_from + 0.1 * step_size
        if step_size > 0:
            cand = np.where(fb >= thr)[0]
        else:
            cand = np.where(fb <= thr)[0]
        step_idx = cand[0] if len(cand) > 0 else 0
    else:
        step_idx = 0
    step_t = t[step_idx]

    # Rise 10-90%
    if abs(step_size) > 1:
        lo = step_from + 0.1 * step_size
        hi = step_from + 0.9 * step_size
        if step_size > 0:
            a10 = np.argmax(fb[step_idx:] >= lo)
            a90 = np.argmax(fb[step_idx:] >= hi)
        else:
            a10 = np.argmax(fb[step_idx:] <= lo)
            a90 = np.argmax(fb[step_idx:] <= hi)
        t10 = t[step_idx + a10] if a10 > 0 or (step_idx + a10 < n and abs(fb[step_idx + a10] - lo) < abs(step_size)) else None
        t90 = t[step_idx + a90] if a90 > 0 or (step_idx + a90 < n and abs(fb[step_idx + a90] - hi) < abs(step_size)) else None
        rise = (t90 - t10) if (t10 is not None and t90 is not None) else None
    else:
        rise = None

    # Overshoot
    if step_size > 0:
        overshoot = max(0, (np.max(fb[step_idx:]) - step_to) / abs(step_size) * 100)
    else:
        overshoot = max(0, (step_to - np.min(fb[step_idx:])) / abs(step_size) * 100)

    # Settling ±5%
    bound = max(abs(step_size) * 0.05, 2.0)
    settle_t = None
    for j in range(step_idx, n - 20):
        if all(abs(fb[j:j+20] - step_to) <= bound):
            settle_t = t[j] - step_t
            break

    # Steady-state (last 100ms)
    ss = fb[-100:]
    ss_std = np.std(ss)
    ss_p2p = float(np.max(ss) - np.min(ss))
    peak_iq = np.max(np.abs(iq[step_idx:]))

    # ===== 振荡分析 =====
    post = fb[step_idx:]
    # 过零计数
    n_cross = 0
    above = post[0] > step_to
    for v in post[1:]:
        now_above = v > step_to
        if now_above != above:
            n_cross += 1
            above = now_above
    # 找局部极值 (忽略前5ms)
    peaks = []
    for i in range(5, len(post) - 1):
        if post[i] > post[i-1] and post[i] > post[i+1]:
            peaks.append((i, post[i]))
        elif post[i] < post[i-1] and post[i] < post[i+1]:
            peaks.append((i, post[i]))
    # 振荡周期/频率 (从过零间隔)
    cross_times = []
    above = post[0] > step_to
    for i, v in enumerate(post[1:], 1):
        now_above = v > step_to
        if now_above != above:
            cross_times.append(i)
            above = now_above
    osc_period = None
    osc_freq = None
    if len(cross_times) >= 4:
        intervals = np.diff(cross_times[1:])  # 跳过第一次过零
        osc_period = float(np.mean(intervals[::2])) * 2  # 完整周期=2×半周期
        osc_freq = 1000.0 / osc_period if osc_period > 0 else None
    # 阻尼比 (先后两个超调峰)
    osc_peaks = [p for p in peaks if p[1] > step_to and (step_size > 0) == (p[1] > step_to)]
    osc_decay = None
    if len(osc_peaks) >= 2:
        a1 = abs(osc_peaks[0][1] - step_to)
        a2 = abs(osc_peaks[1][1] - step_to)
        if a1 > 0.01:
            osc_decay = float(a2 / a1)

    # ===== 单图 =====
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    ax = axes[0]
    ax.plot(t, ref, 'g--', lw=1.0, label='Ref')
    ax.plot(t, fb, 'r', lw=1.5, label='EKF speed_fb')
    if rise:
        if t10: ax.axvline(t10, color='gray', lw=0.5, ls=':')
        if t90: ax.axvline(t90, color='gray', lw=0.5, ls=':')
    if settle_t:
        ax.axvline(t[step_idx] + settle_t, color='orange', lw=0.8, ls='--',
                   label=f'Settle {settle_t:.0f}ms')
    ax.axvline(step_t, color='blue', lw=0.5, ls=':')
    ax.set_ylabel('Speed (RPM)')
    ax.legend(fontsize=7, loc='lower right')
    info = f'{label} Step {step_from}→{step_to} RPM'
    if rise: info += f'  rise={rise:.0f}ms'
    info += f'  OS={overshoot:.1f}%'
    if osc_freq: info += f'  osc={osc_freq:.0f}Hz'
    info += f'  p2p={ss_p2p:.2f}RPM'
    ax.set_title(info)
    ax.grid(alpha=0.3)
    ypad = abs(step_size) * 0.25 if abs(step_size) > 5 else 8
    ax.set_ylim(min(step_from, step_to) - ypad, max(step_from, step_to) + ypad)

    ax = axes[1]
    ax.plot(t, iq, 'r', lw=1.2, label='iq')
    ax.axvline(step_t, color='blue', lw=0.5, ls=':')
    ax.axhline(0, color='k', lw=0.5)
    ax.set_ylabel('Current (A)')
    ax.legend(fontsize=7)
    iq_info = f'Current  peak={peak_iq:.3f}A'
    if osc_decay is not None:
        iq_info += f'  decay={osc_decay:.3f}  n_cross={n_cross}'
    ax.set_title(iq_info)
    ax.grid(alpha=0.3)
    iq_max = max(abs(peak_iq) * 1.5, 0.5)
    ax.set_ylim(-iq_max, iq_max)

    ax = axes[2]
    ax.plot(t, tl * 1000, 'purple', lw=1.2, label='T_load est')
    ax.axvline(step_t, color='blue', lw=0.5, ls=':')
    ax.axhline(0, color='k', lw=0.5)
    ax.set_ylabel('T_load (mN·m)')
    ax.set_xlabel('Time (ms)')
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)
    tl_range = max(np.max(np.abs(tl)) * 1500, 5)
    ax.set_ylim(-tl_range, tl_range)

    plt.tight_layout()
    tag = label.replace(' ', '_')
    path = os.path.join(OUT, f'step_{tag}_{step_from}to{step_to}.png')
    plt.savefig(path, dpi=130)
    plt.close()

    return {
        'label': label, 'step_from': step_from, 'step_to': step_to,
        'rise': rise, 'overshoot': overshoot, 'settle': settle_t,
        'peak_iq': peak_iq, 'ss_std': ss_std, 'ss_p2p': ss_p2p,
        'n_cross': n_cross, 'osc_freq': osc_freq, 'osc_decay': osc_decay,
        'path': path
    }


def make_summary(metrics_list):
    """3×3 总览图"""
    n = len(metrics_list)
    if n == 0:
        return
    cols = 3
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(18, 5 * rows))
    axes = axes.flatten() if n > 1 else [axes]

    for i, m in enumerate(metrics_list):
        # 重新加载数据绘图
        files = sorted(glob.glob(os.path.join(DATA_DIR, f'step_burst_{i}_*.csv')))
        if not files:
            continue
        latest = sorted(files, key=lambda x: x.split('_')[-1])[-1]
        with open(latest) as f:
            r = list(csv.reader(f))
        rows_data = np.array([[float(x) for x in row] for row in r[1:]])
        t = np.arange(len(rows_data))
        fb = rows_data[:, 1]
        ref = rows_data[:, 3]

        ax = axes[i]
        ax.plot(t, ref, 'g--', lw=0.8)
        ax.plot(t, fb, 'r', lw=1.0)
        ax.set_title(f'{m["label"]} {m["step_from"]}→{m["step_to"]} RPM', fontsize=9)
        ax.set_xlabel('ms')
        ax.set_ylabel('RPM')
        ax.grid(alpha=0.3)
        ypad = max(abs(m['step_to'] - m['step_from']) * 0.25, 10)
        ylo = min(m['step_from'], m['step_to']) - ypad
        yhi = max(m['step_from'], m['step_to']) + ypad
        ax.set_ylim(ylo, yhi)

    # 隐藏多余的 subplot
    for j in range(n, len(axes)):
        axes[j].set_visible(False)

    plt.tight_layout()
    path = os.path.join(OUT, 'step_summary.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"\nSummary → {path}")


def main():
    files = sys.argv[1:] if len(sys.argv) > 1 else find_latest_files()
    if len(files) != len(TESTS):
        print(f"Warning: {len(files)} files, expected {len(TESTS)}")

    metrics_list = []
    for i, (csv_path, (label, sf, st)) in enumerate(zip(files, TESTS)):
        print(f"\n{'='*50}")
        m = analyze_one(csv_path, label, sf, st)
        metrics_list.append(m)
        r = m['rise']
        osc = f"osc={m['osc_freq']:.0f}Hz" if m['osc_freq'] else "osc=---"
        p2p = m['ss_p2p']
        ncross = m['n_cross']
        print(f"  {label} {sf}→{st} RPM: "
              f"{'rise=' + f'{r:.0f}ms' if r else 'rise=---'}  "
              f"OS={m['overshoot']:.1f}%  "
              f"settle={'{:.0f}ms'.format(m['settle']) if m['settle'] else '---'}  "
              f"σ={m['ss_std']:.2f}RPM  "
              f"{osc}  p2p={p2p:.2f}RPM  n_cross={ncross}  "
              f"iq={m['peak_iq']:.3f}A")
        print(f"  → {m['path']}")

    make_summary(metrics_list)
    print("\nDone.")


if __name__ == '__main__':
    main()
