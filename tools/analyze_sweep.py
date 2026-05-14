"""
速度扫描数据分析 — 双电机稳态纹波 + 阶跃响应
"""

import csv, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os
from collections import defaultdict

CSV = sys.argv[1] if len(sys.argv) > 1 else 'data/sweep_M1M2_20260512_234624.csv'
OUT = 'figures/sweep'
os.makedirs(OUT, exist_ok=True)

# Load
with open(CSV) as f:
    r = list(csv.reader(f))
cols = r[0]; rows = [[float(x) for x in row] for row in r[1:]]
n = len(rows)
t  = np.array([r[0]/1000.0 for r in rows])
m1_ref = np.array([r[1] for r in rows]); m1_fb = np.array([r[2] for r in rows])
m1_iqr = np.array([r[3] for r in rows]); m1_iq = np.array([r[4] for r in rows]); m1_ang = np.array([r[5] for r in rows])
m2_ref = np.array([r[6] for r in rows]); m2_fb = np.array([r[7] for r in rows])
m2_iqr = np.array([r[8] for r in rows]); m2_iq = np.array([r[9] for r in rows]); m2_ang = np.array([r[10] for r in rows])

DT = (t[1] - t[0]) if n > 1 else 0.005
print(f"Data: {n} frames, {n*DT:.1f}s @ {1/DT:.0f}Hz\n")

# True speed
def true_rpm(angle, edir):
    ac = angle * edir; au = np.unwrap(ac)
    raw = np.zeros(len(au)); raw[1:] = np.diff(au) / DT * 9.549296586
    return np.convolve(raw, np.ones(5)/5, mode='same')

t1 = true_rpm(m1_ang, -1)
t2 = true_rpm(m2_ang, +1)

# ======== Steady-state analysis per motor ========
def steady_stats(ref, fb, true_spd, iq):
    """Extract steady-state stats for each speed plateau"""
    # Detect stable plateaus
    dref = np.abs(np.diff(ref, prepend=ref[0]))
    stable = dref < 1.0
    changes = np.diff(stable.astype(int))
    ss = list(np.where(changes==1)[0]+1); se = list(np.where(changes==-1)[0])
    if len(se) < len(ss): se.append(len(ref)-1)
    if stable[0]: ss.insert(0, 0)

    results = []
    for s, e in zip(ss, se):
        dur = (e-s)*DT
        if dur < 1.0: continue
        rm = np.median(ref[s:e])
        if abs(rm) < 0.5: continue  # skip V0
        fm = np.mean(fb[s:e])
        fs = np.std(fb[s:e])
        ts = np.std(true_spd[s:e])
        im = np.mean(np.abs(iq[s:e]))
        is_std = np.std(iq[s:e])
        results.append((rm, fm, fs, ts, im, is_std, dur))
    return results

print("=" * 75)
print(f"{'Motor':<5} {'Ref':>6} {'fb':>7} {'σ_fb':>7} {'σ/Ref':>7} {'iq mA':>7} {'iq_σ':>7} {'Steady':>6}")
print("-" * 75)

all_data = {}
for label, ref, fb, true_spd, iq, edir in [
    ('M1', m1_ref, m1_fb, t1, m1_iq, -1),
    ('M2', m2_ref, m2_fb, t2, m2_iq, +1)]:
    stats = steady_stats(ref, fb, true_spd, iq)
    all_data[label] = stats
    for rm, fm, fs, ts, im, is_std, dur in stats:
        pct = fs/abs(rm)*100 if abs(rm)>0.5 else 0
        print(f"{label:<5} {rm:6.0f} {fm:7.1f} {fs:7.3f} {pct:6.1f}% {im*1000:6.1f} {is_std*1000:6.1f} {dur:5.1f}s")

# ======== Step response analysis ========
def find_steps(ref, fb, true_spd, iq, t_arr):
    """Detect step transitions and measure response"""
    dref = np.abs(np.diff(ref, prepend=ref[0]))
    # A "step" is when ref changes by ≥5 RPM from one plateau to next
    stable = dref < 1.0
    ch = np.diff(stable.astype(int))
    trans_starts = np.where(ch == 1)[0]+1  # entering unstable zone (ramp starts)
    trans_ends   = np.where(ch == -1)[0]   # exiting unstable zone (ramp ends)

    steps = []
    for ts_idx, te_idx in zip(trans_starts, trans_ends):
        # Ref before and after
        r_before = ref[ts_idx-5] if ts_idx >= 5 else ref[ts_idx]
        r_after  = ref[min(te_idx+5, len(ref)-1)]
        delta = r_after - r_before
        if abs(delta) < 10: continue  # skip tiny steps

        # Analyze response: window from ts_idx to min(te_idx+300, len(ref))
        end_w = min(te_idx+250, len(ref))
        seg_fb = fb[ts_idx:end_w]
        seg_t  = np.arange(len(seg_fb))*DT
        seg_ref = ref[ts_idx:end_w]
        seg_iq  = iq[ts_idx:end_w]

        target = r_after
        start  = r_before

        # Settling time to ±5% of step
        bound = max(0.05*abs(delta), 5.0)  # at least 5 RPM
        settle_t = None
        for j in range(20, len(seg_fb)):
            if abs(seg_fb[j] - target) <= bound:
                ok = True
                for k in range(j, min(j+int(0.05/DT), len(seg_fb))):  # stay 50ms
                    if abs(seg_fb[k] - target) > bound:
                        ok = False; break
                if ok:
                    settle_t = seg_t[j]
                    break

        # Overshoot
        if delta > 0:
            overshoot = (np.max(seg_fb[:min(200, len(seg_fb))]) - target) / abs(delta) * 100
        else:
            overshoot = (target - np.min(seg_fb[:min(200, len(seg_fb))])) / abs(delta) * 100
        overshoot = max(0, overshoot)

        # Peak current
        peak_iq = np.max(np.abs(iq[ts_idx:end_w]))

        steps.append({
            't': t_arr[ts_idx], 'from': start, 'to': target, 'delta': delta,
            'settle': settle_t, 'overshoot': overshoot, 'peak_iq': peak_iq,
            'ts_idx': ts_idx, 'te_idx': te_idx
        })

    return steps

print(f"\n{'='*85}")
print(f"Step Response Analysis")
print(f"{'='*85}")
print(f"{'Motor':<5} {'Transition':>18} {'ΔRPM':>7} {'Settle':>8} {'Overshoot':>10} {'Peak iq':>8}")
print("-" * 85)

for label, ref, fb, true_spd, iq in [
    ('M1', m1_ref, m1_fb, t1, m1_iq),
    ('M2', m2_ref, m2_fb, t2, m2_iq)]:
    steps = find_steps(ref, fb, true_spd, iq, t)
    all_data[label+'_steps'] = steps
    for s in steps:
        st = f"{s['settle']*1000:.0f}ms" if s['settle'] else "---"
        tr = f"{s['from']:.0f}→{s['to']:.0f}"
        print(f"{label:<5} {tr:>18} {s['delta']:+7.0f} {st:>8} {s['overshoot']:9.1f}% {s['peak_iq']:7.2f}A")

# ======== Full overview plot ========
fig, axes = plt.subplots(2, 2, figsize=(18, 10))

for idx, (label, ref, fb, true_spd, iq, iqr) in enumerate([
    ('M1', m1_ref, m1_fb, t1, m1_iq, m1_iqr),
    ('M2', m2_ref, m2_fb, t2, m2_iq, m2_iqr)]):
    ax = axes[idx, 0]
    ax.plot(t, true_spd, 'k-', alpha=0.12, lw=0.3)
    ax.plot(t, fb, 'r-', lw=0.7, label=f'{label} EKF')
    ax.plot(t, ref, 'g--', lw=0.5, label='Ref')
    ax.set_ylabel('RPM'); ax.set_title(f'{label} Speed Profile')
    ax.legend(fontsize=7); ax.grid(alpha=0.3)

    ax = axes[idx, 1]
    ax.plot(t, iq, 'r-', lw=0.6, alpha=0.5, label='iq')
    ax.plot(t, iqr, 'steelblue', lw=0.5, alpha=0.5, label='iq ref')
    ax.set_ylabel('A'); ax.set_title(f'{label} Current')
    ax.legend(fontsize=7); ax.grid(alpha=0.3)
    if idx == 1: ax.set_xlabel('Time (s)')

plt.tight_layout()
path = os.path.join(OUT, '01_overview.png')
plt.savefig(path, dpi=130); plt.close()
print(f"\n→ {path}")

# ======== Ripple comparison ========
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
colors = {'M1': 'coral', 'M2': 'steelblue'}
for label in ['M1', 'M2']:
    stats = all_data[label]
    refs = [s[0] for s in stats]
    stds = [s[2] for s in stats]
    pcts = [s[2]/abs(s[0])*100 if abs(s[0])>0.5 else 0 for s in stats]
    ax1.plot(refs, stds, 'o-', color=colors[label], lw=1.5, label=label)
    ax2.plot(refs, pcts, 'o-', color=colors[label], lw=1.5, label=label)

ax1.set_xlabel('Ref (RPM)'); ax1.set_ylabel('σ (RPM)'); ax1.set_title('Speed Ripple (absolute)')
ax1.legend(); ax1.grid(alpha=0.3)
ax2.set_xlabel('Ref (RPM)'); ax2.set_ylabel('σ/Ref (%)'); ax2.set_title('Speed Ripple (relative)')
ax2.legend(); ax2.grid(alpha=0.3)
plt.tight_layout()
path = os.path.join(OUT, '02_ripple.png')
plt.savefig(path, dpi=130); plt.close()
print(f"→ {path}")

# ======== Key step response zoom ========
# Identify the 500→-500 reversal for each motor
for label, ref, fb, true_spd, iq, iqr in [
    ('M1', m1_ref, m1_fb, t1, m1_iq, m1_iqr),
    ('M2', m2_ref, m2_fb, t2, m2_iq, m2_iqr)]:
    steps = all_data[label+'_steps']
    rev = [s for s in steps if abs(s['delta']) > 800]  # >800 RPM = reversal
    if not rev: continue
    rev = rev[0]
    s0 = max(0, rev['ts_idx'] - int(0.5/DT))
    e0 = min(n-1, rev['ts_idx'] + int(2.5/DT))
    t_seg = t[s0:e0]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    ax1.plot(t_seg, true_spd[s0:e0], 'k-', alpha=0.2, lw=0.5, label='True')
    ax1.plot(t_seg, fb[s0:e0], 'r-', lw=1.2, label='EKF')
    ax1.plot(t_seg, ref[s0:e0], 'g--', lw=0.8, label='Ref')
    ax1.axvline(t[rev['ts_idx']], color='orange', lw=1, ls='--')
    ax1.set_ylabel('RPM')
    st = f"{rev['settle']*1000:.0f}ms" if rev['settle'] else '---'
    ax1.set_title(f'{label} Step {rev["from"]:.0f}→{rev["to"]:.0f} RPM  '
                  f'Settle={st}  Overshoot={rev["overshoot"]:.1f}%  Peak iq={rev["peak_iq"]:.2f}A')
    ax1.legend(fontsize=7); ax1.grid(alpha=0.3)

    ax2.plot(t_seg, iq[s0:e0], 'r-', lw=0.8, label='iq')
    ax2.plot(t_seg, iqr[s0:e0], 'steelblue', lw=0.6, label='iq ref')
    ax2.axvline(t[rev['ts_idx']], color='orange', lw=1, ls='--')
    ax2.set_ylabel('Current (A)')
    ax2.set_xlabel('Time (s)')
    ax2.legend(fontsize=7); ax2.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(OUT, f'03_reversal_{label}.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f"→ {path}")

# ======== Small step zoom ========
# 10→30 RPM for each motor
for label, ref, fb, true_spd, iq, iqr in [
    ('M1', m1_ref, m1_fb, t1, m1_iq, m1_iqr),
    ('M2', m2_ref, m2_fb, t2, m2_iq, m2_iqr)]:
    steps = all_data[label+'_steps']
    small = [s for s in steps if 15 < abs(s['delta']) < 30]
    if not small: continue
    s = small[0]
    s0 = max(0, s['ts_idx'] - int(0.3/DT))
    e0 = min(n-1, s['ts_idx'] + int(1.5/DT))
    t_seg = t[s0:e0]

    fig, ax = plt.subplots(figsize=(13, 4))
    ax.plot(t_seg, true_spd[s0:e0], 'k-', alpha=0.2, lw=0.5, label='True')
    ax.plot(t_seg, fb[s0:e0], 'r-', lw=1.2, label='EKF')
    ax.plot(t_seg, ref[s0:e0], 'g--', lw=0.8, label='Ref')
    ax.axvline(t[s['ts_idx']], color='orange', lw=1, ls='--')
    ax.set_ylabel('RPM'); ax.set_xlabel('Time (s)')
    st = f"{s['settle']*1000:.0f}ms" if s['settle'] else '---'
    ax.set_title(f'{label} Small Step: {s["from"]:.0f}→{s["to"]:.0f} RPM  '
                 f'Settle={st}  Overshoot={s["overshoot"]:.1f}%')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(OUT, f'04_small_step_{label}.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f"→ {path}")

# ======== Summary ========
print(f"\n{'='*65}")
print(f"SUMMARY")
print(f"{'='*65}")
for label in ['M1', 'M2']:
    stats = all_data[label]
    stds_all = [s[2] for s in stats]
    lo = [s for s in stats if abs(s[0]) <= 100]; hi = [s for s in stats if abs(s[0]) > 100]
    print(f"\n{label}:")
    print(f"  Overall σ median={np.median(stds_all):.2f}  max={np.max(stds_all):.2f} RPM")
    if lo:
        print(f"  ≤100 RPM: σ_avg={np.mean([s[2] for s in lo]):.2f}  σ/ref={np.mean([s[2]/abs(s[0])*100 for s in lo]):.1f}%")
    if hi:
        print(f"  >100 RPM: σ_avg={np.mean([s[2] for s in hi]):.2f}  σ/ref={np.mean([s[2]/abs(s[0])*100 for s in hi]):.1f}%")
    steps = all_data[label+'_steps']
    settles = [s['settle'] for s in steps if s['settle']]
    if settles:
        print(f"  Step settle: median={np.median(settles)*1000:.0f}ms  max={np.max(settles)*1000:.0f}ms")

print(f"\nOutput: {OUT}/")
print("Done!")
