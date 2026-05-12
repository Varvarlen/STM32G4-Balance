"""
速度环性能评估 v5 — 简化方法: 按 ref 稳定值分组计算纹波
"""

import csv, numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys
from collections import defaultdict

csv_path = sys.argv[1] if len(sys.argv) > 1 else 'data/ekf_exp_20260512_230807.csv'
ENC_DIR = -1

# Load
with open(csv_path) as f:
    r = list(csv.reader(f))
cols = r[0]; data_rows = [[float(x) for x in row] for row in r[1:]]
n = len(data_rows)
t = np.array([row[0]/1000.0 for row in data_rows])
# Auto-detect frame interval from t_ms column
DT = (t[1] - t[0]) if n > 1 else 0.01
print(f"DT = {DT*1000:.0f}ms ({1/DT:.0f}Hz)")
ref   = np.array([row[1] for row in data_rows])  # speed_ref_ramp
fb    = np.array([row[2] for row in data_rows])  # EKF speed
iq_r  = np.array([row[3] for row in data_rows])  # iq_ref
iq_a  = np.array([row[4] for row in data_rows])  # iq actual
angle = np.array([row[5] for row in data_rows])  # mech_angle

# True speed
ac = angle * ENC_DIR; au = np.unwrap(ac)
true = np.zeros(n); true[1:] = np.diff(au) / DT * 9.549296586
fir = np.ones(5)/5.0; true = np.convolve(true, fir, mode='same')

print(f"Data: {n} frames = {n*DT:.1f}s\n")

# ======== Simple approach: group by ref value rounded to nearest 10 ========
# For each frame, compute the "intended ref" by looking forward/backward
# to find what the ramp is converging to.
# Simpler: use the ref value directly at frames where ref is stable
# (|d(ref)/dt| < 0.5 RPM/sample)

ref_d = np.abs(np.diff(ref, prepend=ref[0]))
stable = ref_d < 0.5

# Group contiguous stable regions
changes = np.diff(stable.astype(int))
s_starts = list(np.where(changes == 1)[0] + 1)
s_ends   = list(np.where(changes == -1)[0])
if len(s_ends) < len(s_starts): s_ends.append(n-1)
if stable[0]: s_starts.insert(0, 0)

# Collect stats per "stable plateau"
plateaus = []
for s, e in zip(s_starts, s_ends):
    dur = (e - s) * DT
    if dur < 1.0: continue  # skip very short plateaus
    r_med = np.median(ref[s:e])
    f_mean = np.mean(fb[s:e])
    f_std  = np.std(fb[s:e])
    t_std  = np.std(true[s:e])
    i_std  = np.std(iq_a[s:e])
    i_mean = np.mean(np.abs(iq_a[s:e]))
    plateaus.append((r_med, f_mean, f_std, t_std, i_std, i_mean, dur))

# Group by ref (rounded to nearest 10 or 100)
def group_key(rpm):
    rpm = abs(rpm)
    if rpm <= 100: return round(rpm / 10) * 10
    return round(rpm / 100) * 100

grouped = defaultdict(list)
for r, fm, fs, ts, iq_s, iq_m, dur in plateaus:
    grouped[group_key(r)].append((r, fm, fs, ts, iq_s, iq_m, dur))

print(f"{'Ref':>6} {'fb':>8} {'σ_fb':>8} {'σ_true':>8} {'σ/Ref':>8} {'iq_rms':>8} {'dur':>6}  Source")
print("-" * 70)
for gk in sorted(grouped.keys()):
    items = grouped[gk]
    # Take the longest plateau for each group
    best = max(items, key=lambda x: x[6])
    r, fm, fs, ts, iq_s, iq_m, dur = best
    pct = fs / abs(r) * 100 if abs(r) > 0.5 else float('nan')
    print(f"{r:6.0f} {fm:8.1f} {fs:8.3f} {ts:8.3f} {pct:7.1f}% {iq_s*1000:7.2f} {dur:5.1f}s")

# Also show all plateaus for reference
print(f"\n--- All plateaus ---")
for p in plateaus:
    r, fm, fs, ts, iq_s, iq_m, dur = p
    pct = fs/abs(r)*100 if abs(r)>0.5 else 0
    print(f"  ref={r:6.0f}  fb={fm:7.1f}  σ={fs:.3f}  σ_true={ts:.3f}  "
          f"σ%={pct:.1f}%  iq_rms={iq_s*1000:.2f}mA  {dur:.1f}s")

# ======== Key large step transients ========
print(f"\n{'='*65}")
print(f"Major Step Transients")
print(f"{'='*65}")

# Find the big ref changes (>100 RPM between stable plateaus)
for i in range(len(plateaus)-1):
    r1 = plateaus[i][0]
    r2 = plateaus[i+1][0]
    delta = abs(r2 - r1)
    if delta < 20: continue  # skip small changes

    # Find the transition period
    # plateau i ends at s_ends[i], plateau i+1 starts at s_starts[i+1]
    # Actually plateaus don't have indices stored... let me reconstruct

# Simpler: manually analyze V10→V50, V100→V200, V500→V-500
# Find frame indices for key transitions
def find_transition(from_rpm, to_rpm, tol=5):
    """Find time of transition between two ref levels"""
    for i, p in enumerate(plateaus):
        if abs(p[0] - from_rpm) < tol:
            for j in range(i+1, len(plateaus)):
                if abs(plateaus[j][0] - to_rpm) < tol:
                    # Return the range spanning both plateaus
                    return i, j
    return None

key_transitions = [
    (0, 50, "V50"),
    (0, -500, "V-500"),
    (500, -500, "V500→V-500"),
]

for from_r, to_r, label in key_transitions:
    result = find_transition(from_r, to_r, tol=max(3, abs(to_r-from_r)*0.1))
    if result is None:
        # Find closest matching plateaus
        best_from = min(plateaus, key=lambda p: abs(p[0]-from_r))
        best_to   = min(plateaus, key=lambda p: abs(p[0]-to_r))
        print(f"  {label}: from~{best_from[0]:.0f}RPM to~{best_to[0]:.0f}RPM")
        continue

    pi, pj = result
    # Widen to catch the ramp
    s_idx = max(0, pi - 5)
    e_idx = min(n-1, pj + 150)  # approximate, since plateaus don't have frame indices

# ======== Plot ========
fig, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)

ax = axes[0]
ax.plot(t, true, 'k-', alpha=0.15, lw=0.4, label='True (Δθ)')
ax.plot(t, fb, 'r-', lw=0.8, label='EKF fb')
ax.plot(t, ref, 'g--', lw=0.5, label='Ref ramp')
ax.set_ylabel('RPM')
ax.legend(fontsize=8)
ax.set_title('Speed Profile: V10→V100→V200→V300→V400→V500→V-500')
ax.grid(alpha=0.3)

# Highlight steady-state windows
for r, fm, fs, ts, iq_s, iq_m, dur in plateaus:
    if dur > 1.5:
        # Find approximate frame range
        matches = np.where(np.abs(ref - r) < 2)[0]
        if len(matches) > 0:
            s_m = matches[0]; e_m = matches[-1]
            ax.axvspan(t[s_m], t[e_m], alpha=0.06, color='green')

ax = axes[1]
ax.plot(t, fb - ref, 'b-', lw=0.5, alpha=0.5, label='fb - ref (tracking err)')
ax.axhline(0, color='k', lw=0.5)
ax.set_ylabel('Err (RPM)')
ax.legend(fontsize=8)
ax.grid(alpha=0.3)

ax = axes[2]
ax.plot(t, iq_a, 'r-', lw=0.6, alpha=0.6, label='iq')
ax.plot(t, iq_r, 'steelblue', lw=0.5, alpha=0.5, label='iq ref')
ax.set_ylabel('Current (A)')
ax.set_xlabel('Time (s)')
ax.legend(fontsize=8)
ax.grid(alpha=0.3)

plt.tight_layout()
plt.savefig('ekf_speed_profile.png', dpi=120)
plt.close()
print("\n→ ekf_speed_profile.png")

# ======== Ripple bar chart ========
gkeys = sorted(grouped.keys())
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

ripple_abs = [min([p[2] for p in grouped[gk]]) for gk in gkeys]
ripple_pct = [min([p[2] for p in grouped[gk]]) / abs(gk) * 100 if gk != 0 else 0 for gk in gkeys]
labels = [str(gk) for gk in gkeys]

ax1.bar(range(len(gkeys)), ripple_abs, color='steelblue', edgecolor='navy')
ax1.set_xticks(range(len(gkeys)))
ax1.set_xticklabels(labels, rotation=45, fontsize=8)
ax1.set_ylabel('σ (RPM)')
ax1.set_title('Steady-State Speed Ripple')
ax1.grid(alpha=0.3, axis='y')

# Only show non-zero refs for %
nz = [(i, gk, p) for i, (gk, p) in enumerate(zip(gkeys, ripple_pct)) if gk != 0]
ax2.plot([gk for _, gk, _ in nz], [p for _, _, p in nz], 'o-', color='coral', lw=2)
ax2.set_xlabel('Ref (RPM)')
ax2.set_ylabel('σ / |Ref| (%)')
ax2.set_title('Relative Ripple')
ax2.grid(alpha=0.3)

plt.tight_layout()
plt.savefig('ekf_ripple_analysis.png', dpi=120)
plt.close()
print("→ ekf_ripple_analysis.png")

# ======== Summary ========
all_fb_std = [p[2] for p in plateaus]
all_true_std = [p[3] for p in plateaus]
print(f"\n{'='*55}")
print(f"SUMMARY")
print(f"{'='*55}")
print(f"  σ_fb mean={np.mean(all_fb_std):.3f}  median={np.median(all_fb_std):.3f}  max={np.max(all_fb_std):.3f} RPM")
print(f"  σ_true mean={np.mean(all_true_std):.3f}  median={np.median(all_true_std):.3f}  max={np.max(all_true_std):.3f} RPM")

lo = [p for p in plateaus if abs(p[0]) <= 100 and abs(p[0]) > 0.5]
hi = [p for p in plateaus if abs(p[0]) > 100]
if lo:
    print(f"  ≤100 RPM: σ_fb={np.mean([p[2] for p in lo]):.3f}  "
          f"σ/ref={np.mean([p[2]/abs(p[0])*100 for p in lo]):.1f}%")
if hi:
    print(f"  >100 RPM: σ_fb={np.mean([p[2] for p in hi]):.3f}  "
          f"σ/ref={np.mean([p[2]/abs(p[0])*100 for p in hi]):.1f}%")

print("\nDone!")
