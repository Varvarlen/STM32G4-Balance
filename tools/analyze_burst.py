"""
1kHz Burst 阶跃响应分析
"""

import csv, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os

CSV = sys.argv[1] if len(sys.argv) > 1 else 'data/step_burst_0_20260513_001347.csv'
LABEL = sys.argv[2] if len(sys.argv) > 2 else 'M1'
_TARGET = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

OUT = 'figures/step_test'
os.makedirs(OUT, exist_ok=True)

# Load
with open(CSV) as f:
    r = list(csv.reader(f))
cols = r[0]
rows = np.array([[float(x) for x in row] for row in r[1:]])
n = len(rows)
dt = 0.001  # 1kHz
t = np.arange(n) * dt * 1000  # ms

fb  = rows[:, 1]  # speed_fb
iq  = rows[:, 2]  # iq actual
ref = rows[:, 3]  # speed_ref (固定=目标值, 阶跃前已设定)
tl  = rows[:, 4] if rows.shape[1] > 4 else np.zeros(n)  # T_load_est

print(f"Burst: {n} samples @ 1kHz = {n}ms")
print(f"Fields: {cols}")

# 阶跃检测: ref 在 burst 开始前已设定，fb 从静止跳变到目标
step_from = fb[0]                  # burst 开始时 fb (阶跃即刻)
step_to   = np.mean(ref[-50:])     # 目标值
step_size = step_to - step_from
# 找到 fb 跨过 10% 步长的时刻作为阶跃起点
thresh = step_from + 0.1 * abs(step_size)
candidates = np.where(np.abs(fb - step_from) > abs(thresh - step_from))[0]
step_idx = candidates[0] if len(candidates) > 0 else 1
step_t = t[step_idx]
print(f"Step detected at t={step_t:.0f}ms: fb={fb[step_idx]:.1f} → target={step_to:.0f} RPM  (Δ={step_size:.0f})")

# Rise time 10-90%
if step_size > 0:
    lo = step_from + 0.1 * step_size
    hi = step_from + 0.9 * step_size
    t10 = t[np.argmax(fb[step_idx:] >= lo) + step_idx] if np.any(fb[step_idx:] >= lo) else None
    t90 = t[np.argmax(fb[step_idx:] >= hi) + step_idx] if np.any(fb[step_idx:] >= hi) else None
else:
    lo = step_from + 0.1 * step_size
    hi = step_from + 0.9 * step_size
    t10 = t[np.argmax(fb[step_idx:] <= lo) + step_idx] if np.any(fb[step_idx:] <= lo) else None
    t90 = t[np.argmax(fb[step_idx:] <= hi) + step_idx] if np.any(fb[step_idx:] <= hi) else None

rise = (t90 - t10) if (t10 is not None and t90 is not None) else None

# Overshoot
if step_size > 0:
    overshoot = (np.max(fb[step_idx:]) - step_to) / abs(step_size) * 100
else:
    overshoot = (step_to - np.min(fb[step_idx:])) / abs(step_size) * 100
overshoot = max(0, overshoot)

# Settling time (±5%)
bound = max(abs(step_size) * 0.05, 2.0)
settle_t = None
for j in range(step_idx, n - 10):
    if all(abs(fb[j:j+10] - step_to) <= bound):
        settle_t = t[j]
        break

# Steady-state ripple (last 100ms)
ss_fb = fb[-100:]
ss_std = np.std(ss_fb)
ss_iq_std = np.std(iq[-100:])

# Peak current
peak_iq = np.max(np.abs(iq[step_idx:]))

print(f"\nStep Response:")
print(f"  Rise 10-90%: {rise:.0f}ms" if rise else "  Rise: ---")
print(f"  Overshoot:   {overshoot:.1f}%")
print(f"  Settle ±5%:  {settle_t-step_t:.0f}ms" if settle_t else "  Settle: ---")
print(f"  Peak iq:     {peak_iq:.3f}A")
print(f"  SS σ_fb:     {ss_std:.2f} RPM")
print(f"  SS σ_iq:     {ss_iq_std*1000:.1f} mA")

# ======== Plot ========
fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

ax = axes[0]
ax.plot(t, ref, 'g--', lw=1.0, label='Ref')
ax.plot(t, fb, 'r-', lw=1.5, label='EKF fb')
if t10 and t90:
    ax.axvline(t10, color='gray', lw=0.5, ls=':')
    ax.axvline(t90, color='gray', lw=0.5, ls=':')
if settle_t:
    ax.axvline(settle_t, color='orange', lw=0.8, ls='--', label=f'Settle {settle_t-step_t:.0f}ms')
ax.axvline(step_t, color='blue', lw=0.5, ls=':')
ax.set_ylabel('Speed (RPM)')
ax.legend(fontsize=7)
info = f'{LABEL} Step {step_from:.0f}→{step_to:.0f} RPM'
if rise: info += f'  rise={rise:.0f}ms'
info += f'  overshoot={overshoot:.1f}%  σ={ss_std:.2f}RPM'
ax.set_title(info)
ax.grid(alpha=0.3)

ax = axes[1]
ax.plot(t, iq, 'r-', lw=1.2, label='iq')
ax.axvline(step_t, color='blue', lw=0.5, ls=':')
ax.axhline(0, color='k', lw=0.5)
ax.set_ylabel('Current (A)')
ax.legend(fontsize=7)
ax.set_title(f'Current  (peak={peak_iq:.3f}A)')
ax.grid(alpha=0.3)

ax = axes[2]
ax.plot(t, tl * 1000, 'purple', lw=1.2, label='T_load est')
ax.axvline(step_t, color='blue', lw=0.5, ls=':')
ax.axhline(0, color='k', lw=0.5)
ax.set_ylabel('T_load (mN·m)')
ax.set_xlabel('Time (ms)')
ax.legend(fontsize=7)
ax.grid(alpha=0.3)

plt.tight_layout()
tag = LABEL.replace(' ', '_')
path = os.path.join(OUT, f'step_{tag}_{step_from:.0f}to{step_to:.0f}.png')
plt.savefig(path, dpi=130)
plt.close()
print(f"\n→ {path}")
