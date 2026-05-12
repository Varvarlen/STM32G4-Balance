"""
EKF 参数扫描 + 分段重置
找到最优 Q/R, 对比 α-β vs EKF
"""

import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys
from itertools import product

csv_path = sys.argv[1] if len(sys.argv) > 1 else 'ekf_exp_20260512_213345.csv'
ENC_DIR = -1
DT = 0.01
KT = 0.0290
J = 1.83e-5
KT_OVER_J = 1587.0

# Load data
columns = []
rows = []
with open(csv_path) as f:
    reader = csv.reader(f)
    columns = next(reader)
    for row in reader:
        rows.append([float(v) for v in row])
n = len(rows)
data = {col: np.array([r[i] for r in rows]) for i, col in enumerate(columns)}

t         = data['t_ms'] / 1000.0
m1_ref    = data['M1_speed_ref']
m1_fb_mcu = data['M1_speed_fb']
m1_iq     = data['M1_iq']
m1_angle  = data['M1_mech_angle']

# True speed
angle_c = m1_angle * ENC_DIR
angle_u = np.unwrap(angle_c)
raw_rpm = np.zeros(n)
raw_rpm[1:] = np.diff(angle_u) / DT * 9.549296586
fir = np.ones(5)/5.0
true_rpm = np.convolve(raw_rpm, fir, mode='same')

# Experiment segments
in_exp = np.abs(m1_ref) > 0.5
trans = np.diff(in_exp.astype(int))
starts = np.where(trans == 1)[0] + 1
ends   = np.where(trans == -1)[0]
print(f"Experiments: {len(starts)}")
for i, (s, e) in enumerate(zip(starts, ends)):
    print(f"  [{i}] {t[s]:.0f}s→{t[e]:.0f}s  ref={m1_ref[s]:.0f}RPM  dur={t[e]-t[s]:.1f}s")

def ekf_segment(angle_raw, iq, dt, kt=KT, J=J, enc_dir=ENC_DIR,
                q_accel=100.0, q_tload=20.0, r_meas=0.001):
    """EKF with state reset at start"""
    nn = len(angle_raw)
    meas_raw = angle_raw * enc_dir

    x = np.zeros((nn, 3))
    x[0] = [meas_raw[0], 0.0, 0.0]
    P = np.eye(3) * 0.1

    F = np.array([[1.0, dt, -0.5*dt*dt/J],
                   [0.0, 1.0, -dt/J],
                   [0.0, 0.0, 1.0]])
    H = np.array([[1.0, 0.0, 0.0]])

    for k in range(1, nn):
        diff = meas_raw[k] - meas_raw[k-1]
        if diff > np.pi:       diff -= 2*np.pi
        elif diff < -np.pi:    diff += 2*np.pi

        z = x[k-1, 0] + diff
        u = kt * iq[k] / J

        x_pred = F @ x[k-1]
        x_pred[0] += 0.5 * u * dt * dt
        x_pred[1] += u * dt

        Q = np.diag([1e-10, q_accel*dt*dt, q_tload*dt])
        P_pred = F @ P @ F.T + Q

        y = z - x_pred[0]
        S = H @ P_pred @ H.T + r_meas
        K = P_pred @ H.T / S

        x[k] = x_pred + (K * y).flatten()
        P = (np.eye(3) - K @ H) @ P_pred

    return x[:, 1] * 9.549296586, x[:, 2]

# ============ 参数扫描 ============
q_accel_candidates = [50, 100, 200, 500, 1000]
q_tload_candidates = [10, 30, 80, 200, 500]
r_meas_candidates  = [0.0001, 0.0005, 0.001, 0.005, 0.01]

best_rmse = float('inf')
best_params = None
results = []

print("\nParameter sweep (loaded exps: 0,1,2)...")
for qa, qt, rm in product(q_accel_candidates, q_tload_candidates, r_meas_candidates):
    total_rmse = 0
    for seg_i in [0, 1, 2]:  # Only loaded experiments
        s, e = starts[seg_i], ends[seg_i]
        ekf_r, _ = ekf_segment(m1_angle[s:e], m1_iq[s:e], DT,
                               q_accel=qa, q_tload=qt, r_meas=rm)
        err = ekf_r - true_rpm[s:e]
        total_rmse += np.sqrt(np.mean(err**2))

    avg_rmse = total_rmse / 3
    results.append((qa, qt, rm, avg_rmse))
    if avg_rmse < best_rmse:
        best_rmse = avg_rmse
        best_params = (qa, qt, rm)

print(f"Best: q_accel={best_params[0]} q_tload={best_params[1]} r_meas={best_params[2]:.4f}  RMSE={best_rmse:.1f}")

# Top 10
results.sort(key=lambda x: x[3])
print("\nTop 10 parameter sets:")
for r in results[:10]:
    print(f"  qa={r[0]:4d} qt={r[1]:3d} R={r[2]:.4f} → RMSE={r[3]:.1f}")

# ============ Final run with best params ============
qa_best, qt_best, rm_best = best_params

print(f"\n{'='*60}")
print(f"Final analysis with q_accel={qa_best} q_tload={qt_best} R={rm_best:.4f}")
print(f"{'='*60}")

fig, axes = plt.subplots(len(starts), 2, figsize=(16, 3*len(starts)))
if len(starts) == 1:
    axes = axes.reshape(1, -1)

for seg_i, (s, e) in enumerate(zip(starts, ends)):
    ref_val = m1_ref[s]

    # Extend window for plotting
    s0 = max(0, s - 30)
    e0 = min(n-1, e + 100)

    # EKF on this segment (fresh reset)
    ekf_r, ekf_tl = ekf_segment(m1_angle[s0:e0], m1_iq[s0:e0], DT,
                                 q_accel=qa_best, q_tload=qt_best, r_meas=rm_best)

    # Metrics (only during experiment, allow 500ms settling)
    s_settle = s + 50  # skip first 500ms
    ab_err = m1_fb_mcu[s_settle:e] - true_rpm[s_settle:e]
    ekf_err = ekf_r[s_settle-s0:e-s0] - true_rpm[s_settle:e]
    ab_rmse = np.sqrt(np.mean(ab_err**2))
    ekf_rmse = np.sqrt(np.mean(ekf_err**2))
    ab_max = np.max(np.abs(ab_err))
    ekf_max = np.max(np.abs(ekf_err))

    pct = (1 - ekf_rmse/ab_rmse)*100 if ab_rmse > 0.1 else 0

    # Determine if E load test or V step test
    label = "E (load)" if ref_val in [30, 50] and seg_i < 3 else "V (step)"

    print(f"\nSeg {seg_i} [{label}] ref={ref_val:.0f}RPM:")
    print(f"  α-β  RMSE={ab_rmse:.1f}  max={ab_max:.0f} RPM")
    print(f"  EKF  RMSE={ekf_rmse:.1f}  max={ekf_max:.0f} RPM")
    print(f"  EKF vs α-β: {pct:+.1f}%")

    # Left: speed
    ax = axes[seg_i, 0]
    t_seg = t[s0:e0]
    ax.plot(t_seg, true_rpm[s0:e0], 'k-', alpha=0.2, lw=0.5, label='True')
    ax.plot(t_seg, m1_fb_mcu[s0:e0], 'b-', lw=1.0, label='α-β MCU')
    ax.plot(t_seg, ekf_r, 'r-', lw=1.0, label='EKF')
    ax.plot(t_seg, m1_ref[s0:e0], 'g--', lw=0.7, label='Ref')
    ax.axvspan(t[s], t[e], alpha=0.08, color='orange')
    ax.set_ylabel('RPM')
    ax.legend(fontsize=7)
    ax.set_title(f'[{seg_i}] {label} ref={ref_val:.0f}  αβ={ab_rmse:.0f} EKF={ekf_rmse:.0f} (Δ={pct:+.0f}%)')
    ax.grid(alpha=0.3)

    # Right: current + load estimate
    ax = axes[seg_i, 1]
    ax.plot(t_seg, m1_iq[s0:e0], 'b-', lw=0.6, alpha=0.5, label='iq')
    ax2 = ax.twinx()
    ax2.plot(t_seg, ekf_tl * 1000, 'r-', lw=1.0, label='T_load (mN·m)')
    ax2.set_ylabel('mN·m', color='r')
    ax2.legend(fontsize=7, loc='upper right')
    ax.axvspan(t[s], t[e], alpha=0.08, color='orange')
    ax.set_ylabel('A')
    ax.set_xlabel('s')
    ax.legend(fontsize=7, loc='upper left')
    ax.grid(alpha=0.3)

plt.tight_layout()
plt.savefig('ekf_final_analysis.png', dpi=120)
plt.close()
print("\n→ ekf_final_analysis.png")

# ============ T_load vs iq correlation ============
# Pick seg 0 (first E50, should have load disturbance)
s, e = starts[0], ends[0]
s0 = max(0, s - 30)
e0 = min(n-1, e + 100)
ekf_r0, ekf_tl0 = ekf_segment(m1_angle[s0:e0], m1_iq[s0:e0], DT,
                                q_accel=qa_best, q_tload=qt_best, r_meas=rm_best)

fig, ax = plt.subplots(figsize=(12, 4))
t_seg = t[s0:e0]
ax.plot(t_seg, ekf_tl0 * 1000, 'r-', lw=1.2, label='EKF T_load (mN·m)')
ax.plot(t_seg, m1_iq[s0:e0] * KT * 1000, 'b-', lw=0.7, alpha=0.5, label='Kt*iq (mN·m)')
ax.axvspan(t[s], t[e], alpha=0.08, color='orange')
ax.set_ylabel('Torque (mN·m)')
ax.set_xlabel('Time (s)')
ax.legend()
ax.set_title('EKF Load Estimate vs Motor Torque (Seg 0: E50)')
ax.grid(alpha=0.3)
plt.tight_layout()
plt.savefig('ekf_load_detail.png', dpi=120)
plt.close()
print("→ ekf_load_detail.png")

print("\nDone!")
