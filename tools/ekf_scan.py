"""
EKF 参数扫描 — 离线跑 EKF, 对比编码器差分基准, 选最优 Q/R

用法: python ekf_scan.py data/ekf_cal_telem_YYYYMMDD_HHMMSS.csv
"""

import csv, sys, os, numpy as np, math, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

Kt = 0.029
J = 1.83e-5
TWO_PI = 6.283185307
RPM_PER_RADPS = 9.549296586
DT = 0.005  # 200Hz 遥测

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'figures', 'ekf_scan')
os.makedirs(OUT_DIR, exist_ok=True)


def load_telem(csv_path):
    with open(csv_path) as f:
        rows = list(csv.reader(f))[1:]
    t = np.array([float(r[0]) for r in rows])
    iq = np.array([float(r[4]) for r in rows])        # M1_iq
    mech = np.array([float(r[5]) for r in rows])       # M1_mech_angle
    speed_fb_mcu = np.array([float(r[2]) for r in rows])  # M1_speed_fb (MCU EKF)
    return t, iq, mech, speed_fb_mcu


def make_ground_truth(mech, dt):
    """编码器差分 + 低通滤波 → 基准速度 (RPM)"""
    # 展开角度
    mech_unwrapped = np.unwrap(mech)
    # 差分
    vel_raw = np.diff(mech_unwrapped) / dt  # rad/s
    vel_raw = np.append(vel_raw, vel_raw[-1])
    # 低通 (等效 2ms 平滑)
    alpha = 0.3
    vel_filt = np.zeros_like(vel_raw)
    vel_filt[0] = vel_raw[0]
    for i in range(1, len(vel_raw)):
        vel_filt[i] = alpha * vel_raw[i] + (1 - alpha) * vel_filt[i-1]
    return vel_filt * RPM_PER_RADPS


def run_ekf(iq, mech, dt, q_accel, q_tload, r_meas, enc_dir=1):
    """离线运行 3-state EKF, 返回 speed_fb (RPM)"""
    n = len(mech)
    pos_est = mech[0] * enc_dir
    vel_est = 0.0
    t_load_est = 0.0

    # P 矩阵初始化
    P = np.zeros(9)
    P[0] = 0.1
    P[4] = 0.1
    P[8] = 0.1

    meas_cont = pos_est
    last_meas_raw = mech[0] * enc_dir

    vel_out = np.zeros(n)
    Jinv = 1.0 / J

    for k in range(1, n):
        meas_raw = mech[k] * enc_dir

        # 测量展开
        diff = meas_raw - last_meas_raw
        if diff > math.pi:
            diff -= TWO_PI
        elif diff < -math.pi:
            diff += TWO_PI
        meas_cont += diff
        last_meas_raw = meas_raw

        u = Kt * iq[k] * Jinv

        p0, p1, p2 = pos_est, vel_est, t_load_est
        P00, P01, P02 = P[0], P[1], P[2]
        P11, P12 = P[4], P[5]
        P22 = P[8]

        # 预测
        Hdt = 0.5 * dt * dt * Jinv
        Ddt = dt * Jinv
        accel = u - p2 * Jinv

        pos_pred = p0 + p1 * dt + 0.5 * accel * dt * dt
        vel_pred = p1 + accel * dt
        tl_pred = p2

        # P_pred = F*P*F^T + Q
        FP00 = P00 + dt * P01 - Hdt * P02
        FP01 = P01 + dt * P11 - Hdt * P12
        FP02 = P02 + dt * P12 - Hdt * P22
        FP11 = P11 - Ddt * P12
        FP12 = P12 - Ddt * P22

        pp00 = FP00 + dt * FP01 - Hdt * FP02
        pp01 = FP01 - Ddt * FP02
        pp02 = FP02
        pp11 = FP11 - Ddt * FP12
        pp12 = FP12
        pp22 = P22

        pp11 += q_accel * dt * dt
        pp22 += q_tload * dt

        # 更新
        y = meas_cont - pos_pred
        S = pp00 + r_meas
        K0 = pp00 / S
        K1 = pp01 / S
        K2 = pp02 / S

        pos_est = pos_pred + K0 * y
        vel_est = vel_pred + K1 * y
        t_load_est = tl_pred + K2 * y

        # P 更新
        omk0 = 1.0 - K0
        P[0] = omk0 * pp00
        P[1] = omk0 * pp01
        P[2] = omk0 * pp02
        P[3] = pp01 - K1 * pp00
        P[4] = pp11 - K1 * pp01
        P[5] = pp12 - K1 * pp02
        P[6] = pp02 - K2 * pp00
        P[7] = pp12 - K2 * pp01
        P[8] = pp22 - K2 * pp02

        vel_out[k] = vel_est * RPM_PER_RADPS

    return vel_out


def evaluate(vel_est, vel_ref, t, label=""):
    """评估 EKF 估计质量"""
    error = vel_est - vel_ref

    # 跳过前 1 秒 (启动瞬态)
    skip = 200
    if len(error) > skip + 100:
        err_eval = error[skip:]
    else:
        err_eval = error

    rmse = float(np.sqrt(np.mean(err_eval ** 2)))
    lag_samples = np.argmax(np.correlate(err_eval, err_eval, mode='full')) - len(err_eval) + 1
    std_err = float(np.std(err_eval))

    return rmse, std_err, error


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else None
    if not csv_path:
        # 自动找最新
        import glob
        files = sorted(glob.glob(os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'data', 'ekf_cal_telem_*.csv')))
        if not files:
            print("未找到遥测文件, 请指定路径")
            sys.exit(1)
        csv_path = files[-1]
        print(f"使用: {csv_path}")

    t, iq, mech, speed_fb_mcu = load_telem(csv_path)
    print(f"加载 {len(t)} 帧 @ 200Hz ({t[-1]:.1f}s)")

    # 基准速度
    vel_ref = make_ground_truth(mech, DT)
    # 再做一次平滑作为最终基准
    alpha2 = 0.15
    vel_ref_smooth = np.zeros_like(vel_ref)
    vel_ref_smooth[0] = vel_ref[0]
    for i in range(1, len(vel_ref)):
        vel_ref_smooth[i] = alpha2 * vel_ref[i] + (1 - alpha2) * vel_ref_smooth[i-1]

    print("\n=== EKF 参数扫描 ===")
    results = []

    for Qa in [50, 100, 200, 500, 1000, 2000, 5000]:
        for Qt in [1, 5, 10, 25, 50]:
            for R in [0.001, 0.005, 0.01, 0.05, 0.1]:
                vel_est = run_ekf(iq, mech, DT, Qa, Qt, R)
                rmse, std_err, error = evaluate(vel_est, vel_ref_smooth, t)
                results.append({
                    'Qa': Qa, 'Qt': Qt, 'R': R,
                    'rmse': rmse, 'std_err': std_err
                })
    print(f"  扫描 {len(results)} 组参数...")

    # 排序, 取最优 10 组
    results.sort(key=lambda x: x['rmse'])
    best = results[0]

    # 也跟 MCU EKF 对比
    rmse_mcu, std_mcu, _ = evaluate(speed_fb_mcu, vel_ref_smooth, t)

    print(f"\n最优: Q_accel={best['Qa']}, Q_tload={best['Qt']}, R_meas={best['R']}")
    print(f"  RMSE={best['rmse']:.3f} RPM")
    print(f"  MCU EKF RMSE={rmse_mcu:.3f} RPM  (当前 Qa=500 Qt=10 R=0.01)")

    print(f"\nTop 10:")
    for r in results[:10]:
        flag = " ← OPTIMAL" if r == best else ""
        print(f"  Qa={r['Qa']:4d} Qt={r['Qt']:2d} R={r['R']:.3f}  RMSE={r['rmse']:.3f} RPM{flag}")

    # ===== 绘图 =====
    fig, axes = plt.subplots(3, 1, figsize=(16, 12))

    # 最优 EKF vs 基准
    vel_best = run_ekf(iq, mech, DT, best['Qa'], best['Qt'], best['R'])
    ax = axes[0]
    ax.plot(t, vel_ref_smooth, 'gray', lw=1.0, alpha=0.6, label='Ground truth (enc diff)')
    ax.plot(t, vel_best, 'r', lw=1.5, label=f'Optimal EKF (Qa={best["Qa"]})')
    ax.plot(t, speed_fb_mcu, 'b--', lw=1.0, label='MCU EKF (Qa=500)')
    ax.set_ylabel('Speed (RPM)')
    ax.set_title(f'EKF Comparison  (Optimal: Qa={best["Qa"]} Qt={best["Qt"]} R={best["R"]})')
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)

    # 误差
    err_best = vel_best - vel_ref_smooth
    err_mcu = speed_fb_mcu - vel_ref_smooth
    ax = axes[1]
    ax.plot(t, err_mcu, 'b', lw=0.8, alpha=0.6, label=f'MCU EKF err (RMSE={rmse_mcu:.2f})')
    ax.plot(t, err_best, 'r', lw=0.8, label=f'Optimal EKF err (RMSE={best["rmse"]:.2f})')
    ax.set_ylabel('Error (RPM)')
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)

    # RMSE vs Q_accel (固定最优 Qt, R)
    best_Qt = best['Qt']
    best_R = best['R']
    qa_vals = sorted(set(r['Qa'] for r in results if r['Qt'] == best_Qt and r['R'] == best_R))
    qa_rmse = []
    for qa in qa_vals:
        match = [r for r in results if r['Qa'] == qa and r['Qt'] == best_Qt and r['R'] == best_R]
        if match:
            qa_rmse.append(match[0]['rmse'])
        else:
            qa_rmse.append(None)
    ax = axes[2]
    ax.semilogx(qa_vals, qa_rmse, 'o-', lw=1.5)
    ax.axvline(best['Qa'], color='r', ls='--', lw=0.8, label=f'Opt Qa={best["Qa"]}')
    ax.axhline(rmse_mcu, color='b', ls=':', lw=0.8, label=f'MCU RMSE={rmse_mcu:.2f}')
    ax.set_xlabel('Q_accel')
    ax.set_ylabel('RMSE (RPM)')
    ax.set_title(f'RMSE vs Q_accel  (Qt={best_Qt} R={best_R})')
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    path = os.path.join(OUT_DIR, 'ekf_scan_result.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f"\n→ {path}")


if __name__ == '__main__':
    main()
