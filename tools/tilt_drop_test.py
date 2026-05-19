"""平衡车自由倾倒测试 — 采集2s遥测, 小角度拟合推算平衡PID初值"""
import serial
import struct
import sys
import time
import math
import os

PORT = 'COM5'
BAUD = 230400
CHANNELS = 10
FRAME_BYTES = CHANNELS * 4
FOOTER = b'\x00\x00\x80\x7f'
CAPTURE_S = 2
SMALL_ANGLE_LIMIT = 8.0  # 小角度拟合上限 (°)
GYRO_THRESHOLD = 5.0     # 倾倒检测阈值 (°/s)

def sync_to_footer(ser, timeout=5):
    buf = b''
    t0 = time.time()
    while time.time() - t0 < timeout:
        buf += ser.read(1)
        if buf.endswith(FOOTER):
            return True
    return False

def main():
    print(f"Opening {PORT} @ {BAUD}...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {PORT}: {e}")
        sys.exit(1)

    ser.reset_input_buffer()
    ser.write(b'T\r\n')
    time.sleep(0.3)
    ser.reset_input_buffer()

    print("Waiting for telemetry...")
    if not sync_to_footer(ser):
        print("ERROR: No telemetry. Is MCU running?")
        ser.close()
        sys.exit(1)
    print("Synced.\n")

    input(">>> 扶正小车, 按 Enter 开始采集...")

    ser.reset_input_buffer()
    t0 = time.time()
    frames = []
    buf = b''

    while time.time() - t0 < CAPTURE_S:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk
        while FOOTER in buf:
            idx = buf.index(FOOTER)
            frame_data = buf[:idx]
            buf = buf[idx + len(FOOTER):]
            if len(frame_data) == FRAME_BYTES:
                try:
                    vals = struct.unpack(f'<{CHANNELS}f', frame_data)
                    frames.append((time.time() - t0, vals))
                except struct.error:
                    pass

    ser.close()
    print(f"Captured {len(frames)} frames (~{len(frames)*5:.0f}ms effective)")

    if len(frames) < 20:
        print("ERROR: Too few frames.")
        sys.exit(1)

    # 保存 CSV
    os.makedirs('tools/data', exist_ok=True)
    csv_path = f'tools/data/tilt_drop_{time.strftime("%Y%m%d_%H%M%S")}.csv'
    with open(csv_path, 'w') as f:
        f.write("t_ms,tilt_angle,gyro_rate,balance_out,spd_fb_L,spd_fb_R,iq_L,iq_R,accel_X,accel_Y,accel_Z\n")
        for t, row in frames:
            f.write(f"{t*1000:.1f}," + ",".join(f"{v:.6f}" for v in row) + "\n")
    print(f"Saved: {csv_path}")

    # 分析
    tilt = [f[1][0] for f in frames]
    gyro = [f[1][1] for f in frames]
    ts   = [f[0] for f in frames]

    # 找倾倒起点: gyro连续3帧 > 阈值
    gyro_abs = [abs(g) for g in gyro]
    fall_start_idx = 0
    for i in range(10, len(gyro_abs)):
        if (gyro_abs[i] > GYRO_THRESHOLD and
            gyro_abs[i-1] > GYRO_THRESHOLD and
            gyro_abs[i-2] > GYRO_THRESHOLD):
            fall_start_idx = max(0, i - 5)
            break

    theta0 = tilt[fall_start_idx]
    gyro0 = gyro[fall_start_idx]

    # 小角度区间: |tilt - θ0| < LIMIT
    small_angle_end = len(tilt) - 1
    for i in range(fall_start_idx, len(tilt)):
        if abs(tilt[i] - theta0) > SMALL_ANGLE_LIMIT:
            small_angle_end = i
            break

    fall_tilt = tilt[fall_start_idx:small_angle_end]
    fall_gyro = gyro[fall_start_idx:small_angle_end]
    fall_time = [ts[i] - ts[fall_start_idx] for i in range(fall_start_idx, small_angle_end)]
    n = len(fall_tilt)

    print(f"\n=== 倾倒分析 (小角度 |Δθ| < {SMALL_ANGLE_LIMIT}°) ===")
    print(f"静止角 θ0 = {theta0:.2f}°")
    print(f"初始角速度 = {gyro0:.1f}°/s")
    print(f"小角度区间: {ts[fall_start_idx]:.3f}s → {ts[small_angle_end]:.3f}s")
    print(f"持续: {ts[small_angle_end]-ts[fall_start_idx]:.3f}s, 数据点: {n}")
    tilt_range = fall_tilt[-1] - fall_tilt[0]
    print(f"倾角变化: {fall_tilt[0]:.2f}° → {fall_tilt[-1]:.2f}° (Δ={tilt_range:.1f}°)")

    if n < 5:
        print("ERROR: 小角度数据点不足, 倾倒太快或噪声过大")
        sys.exit(1)

    # 指数拟合 Δθ(t) = ε·e^(ωt)
    y_data = [max(0.02, abs(t - theta0)) for t in fall_tilt]
    lny = [math.log(y) for y in y_data]

    sum_t = sum(fall_time)
    sum_lny = sum(lny)
    sum_t2 = sum(t*t for t in fall_time)
    sum_t_lny = sum(t*ly for t, ly in zip(fall_time, lny))

    omega = (n * sum_t_lny - sum_t * sum_lny) / (n * sum_t2 - sum_t * sum_t)

    # 计算 R²
    y_mean = math.exp(sum_lny / n)
    ss_tot = sum((y - y_mean)**2 for y in y_data)
    lnA = (sum_lny - omega * sum_t) / n
    y_pred = [math.exp(lnA + omega * t) for t in fall_time]
    ss_res = sum((y - yp)**2 for y, yp in zip(y_data, y_pred))
    r_sq = 1 - ss_res / ss_tot if ss_tot > 0 else 0

    # 也用角速度直接估计 ω = gyro / tilt_error (在小角度线性区)
    # 取前几个点的平均值
    gyro_over_tilt = []
    for i in range(min(20, n)):
        dt = abs(fall_tilt[i] - theta0)
        if dt > 0.1:
            gyro_over_tilt.append(abs(fall_gyro[i]) / dt)
    omega_from_gyro = sum(gyro_over_tilt) / len(gyro_over_tilt) if gyro_over_tilt else 0

    print(f"\n指数拟合 Δθ = ε·e^(ωt):")
    print(f"  ω = {omega:.2f} rad/s  (拟合)")
    print(f"  ω = {omega_from_gyro:.2f} rad/s  (gyro/angle直接估计)")
    print(f"  τ_e = {1/max(omega,0.1)*1000:.0f} ms  (发散时间常数)")
    print(f"  R² = {r_sq:.4f}")

    # 用两个估计的平均值
    omega_est = (omega + omega_from_gyro) / 2

    # PID 推算
    # 平衡车 PID 与 ω 的经验关系:
    #   Kp (RPM/°) ~ ω * (0.15~0.5)
    #   Kd (RPM per °/s) ~ Kp * 0.08~0.2
    # 系数取决于: 轮径, 减速比, 电机常数, 速度环响应速度

    Kp_low  = round(omega_est * 0.12, 1)
    Kp_mid  = round(omega_est * 0.20, 1)
    Kp_high = round(omega_est * 0.35, 1)

    print(f"\n=== PID 参数建议 (基于 ω≈{omega_est:.1f} rad/s) ===")
    print(f"  保守: PK ANG={Kp_low}")
    print(f"  中等: PK ANG={Kp_mid}, PK GYR=0.1~{round(Kp_mid*0.12,2)}")
    print(f"  激进: PK ANG={Kp_high}, PK GYR=0.2~{round(Kp_high*0.12,2)}")
    print(f"  MAX:  PK MAX=200")

    print(f"\n推荐步骤:")
    print(f"  1. PK GYR=0 PK ANG={Kp_low} PK MAX=200")
    print(f"  2. B 激活, 看能否稳>2s")
    print(f"  3. 稳不住: PK ANG+1 每次加1")
    print(f"  4. 出现摆动: PK GYR+0.3 加阻尼")
    print(f"  5. 方向怀疑: PK ANG=-X 试反向")

if __name__ == '__main__':
    main()
