"""平衡车自由倾倒测试 — 采集2s遥测, 分析倾倒动力学推算PID参数"""
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
FOOTER = b'\x00\x00\x80\x7f'  # +infinity LE
CAPTURE_S = 2  # seconds

def sync_to_footer(ser, timeout=5):
    """同步到遥测帧尾"""
    buf = b''
    t0 = time.time()
    while time.time() - t0 < timeout:
        buf += ser.read(1)
        if buf.endswith(FOOTER):
            return True
    return False

def main():
    # 1. 打开串口
    print(f"Opening {PORT} @ {BAUD}...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {PORT}: {e}")
        sys.exit(1)

    # 2. 先清空缓冲区, 再发T开启遥测
    ser.reset_input_buffer()
    ser.write(b'T\r\n')
    time.sleep(0.3)
    ser.reset_input_buffer()

    # 3. 同步帧尾
    print("Waiting for telemetry frames...")
    if not sync_to_footer(ser):
        print("ERROR: No telemetry data received. Is T enabled? Is MCU running?")
        ser.close()
        sys.exit(1)
    print("Synced to telemetry.\n")

    # 4. 提示用户
    input(">>> 扶正小车, 准备松手, 按 Enter 开始采集...")

    # 5. 采集2s
    print(f"\n>>> 松手! 采集 {CAPTURE_S}s...")
    ser.reset_input_buffer()
    t0 = time.time()
    frames = []
    buf = b''

    while time.time() - t0 < CAPTURE_S:
        # 读取可用数据
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk

        # 从缓冲区提取完整帧
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
        print("ERROR: Too few frames. Check telemetry rate and serial connection.")
        sys.exit(1)

    # 6. 保存 CSV
    os.makedirs('tools/data', exist_ok=True)
    csv_path = f'tools/data/tilt_drop_{time.strftime("%Y%m%d_%H%M%S")}.csv'
    with open(csv_path, 'w') as f:
        f.write("t_ms,tilt_angle,gyro_rate,balance_out,spd_fb_L,spd_fb_R,iq_L,iq_R,accel_X,accel_Y,accel_Z\n")
        for t, row in frames:
            f.write(f"{t*1000:.1f}," + ",".join(f"{v:.6f}" for v in row) + "\n")
    print(f"Saved: {csv_path}")

    # 7. 快速分析
    tilt = [f[1][0] for f in frames]  # ch0 = tilt_angle
    gyro = [f[1][1] for f in frames]  # ch1 = gyro_rate
    ts   = [f[0] for f in frames]

    # 找倾倒区间 (gyro显著偏离0的时刻)
    gyro_abs = [abs(g) for g in gyro]
    threshold = max(gyro_abs) * 0.1  # 10% of max
    fall_start_idx = None
    fall_end_idx = None
    for i, ga in enumerate(gyro_abs):
        if fall_start_idx is None and ga > threshold:
            fall_start_idx = max(0, i - 5)
        if fall_start_idx is not None and ga < threshold and i > fall_start_idx + 10:
            fall_end_idx = i
            break
    if fall_end_idx is None:
        fall_end_idx = len(gyro) - 1

    fall_tilt = tilt[fall_start_idx:fall_end_idx]
    fall_time = [ts[i] - ts[fall_start_idx] for i in range(fall_start_idx, fall_end_idx)]

    print(f"\n=== 倾倒分析 ===")
    print(f"倾倒起点: {ts[fall_start_idx]:.3f}s, tilt={tilt[fall_start_idx]:.2f}°")
    print(f"倾倒终点: {ts[fall_end_idx]:.3f}s, tilt={tilt[fall_end_idx]:.2f}°")
    print(f"倾倒持续: {ts[fall_end_idx]-ts[fall_start_idx]:.3f}s")

    # 拟合指数发散 θ(t) = θ0 + A * exp(ω*t)
    # 线性化: ln|θ(t) - θ0| = ln(A) + ω*t
    theta0 = tilt[fall_start_idx]
    y_data = [max(0.01, abs(t - theta0)) for t in fall_tilt]  # abs deviation, clamp for log
    t_data = fall_time

    # 简单线性回归 ln(y) vs t
    n = len(y_data)
    lny = [math.log(y) for y in y_data]
    sum_t = sum(t_data)
    sum_lny = sum(lny)
    sum_t2 = sum(t*t for t in t_data)
    sum_t_lny = sum(t*ly for t, ly in zip(t_data, lny))

    omega = (n * sum_t_lny - sum_t * sum_lny) / (n * sum_t2 - sum_t * sum_t)  # 1/s
    lnA = (sum_lny - omega * sum_t) / n
    r_sq = 1 - sum((ly - (lnA + omega * t))**2 for t, ly in zip(t_data, lny)) / sum((ly - sum_lny/n)**2 for ly in lny)

    print(f"\n指数拟合 θ(t) = θ0 + A·e^(ωt):")
    print(f"  ω = {omega:.2f} rad/s  (等效摆长 L_eff = {9.81/(omega**2):.3f}m)")
    print(f"  R² = {r_sq:.4f}")

    # 推算 PID 初值
    Kt = 0.029   # N·m/A (电机转矩常数)
    J  = 1.83e-5 # kg·m² (转子惯量)

    # 倒立摆理论: Kp_min = ω² * J/Kt / (RPM→rad/s)
    RPM_TO_RADPS = 0.10472  # 1 RPM = 0.10472 rad/s
    Kp_min_radps = omega * omega * J / Kt  # (rad/s²)/(rad) = rad/s / rad = 1/s² * kg·m² / (N·m/A) = A/(kg·m²) * kg·m²/(N·m) ... this doesn't work dimensionally

    # 更实用: Kp ≈ ω² × J/Kt, 直接算 RPM/°
    # balance_output = Kp × tilt_err(°) → RPM
    # tilt_err(°) → need to convert to effective angular acceleration
    # For inverted pendulum: α = (g/L) * θ = ω² * θ
    # Required torque: τ = J_eff * α
    # Required current: iq = τ / Kt
    # Required speed: speed = iq * Kt / (damping) ... this is getting complex

    # Practical estimation (Ziegler-Nichols style): Ku ≈ Kp that causes sustained oscillation
    # Based on ω of the free-fall:
    # Kp_start = ω * 10 (empirical for RPM/° to rad/s^2 mapping)
    Kp_suggest = max(1.0, omega * 2.5)  # conservative
    Kd_suggest = Kp_suggest * 0.08 * omega  # Kd ≈ Kp * τ_d with τ_d proportional to 1/ω

    print(f"\n建议 PID 初值 (保守):")
    print(f"  PK ANG={Kp_suggest:.1f}   (角度 Kp, RPM/°)")
    print(f"  PK GYR={Kd_suggest:.2f}   (角速度 Kd, RPM per °/s)")
    print(f"  PK MAX={min(500, int(Kp_suggest * 30))}")

    print(f"\n调试流程:")
    print(f"  1. PK ANG={Kp_suggest:.1f} PK GYR=0")
    print(f"  2. B 激活后如果不倒, 逐步 +0.2 Kd")
    print(f"  3. 如果立即倒, Kp ±30% 重试")
    print(f"  4. 稳定后 Kp 每次 +1 直到临界, 再加 Kd 消除振荡")

if __name__ == '__main__':
    main()
