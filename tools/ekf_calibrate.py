"""
EKF 参数标定 — 采集遥测数据用于离线参数扫描
用法: python ekf_calibrate.py COM5

流程:
  1. 开遥测 (T)
  2. 跑速度阶跃序列
  3. 关遥测 (T)
  4. 保存遥测帧 + burst 块到 CSV
"""

import serial, time, sys, os, struct, csv, re

BAUD = 230400
FRAME_FLOATS = 10
FOOTER = b'\x00\x00\x80\x7F'

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
os.makedirs(DATA_DIR, exist_ok=True)

# 标定用阶跃序列
CAL_SEQUENCE = [
    "RT30", "RT50", "RT100", "RT200", "RT50 -50",
]
CAL_INTERVAL = 5.0


def send_cmd(ser, cmd):
    ser.reset_input_buffer()
    ser.write((cmd + '\r').encode('ascii'))
    time.sleep(0.3)
    buf = b''
    deadline = time.time() + 0.5
    while time.time() < deadline:
        n = ser.in_waiting
        if n > 0:
            buf += ser.read(n)
    return buf.decode('utf-8', errors='replace')


def valid_telem_frame(vals):
    """校验遥测帧数据合理性, 过滤 burst 二进制误匹配"""
    # speed 范围: ±2000 RPM
    for i in [0, 1, 2, 5, 6, 7]:
        if abs(vals[i]) > 2000:
            return False
    # mech_angle 范围: ±2π
    for i in [4, 9]:
        if abs(vals[i]) > 6.3:
            return False
    # iq 范围: ±5A
    for i in [3, 8]:
        if abs(vals[i]) > 5.0:
            return False
    return True


def parse_telemetry(data):
    """从原始字节中提取 10-float 遥测帧, 过滤误匹配"""
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
                if valid_telem_frame(vals):
                    frames.append(vals)
            except struct.error:
                pass
        pos = idx + 4
    return frames


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM5'
    ser = serial.Serial(port, BAUD, timeout=0.5)
    time.sleep(1)

    # 限时排空
    t0 = time.time()
    while time.time() - t0 < 1.0:
        if ser.in_waiting:
            ser.read(ser.in_waiting)
        time.sleep(0.02)

    # 关遥测 → 读 PI
    send_cmd(ser, 'T')
    time.sleep(0.3)
    resp = send_cmd(ser, 'PRS')
    kp_s = ki_s = None
    m = re.search(r'Kp=([\d.]+)\s+Ki=([\d.]+)', resp)
    if m: kp_s, ki_s = float(m.group(1)), float(m.group(2))
    print(f"速度PI: Kp={kp_s} Ki={ki_s}")

    resp = send_cmd(ser, 'PRC')
    kp_c = ki_c = None
    m = re.search(r'Kp=([\d.]+)\s+Ki=([\d.]+)', resp)
    if m: kp_c, ki_c = float(m.group(1)), float(m.group(2))
    print(f"电流PI: Kp={kp_c} Ki={ki_c}")

    # 开遥测, 确认 ON
    for _ in range(3):
        resp = send_cmd(ser, 'T')
        if 'ON' in resp:
            break
    print("开遥测...")

    all_telem = []
    telem_start_idx = [0]

    def collect_telem(duration):
        data = bytearray()
        deadline = time.time() + duration
        while time.time() < deadline:
            n = ser.in_waiting
            if n > 0:
                data.extend(ser.read(n))
            time.sleep(0.01)
        frames = parse_telemetry(bytes(data))
        return frames, bytes(data)

    # 基线
    frames, _ = collect_telem(2.0)
    if frames:
        all_telem.extend(frames)
        print(f"  遥测基线: {len(frames)} 帧")
    else:
        # debug
        _, raw = collect_telem(1.0)
        print(f"  基线 0 帧! raw={len(raw)}B, 前40B={raw[:40].hex()}")

    # 跑阶跃
    for i, cmd in enumerate(CAL_SEQUENCE):
        print(f"\n[{i+1}/{len(CAL_SEQUENCE)}] {cmd}")
        frames, raw = collect_telem(CAL_INTERVAL)
        if frames:
            telem_start_idx.append(len(all_telem))
            all_telem.extend(frames)
            print(f"  遥测: {len(frames)} 帧")
        else:
            print(f"  0 帧! raw={len(raw)}B, FOOTER count={raw.count(FOOTER)}")

    telem_start_idx.append(len(all_telem))

    # 关遥测
    print("\n关遥测")
    send_cmd(ser, 'T')
    ser.close()

    if not all_telem:
        print("未采集到遥测数据, 退出")
        return

    # 保存
    ts = time.strftime('%Y%m%d_%H%M%S')

    # 遥测 CSV
    telem_path = os.path.join(DATA_DIR, f'ekf_cal_telem_{ts}.csv')
    with open(telem_path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['t_ms', 'M1_speed_ref', 'M1_speed_fb', 'M1_iq_ref', 'M1_iq',
                     'M1_mech_angle', 'M2_speed_ref', 'M2_speed_fb', 'M2_iq_ref',
                     'M2_iq', 'M2_mech_angle'])
        for i, row in enumerate(all_telem):
            w.writerow([f"{i*5:.1f}"] + [f"{v:.6f}" for v in row])

    # 索引文件
    idx_path = os.path.join(DATA_DIR, f'ekf_cal_index_{ts}.csv')
    with open(idx_path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['test', 'cmd', 'telem_start', 'telem_end', 'n_frames'])
        for i, cmd in enumerate(CAL_SEQUENCE):
            w.writerow([i, cmd, telem_start_idx[i], telem_start_idx[i+1],
                         telem_start_idx[i+1] - telem_start_idx[i]])

    # 参数文件
    param_path = os.path.join(DATA_DIR, f'ekf_cal_params_{ts}.txt')
    with open(param_path, 'w') as f:
        f.write(f"speed_kp={kp_s}\n")
        f.write(f"speed_ki={ki_s}\n")
        f.write(f"current_kp={kp_c}\n")
        f.write(f"current_ki={ki_c}\n")
        f.write(f"motor_kt=0.029\n")
        f.write(f"motor_j=1.83e-5\n")
        f.write(f"ekf_q_accel=500\n")
        f.write(f"ekf_q_tload=10\n")
        f.write(f"ekf_r_meas=0.01\n")

    print(f"\n遥测: {telem_path}  ({len(all_telem)} 帧 @ 200Hz)")
    print(f"索引: {idx_path}")
    print(f"参数: {param_path}")


if __name__ == '__main__':
    main()
