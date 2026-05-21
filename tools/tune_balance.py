"""
平衡车 PID 调参脚本

用法:
    python tune_balance.py status            查询当前状态和参数
    python tune_balance.py set ANG=30 GYR=3  设置 PD 参数
    python tune_balance.py telem 5           采集 5 秒遥测 → tune_telem.csv
    python tune_balance.py telem 5 out.csv   采集到指定文件

帧格式 (44 字节/帧): 10×float LE + 0x0000807F 帧尾
"""

import sys, time, struct, csv
import serial, serial.tools.list_ports

BAUD = 230400
TIMEOUT = 0.5
TELEM_CHANNELS = 10
FRAME_BYTES = TELEM_CHANNELS * 4 + 4  # 10 floats + footer
FOOTER = b'\x00\x00\x80\x7F'

CH_NAMES = [
    "tilt_angle", "gyro_rate", "balance_out",
    "speed_R", "speed_L", "iq_R", "iq_L",
    "target_angle", "ch8", "ch9"
]


def find_port():
    ports = [p.device for p in serial.tools.list_ports.comports()
             if 'STLink' in (p.description or '') or 'STMicro' in (p.description or '')]
    if ports:
        return ports[0]
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if ports:
        print(f"未检测到 ST-Link，使用第一个串口: {ports[0]}")
        return ports[0]
    print("ERROR: 未找到串口")
    sys.exit(1)


def send_cmd(ser, cmd):
    """发送命令，读取文本回显"""
    ser.reset_input_buffer()
    ser.write((cmd + '\r').encode('ascii'))
    buf = b''
    deadline = time.time() + TIMEOUT
    idle_start = time.time()
    while time.time() < deadline:
        n = ser.in_waiting
        if n > 0:
            buf += ser.read(n)
            idle_start = time.time()
        elif buf and time.time() - idle_start > 0.05:
            break
        else:
            time.sleep(0.01)
    return buf.decode('utf-8', errors='replace')


def cmd_status(ser):
    """查询状态并打印"""
    print("--- Balance Status ---")
    resp = send_cmd(ser, 'P')
    print(resp)


def cmd_set(ser, args):
    """设置 PK 参数: python tune_balance.py set ANG=30 GYR=3"""
    for arg in args:
        resp = send_cmd(ser, f'PK {arg}')
        print(resp.strip())


def cmd_telem(ser, duration_s, outfile='tune_telem.csv'):
    """采集遥测数据"""
    print(f"开启遥测, 采集 {duration_s}s → {outfile} ...")

    # 确保遥测开启
    send_cmd(ser, 'T')  # 开启
    resp = send_cmd(ser, 'T')
    if 'ON' in resp:
        send_cmd(ser, 'T')  # 如果当前是 ON 则 toggle 到 OFF
    send_cmd(ser, 'T')  # 开启
    time.sleep(0.1)

    # 清空缓冲区
    ser.reset_input_buffer()

    # 采集: 读取字节流, 用帧尾同步
    raw = b''
    deadline = time.time() + duration_s
    while time.time() < deadline:
        n = ser.in_waiting
        if n > 0:
            raw += ser.read(n)
        else:
            time.sleep(0.005)

    # 关闭遥测
    send_cmd(ser, 'T')

    # 解析帧
    frames = []
    pos = 0
    while pos + FRAME_BYTES <= len(raw):
        # 查找帧尾
        footer_pos = raw.find(FOOTER, pos)
        if footer_pos == -1:
            break
        # 帧数据在帧尾前
        data_start = footer_pos - TELEM_CHANNELS * 4
        if data_start >= pos:
            data = raw[data_start:footer_pos]
            if len(data) == TELEM_CHANNELS * 4:
                floats = list(struct.unpack(f'<{TELEM_CHANNELS}f', data))
                frames.append(floats)
        pos = footer_pos + 4

    # 写 CSV
    with open(outfile, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(CH_NAMES)
        for frame in frames:
            w.writerow(frame)

    if frames:
        dt = duration_s / len(frames) if len(frames) > 0 else 0
        print(f"  {len(frames)} 帧 (≈{1/dt:.0f}Hz)")
        # 统计摘要
        s = frames[-1]  # 最后一帧
        print(f"  tilt={s[0]:.2f}° target={s[7]:.2f}° out={s[2]:.0f}RPM "
              f"speed=({s[3]:.0f},{s[4]:.0f})RPM iq=({s[5]:.2f},{s[6]:.2f})A")
    else:
        print("  未采集到数据 (遥测帧同步失败)")

    print(f"  已保存 → {outfile}")


def main():
    port = sys.argv[2] if len(sys.argv) > 2 and sys.argv[1] in ('telem', 'status', 'set') and not sys.argv[2].startswith('COM') else None
    # 简化参数解析
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    action = sys.argv[1]
    args = sys.argv[2:]

    # 自动找端口
    port = None
    rest = []
    for a in args:
        if a.upper().startswith('COM'):
            port = a.upper()
        else:
            rest.append(a)
    if port is None:
        port = find_port()
    print(f"串口: {port}")

    ser = serial.Serial(port, BAUD, timeout=0.1)

    if action == 'status':
        cmd_status(ser)
    elif action == 'set':
        cmd_set(ser, rest)
    elif action == 'telem':
        duration = int(rest[0]) if rest else 5
        outfile = rest[1] if len(rest) > 1 else 'tune_telem.csv'
        cmd_telem(ser, duration, outfile)
    else:
        print(f"未知操作: {action}")
        print(__doc__)

    ser.close()


if __name__ == '__main__':
    main()
