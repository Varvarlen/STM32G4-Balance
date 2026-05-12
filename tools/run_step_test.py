"""
阶跃测试采集 — 遥测 200Hz + 1kHz burst dump 提取

用法:
    python run_step_test.py                # 自动找 STLink
    python run_step_test.py COM5           # 指定串口

命令:
    S50      M1: 0→50 RPM         T50      M2: 0→50 RPM
    S50_100  M1: 50→100 RPM       T-50_50  M2: -50→50 RPM
    q        退出
"""

import serial, serial.tools.list_ports, struct, csv, sys, time, os, threading

BURST_MAGIC = b'RBUS'
FRAME_FLOATS = 10
FOOTER = b'\x00\x00\x80\x7F'

TELEM_COLS = [
    't_ms', 'M1_speed_ref', 'M1_speed_fb', 'M1_iq_ref', 'M1_iq', 'M1_mech_angle',
    'M2_speed_ref', 'M2_speed_fb', 'M2_iq_ref', 'M2_iq', 'M2_mech_angle',
]
BURST_COLS = ['t_ms', 'speed_fb', 'iq', 'speed_ref', 'T_load_est']

STOP = False
buf_lock = threading.Lock()
raw_bytes = bytearray()
burst_blocks = []
telem_rows = []
telem_drop = 0


def parse_float_frame(buf):
    idx = buf.find(FOOTER)
    if idx < 0 or idx < 40:
        return None, 0
    data = buf[idx - 40:idx]
    try:
        return struct.unpack(f'<{FRAME_FLOATS}f', data), idx + 4
    except struct.error:
        return None, idx + 4


def serial_reader(ser):
    global STOP, raw_bytes
    while not STOP:
        try:
            w = ser.in_waiting
            if w > 0:
                with buf_lock:
                    raw_bytes.extend(ser.read(w))
        except serial.SerialException:
            break
        time.sleep(0.001)


def extract_and_echo():
    """提取 burst/遥测帧, 同时回显 MCU 文本输出"""
    global raw_bytes, burst_blocks, telem_rows, telem_drop

    with buf_lock:
        data = bytes(raw_bytes)
        new_start = 0  # 处理完后从这里开始保留

        while new_start < len(data):
            # 找到下一个 burst magic 或 telemetry sentinel
            burst_idx = data.find(BURST_MAGIC, new_start)
            sentinel_idx = data.find(FOOTER, new_start)

            # 处理 magic/sentinel 之前的文本回显
            next_event = min(
                burst_idx if burst_idx >= 0 else len(data),
                sentinel_idx if sentinel_idx >= 0 else len(data)
            )
            if next_event > new_start:
                # 提取文本回显
                text_chunk = data[new_start:next_event]
                echo_mcu_text(text_chunk)
                new_start = next_event

            if burst_idx == new_start and burst_idx >= 0:
                # 提取 burst
                if len(data) >= burst_idx + 8:
                    count = struct.unpack('<H', data[burst_idx+4:burst_idx+6])[0]
                    fields = data[burst_idx+6]
                    data_bytes = count * fields * 4
                    total = 8 + data_bytes
                    if len(data) >= burst_idx + total:
                        block = data[burst_idx+8:burst_idx+total]
                        rows = []
                        for i in range(count):
                            off = i * fields * 4
                            vals = struct.unpack(f'<{fields}f', block[off:off+fields*4])
                            rows.append(vals)
                        burst_blocks.append((fields, rows))
                        new_start = burst_idx + total
                        continue
                break  # 数据不完整, 等待更多

            elif sentinel_idx == new_start and sentinel_idx >= 0:
                # 提取遥测帧
                if sentinel_idx >= 40:
                    frame_data = data[sentinel_idx-40:sentinel_idx]
                    try:
                        floats = struct.unpack(f'<{FRAME_FLOATS}f', frame_data)
                        telem_rows.append(floats)
                        new_start = sentinel_idx + 4
                        continue
                    except struct.error:
                        new_start = sentinel_idx + 4
                        telem_drop += 1
                        continue
                else:
                    new_start = sentinel_idx + 4
                    telem_drop += 1
                    continue
            else:
                # 没有找到任何帧标记, 保留末尾
                break

        raw_bytes = bytearray(data[new_start:])


def echo_mcu_text(chunk):
    """打印 MCU 输出的可读文本"""
    # 查找 \r\n 分隔的行
    text = chunk.decode('ascii', errors='replace')
    lines = text.split('\r\n')
    for line in lines:
        line = line.strip()
        if not line:
            continue
        # 过滤掉纯乱码 (包含太多不可打印字符)
        printable = sum(1 for c in line if 32 <= ord(c) <= 126 or c in '\t')
        if len(line) > 0 and printable / len(line) > 0.6:
            print(f"  [MCU] {line}")


def keyboard_thread(ser):
    global STOP
    print("\n  S<rpm> / S<from>_<to> (M1)   T<rpm> / T<from>_<to> (M2)   q=quit\n")
    while not STOP:
        try:
            line = input()
        except (EOFError, OSError):
            break
        if STOP or not line:
            continue
        if line.lower() == 'q':
            STOP = True
            break
        try:
            ser.write((line + '\r').encode('ascii'))
            print(f"  -> sent: {line}")
        except serial.SerialException:
            break


def find_stlink_port():
    """自动查找 STLink 串口, 找到直接返回"""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if 'stlink' in (p.description + p.hwid).lower():
            return p.device
    return None


def main():
    global STOP, raw_bytes, burst_blocks, telem_rows, telem_drop

    # 串口: 命令行 > 自动 STLink > 报错
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = find_stlink_port()
        if port is None:
            print("No STLink port found. Specify: python run_step_test.py COM5")
            sys.exit(1)

    print(f"Using {port}")
    ser = serial.Serial(port, 230400, timeout=0.5)
    ser.reset_input_buffer()

    reader = threading.Thread(target=serial_reader, args=(ser,), daemon=True)
    reader.start()
    kb = threading.Thread(target=keyboard_thread, args=(ser,), daemon=True)
    kb.start()

    try:
        while not STOP:
            extract_and_echo()
            time.sleep(0.02)
    except KeyboardInterrupt:
        STOP = True

    time.sleep(0.5)
    ser.close()
    reader.join(timeout=1)

    # 最后处理
    extract_and_echo()

    ts = time.strftime('%Y%m%d_%H%M%S')

    if telem_rows:
        fname = f"step_telem_{ts}.csv"
        path = os.path.join('data', fname)
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(TELEM_COLS)
            for i, row in enumerate(telem_rows):
                w.writerow([f"{i*5:.1f}"] + [f"{v:.6f}" for v in row])
        print(f"Telemetry: {path}  ({len(telem_rows)} frames @ 200Hz)")

    for bi, (fields, rows) in enumerate(burst_blocks):
        fname = f"step_burst_{bi}_{ts}.csv"
        path = os.path.join('data', fname)
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(BURST_COLS[:fields+1])  # +1 for t_ms header
            for i, row in enumerate(rows):
                w.writerow([f"{i*1.0:.1f}"] + [f"{v:.6f}" for v in row])
        dur_ms = len(rows)
        print(f"Burst[{bi}]: {path}  ({len(rows)} samples @ 1kHz = {dur_ms}ms)")

    if not telem_rows and not burst_blocks:
        print("No data captured.")
    else:
        print("Done.")


if __name__ == '__main__':
    main()
