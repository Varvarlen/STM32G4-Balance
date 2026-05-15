"""
阶跃测试采集 — 遥测 200Hz + 1kHz burst dump 提取

用法:
    python run_step_test.py                # 手动模式
    python run_step_test.py --auto         # 自动跑完整阶跃矩阵
    python run_step_test.py COM5           # 指定串口

手动命令:
    RT50      M1: 0→50 RPM         LT50      M2: 0→50 RPM
    RT50 100  M1: 50→100 RPM        LT-50 50  M2: -50→50 RPM
    q        退出
"""

# 自动序列
AUTO_SEQUENCE = [
    "RT30",      # M1 0→30
    "RT100",     # M1 0→100
    "RT200",     # M1 0→200
    "RT300",     # M1 0→300
    "RT50 -50",  # M1 50→-50
    "LT30",      # M2 0→30
    "LT200",     # M2 0→200
    "LT300",     # M2 0→300
    "LT50 -50",  # M2 50→-50
]
AUTO_INTERVAL = 4.0  # 每条命令间隔(秒), 阶跃测试完整周期 ~1.3s

import serial, serial.tools.list_ports, struct, csv, sys, time, os, threading

BURST_MAGIC = b'RBUS'
FRAME_FLOATS = 10
FOOTER = b'\x00\x00\x80\x7F'

TELEM_COLS = [
    't_ms',
    'M1_pos_ref', 'M1_pos_est', 'M1_speed_fb', 'M1_iq', 'M1_speed_ref',
    'M2_pos_ref', 'M2_pos_est', 'M2_speed_fb', 'M2_iq', 'M2_speed_ref',
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
    """提取 burst/遥测帧, 遥测帧间隙中的 \r\n 文本回显"""
    global raw_bytes, burst_blocks, telem_rows, telem_drop

    with buf_lock:
        data = bytes(raw_bytes)
        new_start = 0
        last_frame_end = 0  # 上一帧结束位置, 用于找文本间隙

        while new_start < len(data):
            burst_idx = data.find(BURST_MAGIC, new_start)
            sentinel_idx = data.find(FOOTER, new_start)

            if burst_idx >= 0:
                # 有 printf 文本在 burst 之前 → 先打印再跳过
                if burst_idx > last_frame_end:
                    echo_mcu_text(data[last_frame_end:burst_idx])

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
                        last_frame_end = burst_idx + total
                        new_start = last_frame_end
                        continue
                break

            elif sentinel_idx >= 0 and sentinel_idx >= 40:
                # 遥测帧: sentinel 前 40B + sentinel 4B = 44B
                frame_start = sentinel_idx - 40

                # 帧间隙文本 (上一帧结束 到 这一帧开始)
                if frame_start > last_frame_end:
                    gap = data[last_frame_end:frame_start]
                    echo_mcu_text(gap)

                frame_data = data[frame_start:sentinel_idx]
                try:
                    floats = struct.unpack(f'<{FRAME_FLOATS}f', frame_data)
                    telem_rows.append(floats)
                except struct.error:
                    telem_drop += 1

                last_frame_end = sentinel_idx + 4
                new_start = last_frame_end
                continue

            elif sentinel_idx >= 0 and sentinel_idx < 40:
                # sentinel 出现但前面数据不够 40B, 跳过
                last_frame_end = sentinel_idx + 4
                new_start = last_frame_end
                telem_drop += 1
                continue

            else:
                break

        raw_bytes = bytearray(data[new_start:])


def echo_mcu_text(chunk):
    """打印 MCU 输出的可读文本 — 仅提取 \r\n 分隔的高可读行"""
    # 在 chunk 中搜索 \r\n 包裹的文本行
    text = chunk.decode('utf-8', errors='replace')
    i = 0
    while i < len(text):
        # 找下一个 \n
        nl = text.find('\n', i)
        if nl < 0:
            break
        line = text[i:nl].rstrip('\r')
        i = nl + 1
        # 过滤: 3-200 字符, 无不可打印字符, 无 null
        if len(line) < 3 or len(line) > 200:
            continue
        if '\x00' in line:
            continue
        printable = sum(1 for c in line if 32 <= ord(c) <= 126 or c == '\t')
        if printable / len(line) > 0.85:  # 严格: >85% 可打印
            print(f"  [MCU] {line}")


def auto_thread(ser):
    """自动模式: 按序列发送测试命令"""
    global STOP
    print(f"\n  Auto sequence: {len(AUTO_SEQUENCE)} tests, ~{len(AUTO_SEQUENCE)*AUTO_INTERVAL:.0f}s\n")
    for i, cmd in enumerate(AUTO_SEQUENCE):
        if STOP:
            break
        print(f"  [{i+1}/{len(AUTO_SEQUENCE)}] {cmd} ...", end=' ', flush=True)
        try:
            ser.write((cmd + '\r').encode('ascii'))
        except serial.SerialException:
            break
        time.sleep(AUTO_INTERVAL)


def keyboard_thread(ser):
    global STOP
    print("\n  RT<rpm> / RT<from> <to> (M1)   LT<rpm> / LT<from> <to> (M2)   q=quit\n")
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

    auto_mode = '--auto' in sys.argv

    # 串口: 命令行参数(非--auto) > 自动 STLink > 报错
    port = None
    for a in sys.argv[1:]:
        if a != '--auto':
            port = a
            break
    if port is None:
        port = find_stlink_port()
        if port is None:
            print("No STLink port found. Specify: python run_step_test.py COM5")
            sys.exit(1)

    print(f"Using {port}")
    ser = serial.Serial(port, 230400, timeout=0.5)
    ser.reset_input_buffer()

    reader = threading.Thread(target=serial_reader, args=(ser,), daemon=True)
    reader.start()

    # 等 MCU 启动完成 (避免启动消息前的命令被丢弃)
    print("Waiting for MCU...")
    time.sleep(3.0)
    # 排空启动文本
    extract_and_echo()

    if auto_mode:
        print("Mode: AUTO")
        cmd_thread = threading.Thread(target=auto_thread, args=(ser,), daemon=True)
    else:
        cmd_thread = threading.Thread(target=keyboard_thread, args=(ser,), daemon=True)
    cmd_thread.start()

    try:
        while not STOP:
            extract_and_echo()
            time.sleep(0.02)
            # 自动模式: 命令线程结束后, 等 3s 排空数据, 自动退出
            if auto_mode and not cmd_thread.is_alive():
                time.sleep(3.0)
                STOP = True
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
