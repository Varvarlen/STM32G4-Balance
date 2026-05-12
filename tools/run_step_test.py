"""
阶跃测试采集 — 遥测 200Hz + 1kHz burst dump 提取

用法:
    python run_step_test.py              # 交互选串口
    S50      M1: 0→50 RPM
    S50_100  M1: 50→100 RPM
    T100     M2: 0→100 RPM
    T-50_50  M2: -50→50 RPM
    q        退出

Burst 格式:
    MAGIC(4B "RBUS") COUNT(2B LE) FIELDS(1B) RESV(1B) DATA(N*F*4B float LE)
"""

import serial, serial.tools.list_ports, struct, csv, sys, time, os, threading

BURST_MAGIC = b'RBUS'  # 0x53425552 LE → "RBUS" in byte order
FRAME_FLOATS = 10
FOOTER = b'\x00\x00\x80\x7F'

TELEM_COLS = [
    't_ms', 'M1_speed_ref', 'M1_speed_fb', 'M1_iq_ref', 'M1_iq', 'M1_mech_angle',
    'M2_speed_ref', 'M2_speed_fb', 'M2_iq_ref', 'M2_iq', 'M2_mech_angle',
]
BURST_COLS = ['t_ms', 'speed_fb', 'iq', 'speed_ref', 'T_load_est']

STOP = False
raw_bytes = bytearray()
burst_blocks = []  # list of (fields, [rows])
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
                raw_bytes.extend(ser.read(w))
        except serial.SerialException:
            break
        time.sleep(0.001)


def extract_bursts():
    """扫描 raw_bytes, 提取 burst 块和遥测帧"""
    global raw_bytes, burst_blocks, telem_rows, telem_drop

    while len(raw_bytes) >= 8:
        # 先检查 burst magic
        idx = raw_bytes.find(BURST_MAGIC)
        telemetry_start = 0

        if idx >= 0:
            # 处理 magic 之前的遥测数据
            if idx > 0:
                telem_buf = raw_bytes[:idx]
                telem_rows, telem_drop = _parse_telem(telem_buf, telem_rows, telem_drop)

            # 提取 burst 数据
            if len(raw_bytes) >= idx + 8:
                hdr = raw_bytes[idx:idx+8]
                count = struct.unpack('<H', hdr[4:6])[0]
                fields = hdr[6]
                data_bytes = count * fields * 4
                total = 8 + data_bytes
                if len(raw_bytes) >= idx + total:
                    data = raw_bytes[idx+8:idx+total]
                    rows = []
                    for i in range(count):
                        off = i * fields * 4
                        vals = struct.unpack(f'<{fields}f', data[off:off+fields*4])
                        rows.append(vals)
                    burst_blocks.append((fields, rows))
                    raw_bytes = raw_bytes[idx+total:]
                    continue  # 继续扫描下一个 burst
                else:
                    # 数据不完整, 等更多字节
                    break
            else:
                break
        else:
            # 无 magic, 全是遥测数据
            if len(raw_bytes) > 44:
                telem_rows, telem_drop = _parse_telem(raw_bytes, telem_rows, telem_drop)
                raw_bytes = raw_bytes[-200:]  # 保留末尾防截断
            break


def _parse_telem(buf, rows, drop):
    while len(buf) >= 44:
        floats, consumed = parse_float_frame(buf)
        if floats is not None:
            rows.append(floats)
            buf = buf[consumed:]
        elif consumed > 0:
            buf = buf[consumed:]
            drop += 1
        else:
            break
    return rows, drop


def keyboard_thread(ser):
    global STOP
    print("\nCommands: S<rpm> S<from>_<to> (M1)  T<rpm> T<from>_<to> (M2)  q=quit\n")
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


def select_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports: return None
    ports.sort(key=lambda p: (0, p.device) if 'stlink' in (p.description+p.hwid).lower() else (1, p.device))
    print("Available serial ports:")
    for i, p in enumerate(ports):
        tag = " <- STLink?" if 'stlink' in (p.description+p.hwid).lower() else ""
        print(f"  [{i}] {p.device}  {p.description}{tag}")
    print(f"  [q] quit\n")
    while True:
        sel = input("Select port: ").strip()
        if sel.lower() == 'q': return None
        try:
            idx = int(sel)
            if 0 <= idx < len(ports): return ports[idx].device
        except ValueError: pass
        print(f"Enter 0-{len(ports)-1} or 'q'")


def main():
    global STOP, raw_bytes, burst_blocks, telem_rows, telem_drop

    port = select_port()
    if port is None: sys.exit(1)

    ser = serial.Serial(port, 230400, timeout=0.5)
    ser.reset_input_buffer()

    reader = threading.Thread(target=serial_reader, args=(ser,), daemon=True)
    reader.start()
    kb = threading.Thread(target=keyboard_thread, args=(ser,), daemon=True)
    kb.start()

    try:
        while not STOP:
            extract_bursts()
            time.sleep(0.05)
    except KeyboardInterrupt:
        STOP = True

    time.sleep(0.5)
    ser.close()
    reader.join(timeout=1)

    # 最后处理一次
    extract_bursts()

    ts = time.strftime('%Y%m%d_%H%M%S')

    # 保存遥测 CSV
    if telem_rows:
        fname = f"step_telem_{ts}.csv"
        path = os.path.join('data', fname)
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(TELEM_COLS)
            for i, row in enumerate(telem_rows):
                w.writerow([f"{i*5:.1f}"] + [f"{v:.6f}" for v in row])
        print(f"Telemetry: {path}  ({len(telem_rows)} frames @ 200Hz)")

    # 保存 burst CSVs
    for bi, (fields, rows) in enumerate(burst_blocks):
        fname = f"step_burst_{bi}_{ts}.csv"
        path = os.path.join('data', fname)
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(BURST_COLS[:fields])
            for i, row in enumerate(rows):
                w.writerow([f"{i*1.0:.1f}"] + [f"{v:.6f}" for v in row])
        dur = len(rows) / 1000.0
        print(f"Burst[{bi}]: {path}  ({len(rows)} samples @ 1kHz = {dur:.0f}ms)")

    if not telem_rows and not burst_blocks:
        print("No data captured.")
    else:
        print("Done.")


if __name__ == '__main__':
    main()
