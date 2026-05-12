"""
EKF 实验数据采集 — 串口接收遥测帧, 保存为 CSV

用法:
    python capture_ekf.py              # 列出串口, 交互选择
    python capture_ekf.py 30           # 指定超时时间(秒), 0=无超时

帧格式 (Cortex-M4 小端):
    [f0][f1]...[f9][00 00 80 7F]
    f0-f4: M1 speed_ref, speed_fb, iq_ref, iq, mech_angle
    f5-f9: M2 speed_ref, speed_fb, iq_ref, iq, mech_angle

在终端内直接输入 E50/E100/V50 等命令, 回车发送。
"""

import serial
import serial.tools.list_ports
import struct
import csv
import sys
import time
import os
import threading

FRAME_FLOATS = 10
FRAME_DATA_BYTES = FRAME_FLOATS * 4
FRAME_TAIL_BYTES = 4
FRAME_TOTAL_BYTES = FRAME_DATA_BYTES + FRAME_TAIL_BYTES

FOOTER = b'\x00\x00\x80\x7F'

COLUMNS = [
    't_ms',
    'M1_speed_ref', 'M1_speed_fb', 'M1_iq_ref', 'M1_iq', 'M1_mech_angle',
    'M2_speed_ref', 'M2_speed_fb', 'M2_iq_ref', 'M2_iq', 'M2_mech_angle',
]

STOP = False  # 全局停止标志


def parse_float_frame(buf):
    """查找帧尾, 解析一帧; 返回 (floats_tuple, consumed_bytes) 或 (None, 0)"""
    idx = buf.find(FOOTER)
    if idx < 0:
        return None, 0
    if idx < FRAME_DATA_BYTES:
        return None, idx + FRAME_TAIL_BYTES
    data = buf[idx - FRAME_DATA_BYTES:idx]
    try:
        floats = struct.unpack(f'<{FRAME_FLOATS}f', data)
    except struct.error:
        return None, idx + FRAME_TAIL_BYTES
    return floats, idx + FRAME_TAIL_BYTES


def keyboard_thread(ser):
    """后台线程: 读取键盘输入, 发送到串口"""
    global STOP
    while not STOP:
        try:
            line = input()
        except (EOFError, OSError):
            break
        if STOP:
            break
        if not line:
            continue
        payload = (line + '\r').encode('ascii')
        try:
            ser.write(payload)
            print(f"  -> sent: {line}")
        except serial.SerialException:
            break


def select_port():
    """列出可用串口, 让用户选择"""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports detected. Check USB connection.")
        return None

    def sort_key(p):
        name_lower = (p.description + p.hwid).lower()
        if 'stlink' in name_lower or 'st-link' in name_lower:
            return (0, p.device)
        return (1, p.device)

    ports.sort(key=sort_key)

    print("Available serial ports:")
    for i, p in enumerate(ports):
        marker = " <- STLink?" if 'stlink' in (p.description + p.hwid).lower() else ""
        print(f"  [{i}] {p.device}  {p.description}{marker}")

    print(f"  [q] quit")
    print()

    while True:
        sel = input("Select port: ").strip()
        if sel.lower() == 'q':
            return None
        try:
            idx = int(sel)
            if 0 <= idx < len(ports):
                return ports[idx].device
        except ValueError:
            pass
        print(f"Invalid. Enter 0-{len(ports)-1} or 'q'")


def main():
    global STOP
    timeout = 15.0
    if len(sys.argv) > 1:
        timeout = float(sys.argv[1])

    port = select_port()
    if port is None:
        sys.exit(1)

    ser = serial.Serial(port, 230400, timeout=0.5)
    ser.reset_input_buffer()

    # 启动键盘输入线程
    kb = threading.Thread(target=keyboard_thread, args=(ser,), daemon=True)
    kb.start()

    buf = bytearray()
    deadline = time.time() + timeout if timeout > 0 else float('inf')
    rows = []
    frame_interval_ms = 10.0
    drop_count = 0

    print(f"\nCapturing from {port} @ 230400...")
    if timeout > 0:
        print(f"  Duration: {timeout}s  (Ctrl+C to stop early)")
    else:
        print(f"  No timeout — Ctrl+C to stop")
    print(f"  Type E50/E100/V50 then Enter\n")

    try:
        while time.time() < deadline:
            waiting = ser.in_waiting
            if waiting > 0:
                buf.extend(ser.read(waiting))

            while len(buf) >= FRAME_TOTAL_BYTES:
                floats, consumed = parse_float_frame(buf)
                if floats is not None:
                    rows.append(floats)
                    buf = buf[consumed:]
                elif consumed > 0:
                    buf = buf[consumed:]
                    drop_count += 1
                else:
                    if len(buf) > FRAME_TOTAL_BYTES * 2:
                        buf = buf[-(FRAME_TOTAL_BYTES * 2):]
                    break
    except KeyboardInterrupt:
        print("\nInterrupted")

    STOP = True
    ser.close()

    if not rows:
        print("No data captured. Is telemetry running?")
        return

    timestamp = time.strftime('%Y%m%d_%H%M%S')
    fname = f"ekf_exp_{timestamp}.csv"
    with open(fname, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for i, row in enumerate(rows):
            w.writerow([f"{i * frame_interval_ms:.1f}"] + [f"{v:.6f}" for v in row])

    duration = len(rows) * frame_interval_ms / 1000.0
    print(f"\nSaved: {fname}")
    print(f"  Frames: {len(rows)}  ({duration:.1f}s @ 100Hz)")
    if drop_count > 0:
        print(f"  Dropped: {drop_count} malformed frames")


if __name__ == '__main__':
    main()
