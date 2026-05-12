"""
自动速度扫描实验 — 科学对数间距序列, 双电机

用法:
    python run_speed_sweep.py            # 交互选串口, 双电机扫描
    python run_speed_sweep.py --m1       # 仅 M1

序列 (对数间距, 覆盖 1.7 decades):
  M1: V10→V30→V50→V100→V200→V500→V-500→V-100→V0
  M2: F10→F30→F50→F100→F200→F500→F-500→F-100→F0
每速度点 4s, 阶跃嵌入在切换中自然产生
"""

import serial
import serial.tools.list_ports
import csv
import struct
import sys
import time
import os
import threading

# ======== 实验序列 ========
DWELL = 4.0  # 每速度点驻留时间 (秒)

# M1 序列
M1_SEQ = [10, 30, 50, 100, 200, 500, -500, -100, 0]

# M2 序列 (对称)
M2_SEQ = [10, 30, 50, 100, 200, 500, -500, -100, 0]

# ======== 遥测帧解析 ========
FRAME_FLOATS = 10
FRAME_SIZE = 44  # 10*4 + 4(sentinel)
FOOTER = b'\x00\x00\x80\x7F'

COLUMNS = [
    't_ms', 'M1_speed_ref', 'M1_speed_fb', 'M1_iq_ref', 'M1_iq', 'M1_mech_angle',
    'M2_speed_ref', 'M2_speed_fb', 'M2_iq_ref', 'M2_iq', 'M2_mech_angle',
]

STOP = False


def parse_float_frame(buf):
    idx = buf.find(FOOTER)
    if idx < 0 or idx < 40:
        return None, 0
    data = buf[idx - 40:idx]
    try:
        return struct.unpack(f'<{FRAME_FLOATS}f', data), idx + 4
    except struct.error:
        return None, idx + 4


def keyboard_thread(ser):
    global STOP
    while not STOP:
        try:
            line = input()
        except (EOFError, OSError):
            break
        if STOP or not line:
            continue
        if line.lower() == 'q':
            print("\n[User abort]")
            STOP = True
            break
        try:
            ser.write((line + '\r').encode('ascii'))
        except serial.SerialException:
            break


def select_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports detected.")
        return None

    def sort_key(p):
        n = (p.description + p.hwid).lower()
        return (0, p.device) if 'stlink' in n else (1, p.device)

    ports.sort(key=sort_key)
    print("Available serial ports:")
    for i, p in enumerate(ports):
        tag = " <- STLink?" if 'stlink' in (p.description + p.hwid).lower() else ""
        print(f"  [{i}] {p.device}  {p.description}{tag}")
    print(f"  [q] quit\n")

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
        print(f"Enter 0-{len(ports)-1} or 'q'")


def run_sweep(ser, motor_idx, seq, cmd_letter):
    """
    执行一个电机的速度扫描.
    motor_idx: 0=M1(V), 1=M2(F 映射到 F)
    cmd_letter: 'V' for M1, 'F' for M2
    """
    for rpm in seq:
        if STOP:
            return
        sign = '+' if rpm >= 0 else ''
        tag = f"M{motor_idx+1}"
        print(f"  [{tag}] {cmd_letter}{sign}{rpm} RPM ...", end=' ', flush=True)
        ser.write(f"{cmd_letter}{rpm}\r".encode('ascii'))
        # Wait dwell time
        for _ in range(int(DWELL * 10)):
            if STOP:
                return
            time.sleep(0.1)
        print("done")


def main():
    global STOP

    do_m1 = True
    do_m2 = True
    if '--m1' in sys.argv:
        do_m2 = False
    if '--m2' in sys.argv:
        do_m1 = False

    port = select_port()
    if port is None:
        sys.exit(1)

    ser = serial.Serial(port, 230400, timeout=0.5)
    ser.reset_input_buffer()

    # Keyboard thread (press 'q' to abort)
    kb = threading.Thread(target=keyboard_thread, args=(ser,), daemon=True)
    kb.start()

    # Total duration estimate
    n_pts = len(M1_SEQ)
    total_s = 0
    if do_m1:
        total_s += n_pts * DWELL + 2
    if do_m2:
        total_s += n_pts * DWELL + 2

    print(f"\n{'='*55}")
    print(f"Speed Sweep Experiment")
    print(f"{'='*55}")
    print(f"Sequence: {M1_SEQ}")
    print(f"Dwell: {DWELL}s per point")
    if do_m1:
        print(f"M1: V10→V30→V50→V100→V200→V500→V-500→V-100→V0")
    if do_m2:
        print(f"M2: F10→F30→F50→F100→F200→F500→F-500→F-100→F0")
    print(f"Estimated duration: ~{total_s:.0f}s")
    print(f"Type 'q' to abort early")
    print(f"{'='*55}\n")

    # Data capture
    buf = bytearray()
    rows = []
    frame_dt = 5.0  # 200Hz → 5ms per frame
    drop_count = 0

    capture_start = time.time()

    # Run sweeps
    if do_m1:
        print("--- M1 Sweep ---")
        run_sweep(ser, 0, M1_SEQ, 'V')
        print("M1 done.\n")

    if do_m2 and not STOP:
        print("--- M2 Sweep ---")
        run_sweep(ser, 1, M2_SEQ, 'F')
        print("M2 done.\n")

    # Drain remaining telemetry (last second)
    if not STOP:
        print("Draining buffer...")
        time.sleep(1.0)

    STOP = True
    drain_deadline = time.time() + 0.5

    # Parse all buffered data
    while time.time() < drain_deadline:
        w = ser.in_waiting
        if w > 0:
            buf.extend(ser.read(w))
        while len(buf) >= FRAME_SIZE:
            floats, consumed = parse_float_frame(buf)
            if floats is not None:
                rows.append(floats)
                buf = buf[consumed:]
            elif consumed > 0:
                buf = buf[consumed:]
                drop_count += 1
            else:
                if len(buf) > FRAME_SIZE * 3:
                    buf = buf[-(FRAME_SIZE * 3):]
                break

    ser.close()

    if not rows:
        print("No data captured!")
        return

    # Save CSV
    ts = time.strftime('%Y%m%d_%H%M%S')
    motors = ('M1' if do_m1 else '') + ('M2' if do_m2 else '')
    motors = motors or 'XX'
    fname = f"sweep_{motors}_{ts}.csv"
    path = os.path.join('data', fname)
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for i, row in enumerate(rows):
            w.writerow([f"{i*frame_dt:.1f}"] + [f"{v:.6f}" for v in row])

    duration = len(rows) * frame_dt / 1000.0
    print(f"\nSaved: {path}")
    print(f"  Frames: {len(rows)} ({duration:.1f}s @ 200Hz)")
    if drop_count:
        print(f"  Dropped: {drop_count}")
    print("Done.")


if __name__ == '__main__':
    main()
