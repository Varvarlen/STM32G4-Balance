"""采集遥测帧并保存为 CSV"""
import serial
import struct
import sys
import time

PORT = 'COM5'
BAUD = 230400
CHANNELS = 10
FRAME_BYTES = CHANNELS * 4
FOOTER = b'\x00\x00\x80\x7f'
TIMEOUT = 5  # seconds

ser = serial.Serial(PORT, BAUD, timeout=1)
print(f"Opened {PORT} @ {BAUD}")

# 同步到帧尾
print("Syncing to frame footer...")
buf = b''
while True:
    buf += ser.read(1)
    if buf.endswith(FOOTER):
        print("Synced!")
        break

# 采集
print(f"Capturing {TIMEOUT}s...")
start = time.time()
frames = []
while time.time() - start < TIMEOUT:
    data = ser.read(FRAME_BYTES)
    if len(data) != FRAME_BYTES:
        continue
    f = struct.unpack('<10f', data)
    frames.append(f)

ser.close()
print(f"Captured {len(frames)} frames ({len(frames)*5}ms)")

# 保存
csv_path = f'tools/data/telem_analysis_{time.strftime("%Y%m%d_%H%M%S")}.csv'
with open(csv_path, 'w') as f:
    f.write("t_ms,M1_pos_ref,M1_pos_est,M1_speed_fb,M1_iq,M1_speed_ref,"
            "M2_pos_ref,M2_pos_est,M2_speed_fb,M2_iq,M2_speed_ref\n")
    for i, row in enumerate(frames):
        t = i * 5.0
        f.write(f"{t:.1f}," + ",".join(f"{v:.6f}" for v in row) + "\n")

print(f"Saved: {csv_path}")
print("\n=== M1 快速统计 (最后3秒) ===")
recent = frames[-600:]  # last 3s
if recent:
    m1_speed_ref = [r[4] for r in recent]
    m1_speed_fb  = [r[2] for r in recent]
    m1_iq        = [r[3] for r in recent]
    print(f"M1 speed_ref: mean={sum(m1_speed_ref)/len(m1_speed_ref):.4f} "
          f"min={min(m1_speed_ref):.4f} max={max(m1_speed_ref):.4f}")
    print(f"M1 speed_fb:  mean={sum(m1_speed_fb)/len(m1_speed_fb):.4f} "
          f"min={min(m1_speed_fb):.4f} max={max(m1_speed_fb):.4f}")
    print(f"M1 iq:        mean={sum(m1_iq)/len(m1_iq):.4f} "
          f"min={min(m1_iq):.4f} max={max(m1_iq):.4f}")
