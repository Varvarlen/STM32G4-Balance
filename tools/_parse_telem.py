import struct
import numpy as np

data_hex = """
00 00 00 00 AA 4C 05 3C D5 47 A4 BC E0 C7 F1 3B 00 00 00 00 00 00 00 00 C9 96 20 41 99 B0 2A BC 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 EE 6E 05 3C E0 57 04 BD E0 C7 F1 3B 00 00 00 00 00 00 00 00 C9 96 20 41 50 3C 11 BC 7C 1B 09 BA 00 00 00 00 00 00 80 7F
00 00 00 00 3B 7E 05 3C 13 86 99 BC 1E A2 DB 3B 00 00 00 00 00 00 00 00 C6 96 20 41 0F 7E B6 BB 7C 1B 09 3A 00 00 00 00 00 00 80 7F
00 00 00 00 42 23 01 3C 2B 8F 2C BE 7E 2D 80 3C 00 00 00 00 00 00 00 00 CA 96 20 41 5B D9 F1 3B 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 BE ED F3 3B 4A 8A 9F BD 34 52 94 3C 00 00 00 00 00 00 00 00 CB 96 20 41 D4 CE 2C 3C 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 26 50 F2 3B 8F BA 8C 3C 0D B4 66 3C 00 00 00 00 00 00 00 00 CB 96 20 41 1C AD 99 3C 7C 1B 09 BA 00 00 00 00 00 00 80 7F
00 00 00 00 47 2B F2 3B 5C 9A C2 3B 89 56 75 3C 00 00 00 00 00 00 00 00 CC 96 20 41 5E 38 CD 3B 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 9B 29 F2 3B 84 9A BB 3B 89 56 75 3C 00 00 00 00 00 00 00 00 CB 96 20 41 78 58 8F 3A 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 97 12 F2 3B 50 0A D2 BB 83 FC 81 3C 00 00 00 00 00 00 00 00 CB 96 20 41 9B 3B 45 3B 02 55 D4 39 00 00 00 00 00 00 80 7F
00 00 00 00 1E 1D F2 3B 99 44 B9 3C 0D B4 66 3C 00 00 00 00 00 00 00 00 CB 96 20 41 6A 38 13 BB 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 21 DE FA 3B 67 1D 2A 3E 84 6A BE 3B 00 00 00 00 00 00 00 00 C9 96 20 41 A0 27 B5 BB FD 45 73 3A 00 00 00 00 00 00 80 7F
00 00 00 00 95 2F FE 3B 1F EF D3 3B F1 CA 2F 3C 00 00 00 00 00 00 00 00 CB 96 20 41 5E 24 19 BB 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 0A 81 FE 3B 15 CC F2 3B 04 69 3E 3C 00 00 00 00 00 00 00 00 C9 96 20 41 5C 62 EF BB 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 FE C3 04 3C 5F 5F 41 3D 04 69 BE 3B 00 00 00 00 00 00 00 00 C9 96 20 41 B8 48 AB BB 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 55 68 05 3C F7 A2 9C BC E0 C7 F1 3B 00 00 00 00 00 00 00 00 C9 96 20 41 87 69 66 3B 7C 1B 09 3A 00 00 00 00 00 00 80 7F
00 00 00 00 8E 2A 02 3C 44 93 52 BE 1A 80 87 3C 00 00 00 00 00 00 00 00 CB 96 20 41 40 93 61 3B 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 07 5F F6 3B 44 96 33 BE 27 C8 AF 3C 00 00 00 00 00 00 00 00 C9 96 20 41 5D 3B A5 BB 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 F9 A0 F2 3B 27 7D B6 BB 89 56 75 3C 00 00 00 00 00 00 00 00 CB 96 20 41 E0 CA 86 3A 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 3C 3B F2 3B EB E4 2C 3B 89 56 75 3C 00 00 00 00 00 00 00 00 CB 96 20 41 4A 04 43 BB 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 1C 25 F2 3B B3 D7 0B 3D 9A AF 5B 3C 00 00 00 00 00 00 00 00 C9 96 20 41 96 6D 36 BC 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 A2 1E F2 3B 91 72 09 3C 89 56 75 3C 00 00 00 00 00 00 00 00 C9 96 20 41 DF 63 A1 BA 02 55 D4 B9 00 00 00 00 00 00 80 7F
00 00 00 00 35 22 F2 3B 1A D6 93 3C 0D B4 66 3C 00 00 00 00 00 00 00 00 C9 96 20 41 CE BA 2D 3B 02 55 D4 39 00 00 00 00 00 00 80 7F
00 00 00 00 52 C3 F8 3B 5C 76 43 3E C8 8E D4 3B 00 00 00 00 00 00 00 00 CB 96 20 41 0B C3 EE BA 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 45 80 04 3C E5 FA B3 3D EB 2F 21 3B 00 00 00 00 00 00 00 00 C9 96 20 41 E5 52 E6 BB 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 19 60 05 3C ED 68 9D BC 7D 80 07 3C 00 00 00 00 00 00 00 00 C9 96 20 41 11 0B 91 3B DB 87 F7 38 00 00 00 00 00 00 80 7F
00 00 00 00 53 7F 05 3C 6E 51 EA BC 1E A2 DB 3B 00 00 00 00 00 00 00 00 CB 96 20 41 57 F7 67 3C 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 EF 79 05 3C 39 9C D3 BC 1A 80 07 3C 00 00 00 00 00 00 00 00 CC 96 20 41 4D 42 2B 3B 02 55 D4 39 00 00 00 00 00 00 80 7F
00 00 00 00 0B 51 F9 3B 79 8C 7A BE 65 19 B7 3C 00 00 00 00 00 00 00 00 CD 96 20 41 96 04 7F 3C 00 00 00 00 00 00 00 00 00 00 80 7F
00 00 00 00 B8 DB F2 3B 50 79 B4 BB 89 56 75 3C 00 00 00 00 00 00 00 00 CE 96 20 41 5C 79 D9 3B 7C 1B 09 3A 00 00 00 00 00 00 80 7F
00 00 00 00 CC 42 F2 3B EE D7 AF 3C 92 11 58 3C 00 00 00 00 00 00 00 00 CC 96 20 41 7D F0 C5 BB 00 00 00 00 00 00 00 00 00 00 80 7F
"""

frames = []
for line in data_hex.strip().split('\n'):
    line = line.strip()
    if not line:
        continue
    hex_bytes = line.split()
    if len(hex_bytes) >= 44:
        payload = bytes.fromhex(''.join(hex_bytes[:40]))
        vals = struct.unpack('<10f', payload)
        frames.append(vals)

print(f'Parsed {len(frames)} frames')

pos_ref = np.array([f[0] for f in frames])
pos_est = np.array([f[1] for f in frames])
speed_fb = np.array([f[2] for f in frames])
iq = np.array([f[3] for f in frames])
speed_ref_out = np.array([f[4] for f in frames])

print(f'pos_ref:  mean={np.mean(pos_ref):.4f} rad, std={np.std(pos_ref):.6f} rad')
print(f'pos_est:  mean={np.mean(pos_est):.4f} rad, std={np.std(pos_est):.6f} rad')
print(f'speed_fb: mean={np.mean(speed_fb):.4f} RPM, std={np.std(speed_fb):.4f} RPM')
print(f'  range:  [{np.min(speed_fb):.4f}, {np.max(speed_fb):.4f}] RPM')
print(f'iq:       mean={np.mean(iq):.4f} A, std={np.std(iq):.6f} A')
print(f'  range:  [{np.min(iq):.4f}, {np.max(iq):.4f}] A')
print(f'speed_ref:mean={np.mean(speed_ref_out):.4f} RPM, std={np.std(speed_ref_out):.6f} RPM')

print(f'\niq peak-to-peak: {np.max(iq)-np.min(iq):.6f} A')
print(f'speed_fb peak-to-peak: {np.max(speed_fb)-np.min(speed_fb):.4f} RPM')

# Check oscillation
print()
print('First 10 samples speed_fb:', [round(x,4) for x in speed_fb[:10].tolist()])
print('First 10 samples iq:', [round(x,4) for x in iq[:10].tolist()])
