"""
速度环带宽测试 — 设置 PI → 阶跃 → 分析 → 恢复

用法: python bandwidth_test.py COM5
"""

import serial, time, sys, os, glob, struct, csv, re, numpy as np

BAUD = 230400
TESTS = [("RT100", "M1", 0, 100), ("RT200", "M1", 0, 200), ("RT50", "M1", 0, 50)]
AUTO_INTERVAL = 6.0
BURST_MAGIC = b'RBUS'


def send(ser, cmd):
    """发送 ASCII 命令 + 读取回显"""
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


def extract_pi(resp):
    m = re.search(r'Kp=([\d.]+)\s+Ki=([\d.]+)', resp)
    return (float(m.group(1)), float(m.group(2))) if m else (None, None)


def run(ser, cmd):
    """发送命令, 直接读串口, 提取 burst"""
    # 排空缓冲 + 等 MCU 稳定
    for _ in range(20):
        while ser.in_waiting:
            ser.read(ser.in_waiting)
        time.sleep(0.05)
    print(f"  TX: {cmd}")
    ser.write((cmd + '\r').encode('ascii'))

    # 直接读串口数据, 等待 AUTO_INTERVAL 秒
    data = bytearray()
    deadline = time.time() + AUTO_INTERVAL
    while time.time() < deadline:
        n = ser.in_waiting
        if n > 0:
            data.extend(ser.read(n))
        time.sleep(0.01)

    # 搜索 burst magic
    blocks = []
    pos = 0
    while pos < len(data):
        idx = data.find(BURST_MAGIC, pos)
        if idx < 0:
            break
        if len(data) >= idx + 8:
            count = struct.unpack('<H', data[idx+4:idx+6])[0]
            fields = data[idx+6]
            total = 8 + count * fields * 4
            if len(data) >= idx + total:
                block = data[idx+8:idx+total]
                rows = []
                for i in range(count):
                    off = i * fields * 4
                    vals = struct.unpack(f'<{fields}f', block[off:off+fields*4])
                    rows.append(vals)
                blocks.append((fields, rows))
                pos = idx + total
                continue
        break
    return blocks


def analyze_one(rows, label, step_from, step_to):
    """单个阶跃分析"""
    n = len(rows)
    if n < 50:
        return None
    t = np.arange(n)
    fb = np.array([r[1] for r in rows])
    ref = np.array([r[3] for r in rows])
    iq = np.array([r[2] for r in rows])
    # 用 speed_ref 的跳变来精确定位阶跃点
    ref_chg = np.abs(np.diff(ref))
    step_idx = int(np.argmax(ref_chg)) + 1  # ref 突变点
    step_t = t[step_idx]
    step_from = float(fb[step_idx - 1])  # 实际起始速度
    step_to_target = float(np.mean(ref[-50:]))  # 实际目标
    step_size = step_to_target - step_from

    if abs(step_size) < 1:
        return None

    # Rise 10-90%
    lo, hi = step_from + 0.1*step_size, step_from + 0.9*step_size
    post = fb[step_idx:]
    if step_size > 0:
        a10 = int(np.argmax(post >= lo))
        a90 = int(np.argmax(post >= hi))
    else:
        a10 = int(np.argmax(post <= lo))
        a90 = int(np.argmax(post <= hi))
    t10 = t[step_idx + a10] if a10 > 0 else None
    t90 = t[step_idx + a90] if a90 > 0 else None
    rise = (t90 - t10) if (t10 is not None and t90 is not None) else None

    # Overshoot
    if step_size > 0:
        os = max(0.0, (float(np.max(post)) - step_to_target) / abs(step_size) * 100)
    else:
        os = max(0.0, (step_to_target - float(np.min(post))) / abs(step_size) * 100)

    # Settling ±5%
    bound = max(abs(step_size) * 0.05, 2.0)
    settle = None
    for j in range(step_idx, n - 20):
        if all(abs(fb[j:j+20] - step_to_target) <= bound):
            settle = t[j] - step_t
            break

    ss_start = max(step_idx + 100, n - 100)
    ss_std = float(np.std(fb[ss_start:])) if n - ss_start > 20 else 0
    peak_iq = float(np.max(np.abs(iq[step_idx:])))

    # 阻尼比和等效带宽
    import math
    if os > 0.5 and os < 90:
        zeta = -math.log(os/100) / math.sqrt(math.pi**2 + math.log(os/100)**2)
        if settle:
            wn = 4 / (zeta * settle / 1000)
            bw = wn * math.sqrt(1 - 2*zeta*zeta + math.sqrt(4*zeta**4 - 4*zeta*zeta + 2))
            bw_hz = bw / (2 * math.pi)
        else:
            bw_hz = None
    else:
        zeta, bw_hz = None, None

    return {'label': label, 'rise': rise, 'overshoot': os, 'settle': settle,
            'ss_std': ss_std, 'peak_iq': peak_iq, 'bw_hz': bw_hz, 'n': n}


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM5'
    new_kp, new_ki = 0.030, 0.200

    print(f"串口: {port}")
    ser = serial.Serial(port, BAUD, timeout=0.5)

    # 读原始 PI
    print("\n=== 当前 PI ===")
    resp = send(ser, 'PRS')
    orig_kp, orig_ki = extract_pi(resp)
    print(f"  M1 Speed PI: Kp={orig_kp} Ki={orig_ki}")

    # 先停车, 排空串口残余
    ser.write(b'R0\r')
    time.sleep(1.0)
    while ser.in_waiting:
        ser.read(ser.in_waiting)
        time.sleep(0.05)

    # 设置新 PI
    print(f"\n=== 设置新 PI: Kp={new_kp} Ki={new_ki} ===")
    resp = send(ser, f'PRS {new_kp} {new_ki}')
    print(f"  {resp.strip()}")

    # 跑阶跃
    print(f"\n=== 阶跃测试 ===")
    results = []
    for cmd, label, sf, st in TESTS:
        blocks = run(ser, cmd)
        if blocks:
            rows = blocks[-1][1]
            m = analyze_one(rows, label, sf, st)
            if m:
                m['to'] = st
                m['from'] = sf
                results.append(m)
                bw = f"{m['bw_hz']:.1f}Hz" if m['bw_hz'] else '---'
                r = f"{m['rise']:.0f}ms" if m['rise'] else '---'
                s = f"{m['settle']:.0f}ms" if m['settle'] else '---'
                print(f"  {label} {sf}→{st}: rise={r} settle={s} OS={m['overshoot']:.1f}% "
                      f"σ={m['ss_std']:.2f}RPM bw={bw} n={m['n']}")
            else:
                print(f"  {label} {sf}→{st}: 数据不足")
        else:
            print(f"  {label} {sf}→{st}: 未收到 burst")

    # 恢复 PI
    print(f"\n=== 恢复 PI: Kp={orig_kp} Ki={orig_ki} ===")
    resp = send(ser, f'PRS {orig_kp} {orig_ki}')
    print(f"  {resp.strip()}")

    ser.close()

    # 与之前数据对比
    prev = {(0, 100): (10, 65, 6.6), (0, 200): (9, 10, 4.9)}
    print(f"\n=== 对比 (Kp=0.015→0.030, Ki=0.10→0.200) ===")
    print(f"{'测试':<16} {'Rise(前)':>8} {'Rise(现)':>8} {'Settle(前)':>10} {'Settle(现)':>10} {'OS(前)':>7} {'OS(现)':>7} {'BW(现)':>7}")
    for m, (cmd, label, sf, st) in zip(results, TESTS):
        pr, ps, pos = prev.get((sf, st), (None, None, None))
        r1 = f"{pr}ms" if pr else '---'
        r2 = f"{m['rise']:.0f}ms" if m['rise'] else '---'
        s1 = f"{ps}ms" if ps else '---'
        s2 = f"{m['settle']:.0f}ms" if m['settle'] else '---'
        o1 = f"{pos}%" if pos else '---'
        o2 = f"{m['overshoot']:.1f}%"
        bw = f"{m['bw_hz']:.1f}Hz" if m['bw_hz'] else '---'
        print(f"  {label} {sf}→{st:<4}  {r1:>8}  {r2:>8}  {s1:>10}  {s2:>10}  {o1:>7}  {o2:>7}  {bw:>7}")

    print("\nDone.")


if __name__ == '__main__':
    main()
