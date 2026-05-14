"""
串口 CLI 自动化测试脚本

用法:
    python test_cli.py                # 自动扫描串口
    python test_cli.py COM5           # 指定串口

测试覆盖:
    ? 帮助, T 遥测开关, PI 查询/设置/校验, 边界情况
PI 参数测试完成后自动恢复为原始值，不污染 MCU 状态。
"""

import sys, time, re, serial, serial.tools.list_ports

BAUD = 230400
TIMEOUT = 0.5   # 每条命令最大等待 (秒)

PASS, FAIL = 0, 0

# 原始 PI 值 —— 测试前记录，测试后恢复
_orig_pi = {}


def log(ok, msg):
    global PASS, FAIL
    if ok:
        PASS += 1
        print(f"  [PASS] {msg}")
    else:
        FAIL += 1
        print(f"  [FAIL] {msg}  ***")


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
    """发送命令，轮询读取回显直到超时"""
    ser.reset_input_buffer()
    payload = (cmd + '\r').encode('ascii')
    ser.write(payload)

    buf = b''
    deadline = time.time() + TIMEOUT
    idle_start = time.time()
    while time.time() < deadline:
        n = ser.in_waiting
        if n > 0:
            buf += ser.read(n)
            idle_start = time.time()  # 有数据，重置空闲计时
        elif buf and time.time() - idle_start > 0.05:
            break  # 已收到数据且暂停 50ms，认为回显结束
        else:
            time.sleep(0.01)

    return buf.decode('ascii', errors='replace')


def extract_pi(resp):
    """从 PRS/PRC 回显中提取 kp, ki 值"""
    m = re.search(r'Kp=([\d.]+)\s+Ki=([\d.]+)', resp)
    if m:
        return float(m.group(1)), float(m.group(2))
    return None, None


def check_contains(resp, expected, context=""):
    """检查回显是否包含期望文本"""
    found = expected.lower() in resp.lower()
    log(found, context if context else f"回显包含 '{expected[:40]}'")
    if not found:
        print(f"      回显: {resp.strip()[:120]}")
    return found


def check_not_contains(resp, unexpected, context=""):
    """检查回显不包含特定文本"""
    ok = unexpected.lower() not in resp.lower()
    log(ok, context if context else f"回显不含 '{unexpected[:40]}'")
    if not ok:
        print(f"      回显: {resp.strip()[:120]}")
    return ok


def record_original_pi(ser):
    """记录所有 PI 控制器原始值"""
    print("\n--- 记录原始 PI 参数 ---")
    for motor, label in [('R', 'M1'), ('L', 'M2')]:
        for subsys, name in [('S', 'Speed'), ('C', 'Current')]:
            cmd = f'P{motor}{subsys}'
            resp = send_cmd(ser, cmd)
            kp, ki = extract_pi(resp)
            if kp is not None:
                _orig_pi[cmd] = (kp, ki)
                log(True, f"{label} {name} PI: Kp={kp} Ki={ki}")
            else:
                log(False, f"{label} {name} PI: 提取失败")
                print(f"      回显: {resp.strip()[:120]}")


def restore_original_pi(ser):
    """恢复所有 PI 控制器为原始值并验证"""
    print("\n--- 恢复原始 PI 参数 ---")
    ok = True
    for cmd, (kp, ki) in _orig_pi.items():
        set_cmd = f'{cmd} {kp} {ki}'
        resp = send_cmd(ser, set_cmd)
        new_kp, new_ki = extract_pi(resp)
        if abs(new_kp - kp) < 0.001 and abs(new_ki - ki) < 0.01:
            log(True, f"{cmd}: 恢复 Kp={kp} Ki={ki}")
        else:
            log(False, f"{cmd}: 恢复失败 期望 Kp={kp} Ki={ki} 实际 Kp={new_kp} Ki={new_ki}")
            ok = False
    return ok


# ===== 测试用例 =====

def test_help(ser):
    print("\n--- ? 帮助命令 ---")
    resp = send_cmd(ser, '?')
    ok = check_contains(resp, '=== STATUS ===', "帮助: 状态标题")
    ok &= check_contains(resp, '=== COMMANDS ===', "帮助: 命令列表标题")
    ok &= check_contains(resp, 'R<A>', "帮助: 包含电流命令说明")
    ok &= check_contains(resp, 'RS<', "帮助: 包含速度命令说明")
    ok &= check_contains(resp, 'RT<', "帮助: 包含阶跃命令说明")
    ok &= check_contains(resp, 'PRS', "帮助: 包含 PI 查询说明")
    ok &= check_contains(resp, 'T', "帮助: 包含遥测开关说明")
    return ok


def test_telemetry_toggle(ser):
    print("\n--- T 遥测开关 ---")
    # 默认关闭，第一次 T 打开
    resp1 = send_cmd(ser, 'T')
    ok1 = check_contains(resp1, 'TELEMETRY ON', "T 命令: 开启遥测")

    resp2 = send_cmd(ser, 'T')
    ok2 = check_contains(resp2, 'TELEMETRY OFF', "T 命令: 关闭遥测")
    return ok1 and ok2


def test_pi_query(ser):
    print("\n--- PI 参数查询 ---")
    ok = True

    resp = send_cmd(ser, 'PRS')
    ok &= check_contains(resp, 'M1 Speed PI:', "PRS: M1 速度 PI 标题")
    ok &= check_contains(resp, 'Kp=', "PRS: 包含 Kp")
    ok &= check_contains(resp, 'Ki=', "PRS: 包含 Ki")

    resp = send_cmd(ser, 'PLS')
    ok &= check_contains(resp, 'M2 Speed PI:', "PLS: M2 速度 PI 标题")

    resp = send_cmd(ser, 'PRC')
    ok &= check_contains(resp, 'M1 Current PI:', "PRC: M1 电流 PI 标题")

    resp = send_cmd(ser, 'PLC')
    ok &= check_contains(resp, 'M2 Current PI:', "PLC: M2 电流 PI 标题")

    resp = send_cmd(ser, 'PR')
    ok &= check_contains(resp, 'Speed  PI:', "PR: 包含速度 PI")
    ok &= check_contains(resp, 'Current PI:', "PR: 包含电流 PI")

    return ok


def test_pi_set_label(ser):
    print("\n--- PI 参数设置 (标签模式) ---")

    # 速度 PI: 用与默认不同的值测试
    resp = send_cmd(ser, 'PRS P=0.010 I=0.050')
    ok = check_contains(resp, 'Kp=0.010', "标签模式: Kp=0.010")
    ok &= check_contains(resp, 'Ki=0.050', "标签模式: Ki=0.050")

    # 电流 PI
    resp = send_cmd(ser, 'PRC P=5.0 I=1765')
    ok &= check_contains(resp, 'Kp=5.0', "标签模式 电流: Kp=5.0")
    ok &= check_contains(resp, 'Ki=1765', "标签模式 电流: Ki=1765")

    return ok


def test_pi_set_positional(ser):
    print("\n--- PI 参数设置 (位置模式) ---")

    resp = send_cmd(ser, 'PRS 0.020 0.150')
    ok = check_contains(resp, 'Kp=0.020', "位置模式: Kp=0.020")
    ok &= check_contains(resp, 'Ki=0.150', "位置模式: Ki=0.150")
    return ok


def test_pi_partial(ser):
    print("\n--- PI 参数设置 (部分更新) ---")

    # 先查当前值
    orig_kp, orig_ki = None, None
    resp = send_cmd(ser, 'PRS P=0.015 I=0.1')
    kp_ref, ki_ref = extract_pi(resp)
    if kp_ref is not None and ki_ref is not None:
        # 只改 Kp
        resp = send_cmd(ser, 'PRS P=0.030')
        kp1, ki1 = extract_pi(resp)
        ok = abs(kp1 - 0.030) < 0.001 and abs(ki1 - ki_ref) < 0.001
        log(ok, f"部分更新: Kp={kp1} (期望 0.030) Ki={ki1} (期望 {ki_ref})")
        # 恢复到基准值
        send_cmd(ser, f'PRS {kp_ref} {ki_ref}')
        return ok
    else:
        log(False, "部分更新: 无法提取基准 PI")
        return False


def test_error_handling(ser):
    print("\n--- 参数校验 ---")

    ok = True
    resp = send_cmd(ser, 'PRS P=')
    ok &= check_not_contains(resp, 'Kp=', "缺值 PRS P=: 不触发设置")

    resp = send_cmd(ser, 'PRX')
    ok &= check_contains(resp, 'S=Speed', "错误子系统: 提示正确用法")

    return ok


def test_rapid_fire(ser):
    print("\n--- 连续快速命令 ---")

    for i in range(5):
        resp = send_cmd(ser, '?')
        if 'STATUS' not in resp:
            log(False, f"快速命令 #{i+1}: 未收到回显")
            return False
    log(True, "快速命令: 5次 ? 全部正常")
    return True


def test_status_consistency(ser):
    """测试完所有 PI 修改后，验证遥测仍然关闭且串口干净"""
    print("\n--- 状态一致性 ---")
    resp = send_cmd(ser, '')
    ok = len(resp.strip()) == 0
    log(ok, "串口空闲无数据 (遥测已关闭)")
    return ok


def ensure_normal_mode(ser):
    """确保 MCU 处于正常模式：复位→等 4s 跳过启动按键窗口→验证"""
    print("复位 MCU 并等待正常启动...")

    # 拉低 DTR 触发 MCU 复位，然后释放
    ser.dtr = True
    time.sleep(0.1)
    ser.dtr = False
    time.sleep(0.1)

    # MCU 启动窗口: 3s 倒计时等待按键选择模式
    # 此期间不发任何数据，确保超时进入正常模式
    for i in range(4):
        time.sleep(1)
        print(f"  {4-i}s...")

    # 清空启动期间的 printf 残留
    ser.reset_input_buffer()
    time.sleep(0.3)
    ser.reset_input_buffer()

    # 验证模式 — 正常模式 ? 回显包含 STATUS
    resp = send_cmd(ser, '?')
    if 'STATUS' in resp:
        print("MCU 已进入正常模式\n")
        # 确保遥测默认关闭
        resp = send_cmd(ser, 'T')
        if 'OFF' in resp:
            send_cmd(ser, 'T')  # 再按一次回到 OFF
        return True
    elif '校准' in resp or 'CALIB' in resp.upper():
        print("ERROR: MCU 处于校准模式。请断电重启后重新运行测试。")
        return False
    elif '阶跃' in resp or 'STEP' in resp.upper():
        print("ERROR: MCU 处于阶跃测试模式。请断电重启后重新运行测试。")
        return False
    else:
        print(f"ERROR: 无法识别 MCU 模式。回显: {resp[:120]}")
        return False


def main():
    global PASS, FAIL

    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    print(f"串口: {port}")

    ser = serial.Serial(port, BAUD, timeout=0.1)

    if not ensure_normal_mode(ser):
        ser.close()
        return 1

    # ===== 1. 记录原始 PI =====
    try:
        record_original_pi(ser)
    except Exception as e:
        log(False, f"记录原始 PI 异常: {e}")
        ser.close()
        return 1

    # ===== 2. 运行只读测试 =====
    read_tests = [
        ("帮助命令 ?",              test_help),
        ("遥测开关 T",              test_telemetry_toggle),
        ("PI 查询",                 test_pi_query),
    ]
    for name, fn in read_tests:
        try:
            fn(ser)
        except Exception as e:
            FAIL += 1
            print(f"  [FAIL] {name}: 异常 {e}")

    # ===== 3. 运行修改 PI 的测试 =====
    write_tests = [
        ("PI 设置 (标签模式)",      test_pi_set_label),
        ("PI 设置 (位置模式)",      test_pi_set_positional),
        ("PI 部分更新",             test_pi_partial),
        ("参数校验",                test_error_handling),
    ]
    for name, fn in write_tests:
        try:
            fn(ser)
        except Exception as e:
            FAIL += 1
            print(f"  [FAIL] {name}: 异常 {e}")

    # ===== 4. 恢复原始 PI 并验证 =====
    try:
        restore_original_pi(ser)
    except Exception as e:
        log(False, f"恢复原始 PI 异常: {e}")

    # ===== 5. 最终检查 =====
    try:
        test_rapid_fire(ser)
        test_status_consistency(ser)
    except Exception as e:
        FAIL += 1
        print(f"  [FAIL] 最终检查: 异常 {e}")

    ser.close()

    total = PASS + FAIL
    print(f"\n{'='*40}")
    print(f"结果: {PASS}/{total} 通过", end="")
    if FAIL > 0:
        print(f", {FAIL} 失败")
    else:
        print(", 全部通过!")

    return 0 if FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
