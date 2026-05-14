"""
串口 CLI 自动化测试脚本

用法:
    python test_cli.py                # 自动扫描串口
    python test_cli.py COM5           # 指定串口

测试覆盖:
    ? 帮助, T 遥测开关, PI 查询/设置, 参数校验, 边界情况
"""

import sys, time, re, serial, serial.tools.list_ports

BAUD = 230400
TIMEOUT = 1.5   # 每条命令等待超时 (秒)
BOOT_WAIT = 3.0 # MCU 启动等待 (秒)

PASS, FAIL = 0, 0

def log(ok, msg):
    global PASS, FAIL
    tag = "PASS" if ok else "FAIL"
    if ok:
        PASS += 1
        print(f"  [{tag}] {msg}")
    else:
        FAIL += 1
        print(f"  [{tag}] {msg}  ***")


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
    """发送命令并读取所有回显，返回回显文本"""
    ser.reset_input_buffer()
    payload = (cmd + '\r').encode('ascii')
    ser.write(payload)
    time.sleep(TIMEOUT)
    buf = b''
    while ser.in_waiting > 0:
        buf += ser.read(ser.in_waiting)
        time.sleep(0.05)
    return buf.decode('ascii', errors='replace')


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
    # 默认关闭，第一次 T 应该打开
    resp1 = send_cmd(ser, 'T')
    ok1 = check_contains(resp1, 'TELEMETRY ON', "T 命令: 开启遥测")

    resp2 = send_cmd(ser, 'T')
    ok2 = check_contains(resp2, 'TELEMETRY OFF', "T 命令: 关闭遥测")
    return ok1 and ok2


def test_pi_query(ser):
    print("\n--- PI 参数查询 ---")

    ok = True
    # 速度 PI 查询
    resp = send_cmd(ser, 'PRS')
    ok &= check_contains(resp, 'M1 Speed PI:', "PRS: M1 速度 PI 标题")
    ok &= check_contains(resp, 'Kp=', "PRS: 包含 Kp")
    ok &= check_contains(resp, 'Ki=', "PRS: 包含 Ki")

    resp = send_cmd(ser, 'PLS')
    ok &= check_contains(resp, 'M2 Speed PI:', "PLS: M2 速度 PI 标题")

    # 电流 PI 查询
    resp = send_cmd(ser, 'PRC')
    ok &= check_contains(resp, 'M1 Current PI:', "PRC: M1 电流 PI 标题")

    resp = send_cmd(ser, 'PLC')
    ok &= check_contains(resp, 'M2 Current PI:', "PLC: M2 电流 PI 标题")

    # 全部 PI 查询 (PR/PL)
    resp = send_cmd(ser, 'PR')
    ok &= check_contains(resp, 'Speed  PI:', "PR: 包含速度 PI")
    ok &= check_contains(resp, 'Current PI:', "PR: 包含电流 PI")

    return ok


def test_pi_set_label(ser):
    print("\n--- PI 参数设置 (标签模式) ---")

    ok = True
    # 速度 PI 设置: P=0.010 I=0.050
    resp = send_cmd(ser, 'PRS P=0.01 I=0.05')
    ok &= check_contains(resp, 'Kp=0.010', "标签模式: Kp=0.010")
    ok &= check_contains(resp, 'Ki=0.050', "标签模式: Ki=0.050")

    # 恢复默认值
    send_cmd(ser, 'PRS P=0.015 I=0.1')

    # 电流 PI 设置
    resp = send_cmd(ser, 'PRC P=5 I=1765')
    ok &= check_contains(resp, 'Kp=5.0', "标签模式 电流: Kp=5.0")
    ok &= check_contains(resp, 'Ki=1765', "标签模式 电流: Ki=1765")

    # 恢复默认
    send_cmd(ser, 'PRC P=12 I=2400')

    return ok


def test_pi_set_positional(ser):
    print("\n--- PI 参数设置 (位置模式) ---")

    # 位置模式: PRS Kp Ki (空格分隔)
    resp = send_cmd(ser, 'PRS 0.020 0.150')
    ok = check_contains(resp, 'Kp=0.020', "位置模式: Kp=0.020")
    ok &= check_contains(resp, 'Ki=0.150', "位置模式: Ki=0.150")

    # 恢复默认
    send_cmd(ser, 'PRS 0.015 0.1')
    return ok


def test_pi_partial(ser):
    print("\n--- PI 参数设置 (部分更新) ---")

    # 只设 Kp，Ki 保持不变
    send_cmd(ser, 'PRS 0.015 0.1')  # reset
    resp1 = send_cmd(ser, 'PRS P=0.030')
    ok = check_contains(resp1, 'Kp=0.030', "部分更新: 只设 Kp")
    ok &= check_contains(resp1, 'Ki=0.100', "部分更新: Ki 保持")

    # 恢复
    send_cmd(ser, 'PRS 0.015 0.1')
    return ok


def test_error_handling(ser):
    print("\n--- 参数校验 ---")

    ok = True
    # 缺少参数
    resp = send_cmd(ser, 'PRS P=')
    ok &= check_not_contains(resp, 'Kp=', "缺值 PRS P=: 不触发设置")

    # 错误的子系统
    resp = send_cmd(ser, 'PRX')
    ok &= check_contains(resp, 'S=Speed', "错误子系统: 提示正确用法")

    return ok


def test_rapid_fire(ser):
    print("\n--- 连续快速命令 ---")

    ok = True
    for i in range(5):
        resp = send_cmd(ser, '?')
        if 'STATUS' not in resp:
            log(False, f"快速命令 #{i+1}: 未收到回显")
            ok = False
            break
    if ok:
        log(True, "快速命令: 5次 ? 全部正常")
    return ok


def main():
    global PASS, FAIL

    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    print(f"串口: {port}")

    ser = serial.Serial(port, BAUD, timeout=0.1)
    print(f"等待 MCU 启动 ({BOOT_WAIT}s)...")
    time.sleep(BOOT_WAIT)

    # 清空启动残余数据
    ser.reset_input_buffer()

    # 先关遥测确保串口干净
    send_cmd(ser, '')
    time.sleep(0.2)
    ser.reset_input_buffer()

    # 确保遥测关闭 (上电默认关，再确认一次)
    send_cmd(ser, 'T')  # may toggle to ON
    send_cmd(ser, 'T')  # toggle back to OFF

    tests = [
        ("帮助命令 ?",              test_help),
        ("遥测开关 T",              test_telemetry_toggle),
        ("PI 查询",                 test_pi_query),
        ("PI 设置 (标签模式)",      test_pi_set_label),
        ("PI 设置 (位置模式)",      test_pi_set_positional),
        ("PI 部分更新",             test_pi_partial),
        ("参数校验",                test_error_handling),
        ("连续快速命令",            test_rapid_fire),
    ]

    for name, fn in tests:
        try:
            fn(ser)
        except Exception as e:
            FAIL += 1
            print(f"  [FAIL] {name}: 异常 {e}")

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
