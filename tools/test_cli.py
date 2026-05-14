"""
串口 CLI 自动化测试脚本

用法:
    python test_cli.py                # 自动扫描串口
    python test_cli.py COM5           # 指定串口

测试覆盖:
    ? 帮助, T 遥测开关, PI 查询 (单电机/两电机)
    PI 设置 — 三种写法:
      key=value: PRS P=0.015 I=0.1  (标签式，单电机)
      位置传参: PRS 0.015 0.1       (位置式，单电机)
      两电机:   PS P=0.020 I=0.150  (标签式，两电机)
    参数校验, 边界情况, 停止测试命令
PI 参数测试完成后自动恢复原始值，不污染 MCU 状态。
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
    return ok


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

    return buf.decode('utf-8', errors='replace')


def extract_pi(resp):
    """从 PRS/PRC/PS/PC 回显中提取 kp, ki 值 (支持多条回显, 取最后一条)"""
    matches = re.findall(r'Kp=([\d.]+)\s+Ki=([\d.]+)', resp)
    if matches:
        return float(matches[-1][0]), float(matches[-1][1])
    return None, None


def extract_pi_all(resp):
    """从回显中提取所有 PI 值对 [(kp, ki), ...]"""
    matches = re.findall(r'Kp=([\d.]+)\s+Ki=([\d.]+)', resp)
    return [(float(m[0]), float(m[1])) for m in matches]


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
    """记录所有 PI 控制器原始值 (单电机 PRS/PRC/PLS/PLC)"""
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
        # 电流 PI 的 Ki 值范围大 (~1765), 用 1% 容差
        tol_ki = max(0.05, abs(ki) * 0.01)
        tol_kp = max(0.001, abs(kp) * 0.01)
        if abs(new_kp - kp) < tol_kp and abs(new_ki - ki) < tol_ki:
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
    ok &= check_contains(resp, 'R<A>', "帮助: 包含电流命令")
    ok &= check_contains(resp, 'RS<', "帮助: 包含速度命令")
    ok &= check_contains(resp, 'RT<', "帮助: 包含阶跃命令")
    ok &= check_contains(resp, 'PRS', "帮助: 包含 PI 查询")
    ok &= check_contains(resp, 'PS', "帮助: 包含两电机 PI")
    ok &= check_contains(resp, 'PC', "帮助: 包含两电机电流 PI")
    ok &= check_contains(resp, 'T', "帮助: 包含遥测开关")
    ok &= check_contains(resp, '停止测试', "帮助: 包含停止提示")
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
    print("\n--- PI 参数查询 (单电机) ---")
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


def test_pi_query_both(ser):
    print("\n--- PI 参数查询 (两电机) ---")
    ok = True

    resp = send_cmd(ser, 'PS')
    ok &= check_contains(resp, 'M1 Speed PI:', "PS: 包含 M1 速度 PI")
    ok &= check_contains(resp, 'M2 Speed PI:', "PS: 包含 M2 速度 PI")
    # 验证回显了两条
    pairs = extract_pi_all(resp)
    ok &= log(len(pairs) >= 2, f"PS: 两电机都有回显 (实际 {len(pairs)} 条)")

    resp = send_cmd(ser, 'PC')
    ok &= check_contains(resp, 'M1 Current PI:', "PC: 包含 M1 电流 PI")
    ok &= check_contains(resp, 'M2 Current PI:', "PC: 包含 M2 电流 PI")

    resp = send_cmd(ser, 'P')
    ok &= check_contains(resp, 'Speed  PI:', "P: 包含速度 PI")
    ok &= check_contains(resp, 'Current PI:', "P: 包含电流 PI")
    # P 应回显 4 条 (两电机 × 两子系统)
    pairs = extract_pi_all(resp)
    ok &= log(len(pairs) >= 4, f"P: 四条 PI 都有回显 (实际 {len(pairs)} 条)")

    return ok


def test_pi_set_label(ser):
    print("\n--- PI 参数设置 (key=value, 单电机) ---")

    # 速度 PI
    resp = send_cmd(ser, 'PRS P=0.010 I=0.050')
    ok = check_contains(resp, 'Kp=0.010', "标签式: Kp=0.010")
    ok &= check_contains(resp, 'Ki=0.050', "标签式: Ki=0.050")

    # 电流 PI
    resp = send_cmd(ser, 'PRC P=5.0 I=1765')
    ok &= check_contains(resp, 'Kp=5.0', "标签式 电流: Kp=5.0")
    ok &= check_contains(resp, 'Ki=1765', "标签式 电流: Ki=1765")

    return ok


def test_pi_set_positional(ser):
    print("\n--- PI 参数设置 (位置传参, 单电机) ---")

    resp = send_cmd(ser, 'PRS 0.020 0.150')
    ok = check_contains(resp, 'Kp=0.020', "位置传参: Kp=0.020")
    ok &= check_contains(resp, 'Ki=0.150', "位置传参: Ki=0.150")
    return ok


def test_pi_set_both(ser):
    print("\n--- PI 参数设置 (两电机同时) ---")

    # 标签式 PS
    resp = send_cmd(ser, 'PS P=0.010 I=0.050')
    ok = check_contains(resp, 'M1 Speed PI:', "两电机标签: M1 回显")
    ok &= check_contains(resp, 'M2 Speed PI:', "两电机标签: M2 回显")
    ok &= check_contains(resp, 'Kp=0.010', "两电机标签: Kp=0.010")

    # 位置式 PS
    resp = send_cmd(ser, 'PS 0.015 0.100')
    ok &= check_contains(resp, 'M1 Speed PI:', "两电机位置: M1 回显")
    ok &= check_contains(resp, 'M2 Speed PI:', "两电机位置: M2 回显")
    ok &= check_contains(resp, 'Kp=0.015', "两电机位置: Kp=0.015")

    # 标签式 PC
    resp = send_cmd(ser, 'PC P=5.0 I=1765')
    ok &= check_contains(resp, 'M1 Current PI:', "两电机电流: M1 回显")
    ok &= check_contains(resp, 'M2 Current PI:', "两电机电流: M2 回显")
    ok &= check_contains(resp, 'Kp=5.0', "两电机电流: Kp=5.0")

    # 验证两电机值一致
    pairs = extract_pi_all(resp)
    if len(pairs) >= 2:
        ok &= log(abs(pairs[0][0] - pairs[1][0]) < 0.001,
                  f"两电机 Kp 一致: {pairs[0][0]:.3f} vs {pairs[1][0]:.3f}")
        ok &= log(abs(pairs[0][1] - pairs[1][1]) < 0.5,
                  f"两电机 Ki 一致: {pairs[0][1]:.1f} vs {pairs[1][1]:.1f}")

    return ok


def test_pi_partial(ser):
    print("\n--- PI 参数设置 (部分更新) ---")

    # 先设基准值
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

    # P 后跟无效字符
    resp = send_cmd(ser, 'PX')
    ok &= check_contains(resp, 'R/L 前缀', "无效前缀: 提示需要 R/L 或 S/C")

    return ok


def test_stop_commands(ser):
    print("\n--- 停止测试命令 (无活跃测试时) ---")

    ok = True
    # RT 不带参数: 无活跃测试 → 提示需要转速值
    resp = send_cmd(ser, 'RT')
    ok &= check_contains(resp, '需要转速值', "RT 无活跃测试: 提示需要转速值")

    # RE 不带参数: 无活跃测试 → 提示需要转速值
    resp = send_cmd(ser, 'RE')
    ok &= check_contains(resp, '需要转速值', "RE 无活跃测试: 提示需要转速值")

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
        print("MCU 已进入正常模式")
        # 确保遥测关闭: 上电默认 OFF, 若意外开启则关掉
        resp = send_cmd(ser, 'T')
        if 'ON' in resp:
            send_cmd(ser, 'T')  # ON → OFF
            print("遥测已关闭\n")
        else:
            print("遥测确认关闭\n")
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
        ("PI 查询 (单电机)",        test_pi_query),
        ("PI 查询 (两电机)",        test_pi_query_both),
    ]
    for name, fn in read_tests:
        try:
            fn(ser)
        except Exception as e:
            FAIL += 1
            print(f"  [FAIL] {name}: 异常 {e}")

    # ===== 3. 运行修改 PI 的测试 =====
    write_tests = [
        ("PI 设置 (key=value, 单电机)",  test_pi_set_label),
        ("PI 设置 (位置传参, 单电机)",   test_pi_set_positional),
        ("PI 设置 (两电机同时)",         test_pi_set_both),
        ("PI 部分更新",                 test_pi_partial),
        ("参数校验",                    test_error_handling),
    ]
    for name, fn in write_tests:
        try:
            fn(ser)
        except Exception as e:
            FAIL += 1
            print(f"  [FAIL] {name}: 异常 {e}")

    # ===== 4. 停止命令测试 =====
    try:
        test_stop_commands(ser)
    except Exception as e:
        FAIL += 1
        print(f"  [FAIL] 停止命令: 异常 {e}")

    # ===== 5. 恢复原始 PI 并验证 =====
    try:
        restore_original_pi(ser)
    except Exception as e:
        log(False, f"恢复原始 PI 异常: {e}")

    # ===== 6. 最终检查 =====
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
