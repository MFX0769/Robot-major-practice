#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""舵机串口实验脚本（2026-11-21 现场用）

用途：一锤定音车上肩/肘/夹爪舵机是"星呗 SP 系列串口总线舵机"还是 PWM 舵机。
接线：CH343 USB-TTL 的 TXD -> 舵机 Signal 线（要回读就把 TXD/RXD 短接成单线），
      GND 共地，舵机供电走原降压链路（别用 USB-TTL 的 5V 带舵机）。
协议：单线半双工 UART 115200-8N1 文本协议，详见
      Develop_Log/20261120_机械臂架构与舵机通信调研.md。

用法：
  python servo_probe.py                # 自动扫所有 COM 口，逐个发 PVER
  python servo_probe.py -p COM7        # 指定串口
  python servo_probe.py -p COM7 -i     # 进入交互模式，手动敲指令（#000PRAD! 等）
判定：
  收到 #000PZServo...G!  → 是串口总线舵机，prot_xb_servo 驱动可激活
  无任何应答             → 再试 PWM 路径（现有代码），或检查波特率/接线
"""
import argparse
import sys
import time

import serial
import serial.tools.list_ports

BAUDS = [115200, 9600, 38400, 19200, 57600, 128000, 256000]
PROBE_CMDS = [
    ("#000PVER!", "读版本（总线舵机铁证）"),
    ("#000PID!",  "ID 探测"),
    ("#000PRAD!", "读当前位置"),
    ("#255PVER!", "广播读版本（ID 不是 000 时的兜底）"),
]


def probe_once(port, baud, cmd, wait=0.30):
    """发一条指令并收集应答，返回应答文本或 None"""
    try:
        s = serial.Serial(port, baud, timeout=0.05)
    except serial.SerialException as e:
        return None
    try:
        s.reset_input_buffer()
        s.write(cmd.encode())
        end = time.time() + wait
        buf = b""
        while time.time() < end:
            n = s.in_waiting
            if n:
                buf += s.read(n)
            time.sleep(0.02)
        return buf.decode("ascii", errors="replace").strip() if buf else None
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-p", "--port", help="串口名，如 COM7（缺省自动扫描全部）")
    ap.add_argument("-i", "--interactive", action="store_true", help="交互模式")
    args = ap.parse_args()

    ports = [args.port] if args.port else [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        print("没找到任何串口——CH343 驱动装了吗？")
        return 1

    verdict = None
    for port in ports:
        for baud in BAUDS:
            for cmd, desc in PROBE_CMDS:
                reply = probe_once(port, baud, cmd)
                tag = f"{port}@{baud} [{desc}] {cmd}"
                if reply:
                    print(f"  {tag}  ->  {reply}")
                    if "Servo" in reply or reply.startswith("#000P") or reply == "#OK!":
                        verdict = (port, baud, reply)
                else:
                    print(f"  {tag}  ->  (无应答)")
                if verdict:
                    break
            if verdict:
                break
        if verdict:
            break

    print("\n========== 结论 ==========")
    if verdict:
        print(f"★ 是串口总线舵机！{verdict[0]} @ {verdict[0]} 收到 {verdict[2]}")
        print("  下一步：告知 agent 激活 prot_xb_servo（UART4@PC10），arm_task 切串口路径")
    else:
        print("× 所有串口/波特率无总线舵机应答。")
        print("  两种可能：a) 是 PWM 舵机（现有 PWM 代码直接能用）")
        print("           b) 接线/供电问题：确认 Signal 接 TXD、共地、舵机供电正常（上电时舵机有没有锁零位力矩？）")

    if args.interactive:
        port = verdict[0] if verdict else (ports[0] if ports else None)
        baud = verdict[1] if verdict else 115200
        print(f"\n交互模式 {port}@{baud}，直接敲指令回车发送，q 退出：")
        s = serial.Serial(port, baud, timeout=0.05)
        while True:
            line = input(">> ").strip()
            if line.lower() == "q":
                break
            if not line:
                continue
            s.write((line + "\r\n").encode())
            time.sleep(0.3)
            while s.in_waiting:
                print(s.read(s.in_waiting).decode("ascii", errors="replace"), end="")
            print()
        s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
