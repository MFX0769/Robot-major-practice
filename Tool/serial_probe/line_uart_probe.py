#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""巡线模块串口实验脚本（2026-11-21 现场用）

模块：YAHBOOM YR-TUK04 8路巡线（手册：Source/八路巡线红外传感器/）
关键机制：上电默认不发送，收到命令后持续流式输出——6 月失败大概率是
         没意识到这点 + 校准/极性问题，不是协议跑不通。
协议：$校准开关,模拟开关,数字开关#   例：$0,1,1#（数字+模拟都开）
上行：数字帧 $D,x1:0,...,x8:0#      模拟帧 $A,x1:4096,...,x8:4096#
接线：CH343 TXD -> 模块 RXD，CH343 RXD -> 模块 TXD，共地（模块是 TTL 3.3/5V）
校准提醒：先按手册做完黑线/白线校准（KEY1 长按 → 黑线 3s → 按键 → 白线 3s → 按键），
         校准值断电保存，不做校准数字量不可信。

用法：
  python line_uart_probe.py            # 自动扫口试波特率，使能数字+模拟持续监听
  python line_uart_probe.py -p COM6 -b 115200
  python line_uart_probe.py --digital-only   # 只开数字
  python line_uart_probe.py --analog-only    # 只开模拟（做置信度分析用这个）
Ctrl-C 退出。
"""
import argparse
import time

import serial
import serial.tools.list_ports

BAUDS = [115200, 9600, 38400]
CMD_FMT = "${},{},{}#"          # 校准,模拟,数字


def try_stream(port, baud, cmd, seconds=2.5):
    """发命令后监听 seconds 秒，返回收到的原始文本"""
    try:
        s = serial.Serial(port, baud, timeout=0.05)
    except serial.SerialException as e:
        return None
    try:
        s.reset_input_buffer()
        s.write(cmd.encode())
        end = time.time() + seconds
        buf = b""
        while time.time() < end:
            n = s.in_waiting
            if n:
                buf += s.read(n)
            time.sleep(0.02)
        return buf.decode("ascii", errors="replace") if buf else None
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-p", "--port")
    ap.add_argument("-b", "--baud", type=int, default=None)
    ap.add_argument("--digital-only", action="store_true")
    ap.add_argument("--analog-only", action="store_true")
    args = ap.parse_args()

    cmd = CMD_FMT.format(0, 0 if args.digital_only else 1, 0 if args.analog_only else 1)
    ports = [args.port] if args.port else [p.device for p in serial.tools.list_ports.comports()]
    bauds = [args.baud] if args.baud else BAUDS

    hit = None
    for port in ports:
        for baud in bauds:
            print(f"试 {port}@{baud} 发 {cmd} 监听 2.5s ...")
            data = try_stream(port, baud, cmd)
            if data:
                print(f"  收到 {len(data)} 字节，样例：{data[:120]!r}")
                hit = (port, baud)
                break
            print("  无数据")
        if hit:
            break

    if not hit:
        print("\n× 无数据。排查清单：1) TXD/RXD 是否接反  2) 共地  3) 波特率是否被改过")
        print("  4) 模块红灯是否常亮闪烁（未校准状态，先做校准）5) 供电是否正常")
        return 1

    port, baud = hit
    print(f"\n锁定 {port}@{baud}，持续监听（Ctrl-C 退出）：")
    s = serial.Serial(port, baud, timeout=0.1)
    s.write(cmd.encode())
    frame_cnt = 0
    try:
        while True:
            n = s.in_waiting
            if n:
                data = s.read(n).decode("ascii", errors="replace")
                frame_cnt += data.count("#")
                print(data, end="", flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print(f"\n退出。共收到 {frame_cnt} 帧。")
    finally:
        s.close()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
