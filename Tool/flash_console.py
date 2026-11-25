#!/usr/bin/env python3
"""taurus_h7_all 调试一条龙：编译 -> 烧录 -> 纯 SWD 放行 -> 读/发控制台

在 taurus_h7_all 根目录下用 py 启动器运行（裸 python 是 WindowsApps 占位符）：

  py Tool\\flash_console.py find                   列出串口和调试器 SN
  py Tool\\flash_console.py build [--rebuild]      Keil 编译，打印体积和 hex SHA256
  py Tool\\flash_console.py flash                  烧录 + 纯 SWD 放行（两步 openocd）
  py Tool\\flash_console.py read [秒]              读控制台，默认 5 秒
  py Tool\\flash_console.py send "yaw status" [秒] 发一条命令并读回复，默认 2 秒
  py Tool\\flash_console.py go                     build + flash + read 6

选项：
  --serial 8E74D081   CMSIS-DAP 的 SN（默认 8E74D081，即现在用的无线 DAP）
  --baud 115200       控制台波特率（USART10 和板载 USB 虚拟串口都是 115200）
  --port COM5         直接指定控制台串口，不按 SN 找。调试器的虚拟串口不通时，
                     换板载 USB 虚拟串口（同一个命令解析器），先跑 find 看是哪个 COM
  --dry-run           只打印将要执行的命令

烧录/放行的 openocd 序列来自 Develop_Log 里 2026-11-23 验证有效的做法：
program 与放行分两次会话；放行 = 清 DHCSR 调试控制 + AIRCR 系统复位。
"""

import argparse
import hashlib
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PROJECT = ROOT / "MDK-ARM" / "CtrBoard-H7_ALL.uvprojx"
HEX = ROOT / "MDK-ARM" / "CtrBoard-H7_ALL" / "CtrBoard-H7_ALL.hex"
BUILD_LOG = ROOT / "MDK-ARM" / "build_tool.log"

UV4_CANDIDATES = [
    r"D:\Software\Keil\Keil_v5\UV4\UV4.exe",
    r"C:\Keil_v5\UV4\UV4.exe",
    r"C:\Program Files\Keil_v5\UV4\UV4.exe",
    r"C:\Program Files (x86)\Keil_v5\UV4\UV4.exe",
]
OPENOCD_CANDIDATES = [
    r"D:\Software\openocd\xpack-openocd-0.12.0-4\bin\openocd.exe",
]

RELEASE = ("init; halt; mww 0xE000EDF0 0xA05F0000; mww 0xE000ED0C 0x05FA0004; "
           "sleep 500; resume; exit")

try:
    sys.stdout.reconfigure(errors="replace")
except Exception:
    pass


def info(msg):
    print(msg, flush=True)


def find_exe(env, candidates, name):
    p = os.environ.get(env)
    if p and Path(p).exists():
        return p
    for c in candidates:
        if Path(c).exists():
            return c
    return shutil.which(name)


def sha256_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def do_build(rebuild, dry):
    uv4 = find_exe("KEIL_UV4", UV4_CANDIDATES, "UV4.exe")
    if uv4 is None:
        info("[ERR] 找不到 UV4.exe，设环境变量 KEIL_UV4 指向它")
        return False
    cmd = [uv4, "-r" if rebuild else "-b", str(PROJECT), "-j0", "-o", str(BUILD_LOG)]
    info("[RUN] " + " ".join(cmd))
    if dry:
        return True
    if BUILD_LOG.exists():
        BUILD_LOG.unlink()
    rc = subprocess.call(cmd)
    text = BUILD_LOG.read_bytes().decode("gbk", "replace") if BUILD_LOG.exists() else ""
    tail = [l for l in text.splitlines() if re.search(r"Program Size|Error\(s\)", l)]
    for l in tail[-3:]:
        info("       " + l.strip())
    if not HEX.exists():
        info("[ERR] 没有生成 hex")
        return False
    info("       hex SHA256 " + sha256_of(HEX))
    ok = ("0 Error(s)" in text) and rc in (0, 1)
    info("[OK] 编译通过" if ok else "[ERR] 编译失败，UV4 退出码 %d，见 %s" % (rc, BUILD_LOG))
    return ok


def write_probe_cfg(serial):
    cfg = Path(tempfile.gettempdir()) / "taurus_dap_swd.cfg"
    cfg.write_text(
        "adapter driver cmsis-dap\n"
        "adapter vid_pid 0xfaed 0x4873\n"
        "adapter serial %s\n"
        "transport select swd\n"
        "adapter speed 1000\n"
        "reset_config none\n"
        "source [find target/stm32h7x.cfg]\n" % serial.lower(),
        encoding="ascii")
    return cfg


def do_flash(serial, dry):
    openocd = find_exe("OPENOCD", OPENOCD_CANDIDATES, "openocd")
    if openocd is None:
        info("[ERR] 找不到 openocd，设环境变量 OPENOCD 或加入 PATH")
        return False
    if not HEX.exists():
        info("[ERR] 没有 hex，先 build")
        return False
    cfg = write_probe_cfg(serial)
    info("       hex SHA256 " + sha256_of(HEX))
    # 第一步：停核 + 烧录 + 校验。不带 reset（DAP 没接 nRESET）
    c1 = [openocd, "-f", str(cfg), "-c",
          "init; halt; program %s verify; exit" % HEX.as_posix()]
    # 第二步：纯 SWD 放行。resume 可能报 not halted，退出码不作数
    c2 = [openocd, "-f", str(cfg), "-c", RELEASE]
    for tag, cmd in (("program", c1), ("release", c2)):
        info("[RUN] " + " ".join(cmd))
        if dry:
            continue
        r = subprocess.run(cmd, capture_output=True, text=True, errors="replace")
        out = (r.stdout or "") + (r.stderr or "")
        key = [l for l in out.splitlines()
               if re.search(r"Verified OK|Programming Finished|Error|error|not halted|shutdown", l)]
        for l in key[-6:]:
            info("       " + l.strip())
        if tag == "program" and "Verified OK" not in out:
            info("[ERR] 烧录未通过校验")
            return False
    info("[OK] 烧录并放行" if not dry else "[DRY] 跳过烧录")
    return True


def find_port(sn):
    from serial.tools import list_ports
    for p in list_ports.comports():
        if (p.serial_number or "").lower() == sn.lower():
            return p.device
    return None


def do_read(port, baud, seconds, send=None, dry=False):
    if port is None:
        info("[ERR] 没有可用串口，先跑 find 确认调试器在线")
        return False
    info("[RUN] open %s @%d for %.1fs%s" % (port, baud, seconds, (" send " + repr(send)) if send else ""))
    if dry:
        return True
    import serial
    got = bytearray()
    with serial.Serial(port, baud, timeout=0.2) as s:
        if send is not None:
            time.sleep(0.3)
            s.reset_input_buffer()
            s.write((send.rstrip("\r\n") + "\r\n").encode("ascii"))
        end = time.time() + seconds
        while time.time() < end:
            got += s.read(4096)
    text = got.decode("ascii", "replace")
    sys.stdout.write(text if text.endswith("\n") or not text else text + "\n")
    info("[OK] %d 字节" % len(got))
    return True


def do_find(sn):
    try:
        from serial.tools import list_ports
    except ImportError:
        info("[ERR] 没有 pyserial：py -m pip install pyserial")
        return
    hit = False
    ports = sorted(list_ports.comports(), key=lambda p: p.device)
    if not ports:
        info("[WARN] 一个串口都没有：调试器和板子的 USB 线都检查一下")
    for p in ports:
        mark = "  <== DAP console" if (p.serial_number or "").lower() == sn.lower() else ""
        hit = hit or bool(mark)
        info("%-8s VID:PID=%s  SN=%s  %s%s" % (p.device, p.vid and ("%04X:%04X" % (p.vid, p.pid)) or "?",
                                               p.serial_number or "?", p.description, mark))
    if not hit:
        info("[WARN] 没看到 SN=%s 的调试器串口：检查无线 DAP 是否连上；"
             "板子的 USB 虚拟串口可以用 --port 直接指定" % sn)
    info("UV4:    %s" % (find_exe("KEIL_UV4", UV4_CANDIDATES, "UV4.exe") or "未找到"))
    info("openocd:%s" % (find_exe("OPENOCD", OPENOCD_CANDIDATES, "openocd") or "未找到"))


def main():
    ap = argparse.ArgumentParser(description="taurus_h7_all build/flash/console")
    ap.add_argument("action", choices=["find", "build", "flash", "read", "send", "go"])
    ap.add_argument("arg", nargs="?", help="send 的命令文本")
    ap.add_argument("sec", nargs="?", type=float, help="读取秒数")
    ap.add_argument("--serial", default="8E74D081")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--port", default=None, help="直接指定控制台串口（例如板载 USB 虚拟串口）")
    ap.add_argument("--rebuild", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    # `read 10` 里的 10 会被当成第二个位置参数，这里把它当秒数
    if a.action == "read" and a.arg is not None and a.sec is None:
        a.sec = float(a.arg)

    if a.action == "find":
        do_find(a.serial)
        return 0

    port = a.port or find_port(a.serial)
    if a.action in ("read", "send", "go") and port is None and not a.dry_run:
        info("[WARN] 没找到可用的控制台串口（--serial %s 或 --port COMx）" % a.serial)

    rc = 0
    if a.action == "build":
        rc = 0 if do_build(a.rebuild, a.dry_run) else 1
    elif a.action == "flash":
        rc = 0 if do_flash(a.serial, a.dry_run) else 1
    elif a.action == "read":
        rc = 0 if do_read(port, a.baud, a.sec if a.sec is not None else 5.0, dry=a.dry_run) else 1
    elif a.action == "send":
        if not a.arg:
            info('用法: send "yaw status" [秒]')
            rc = 2
        else:
            rc = 0 if do_read(port, a.baud, a.sec if a.sec is not None else 2.0, send=a.arg, dry=a.dry_run) else 1
    elif a.action == "go":
        if not do_build(False, a.dry_run):
            rc = 1
        elif not do_flash(a.serial, a.dry_run):
            rc = 1
        else:
            do_read(port, a.baud, 6.0, dry=a.dry_run)
    return rc


if __name__ == "__main__":
    sys.exit(main())
