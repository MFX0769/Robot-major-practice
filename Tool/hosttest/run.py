#!/usr/bin/env python3
"""主机测试台入口：把固件源文件原样编译到电脑上，硬件用模拟的代替，跑场景测试。

  py Tool\\hosttest\\run.py               编译并跑全部测试
  py Tool\\hosttest\\run.py --arm         只跑机械臂状态机
  py Tool\\hosttest\\run.py --key         只跑按键判定
  py Tool\\hosttest\\run.py --console     只跑控制台 + 真实底盘控制环
  py Tool\\hosttest\\run.py --dbus        只跑遥控器接收切帧
  py Tool\\hosttest\\run.py -v            同时打印固件自己的日志

需要主机 C 编译器。本机没有装 gcc/cl，所以用 pip 包 ziglang 自带的 clang，
装在系统临时目录的独立虚拟环境里（%TEMP%\\taurus_hosttest\\venv），不碰全局环境。
首次运行会自动创建，约 80MB，可随时整个目录删掉。
"""

import argparse
import os
import re
import subprocess
import sys
import venv
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent                     # taurus_h7_all
WORK = Path(os.environ.get("TEMP", str(Path.home()))) / "taurus_hosttest"
VENV = WORK / "venv"
GEN = WORK / "gen"

try:
    sys.stdout.reconfigure(errors="replace")
except Exception:
    pass


def get_zig():
    py = VENV / "Scripts" / "python.exe"
    if not py.exists():
        info("首次运行：在 %s 创建独立虚拟环境并安装 ziglang（约 80MB）" % VENV)
        WORK.mkdir(parents=True, exist_ok=True)
        venv.create(str(VENV), with_pip=True)
        r = subprocess.call([str(py), "-m", "pip", "install", "--quiet", "--disable-pip-version-check", "ziglang"])
        if r != 0:
            info("[ERR] ziglang 安装失败。也可以设环境变量 ZIG_EXE 指向任意 zig.exe")
            return None
    exe = os.environ.get("ZIG_EXE")
    if exe:
        return [exe, "cc"]
    return [str(py), "-m", "ziglang", "cc"]


def info(msg):
    print(msg, flush=True)


# 固件里直接读写的串口寄存器 -> 测试台的模拟寄存器。只改临时副本里的这几处，其余字节原样。
REG_NUM = {"UART5": 5, "USART10": 10, "UART8": 8, "UART9": 9}
REG_RE = re.compile(rb"(" + b"|".join(sorted((n.encode() for n in REG_NUM), key=len, reverse=True)) + rb")->(ISR|RDR|ICR)\b")
REG_CALL = {"ISR": b"sim_uart_isr(%d)", "RDR": b"sim_uart_rdr(%d)", "ICR": b"(*sim_uart_icr(%d))"}


def gen_copy(src):
    """把固件文件复制到临时目录，UARTn->ISR/RDR/ICR 换成模拟函数调用。"""
    data = Path(src).read_bytes()
    names = {n.encode(): n for n in REG_NUM}
    count = [0]

    def repl(m):
        count[0] += 1
        return REG_CALL[m.group(2).decode()] % REG_NUM[names[m.group(1)]]

    out = REG_RE.sub(repl, data)
    GEN.mkdir(parents=True, exist_ok=True)
    dst = GEN / Path(src).name
    dst.write_bytes(out)
    info("[GEN] %s: %d 处寄存器访问换成模拟寄存器，其余字节原样" % (Path(src).name, count[0]))
    return dst


def stage(src):
    """把固件文件复制到临时目录：UARTn->ISR/RDR/ICR 换成模拟寄存器；放进临时目录还为了让
    fake/ 里的假头文件（control_def.h、mode_switch_task.h 等）排在同名真头文件前面。"""
    return gen_copy(src)


def build_and_run(zig, name, sources, verbose, defines=()):
    exe = WORK / (name + ".exe")
    inc = [HERE / "fake", ROOT / "Application", ROOT / "Protocol", ROOT / "Math",
           ROOT / "Tool" / "debug", ROOT / "Bsp", ROOT / "Bsp" / "inc"]
    # 必须编成 32 位：固件把 RAM 地址当 uint32_t 传给 HAL_FLASH_Program，在单片机上是对的，
    # 64 位主机上会被截断。片内 Flash 也要映射在 0x08000000 这种低地址上。
    cmd = list(zig) + ["-target", "x86-windows-gnu", "-std=gnu11", "-O0", "-w",
                       "-D__packed=__attribute__((packed))"]   # armcc 关键字，主机编译器用 GCC 写法
    for d in defines:
        cmd += ["-D" + d]
    for d in inc:
        cmd += ["-I", str(d)]
    cmd += [str(HERE / s) if isinstance(s, str) else str(s) for s in sources]
    cmd += ["-o", str(exe)]
    info("[CC] " + name)
    r = subprocess.run(cmd, capture_output=True, text=True, errors="replace")
    if r.returncode != 0:
        info(r.stdout[-4000:])
        info(r.stderr[-6000:])
        info("[ERR] %s 编译失败" % name)
        return 1
    env = dict(os.environ)
    if verbose:
        env["SIM_VERBOSE"] = "1"
    p = subprocess.run([str(exe)], capture_output=True, text=True, errors="replace", env=env)
    info(p.stdout.rstrip())
    if p.stderr.strip():
        info(p.stderr.strip()[-2000:])
    rc = p.returncode
    # test_console 第二遍带参数 boot：上电后头 2 秒的巡线缓冲只出现一次，得从干净状态跑
    if name == "test_console" and rc == 0:
        p = subprocess.run([str(exe), "boot"], capture_output=True, text=True, errors="replace", env=env)
        info(p.stdout.rstrip())
        if p.stderr.strip():
            info(p.stderr.strip()[-2000:])
        rc |= p.returncode
    return rc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", action="store_true")
    ap.add_argument("--key", action="store_true")
    ap.add_argument("--console", action="store_true")
    ap.add_argument("--dbus", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()

    zig = get_zig()
    if zig is None:
        return 2

    app = lambda n: ROOT / "Application" / n
    proto = lambda n: ROOT / "Protocol" / n
    motor = proto("prot_rm_motor.c")            # 真实：电机 -> 0x200/0x1FF 帧打包
    pid = ROOT / "Math" / "pid.c"
    # tfmini.c 是真实驱动：sim.c 的模拟测距传感器要往它的中断里灌字节，所以每个测试都编
    core = ["sim.c", "sim_extra.c", stage(proto("tfmini.c"))]

    jobs = []
    if not a.key and not a.console and not a.dbus:
        jobs.append(("test_arm", core + ["sim_stubs_arm.c", "test_arm.c", pid, app("arm_yaw_limit.c"),
                                         ROOT / "Bsp" / "bsp_user_key.c", motor], ()))
    if not a.arm and not a.console and not a.dbus:
        jobs.append(("test_key", core + ["sim_stubs_arm.c", "test_key.c", motor], ()))
    if not a.arm and not a.key and not a.console:
        jobs.append(("test_dbus", core + ["sim_stubs_console.c", "test_dbus.c", stage(proto("prot_dbus.c")),
                                          stage(proto("prot_sbus.c")), motor], ("SIM_NO_DBUS_STUBS", "SIM_NO_SERVO_STUBS")))
    if not a.arm and not a.key and not a.dbus:
        # arm_task.c 由 test_console.c 直接 #include（要调它的静态函数），所以这里不再单列
        jobs.append(("test_console", core + ["sim_stubs_console.c", "test_console.c", stage(proto("servo_test.c")),
                                             stage(app("chassis_task.c")), pid, app("arm_yaw_limit.c"),
                                             ROOT / "Bsp" / "bsp_user_key.c", motor, stage(proto("prot_sbus.c"))], ()))

    rc = 0
    for name, srcs, defs in jobs:
        rc |= build_and_run(zig, name, srcs, a.verbose, defs)
    info("\n[RESULT] " + ("ALL PASS" if rc == 0 else "FAILURES (exit %d)" % rc))
    return rc


if __name__ == "__main__":
    sys.exit(main())
