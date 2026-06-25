"""
STM32H7 全自动编译烧录调试工具

功能：
1. 自动编译 Keil 项目
2. 通过 OpenOCD 烧录固件
3. 启动 GDB 调试会话
4. 支持有线/无线串口监控
5. 无线调试器发送/接收测试

使用方法：
    python stm32h7_tool.py build              # 仅编译
    python stm32h7_tool.py flash              # 仅烧录
    python stm32h7_tool.py debug              # 编译+烧录+调试
    python stm32h7_tool.py monitor            # 有线串口监控
    python stm32h7_tool.py wireless-monitor   # 无线串口监听
    python stm32h7_tool.py wireless-send "Hello"  # 无线发送
    python stm32h7_tool.py wireless-test      # 无线收发测试
    python stm32h7_tool.py full               # 完整流程：编译+烧录+调试+监控
"""

import subprocess
import os
import sys
import time
import argparse
import serial
import serial.tools.list_ports
from pathlib import Path
from datetime import datetime

# 配置
PROJECT_ROOT = Path(__file__).parent.parent.parent
KEIL_PROJECT = PROJECT_ROOT / "MDK-ARM" / "rm_main.uvprojx"
KEIL_OUTPUT = PROJECT_ROOT / "MDK-ARM" / "rm_main" / "rm_main.axf"
HEX_FILE = PROJECT_ROOT / "MDK-ARM" / "rm_main" / "rm_main.hex"
OPENOCD_CFG = PROJECT_ROOT / "Tool" / "debug" / "stm32h7.cfg"
OPENOCD_SCRIPT = PROJECT_ROOT / "Tool" / "debug" / "openocd_controller.py"

# Keil UV4 路径
KEIL_UV4 = r"D:\Users\Tommy0769\AppData\Local\Keil_v5\UV4\UV4.exe"

class STM32H7Tool:
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.openocd_process = None
        
    def log(self, msg, level="INFO"):
        """日志输出"""
        print(f"[{level}] {msg}")
        
    def run_command(self, cmd, check=True, capture=False):
        """执行命令"""
        if self.verbose:
            self.log(f"执行命令: {cmd}")
        try:
            result = subprocess.run(
                cmd, 
                shell=isinstance(cmd, str),
                check=check,
                capture_output=capture,
                text=True
            )
            return result
        except subprocess.CalledProcessError as e:
            self.log(f"命令执行失败: {e}", "ERROR")
            if capture:
                self.log(f"错误输出: {e.stderr}", "ERROR")
            return None
    
    def find_keil(self):
        """查找 Keil UV4"""
        # 检查默认路径
        if os.path.exists(KEIL_UV4):
            return KEIL_UV4
        
        # 搜索常见安装路径
        common_paths = [
            r"C:\Keil_v5\UV4\UV4.exe",
            r"C:\Program Files\Keil_v5\UV4\UV4.exe",
            r"C:\Program Files (x86)\Keil_v5\UV4\UV4.exe",
            r"D:\Keil_v5\UV4\UV4.exe",
        ]
        
        for path in common_paths:
            if os.path.exists(path):
                return path
        
        # 尝试从 PATH 中查找
        try:
            result = subprocess.run(
                ["where", "UV4.exe"],
                capture_output=True,
                text=True
            )
            if result.returncode == 0:
                return result.stdout.strip().split('\n')[0]
        except:
            pass
        
        return None
    
    def build(self):
        """编译 Keil 项目"""
        self.log("=== 开始编译 ===")
        
        keil_path = self.find_keil()
        if not keil_path:
            self.log("未找到 Keil UV4，请检查安装路径", "ERROR")
            self.log("可在脚本顶部修改 KEIL_UV4 变量")
            return False
        
        self.log(f"使用 Keil: {keil_path}")
        
        # 构建命令
        cmd = [
            keil_path,
            "-r",  # 重新编译
            "-o", str(KEIL_PROJECT),
            "-j0"  # 不显示进度对话框
        ]
        
        result = self.run_command(cmd, check=False)
        
        # 检查编译结果
        if KEIL_OUTPUT.exists():
            self.log(f"编译成功: {KEIL_OUTPUT}")
            return True
        else:
            self.log("编译失败：未找到输出文件", "ERROR")
            return False
    
    def flash_openocd(self):
        """使用 OpenOCD 烧录"""
        self.log("=== 开始烧录 (OpenOCD) ===")
        
        if not KEIL_OUTPUT.exists():
            self.log("固件文件不存在，请先编译", "ERROR")
            return False
        
        if not OPENOCD_CFG.exists():
            self.log("OpenOCD 配置文件不存在", "ERROR")
            return False
        
        # 使用 OpenOCD 烧录
        cmd = [
            "openocd",
            "-f", str(OPENOCD_CFG),
            "-c", f"init; halt; program {KEIL_OUTPUT} verify; reset; shutdown"
        ]
        
        result = self.run_command(cmd, check=False, capture=True)
        
        if result and result.returncode == 0:
            self.log("烧录成功")
            return True
        else:
            self.log("烧录失败", "ERROR")
            if result:
                self.log(f"错误: {result.stderr}", "ERROR")
            return False
    
    def flash_stlink(self):
        """使用 ST-Link 烧录（备选方案）"""
        self.log("=== 开始烧录 (ST-Link) ===")
        
        if not HEX_FILE.exists():
            self.log("HEX 文件不存在，请先编译", "ERROR")
            return False
        
        # 使用 STM32_Programmer_CLI
        cmd = [
            "STM32_Programmer_CLI",
            "-c", "port=SWD",
            "-w", str(HEX_FILE),
            "-v",
            "-rst"
        ]
        
        result = self.run_command(cmd, check=False, capture=True)
        
        if result and result.returncode == 0:
            self.log("烧录成功")
            return True
        else:
            self.log("烧录失败", "ERROR")
            return False
    
    def start_openocd(self):
        """启动 OpenOCD 服务"""
        self.log("=== 启动 OpenOCD ===")
        
        cmd = [
            "openocd",
            "-f", str(OPENOCD_CFG)
        ]
        
        try:
            self.openocd_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            time.sleep(2)  # 等待启动
            
            if self.openocd_process.poll() is None:
                self.log("OpenOCD 启动成功")
                return True
            else:
                self.log("OpenOCD 启动失败", "ERROR")
                return False
        except Exception as e:
            self.log(f"启动 OpenOCD 失败: {e}", "ERROR")
            return False
    
    def stop_openocd(self):
        """停止 OpenOCD"""
        if self.openocd_process:
            self.openocd_process.terminate()
            self.openocd_process.wait(timeout=5)
            self.log("OpenOCD 已停止")
    
    def start_gdb(self):
        """启动 GDB 调试会话"""
        self.log("=== 启动 GDB 调试 ===")
        
        if not KEIL_OUTPUT.exists():
            self.log("固件文件不存在，请先编译", "ERROR")
            return False
        
        # 启动 arm-none-eabi-gdb
        gdb_cmds = [
            "target remote localhost:3333",
            "monitor reset halt",
            f"load",
            "break main",
            "continue"
        ]
        
        cmd = ["arm-none-eabi-gdb", str(KEIL_OUTPUT)]
        
        self.log("启动 GDB，请在终端中操作")
        self.log("常用命令:")
        self.log("  target remote localhost:3333")
        self.log("  monitor reset halt")
        self.log("  load")
        self.log("  break main")
        self.log("  continue")
        self.log("  step / next")
        self.log("  info registers")
        self.log("  quit")
        
        try:
            subprocess.run(cmd)
        except FileNotFoundError:
            self.log("未找到 arm-none-eabi-gdb，请确保已安装", "ERROR")
            return False
        
        return True
    
    def monitor_serial(self, port=None, baud=115200, duration=None):
        """串口监控"""
        self.log("=== 串口监控 ===")
        
        # 自动检测串口
        if not port:
            ports = serial.tools.list_ports.comports()
            stm32_ports = [p for p in ports if 'STM32' in p.description or 'ST-Link' in p.description]
            
            if stm32_ports:
                port = stm32_ports[0].device
                self.log(f"自动检测到串口: {port}")
            elif ports:
                port = ports[0].device
                self.log(f"使用第一个串口: {port}")
            else:
                self.log("未检测到串口", "ERROR")
                return False
        
        self.log(f"连接串口: {port} @ {baud} baud")
        
        try:
            ser = serial.Serial(port, baud, timeout=1)
            self.log("串口已连接，开始监控...")
            self.log("按 Ctrl+C 停止")
            
            start_time = time.time()
            while True:
                if duration and (time.time() - start_time) > duration:
                    break
                
                if ser.in_waiting:
                    data = ser.readline().decode('utf-8', errors='ignore')
                    if data:
                        print(data, end='')
                
                time.sleep(0.01)
                
        except serial.SerialException as e:
            self.log(f"串口连接失败: {e}", "ERROR")
            return False
        except KeyboardInterrupt:
            self.log("\n监控已停止")
        finally:
            if 'ser' in locals() and ser.is_open:
                ser.close()
        
        return True
    
    def wireless_monitor(self, port="COM23", baud=115200, duration=None, log_file=None):
        """无线串口监听"""
        self.log("=== 无线串口监听 ===")
        
        try:
            ser = serial.Serial(port, baud, timeout=1)
            self.log(f"已连接 {port} @ {baud} baud")
            self.log("按 Ctrl+C 停止\n")
            
            log_f = None
            if log_file:
                log_f = open(log_file, 'w', encoding='utf-8')
            
            start_time = time.time()
            while True:
                if duration and (time.time() - start_time) > duration:
                    break
                
                if ser.in_waiting:
                    data = ser.readline()
                    if data:
                        text = data.decode('utf-8', errors='ignore').strip()
                        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                        line = f"[{timestamp}] {text}"
                        print(line)
                        if log_f:
                            log_f.write(line + '\n')
                            log_f.flush()
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            self.log("\n监听停止")
        except Exception as e:
            self.log(f"错误: {e}", "ERROR")
        finally:
            if 'ser' in locals() and ser.is_open:
                ser.close()
            if 'log_f' in locals() and log_f:
                log_f.close()
    
    def wireless_send(self, data, port="COM23", baud=115200):
        """无线发送数据"""
        self.log(f"=== 无线发送 ===")
        
        try:
            ser = serial.Serial(port, baud, timeout=2)
            self.log(f"已连接 {port}")
            
            if isinstance(data, str):
                data = data.encode('utf-8')
            
            ser.write(data)
            self.log(f"TX: {data}")
            
            time.sleep(0.5)
            rx = ser.read(ser.in_waiting or 4096)
            if rx:
                self.log(f"RX: {rx.decode('utf-8', errors='ignore')}")
            
            ser.close()
            return True
        except Exception as e:
            self.log(f"错误: {e}", "ERROR")
            return False
    
    def wireless_test(self, port="COM23", baud=115200):
        """无线收发测试"""
        self.log("=== 无线收发测试 ===")
        
        test_messages = [
            "Hello STM32",
            "AI->H7 Test",
            "1234567890"
        ]
        
        try:
            ser = serial.Serial(port, baud, timeout=2)
            self.log(f"已连接 {port}\n")
            
            for msg in test_messages:
                ser.write((msg + '\r\n').encode())
                self.log(f"TX: {msg}")
                time.sleep(0.5)
                rx = ser.read(ser.in_waiting or 1024)
                self.log(f"RX: {rx.decode('utf-8', errors='ignore').strip() or '(无)'}")
                print()
            
            ser.close()
            self.log("[OK] 测试完成")
            return True
        except Exception as e:
            self.log(f"错误: {e}", "ERROR")
            return False
    
    def full_workflow(self):
        """完整工作流程"""
        self.log("=== 完整工作流程 ===")
        
        # 1. 编译
        if not self.build():
            return False
        
        # 2. 烧录
        if not self.flash_openocd():
            return False
        
        # 3. 启动 OpenOCD
        if not self.start_openocd():
            return False
        
        try:
            # 4. 启动 GDB
            self.start_gdb()
        finally:
            # 5. 停止 OpenOCD
            self.stop_openocd()
        
        return True

def main():
    parser = argparse.ArgumentParser(description='STM32H7 全自动编译烧录调试工具')
    parser.add_argument('action', choices=[
        'build', 'flash', 'flash-stlink', 'debug', 'monitor', 'full',
        'wireless-monitor', 'wireless-send', 'wireless-test'
    ], help='执行的操作')
    parser.add_argument('--port', '-p', default='COM23', help='串口端口')
    parser.add_argument('--baud', '-b', type=int, default=115200, help='波特率')
    parser.add_argument('--data', '-d', help='发送的数据')
    parser.add_argument('--duration', type=int, help='监控时长（秒）')
    parser.add_argument('--log', '-l', action='store_true', help='保存日志')
    parser.add_argument('--verbose', '-v', action='store_true', help='详细输出')
    
    args = parser.parse_args()
    
    tool = STM32H7Tool(verbose=args.verbose)
    
    success = False
    
    if args.action == 'build':
        success = tool.build()
    elif args.action == 'flash':
        success = tool.flash_openocd()
    elif args.action == 'flash-stlink':
        success = tool.flash_stlink()
    elif args.action == 'debug':
        if tool.start_openocd():
            try:
                tool.start_gdb()
                success = True
            finally:
                tool.stop_openocd()
    elif args.action == 'monitor':
        success = tool.monitor_serial(args.port, args.baud, args.duration)
    elif args.action == 'wireless-monitor':
        log_file = None
        if args.log:
            from datetime import datetime
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            log_file = str(PROJECT_ROOT / "Develop_Log" / f"wireless_{timestamp}.log")
        success = tool.wireless_monitor(args.port, args.baud, args.duration, log_file)
    elif args.action == 'wireless-send':
        if not args.data:
            print("[ERROR] 请指定发送数据: --data 'Hello'")
        else:
            success = tool.wireless_send(args.data, args.port, args.baud)
    elif args.action == 'wireless-test':
        success = tool.wireless_test(args.port, args.baud)
    elif args.action == 'full':
        success = tool.full_workflow()
    
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
