"""
OpenOCD控制器 - AI控制OpenOCD，您只需观看VS Code GUI界面

使用方法：
1. 在VS Code中启动OpenOCD GUI界面
2. 运行此脚本控制调试过程
3. 在VS Code中观看结果
"""

import socket
import time
import subprocess
import os
import json

class OpenOCDController:
    def __init__(self, host='localhost', port=4444):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        
    def connect_telnet(self):
        """连接到OpenOCD的telnet接口"""
        print(f"[连接] 尝试连接OpenOCD Telnet接口: {self.host}:{self.port}")
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print("[连接] OpenOCD Telnet接口连接成功")
            
            # 读取欢迎信息
            welcome = self.socket.recv(1024).decode('utf-8', errors='ignore')
            print(f"[信息] {welcome.strip()}")
            return True
        except Exception as e:
            print(f"[错误] 连接失败: {e}")
            print("[提示] 请确保OpenOCD已在VS Code中启动")
            return False
    
    def send_command(self, command):
        """发送命令到OpenOCD"""
        if not self.connected:
            print("[错误] 未连接到OpenOCD")
            return None
        
        try:
            print(f"[命令] {command}")
            self.socket.send(f"{command}\n".encode('utf-8'))
            
            # 读取响应
            time.sleep(0.5)
            response = b""
            while True:
                try:
                    chunk = self.socket.recv(1024)
                    if not chunk:
                        break
                    response += chunk
                    if b"> " in chunk:  # OpenOCD提示符
                        break
                except socket.timeout:
                    break
            
            decoded = response.decode('utf-8', errors='ignore')
            print(f"[响应] {decoded.strip()}")
            return decoded
        except Exception as e:
            print(f"[错误] 发送命令失败: {e}")
            return None
    
    def reset_device(self):
        """复位设备"""
        print("\n=== 复位设备 ===")
        return self.send_command("reset halt")
    
    def read_register(self, reg_name):
        """读取寄存器"""
        print(f"\n=== 读取寄存器: {reg_name} ===")
        return self.send_command(f"reg {reg_name}")
    
    def read_memory(self, address, length=4):
        """读取内存"""
        print(f"\n=== 读取内存: 0x{address:08X}, 长度: {length}字节 ===")
        return self.send_command(f"mdw 0x{address:08X} {length}")
    
    def set_breakpoint(self, address):
        """设置断点"""
        print(f"\n=== 设置断点: 0x{address:08X} ===")
        return self.send_command(f"halt; bp 0x{address:08X} 2 hw")
    
    def resume(self):
        """继续执行"""
        print("\n=== 继续执行 ===")
        return self.send_command("resume")
    
    def halt(self):
        """暂停执行"""
        print("\n=== 暂停执行 ===")
        return self.send_command("halt")
    
    def flash_erase(self, address, length):
        """擦除Flash"""
        print(f"\n=== 擦除Flash: 0x{address:08X}, 长度: {length} ===")
        return self.send_command(f"flash erase_address 0x{address:08X} {length}")
    
    def flash_write(self, file_path, address=0x08000000):
        """写入Flash"""
        print(f"\n=== 写入Flash: {file_path} → 0x{address:08X} ===")
        if not os.path.exists(file_path):
            print(f"[错误] 文件不存在: {file_path}")
            return None
        
        # 使用openocd命令行工具写入
        cmd = [
            "openocd",
            "-f", "interface/cmsis-dap.cfg",
            "-c", f"init; halt; program {file_path} 0x{address:08X}; reset; shutdown"
        ]
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            print(f"[结果] {result.stdout}")
            if result.stderr:
                print(f"[错误] {result.stderr}")
            return result.returncode == 0
        except Exception as e:
            print(f"[错误] 烧录失败: {e}")
            return None
    
    def disconnect(self):
        """断开连接"""
        if self.connected:
            self.send_command("shutdown")
            self.socket.close()
            self.connected = False
            print("[断开] OpenOCD连接已关闭")
    
    def interactive_mode(self):
        """交互式控制模式"""
        print("\n=== 交互式控制模式 ===")
        print("命令列表:")
        print("  reset      - 复位设备")
        print("  reg <name> - 读取寄存器")
        print("  mem <addr> - 读取内存")
        print("  bp <addr>  - 设置断点")
        print("  continue   - 继续执行")
        print("  halt       - 暂停执行")
        print("  flash <file> - 烧录固件")
        print("  help       - 显示帮助")
        print("  exit       - 退出")
        print()
        
        while True:
            try:
                cmd = input("openocd> ").strip()
                if not cmd:
                    continue
                
                parts = cmd.split()
                action = parts[0].lower()
                
                if action == "exit" or action == "quit":
                    break
                elif action == "help":
                    print("可用命令: reset, reg, mem, bp, continue, halt, flash, help, exit")
                elif action == "reset":
                    self.reset_device()
                elif action == "reg" and len(parts) > 1:
                    self.read_register(parts[1])
                elif action == "mem" and len(parts) > 1:
                    addr = int(parts[1], 0)
                    length = int(parts[2]) if len(parts) > 2 else 4
                    self.read_memory(addr, length)
                elif action == "bp" and len(parts) > 1:
                    addr = int(parts[1], 0)
                    self.set_breakpoint(addr)
                elif action == "continue":
                    self.resume()
                elif action == "halt":
                    self.halt()
                elif action == "flash" and len(parts) > 1:
                    self.flash_write(parts[1])
                else:
                    print("未知命令或参数不足")
            except KeyboardInterrupt:
                print("\n用户中断")
                break
            except Exception as e:
                print(f"错误: {e}")

def main():
    print("=== OpenOCD控制器 ===")
    print("功能: AI控制OpenOCD，您只需在VS Code中观看GUI界面")
    print()
    
    # 创建控制器实例
    controller = OpenOCDController()
    
    # 连接到OpenOCD
    if not controller.connect_telnet():
        print("\n[提示] 请先在VS Code中启动OpenOCD:")
        print("1. 打开VS Code")
        print("2. 启动OpenOCD调试会话")
        print("3. 然后运行此脚本")
        return
    
    # 显示可用命令
    print("\n可用命令:")
    print("  interactive - 进入交互式控制模式")
    print("  reset       - 复位设备")
    print("  demo        - 运�示例演示")
    
    # 简单交互
    try:
        cmd = input("\n请选择命令: ").strip().lower()
        
        if cmd == "interactive":
            controller.interactive_mode()
        elif cmd == "reset":
            controller.reset_device()
        elif cmd == "demo":
            # 运行示例演示
            print("\n=== 运行示例演示 ===")
            controller.reset_device()
            controller.read_register("pc")
            controller.read_register("sp")
            controller.read_memory(0x08001000, 8)
            controller.set_breakpoint(0x08001000)
            print("\n演示完成，请在VS Code GUI中查看结果")
        else:
            print("未知命令")
    
    finally:
        controller.disconnect()

if __name__ == "__main__":
    main()
