"""
无线调试器串口工具 - COM23

功能：
1. 监听STM32 debug串口输出
2. 发送数据给STM32
3. 自动检测波特率
4. 集成到自动化调试流程

使用方法：
    python wireless_debug.py monitor              # 监听串口
    python wireless_debug.py send "Hello"         # 发送数据
    python wireless_debug.py monitor --log        # 监听并保存日志
    python wireless_debug.py test                 # 测试收发
"""

import serial
import serial.tools.list_ports
import time
import sys
import argparse
from datetime import datetime
from pathlib import Path

# 配置
DEFAULT_PORT = "COM23"
DEFAULT_BAUD = 115200
LOG_DIR = Path(__file__).parent.parent.parent / "Develop_Log"

class WirelessDebugger:
    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD):
        self.port = port
        self.baud = baud
        self.serial = None
        
    def connect(self):
        """连接串口"""
        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=1)
            print(f"[OK] 已连接 {self.port} @ {self.baud} baud")
            return True
        except Exception as e:
            print(f"[ERROR] 连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print(f"[OK] 已断开 {self.port}")
    
    def send(self, data):
        """发送数据"""
        if not self.serial or not self.serial.is_open:
            print("[ERROR] 未连接")
            return False
        
        if isinstance(data, str):
            data = data.encode('utf-8')
        
        self.serial.write(data)
        print(f"[TX] {data}")
        return True
    
    def receive(self, timeout=1):
        """接收数据"""
        if not self.serial or not self.serial.is_open:
            return b''
        
        time.sleep(timeout)
        data = self.serial.read(self.serial.in_waiting or 4096)
        if data:
            print(f"[RX] {data}")
        return data
    
    def monitor(self, log_file=None, duration=None):
        """监听串口输出"""
        if not self.connect():
            return
        
        print(f"\n=== 监听 {self.port} ===")
        print("按 Ctrl+C 停止\n")
        
        log_f = None
        if log_file:
            log_f = open(log_file, 'w', encoding='utf-8')
            print(f"[LOG] 保存到: {log_file}\n")
        
        start_time = time.time()
        try:
            while True:
                if duration and (time.time() - start_time) > duration:
                    break
                
                if self.serial.in_waiting:
                    data = self.serial.readline()
                    if data:
                        try:
                            text = data.decode('utf-8', errors='ignore').strip()
                            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                            line = f"[{timestamp}] {text}"
                            print(line)
                            if log_f:
                                log_f.write(line + '\n')
                                log_f.flush()
                        except:
                            pass
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n[STOP] 监听停止")
        finally:
            if log_f:
                log_f.close()
            self.disconnect()
    
    def test_loopback(self):
        """测试收发"""
        if not self.connect():
            return
        
        print(f"\n=== 收发测试 ===")
        
        test_messages = [
            "Hello STM32",
            "1234567890",
            "Test message from AI",
            "无线调试测试"
        ]
        
        for msg in test_messages:
            print(f"\n发送: {msg}")
            self.send(msg + '\r\n')
            time.sleep(0.5)
            data = self.receive(timeout=1)
            if data:
                print(f"接收: {data.decode('utf-8', errors='ignore').strip()}")
            else:
                print("接收: (无)")
        
        self.disconnect()
        print("\n[OK] 测试完成")

def detect_wireless_debugger():
    """检测无线调试器串口"""
    ports = serial.tools.list_ports.comports()
    wireless_ports = []
    
    for port in ports:
        if port.vid == 0xEF1A and port.pid == 0x74E5:
            wireless_ports.append(port)
        elif 'EF1A' in str(port.hwid).upper():
            wireless_ports.append(port)
    
    return wireless_ports

def main():
    parser = argparse.ArgumentParser(description='无线调试器串口工具')
    parser.add_argument('action', choices=['monitor', 'send', 'test', 'detect'],
                       help='操作: monitor=监听, send=发送, test=测试, detect=检测')
    parser.add_argument('--port', '-p', default=DEFAULT_PORT, help=f'串口 (默认 {DEFAULT_PORT})')
    parser.add_argument('--baud', '-b', type=int, default=DEFAULT_BAUD, help=f'波特率 (默认 {DEFAULT_BAUD})')
    parser.add_argument('--data', '-d', help='发送的数据')
    parser.add_argument('--log', '-l', action='store_true', help='保存日志')
    parser.add_argument('--duration', type=int, help='监听时长(秒)')
    
    args = parser.parse_args()
    
    if args.action == 'detect':
        print("=== 检测无线调试器 ===")
        ports = detect_wireless_debugger()
        if ports:
            for p in ports:
                print(f"  {p.device} - {p.description}")
                print(f"    VID: 0x{p.vid:04X} PID: 0x{p.pid:04X}")
        else:
            print("  未检测到无线调试器")
        return
    
    debugger = WirelessDebugger(args.port, args.baud)
    
    if args.action == 'monitor':
        log_file = None
        if args.log:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            log_file = LOG_DIR / f"wireless_monitor_{timestamp}.log"
        debugger.monitor(log_file, args.duration)
        
    elif args.action == 'send':
        if not args.data:
            print("[ERROR] 请指定发送数据: --data 'Hello'")
            return
        if debugger.connect():
            debugger.send(args.data + '\r\n')
            time.sleep(0.5)
            debugger.receive(timeout=1)
            debugger.disconnect()
            
    elif args.action == 'test':
        debugger.test_loopback()

if __name__ == "__main__":
    main()
