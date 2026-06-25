"""
nanoDLA 逻辑分析仪 - 标准化接口模块

提供统一的LA数据采集、保存、加载、分析接口。

Usage:
    from la_reader import NanoDLA, capture, load, analyze
"""

import usb.core
import usb.util
import libusb_package
import numpy as np
import json
import os
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple, Dict, Any
from dataclasses import dataclass, asdict

# ==================== 常量 ====================

VID = 0x1D50
PID = 0x608C

CMD_GET_FW_VERSION = 0xB0
CMD_START = 0xB1
CMD_GET_REVID_VERSION = 0xB2

CMD_START_FLAGS_CLK_CTL2 = 1 << 4
CMD_START_FLAGS_SAMPLE_16BIT = 1 << 5
CMD_START_FLAGS_CLK_48MHZ = 1 << 6

# ==================== 数据结构 ====================

@dataclass
class CaptureConfig:
    """采集配置"""
    sample_rate: int = 1_000_000      # 采样率 (Hz)
    duration_ms: float = 10.0         # 采集时长 (ms)
    channels: int = 8                 # 通道数 (8或16)
    use_48mhz_clock: bool = True      # 使用48MHz时钟
    trigger_channel: Optional[int] = None  # 触发通道
    trigger_level: Optional[int] = None    # 触发电平 (0或1)

@dataclass
class CaptureResult:
    """采集结果"""
    timestamp: str                    # 采集时间
    config: CaptureConfig             # 采集配置
    raw_data: np.ndarray              # 原始数据
    channel_data: np.ndarray          # 解码后的通道数据
    actual_sample_rate: int           # 实际采样率
    total_samples: int                # 总采样点数
    duration_ms: float                # 实际时长

    def save(self, filepath: str):
        """保存到文件"""
        data = {
            'timestamp': self.timestamp,
            'config': asdict(self.config),
            'raw_data': self.raw_data.tolist(),
            'actual_sample_rate': self.actual_sample_rate,
            'total_samples': self.total_samples,
            'duration_ms': self.duration_ms
        }
        with open(filepath, 'w') as f:
            json.dump(data, f)
        print(f"数据已保存: {filepath}")

    @classmethod
    def load(cls, filepath: str) -> 'CaptureResult':
        """从文件加载"""
        with open(filepath, 'r') as f:
            data = json.load(f)

        config = CaptureConfig(**data['config'])
        raw_data = np.array(data['raw_data'], dtype=np.uint8)
        channel_data = np.unpackbits(raw_data.reshape(-1, 1), axis=1, bitorder='little')[:, :config.channels].astype(bool)

        return cls(
            timestamp=data['timestamp'],
            config=config,
            raw_data=raw_data,
            channel_data=channel_data,
            actual_sample_rate=data['actual_sample_rate'],
            total_samples=data['total_samples'],
            duration_ms=data['duration_ms']
        )


# ==================== 核心类 ====================

class NanoDLA:
    """nanoDLA 逻辑分析仪控制类"""

    def __init__(self):
        self.dev: Optional[usb.core.Device] = None
        self._connected = False

    def connect(self) -> bool:
        """连接设备"""
        self.dev = usb.core.find(idVendor=VID, idProduct=PID, backend=libusb_package.get_libusb1_backend())
        if self.dev is None:
            print("[ERROR] 未找到 nanoDLA 设备")
            return False

        try:
            self.dev.set_configuration()
            self._connected = True
            print(f"[OK] 已连接 nanoDLA")
            return True
        except usb.core.USBError as e:
            print(f"[ERROR] 连接失败: {e}")
            return False

    def disconnect(self):
        """断开连接"""
        if self.dev:
            usb.util.dispose_resources(self.dev)
            self.dev = None
            self._connected = False
            print("[OK] 已断开连接")

    def is_connected(self) -> bool:
        """检查连接状态"""
        return self._connected and self.dev is not None

    def get_fw_version(self) -> Tuple[int, int]:
        """获取固件版本"""
        self._check_connection()
        data = self.dev.ctrl_transfer(0xC0, CMD_GET_FW_VERSION, 0, 0, 2)
        return data[0], data[1]

    def get_chip_id(self) -> int:
        """获取芯片REV ID"""
        self._check_connection()
        data = self.dev.ctrl_transfer(0xC0, CMD_GET_REVID_VERSION, 0, 0, 1)
        return data[0]

    def start_acquisition(self, config: CaptureConfig) -> bool:
        """启动采样"""
        self._check_connection()

        delay_h, delay_l = self._calc_delay(config.sample_rate, config.use_48mhz_clock)

        flags = 0
        if config.use_48mhz_clock:
            flags |= CMD_START_FLAGS_CLK_48MHZ
        if config.channels == 16:
            flags |= CMD_START_FLAGS_SAMPLE_16BIT

        cmd = bytes([flags, delay_h, delay_l])
        try:
            self.dev.ctrl_transfer(0x40, CMD_START, 0, 0, cmd)
            return True
        except usb.core.USBError as e:
            print(f"[ERROR] 启动采样失败: {e}")
            return False

    def read_data(self, num_samples: int = 1024, timeout: int = 1000) -> np.ndarray:
        """读取原始数据"""
        self._check_connection()
        try:
            data = self.dev.read(0x82, num_samples, timeout)
            return np.array(data, dtype=np.uint8)
        except (usb.core.USBTimeoutError, usb.core.USBError):
            return np.array([], dtype=np.uint8)

    def read_until_done(self, total_bytes: int, chunk_size: int = 4096, timeout: int = 5000) -> np.ndarray:
        """读取指定长度数据"""
        all_data = []
        remaining = total_bytes

        while remaining > 0:
            to_read = min(chunk_size, remaining)
            chunk = self.read_data(to_read, timeout)
            if len(chunk) == 0:
                break
            all_data.append(chunk)
            remaining -= len(chunk)

        return np.concatenate(all_data) if all_data else np.array([], dtype=np.uint8)

    def capture(self, config: CaptureConfig) -> CaptureResult:
        """执行完整采集"""
        import time

        num_samples = int(config.sample_rate * config.duration_ms / 1000)

        print(f"[INFO] 开始采集: {config.sample_rate/1e6:.2f}MHz, {config.duration_ms}ms, {config.channels}通道")

        if not self.start_acquisition(config):
            raise RuntimeError("启动采样失败")

        time.sleep(config.duration_ms / 1000 * 1.5)

        raw_data = self.read_until_done(num_samples)

        if len(raw_data) == 0:
            raise RuntimeError("未采集到数据")

        channel_data = np.unpackbits(raw_data.reshape(-1, 1), axis=1, bitorder='little')[:, :config.channels].astype(bool)

        return CaptureResult(
            timestamp=datetime.now().isoformat(),
            config=config,
            raw_data=raw_data,
            channel_data=channel_data,
            actual_sample_rate=config.sample_rate,
            total_samples=len(raw_data),
            duration_ms=len(raw_data) / config.sample_rate * 1000
        )

    def _check_connection(self):
        if not self.is_connected():
            raise RuntimeError("设备未连接，请先调用 connect()")

    def _calc_delay(self, sample_rate: int, use_48mhz: bool) -> Tuple[int, int]:
        clock = 48_000_000 if use_48mhz else 30_000_000
        if sample_rate >= clock:
            return 0, 0
        delay = (clock // sample_rate) - 1
        return min(delay >> 8, 5), delay & 0xFF


# ==================== 标准化接口函数 ====================

# 默认存储目录
DATA_DIR = Path(__file__).parent / "data"

def ensure_data_dir():
    """确保数据目录存在"""
    DATA_DIR.mkdir(exist_ok=True)

def capture(
    sample_rate: int = 1_000_000,
    duration_ms: float = 10.0,
    channels: int = 8,
    save: bool = True,
    tag: str = ""
) -> CaptureResult:
    """
    标准采集接口

    Args:
        sample_rate: 采样率 (Hz)
        duration_ms: 采集时长 (ms)
        channels: 通道数 (8或16)
        save: 是否自动保存到文件
        tag: 文件名标签

    Returns:
        CaptureResult 对象

    Example:
        result = capture(sample_rate=1_000_000, duration_ms=100)
        print(result.channel_data.shape)
    """
    la = NanoDLA()
    if not la.connect():
        raise ConnectionError("无法连接 nanoDLA 设备")

    try:
        config = CaptureConfig(
            sample_rate=sample_rate,
            duration_ms=duration_ms,
            channels=channels
        )
        result = la.capture(config)

        if save:
            ensure_data_dir()
            filename = generate_filename(tag)
            result.save(str(DATA_DIR / filename))

        return result
    finally:
        la.disconnect()


def load(filepath: str) -> CaptureResult:
    """
    加载已保存的采集数据

    Args:
        filepath: 文件路径

    Returns:
        CaptureResult 对象

    Example:
        result = load("data/capture_20240101_120000.json")
        print(result.channel_data[:, 0])  # 通道0的数据
    """
    return CaptureResult.load(filepath)


def list_captures() -> list:
    """列出所有已保存的采集文件"""
    ensure_data_dir()
    return sorted(DATA_DIR.glob("*.json"))


def analyze(result: CaptureResult, channels: list = None) -> Dict[str, Any]:
    """
    分析采集结果

    Args:
        result: CaptureResult 对象
        channels: 要分析的通道列表，None表示全部

    Returns:
        分析结果字典

    Example:
        result = capture()
        stats = analyze(result)
        print(stats['channel_stats'])
    """
    if channels is None:
        channels = list(range(result.config.channels))

    analysis = {
        'timestamp': result.timestamp,
        'sample_rate': result.actual_sample_rate,
        'total_samples': result.total_samples,
        'duration_ms': result.duration_ms,
        'channel_stats': {}
    }

    for ch in channels:
        ch_data = result.channel_data[:, ch]
        analysis['channel_stats'][f'CH{ch}'] = {
            'high_count': int(np.sum(ch_data)),
            'low_count': int(np.sum(~ch_data)),
            'high_ratio': float(np.mean(ch_data)),
            'toggle_count': int(np.sum(ch_data[:-1] != ch_data[1:])),
        }

    return analysis


def generate_filename(tag: str = "") -> str:
    """生成标准文件名"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if tag:
        return f"capture_{timestamp}_{tag}.json"
    return f"capture_{timestamp}.json"


# ==================== 快捷函数 ====================

def quick_capture(duration_ms: float = 10.0, sample_rate: int = 1_000_000) -> np.ndarray:
    """
    快速采集，直接返回通道数据

    Returns:
        shape为(num_samples, channels)的布尔数组
    """
    result = capture(sample_rate=sample_rate, duration_ms=duration_ms, save=False)
    return result.channel_data


def capture_loop(interval_ms: float = 1000, count: int = 5, **kwargs):
    """
    循环采集

    Args:
        interval_ms: 采集间隔 (ms)
        count: 采集次数
        **kwargs: 传递给capture()的参数
    """
    import time

    results = []
    for i in range(count):
        print(f"\n--- 第 {i+1}/{count} 次采集 ---")
        try:
            result = capture(tag=f"loop_{i+1}", **kwargs)
            results.append(result)
        except Exception as e:
            print(f"[ERROR] 第{i+1}次采集失败: {e}")

        if i < count - 1:
            time.sleep(interval_ms / 1000)

    return results


# ==================== 主函数 ====================

if __name__ == "__main__":
    print("=== nanoDLA 标准化接口测试 ===\n")

    # 测试连接
    la = NanoDLA()
    if la.connect():
        major, minor = la.get_fw_version()
        print(f"固件版本: v{major}.{minor}")
        print(f"芯片ID: 0x{la.get_chip_id():02X}")
        la.disconnect()

    # 测试采集
    print("\n--- 测试采集 ---")
    try:
        result = capture(
            sample_rate=1_000_000,
            duration_ms=10.0,
            tag="test"
        )
        print(f"采集完成: {result.total_samples} 个采样点")
        print(f"数据形状: {result.channel_data.shape}")

        # 分析
        stats = analyze(result)
        print(f"\n分析结果:")
        for ch, stat in stats['channel_stats'].items():
            print(f"  {ch}: 高电平占比 {stat['high_ratio']:.2%}")

    except Exception as e:
        print(f"采集失败: {e}")

    # 列出所有文件
    print(f"\n已保存的文件:")
    for f in list_captures():
        print(f"  {f.name}")
