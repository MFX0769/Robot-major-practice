# nanoDLA 自动化工具

标准化的 nanoDLA 逻辑分析仪数据采集、保存、分析接口。

## 快速使用

```python
from la_reader import capture, load, analyze

# 一键采集并保存
result = capture(sample_rate=1_000_000, duration_ms=100)

# 加载已保存的数据
result = load("data/capture_20240101_120000.json")

# 分析数据
stats = analyze(result)
print(stats['channel_stats'])
```

## 标准接口

| 函数 | 说明 |
|------|------|
| `capture()` | 采集并保存到文件 |
| `load()` | 从文件加载数据 |
| `analyze()` | 分析通道数据 |
| `list_captures()` | 列出所有采集文件 |
| `quick_capture()` | 快速采集，返回numpy数组 |
| `capture_loop()` | 循环多次采集 |

## 数据格式

采集文件保存为JSON格式，包含：
- `timestamp`: 采集时间
- `config`: 采集配置
- `raw_data`: 原始数据
- `actual_sample_rate`: 实际采样率
- `total_samples`: 总采样点数

## 文件存储

所有采集数据保存在 `data/` 目录下，文件名格式：
```
capture_YYYYMMDD_HHMMSS[_tag].json
```

## 依赖

```bash
pip install -r requirements.txt
```
