---
name: emk850
description: EMK850+ 低功耗分析仪（COM15）纯 Python 直控电流监测——替代官方 GUI 持口，整夜电流日志/堵转告警/功耗基线。当用户说"低功耗分析仪/EMK850/电流监测/功耗电流/堵转电流/台架电流/钳表读数/量一下功耗"，或要排查"GUI 找不到设备/COM 抢占/频谱仪和分析仪冲突"，或夜间压测要盯电流异常时使用。覆盖：启动监测、协议速查（0x33/64B 帧/cmd132 数据流/校准自读/电流换算式）、与 ZS406 频谱仪的 ST-CDC 共存纪律、GUI 复原。
---

# /emk850 — EMK850+ 低功耗分析仪直控

台架电流监测的正道：**不开 GUI，直接 python 读串口**。GUI 会抢 ST-CDC 口（知识卡正本=develop_log/knowledge/EMK850上位机COM抢占与ST-CDC共存_20260930.md），直控脚本零冲突、可整夜挂、带告警。

## 快速上手

```bash
python tools/emk850_mon.py                      # 默认 COM15@115200，1Hz 落 temp/emk850/current.csv，无限跑
python tools/emk850_mon.py --duration 3600      # 有界采集 1h 后干净退出
python tools/emk850_mon.py --warn-ma 200        # 堵转告警线（默认 200mA）
type temp\emk850\last.json                      # 最新一条 + 今日 min/max/mean
```

台架基线（2026-10-02 实测，压力测试台主机=传感器群+GD32+BK3633 无睡眠）：**~32.7mA 均值 / 24.66V 轨**；电流突增数百 mA=电机堵转（该台架按压即堵转，平常测试不触发）。

## 文件地图

| 文件 | 作用 |
|---|---|
| `tools/emk850_mon.py` | 唯一脚本（握手/校准自读/数据流解析/CSV/告警/断线重连） |
| `temp/emk850/current.csv` | 逐窗记录：timestamp,i_mean/min/max,v_mean,samples,active_ch,warn |
| `temp/emk850/last.json` | 最新快照+当日统计（原子写） |
| 知识卡（正本） | `develop_log/knowledge/EMK850上位机COM抢占与ST-CDC共存_20260930.md`（GUI 抢 COM 根因+拓扑方案） |

## 协议要点（2026-10-02 反编译+实测定案，已固化进脚本注释）

- 帧=64B 定长：`magic 0x33 | cmd | len | zero | payload(60)`；设备=PA-EMK850+-KL（STM32 CDC，波特率名义值）。
- 握手 `REQ_VERSION(16)→cmd17`；**校准自读** `REQ_READ_CONFIG(50)→cmd66+64`（Vref/om1..om5/p,o 校准对，不硬编码）+ `REQ_READ_ALARMCONFIG(104)→cmd102`（max_1=PGA 增益, avg_1=ADC 压扩）。
- 数据流 `REQ_START(22)→cmd132 @~10kHz`：`payload[0:9]=通道位图(3bit/样本), [9:59]=25×u16（[0]电压 ADC,[1:25]电流 ADC）, [59]=模式`。
- 换算（GUI HandleSampleHighSpeed 同源）：`I_mA=(raw×Vref/65536/gain/om×1000+0.0011/om)×(1+p)+o`；`V=(vraw×Vref/65535×adc_mag)×(1+pv)+ov`。
- **REQ_STOP(24) 永不发送**：退出保持仪器采集态（台架不被扰动），下次连接自动续流。

## 共存纪律（ST-CDC 三件套）

| 设备 | 口 | 规则 |
|---|---|---|
| EMK850+ 分析仪 | **COM15**（序列 1E0047） | 本脚本专用；GUI 想用=桌面快捷方式"低功耗分析仪110047"（先停本脚本） |
| uA电源 850L+ 第二台 | COM93 | 同协议可测（0x33 探针已验证）；**不是频谱仪**（2026-10-02 仲裁） |
| ZS406 频谱仪 | COM87 | 与 EMK **GUI** 互斥（GUI 抓第一个 ST-CDC）；**两个 python 直控不同口=无冲突**，可同时整夜挂 |

## 排错速查

| 现象 | 处置 |
|---|---|
| PermissionError COM15 | GUI 还开着（窗口"低功耗分析仪-1E0047"）→ `(Get-Process -Id <pid>).CloseMainWindow()` 优雅关，勿 taskkill /F |
| 握手失败/无 cmd132 | 先 `REQ_VERSION` 探活；设备可能在别状态 → 重跑脚本（ensure_stream 会补 REQ_START） |
| 数值离谱 | 看 config 日志行 Vref/om/gain 是否读到（全零=校准没读到，查线） |
| 端口号漂了 | COM 非恒久映射，`python -c "from serial.tools import list_ports; [print(p.device,p.description) for p in list_ports.comports()]"` 实测 |

## 维护规则

- 改脚本后跑 `--duration 30` 冒烟（30s 出 30 行 CSV 且无 error 即过）。
- 电流结论落 daily 时必须带：设备版本串、均值、窗长、warn 次数。
- 本 skill 与 zs406 skill 的共存矩阵如有变化（换口/加设备），**当场改本表并同步知识卡**。
