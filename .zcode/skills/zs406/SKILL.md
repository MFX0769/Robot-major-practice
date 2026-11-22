# ZS-406 频谱仪操作手册（tinySA ULTRA Plus 贴牌）

> 用途：2.4G 跳频实验的频域观测——频点校验、时隙占用目视、环境干扰普查。
> 来源：2026-09-30 调研（tinysa.org wiki 等，**未接实机验证的条目已标注 ⚠**）。
> 实机第一次使用时：逐条验证 ⚠ 项，验证通过的删掉 ⚠，不通过的改正本文件。

## 1. 这台仪器是什么、不是什么

- 是：手持频谱仪，100kHz–5.4GHz，两个 SMA 口（**RF 输入**接天线/馈线；**CAL 输出**是它自己的校准信号源，不是触发输入）+ Type-C + 触摸屏。
- 不是：射频示波器。零跨度模式下约 **0.3ms 才出一个测量点**（最宽 RBW 600kHz 时），**看不到 100µs 级的瞬态**，3ms 的事件也只有约 10 个点。µs 级时序问题用嗅探器/逻辑分析仪，不要用它。
- 没有外部触发，只有射频电平触发（trigger normal/single + 电平门限）。

## 2. 连电脑

1. Type-C 线接电脑，枚举为 USB 虚拟串口（免驱）。实测 CDC 115200 可用（2026-09-30）。
2. 找口：插拔前后各查一次注册表（Win32_SerialPort 漏 VCP）：
   `reg query "HKLM\HARDWARE\DEVICEMAP\SERIALCOMM"`
   ⚠ 坑（09-30 实测）：它挂 **ST VCP 驱动**，友好名显示 "STMicroelectronics Virtual COM Port"——和 ST-Link 同名同 VID(0x0483)，**按驱动名排除会把仪器本体误判成调试器**；本台=COM87，与另一块真 ST-Link(COM15) 并存时逐个 `version\r\n` 探活区分。
3. 探活：发 `help\r\n`，应回命令列表。实测固件=**tinySA4_v1.4-217-gc5dd31f / HW V0.4.6（官方线，非贴牌分叉）**，scan/scanraw/sweep/trigger/capture/help 全部可用（2026-09-30）。
   注意：`scan {a} {b} {n} 3` 回行是**三列**（freq level 0.0），level 为科学计数法 dBm，解析按前两列取。

## 3. 上位机三选一

| 工具 | 干什么 | 来源 |
|---|---|---|
| tinyRemote.exe | 屏幕镜像+鼠标遥控（看图用） | http://dfu.tinydevices.org/tinySA4/Remote/tinyRemote.exe |
| tsapython | Python 编程控制（`pip install tsapython`，Ultra 每扫 ≤450 点） | PyPI |
| 裸 pyserial | 直接发文本命令（最简单，推荐起步） | — |

⚠ NanoVNA-Saver 不兼容 tinySA；PyPI 上没有 pytinysa。

## 4. 常用命令速查（\r\n 结尾；正本=tinysa.org/wiki USBInterface 页）

```
scan {start} {stop} [points] [outmask]   # 扫频取数：outmask 3=频率+实测 dBm
scanraw {start} {stop} [points]          # 二进制版：dBm = raw/32 - 174（Ultra 专用常数）
sweep cw {freq}                          # 零跨度：钉在一个频率上看时间轴
sweep 0                                  # 扫描时间=自动最快
rbw 600k                                 # 最宽分辨率带宽=最快出点
trigger auto|normal|single               # normal+电平=来信号才刷新
trigger -60                              # 触发电平 dBm
data 0|1|2                               # 0=临时 1=存储 2=测量值
capture                                  # 屏幕位图 dump（2 字节/像素）⚠ Ultra 屏 480x320 未实测
pause / resume
```

pyserial 最小可用：

```python
import serial
sa = serial.Serial('COMx', 115200, timeout=1)
sa.write(b'scan 2400M 2483M 450 3\r')     # 全 2.4G 扫一遍
print(sa.read(65536).decode(errors='ignore'))
```

## 5. 本项目实验配方

### 配方 A：频点校验（跳频表核对）
- 条件：跳频固件在跑，发射持续。
- 做法：`scan 2400M 2483M 450 3` 存 CSV，提取有信号的频点，与 39 值表（exp 仓 rf_hopper.c:27 的 s_hop_table，BK 道号 → 频率 = 2402 + 道号 MHz，**⚠ 此换算式需用定频包实测一次核对**）比对。
- 判读：39 个频点全部出现且位置吻合=映射正确；缺道/错位=寄存器映射问题。
- 注意：只证明"发在正确的频率上"，不解释丢包（2026-09-30 已有 61048 帧按道计数证明信道值均匀，见 daily 15:14 条）。

### 配方 B：零跨度监听变频节奏（09-30 实测可行）
- 做法：`sweep cw 2430M` + `rbw 600k` + `sweep 0` + `resume`，测试臂跑起来后 `data 2` 取迹。
- 实测样例（28:32 测试臂 period=2ms）：450 点/迹（0.3ms/点，~135ms 窗），A 道突发 119-148 次/s、单突发宽 0.7-0.8ms、占空比 ~10%——**节奏/速率/占空比可测**。
- 两个解析坑：①零跨度 `data 2` 回**单列** dBm（扫频模式才是三列）；②**第一个值 ~+8dBm 是状态残留不是电平**，丢弃。
- ⚠ 时基陷阱（09-30 用户指正）：`data 2` = **最近一次完整扫描的快照，非实时流**——"扫一下绘一次"。换算节奏前必须先查实际扫描时间（`status`/`sweep time`），要固定时基就用 `sweeptime` 显式设定并读回确认；status 显示 Paused 先 `resume`，否则按想当然的时基换算会得出错误速率。
- 仍不能：单突发内部结构（0.7ms 突发只有 2-3 个点），µs 级换频瞬态还是 LA 的活。
- 主动输出：`caloutput` = 固定晶振档 **30/15/10/4/3/2/1 MHz**（非任意频率，上不了 2.4G）；`output on|off|normal|mixer` = **本振泄漏输出**（当前调谐频率可到 2.4G，低电平未验证，RF 口角色反转）——可作 E-E 接收侧实验的已知 CW 源候选，用前实测电平。

### 配方 C：环境干扰普查
- 做法：配方 A 的扫频曲线在无发射时采一遍=底噪；有发射时再采一遍。对照找常驻干扰（WiFi 核）与游走干扰。
- 与 fcc_dtm 双板丢包测试互补：频谱仪说"哪里有人"，丢包测试说"哪里疼"。
- 外场用法见 skill fcc-bench 的采样律。

## 6. 安全与纪律

- 天线近场接收即可，**禁止 RF 口直连任何发射源或天线直贴设备**（输入功率上限，烧前端）。
- 不占用台架既有串口（桥 COM43/遥控 COM74 等）。
- 任何数据/结论当场 append 到 develop_log/daily_review.md（实测时间戳）。
- 仪器分辨率先声明再下结论：ms 级以下的事件一律说"看不到"，不许拿包络猜 µs。

## 7. 已知不确定项清单（验证后更新本节）

1. ~~CDC 波特率~~ 已验：115200 可用（09-30）
2. ⚠ capture 在 Ultra 上的确切图像尺寸（480×320？）
3. ~~贴牌固件兼容性~~ 已验：官方 tinySA4 v1.4-217，命令集全（09-30）
4. ~~道号→频率换算~~ 已验：ch28→2430 / ch70→2470 两点实测吻合 2402+ch（09-30，配方A）
5. 触发功能齐不齐（edge/pre-post 触发是否有）——wiki 说有，实机确认

## 8. 维护

- 本 skill 依据=2026-09-30 调研（来源：tinysa.org wiki USBInterface/TRIGGER/FAQ/PCSW、groups.io Erik zero-span 建议帖）；正本调研结论另存 develop_log/handoffs/handoff_20260930_下午_频谱仪部署专项.md。
- 实机验证过的条目改写进正文并去掉 ⚠；发现与文档不符的命令行为直接改正本节并记 daily。
