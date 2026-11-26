---
name: kingst-la
description: KingstVIS（Kingst LA1010 逻辑分析仪）上位机全自动驱动——采集→导出原始跳变 CSV→离线 UART 解码，替代手工点 UI 和 UI 解析器导出。当用户说"LA 自动采集/逻辑分析仪导出 CSV/抓一帧串口数据/解析器导出/解码导出/la 自动化"，或要重复执行采集-导出流水线、离线解码 UART 遥测，或排查"KingstVIS 点不动/菜单弹不出/导出没反应/对话框确定按不了"时使用。覆盖：标定坐标、合成输入免疫地图、盲点协议、原生对话框消息通道、离线解码、重启代价。
---

# /kingst-la — KingstVIS(LA1010) 全自动采集-导出-解码

三段管线，全部免人工：

```
kingst_auto_capture.ps1  → 原始跳变 CSV（点开始→等自动停→菜单盲点→原生对话框写路径→保存）
la_decode_offline.py     → 原始 CSV 离线 UART 解码（8N1/1M，TX 三路），替代 UI 解析器导出
```

## 快速上手

```powershell
# 前提: KingstVIS 已连接设备(LA1010 已连接)、采集配置能自动停(如 50MSa@20MHz=2.5s)、窗口未被挪动(脚本自动钉 10,10 1680x930)
powershell -ExecutionPolicy Bypass -File temp\kingst_auto_capture.ps1                               # → temp\la_capture.csv
powershell -ExecutionPolicy Bypass -File temp\kingst_auto_capture.ps1 -OutPath D:\x\a.csv -CaptureWaitMs 5000
python temp\la_decode_offline.py temp\la_capture.csv            # → la_capture_decoded.txt (RC/CAD/BRI 三路)
```
注意：采集配置必须能在 CaptureWaitMs 内自动停止，否则导出时无完整数据；**采样率/深度两个下拉不接受合成输入**，改配置用真鼠标（两下下拉）。

## 文件地图

| 文件 | 作用 |
|---|---|
| `temp/kingst_auto_capture.ps1` | 采集+导出主脚本（参数 OutPath/CaptureWaitMs，输出 PASS/FAIL） |
| `temp/la_decode_offline.py`    | 原始 CSV → UART 解码文本（参数 csv/baud/out） |
| `temp/kx.ps1`                  | 坐标点击/右键/悬停/截屏/钉窗 辅助（Action 参数） |
| `temp/kx_menu.ps1`             | 全量窗口枚举（句柄/rect/标题/可见性） |
| `D:\Program Files\KingstVIS\Resource\ug-cn.pdf` | 官方手册 v3.5（导出数据=§10 p24，解析器=§四.6 p17） |
| `D:\Program Files\KingstVIS\Language\zh-CN.xml` | 全部 UI 字符串（查菜单项/按钮文字） |

## 已标定流程链（窗口钉 10,10 1680x930 时有效）

| 步骤 | 操作 | 验证 |
|---|---|---|
| 开始采集 | 左键 (609,95) | 预期采样时间内自动停 |
| 主菜单 | 左键 **(1650,50)**（隐形按钮 btnMain） | 菜单 popup vis 0→1（rect≈176x248） |
| 导出数据行 | 左键 (menu.left+88, **menu.top+87**) | 新 popup ≈145x67（行偏移随菜单顶浮动，87 是实测） |
| 导出设置确认 | 对话框前置后 **Shift+Tab×2 + Enter** | 另存为对话框出现（854x480, 标题=另存为） |
| 写路径+保存 | 枚举子控件：`Edit` 类 WM_SETTEXT 路径 → `保存(&S)` 按钮 BM_CLICK | 文件存在+大小稳定+表头含 Time |
| 离线解码 | `python la_decode_offline.py <csv>` | *_decoded.txt 出现 `[SBt]`/`[CSl]` 等真实遥测 |

## 铁律（每条都是实测学费）

1. **合成输入免疫地图**：QMenu popup（主菜单/子菜单）盲点可交互 ✓；原生对话框走子控件消息（WM_SETTEXT/BM_CLICK）✓；**原位控件全部免疫**——对话框确认按钮、采样率/深度下拉（含 F4/Alt+Down）、通道齿轮、右侧面板按钮，合成点击/PostMessage/键盘全无效。绕行=键盘焦点导航或子控件消息，别再试坐标点击。
2. **隐形渲染**：btnMain 白底白字、菜单/列表 popup 渲染透明但可交互（class `Qt5QWindowPopupDropShadowSaveBits`）。找按钮靠：exe 内 QSS 字符串（`MainMenu>QPushButton#btnMain`）+ WindowFromPoint + vis 0→1 跃迁检测；禁用视觉扫像素（白底白字扫不到）。
3. **每步证据闭环**：每步操作后用窗口枚举（可见性/rect/标题）或文件系统证据确认，失败 Esc×2 重试一次再停。禁止无证据声称成功（2026-09-28 幻觉事故教训，daily:5245）。
4. **PID 陷阱**：枚举窗口必须用**当前**进程 PID——重启后 PID 变，旧 PID 过滤=全部假阴性（本次踩过：菜单明明开着却报空）。
5. **GBK/BOM**：.ps1 含中文必须 UTF-8 **带 BOM**（PowerShell 5.1 无 BOM 按 GBK 读，中文字面量匹配全部静默失效）。
6. **popup 栈会卡死**：对菜单 popup 做 ShowWindow/PostMessage 实验后，btnMain 可能永久失灵（Qt popup 簿记混乱），Esc/WM_CLOSE 救不回。恢复=重启应用。
7. **重启代价**：采集配置（采样率/深度/解码器/通道名）只在进程内存，**不持久化**（vis.config 是 09-23 的旧文件、无注册表键）。重启=回退到上次保存态，实验配置需真鼠标重设。重启前先问用户/确认实验边界。
8. **SetForegroundWindow** 后台进程调用会被系统拒绝——用 AttachThreadInput 技巧（kx.ps1 的 ForceForeground），且发键前校验 `GetForegroundWindow()==目标`。
9. 导出设置对话框默认勾选=当前启用通道+全部时间，一般直接确认即可；原生对话框左下"保存类型"已是 CSV。

## 离线解码替代 UI 解析器导出

UI 解析器面板（齿轮→导出数据）在 1680x930 布局下不可达且控件免疫；等价物=对原始 CSV 离线解码（la_decode_offline.py，通道映射读 CSV 表头自适应，TX 列索引在 main() 改）。实测 2.5s 采集解出 RC_TX 1027B / CAD_TX 1003B / BRI_TX 3016B 真实遥测（[SBt]/[CSl]/[GB]）。

## USB 直控（已建成 2026-10-03，优先用这条，GUI 管线作后备）

**`tools/la_reverse/la_direct.py` = 免 GUI 采集正本**（pyusb + libusb-package）：

```powershell
python tools\la_reverse\la_direct.py probe                       # 探测(枚举/EEPROM/FPGA态)
python tools\la_reverse\la_direct.py capture --out raw.bin --srate 10e6 --depth 100e6 --channels 6 --force-upload
python tools\la_reverse\la_compare.py <vis.csv> raw.bin          # 与 KingstVIS 导出对拍
```

- 协议正本=`develop_log/reference/la_reverse/la1010_protocol.md`（端点/命令/FPGA 寄存器/数据格式/6 坑）；信源=sigrok 官方 kingst-la2016 驱动 + AlexUg fork 双源交叉验证
- **前置=必须退出 KingstVIS**（运行中独占 USB 句柄，libusb_open 直接 Access denied）
- **KingstVIS 退出会停 FPGA**（REG_RUN=0x0000、采集 0 数据）→ 用 `--force-upload` 强制重传 bitstream（fw_out/ 自 Linux 版 KingstVIS 提取，全系列 18 文件）
- 数据格式：u16 LE 交织流，第 k 个 u16=第 k 启用通道的 16 采样点（LSB 先）；D0-D5 时输出 bit0-5=D0-D5，1Mbaud UART 解码已实测出 `[SBt]` 遥测
- reader-first 铁律：bulk IN 先挂起再发 REG_RUN=3+BULK_START（FIFO 0.27ms 满），la_direct.py 已内置
- GUI 管线（下文坐标链）仅在 pyusb 不可用时后备

## 探针脉冲簇解析（切换实验/打点脚捕获 · 2026-10-12 增补）

**正本=** `develop_log/sop/LA数据解析SOP_20261012.md`（数据格式/双码表/配对公式/11 项体检门/14 坑全量）；**解析器=** `tools/la_swt_decode.py`（复用 la_analyze_v2 原语+双码表+事件对配对，输出五件：uart_lines.txt / events.csv / report.txt）。当 CSV 里除串口列还有**打点脚列**（P0.1，n 个脉冲一簇=事件码 n）时用这条，不要拿 la_decode_offline 硬解。

```powershell
# 桥打点脚捕获（通道名按 CSV 表头改）
python tools\la_swt_decode.py temp\<csv> --uart-ch BRIDGE_TX --probe-ch BRIDGE_RX --codetable bridge
# 遥控打点脚捕获（码表不同必须切）
python tools\la_swt_decode.py temp\<csv> --probe-ch RC_P01 --codetable remote --burst-gap-us 2
```

速查（细节查 SOP）：
- **宽度过滤 0.3µs 强制开**（串口串扰假脉冲可占 85%）；**簇间隔阈值桥=6µs、遥控=2µs**（20µs 会并簇把码吃掉）
- 码表：桥 1-7 旧探针+8-14 切换事件（8/9=释放起止、10/11=取回起止、12/13=广播停起止、14=信标 TX_DS），正本=kmo `app/api/swtime_instr.h`；遥控 1-8 另一套且 10+格内相位——**跨设备必须切表，混表=全错**
- 验收基准（2026-10-12_10-53-10.csv 实测）：UART 帧错=0、[HB]数=时长秒数、码 6 速率≈100/s（10ms 格）、码 4≈10/s（双信标/块）；不吻合先查接线/采样率再查码表
- 已知挂账：码 14 有"在飞闸"门可能零出现（rf_hopper.c:803）——解析器报 n=0 不代表没发信标，先核固件闸
- 红线：PC/USB 时间不当事件时间；无宽度过滤不出簇结论；N/A 不填 0；阈值不中途调
