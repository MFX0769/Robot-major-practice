# Handoff · 2026-11-26 · MKS CANable V2.0 监听链路部署

## 一句话状态

🟢 **资料包已交付，等待目标电脑执行**：本机只做离线准备；目标电脑的另一 AI 会话按本文件完成设备枚举 → 只读监听 → 可选 CAN hello → 判定。

## 已确认事实（别重复勘察）

- 硬件：MKS CANable V2.0，STM32G431C8T6 + TJA1051T/3，非隔离；资料来自厂商仓库固定提交 `b5c7a39f`。
- 协议：CANable2 slcan 固件固定提交 `b158aa7`。`S8`=1000 kbit/s，`M1`=silent，`O/C`=开/关，`V`=版本，`E`=错误寄存器。配置命令必须在 `O` 前；串口命令无 ACK/NACK。
- 本项目主线：CAN1、CAN2 均 **1 Mbit/s**。预期 CAN1 `0x201`–`0x204`（4×2006，约 1 kHz），CAN2 `0x208`（6020 yaw）、`0x1FF`（yaw 控制，10 Hz）。
- 接线：CANH、CANL、**GND 三根都要接**。厂商不隔离；同一 220 V 插排 ≠ 共地。终端电阻保持 OFF，除非确认总线上不足两个 120 Ω 终端。
- 之前本机 COM45 状态属于本机，**不是目标电脑事实**；目标电脑必须重新枚举。
- 官方 Windows 图形工具 cangaroo 官方便携 ZIP 已下载；源码归档与 pyserial 离线包已备齐。
- “A 检查”按用户澄清理解为“另一个 AI 会话继续执行”；未发现需单独安装的 A 工具。
- “AI 绘画”在本任务语境中同样按“另一个 AI 会话”理解；未把它解释为图像生成软件。

## 交付包结构

```text
MKS CANable V2.0_交付包_20261126/
├── README.md
├── SHA256SUMS
├── 00_源码与固件/
│   ├── 01_厂商手册/            16 页 PDF、原理图、尺寸图、README
│   ├── 02_固件/                candlelight.zip、slcan.zip
│   ├── 03_官方上位机/          cangaroo-win32-ccdcb64.zip
│   ├── 04_源码归档/            canable2-fw.tar.gz、cangaroo.tar.gz
│   ├── 05_社区案例/            openpilot/H753zi_AMR/1PengYuyan1 固定版本
│   └── 06_离线依赖/            pyserial-3.5 wheel
├── 01_脚本/
│   ├── canable_tool.py         部署/枚举/监听/受保护发送
│   └── deploy.ps1              Windows 一键部署脚本
└── 02_文档/
    ├── 调研报告.md
    ├── 主会话使用摘要.md
    └── handoff_20261126_CANable监听部署.md
```

## 目标电脑执行顺序

### 步骤 0：物理连接

1. 断电接线：CANable CANH → 板子 CANH，CANL → CANL，GND → GND。
2. 终端电阻 OFF；仅当确认总线缺终端才打开。
3. CANable USB 接入目标电脑；BOOT 跳帽断开。若进入 DFU，则按厂商手册短接 BOOT 进入烧录，再恢复正常。

### 步骤 1：部署与枚举

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\01_脚本\deploy.ps1
py -I .\01_脚本\canable_tool.py probe --write-report .\logs\probe.json
```

从报告中找新增的 USB CDC（通常 VID/PID `0483:5740`）；不要猜 COM 口。

### 步骤 2：只读监听

```powershell
py -I .\01_脚本\canable_tool.py monitor --port COMx --bitrate 1000 --silent --duration 10 --output-dir .\logs
```

验收：`0x201`–`0x204` 或 `0x208` 出现，计数 >0。`monitor` 从不发 CAN 帧。若 `probe --port COMx` 查询版本，仅发 `V` 命令，不发 CAN 帧。

### 步骤 3：可选 CAN hello

前提：轮子离地或电机动力断开。

```powershell
py -I .\01_脚本\canable_tool.py send-hello --port COMx --bitrate 1000 --confirm-send
```

同时在另一个 session 运行 `monitor` 验证是否收到 `ID=0x123`、数据 `01 02 03 04 05 06 07 08`。收到测试帧只证明 PC 能监听 MCU 发送，不证明电机执行。

### 步骤 4：停止与恢复

- `monitor` 退出时自动 `C` 关闭通道。
- 不修改主工程 CAN 位定时；不改 H/L 极性；一次只改一个变量。
- 现场日志保留在 `logs/`，含 CSV 统计和原始帧文本。

## 故障树（按顺序，每步只改一个变量）

| 现象 | 先查 |
|---|---|
| 枚举不到串口 | USB 数据线、设备管理器、是否 DFU、驱动 |
| 有串口但无帧 | H/L 反接、波特率、接错 CAN1/CAN2、GND、终端 |
| 有帧但 ID 不对 | 对端固件/ID 配置；不要改 CANable |
| 帧计数正常但电机不动 | 电机动力/使能/电流指令；监听只证明总线有帧 |
| CAN hello 不可见 | 是否同时监听、ID 是否一致、是否发错通道、是否被过滤 |

## 禁止事项

- 不在未确认固件和端口前发送任何 CAN 帧。
- 不把 CANable 的 5 V 输出接进 CAN 总线。
- 不把“收到 CAN hello”说成“电机执行成功”。
- 不在公开 Git 写私人 `D:/life_log/compass.md` 内容。
- 不修改主工程 CAN 位定时或已验证的 H/L 交换结论。

## 后续工作建议

1. **P1** 目标电脑执行步骤 0–2，产出 `probe.json` + 10 秒监听 CSV；预计 20–30 min；依赖实物接线；产出监听链路判定。
2. **P1** 若主会话需要验证 MCU 发送，执行步骤 3 CAN hello；预计 10 min；依赖轮子离地/动力断开；产出 PC 可见发送帧证据。
3. **P2** 如需图形界面，解包官方便携 cangaroo，slcan 模式 1000 kbit/s；预计 10 min；依赖目标电脑权限；产出 GUI 截图或人工确认。

**建议先做第 1 项**：它零发送、最低风险，能一次性回答“能不能监听自己的发送”。
