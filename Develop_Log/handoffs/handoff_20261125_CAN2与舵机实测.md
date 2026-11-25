# Handoff · 2026-11-25 · CAN2/6020 实测与双舵机通信

## 一句话状态
🟢 **CAN2/6020 已打通**：板上运行包 `01d4b265`（CAN2 位定时已修为 1Mbps），交换 CAN_H/L 后 `rx` 以约 1.1k/s 连续增长、`tec=0 rec=0 err=0` 持续 35s；6020=ID4、反馈 0x208、控制 0x1FF。双舵机 ID002/001 ping/read 通过，映射仅 RAM。

## 资产与锚点
| 项 | 路径/值 | 状态 |
|---|---|---|
| 固件工程 | `D:/ZMS_Desktop/RobotWorkspace/taurus_h7_all/` | 不在 Git |
| 板上固件 hex | SHA256 `28ab70b1c6701cd96b7732171d59d972c307a9bf832f7274b54852589ba02c9c` | 已烧录、Verified OK、已运行 |
| 回退固件 | `_backup/20261125_can_cleanup/build/CtrBoard-H7_ALL.hex`，`79cf419c...` | 未改动 |
| 恢复点 | `_backup/20261125_can_cleanup/`，46 个文件（39 源码+原 hex/axf/map） | 可恢复 |
| CAN2 烧录日志 | `MDK-ARM/flash_can2_sp875.log` | Verified OK |
| 当前源码锚点 | fdcan `8b3639…`；can_comm `0f3d4f…`；bsp_can `cc3520…`；prot_rm_motor `97e20b…` | 以磁盘当前值为准 |
| 无线 DAP | FAED:4873，SN `8E74D081`，COM141@115200 | 已通 |
| 主仓 | `Robot-major-practice` master@e46f31d，57项工作树改动，29未推送，单 worktree | 未 commit/push |

## 已验结论（防重验）
| 结论 | 证据 | 等级 |
|---|---|---|
| 6020 绿灯每秒闪4次 = 驱动器 ID4；反馈 0x208、控制 0x1FF | 用户手册截图 + 用户确认灯语 | 事实 |
| 双舵机 UART1 通信正常，肩ID002/肘ID001 | ping rc=0；read pulse 1496/1491 | L3 通信 |
| 6020 在板上 FDCAN2，PB5/PB6 | 用户答复 + 原理图 CAN2_TX(PB06)/RX(PB05) | 接线事实 |
| CAN2 双向通信成立 | 交换 H/L 后 `rx 36508→77321`（~1.1k/s）、`tec=0 rec=0 err=0` 连续 35s | L4 闭环 |
| 根因1：CAN2 位定时 250kbps | 原 `60/6/1`=120MHz/(60×8)；改 `24/2/2`=1Mbps 后 hex `01d4b265` | 已证 |
| 根因2：现场 CAN_H/L 与丝印相反 | 未换线时 `tec=248 rx=0`；交换后立即 `tec=0 rx` 增长 | 已证 |
| 参考工程 yaw=6020 为 1Mbps | `_ref_gimbal_6020` FDCAN2 96MHz/8/8/3 | 事实 |
| GM6020 手册：1Mbps、ID4→0x208/0x1FF | 手册 p3/p6/p8 | 事实 |
| USB-CAN 当前未枚举 | Windows 仅见 CH343/FTDI/CH340/DAP，无 CAN 类设备 | 工具未接入 |

## Dead Ends
1. **不要把 prescaler 当采样点调**：`60/6/1` 实为 250kbps，不是"87.5% 采样点"。6020 必须 1Mbps，用 `24/2/2`。
2. **不要把 `green flash 4` 当成 0x208 灯码**：它表示 ID=4，0x208 是 ID 映射出的反馈 ID。
3. **本板 CAN2 现场 H/L 与 6020 线色相反**：红(A/H)/黑(B/L) 按丝印接会 `tec=248`；交换后即通，后续接线沿用交换后的极性。

## 没测清单
1. 6020 的 0x208 帧内编码器/转速/温度数值核对（当前固件只计数，不打印字段）。
2. 控制帧 0x1FF 实际输出转矩的有效性（本轮只发零电流）。
3. 两个舵机的同时动作/负载行为；本轮只读，无位置指令。
4. CAN1 上 4 个 2006 的真实反馈/总线错误（未驱动底盘）。

## 下一会话第一动作
CAN2/6020 已闭环，下一步按比赛主线推进：
1. 给 `[CAN2]` 或 status 增加 6020 反馈内容打印（编码器/转速/温度），确认 0x208 帧字段数值合理；仍不发非零转矩。
2. 依次验证 4 个 2006 在 CAN1 的反馈（0x201–0x204），确认底盘总线拓扑。
3. 之后回到 25 号下午计划：片内 Flash 示教、按键、WS2812。

接线沿用本轮结论：CAN2 需按"交换后"的 H/L 极性；6020 必须 24V 供电且拨码4=ON。

## 当前关键配置
- FDCAN1：PD0/PD1，Classic 1Mbit/s，4 条规则（IMU、0x201~0x204、0x208、0x205~0x207），全 FIFO0。
- FDCAN2：PB5/PB6，Classic 1Mbit/s，1 条规则 0x208→FIFO0；采样点 CI 60% 为默认，后改为 87.5%（120MHz/60，TS1=6，TS2=1），实测无改善；当前源码为 87.5%。
- 6020：`arm_yaw_motor`，ID 0x208，控制 0x1FF，`arm_task` 每100ms发零电流，仅保活。
- 双舵机：`servo map` RAM 当前 shoulder=2/elbow=1；`servo_test` 走 U10/COM141；本轮未持久化。

## 环境假设清单
| 假设 | 证据/状态 |
|---|---|
| 板子与 6020 的 24V 已就绪 | **未确认**（用户只勾选拨码4=ON） |
| CAN2 H/L 与 6020 对应、未反接 | **未确认** |
| 6020 拨码4=ON（终端电阻） | 用户已勾选 ✅ |
| CAN1↔CAN2 物理桥接 | 尚未执行；用户将在压缩后接线 |
| USB-CAN 小工具 | 当前 Windows 未枚举；不能用它做旁听 |
| 关联仓环境 | 已查 rc_remote 18:37、kmo_mc_rcv 18:08、cadence 16:09：均为跳频/桥/OTA 事项，与本板 CAN/舵机无直接共享环境变更 |
