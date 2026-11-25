# Daily Review

> 开发流水日志正本。按时间追加，禁止覆盖。

---

## 2026-11-22 ~ 11-23 · 达妙 H723 板 bring-up + 全量基座迁移 + U7 寻线串口攻坚

**执行者标注**：【用户】【AI】【AI执行/用户判读】

---

### 2026-11-22 15:40 — ⏱️ 工作区搬迁落地 + 硬件枚举

【用户】4 个仓库 + 临时脚本目录全在 `D:\ZMS_Desktop\RobotWorkspace\`（主工程 5385 文件 / dm-mc02 4972 / damiao 46 / 25-aiming-cart 608），git 全 clean。

【AI】硬件枚举（`Get-PnpDevice`）：
- 新购调试器 = **Horco CMSIS-DAP v2**，VID `FAED:4873`，SN `90bbd2c8`，VCP **COM134**，附 U 盘 F:（固件 V208）
- 老调试器 ×2 = `EF1A:74E5` → COM34 / COM43
- 其他 CMSIS-DAP ×2 = `0D28:0204`
- 板载 USB CDC（BMI088 输出）= VID `0483:5740`，SN `395F396C3135` → **COM137**

---

### 2026-11-22 16:00 — ⏱️ 【失败返工】U 盘拖拽烧录是误判

【AI】把 hex 拖进 DAP 的 F: 盘，以为这是烧录方式。

【排查/判据】读塔克官方手册 V3.0.1 §2.2.2：

> F: 盘内 `CONF.TXT` 仅配置无线主从模式与地址（`mode=master/slave`、`addr`、`rate=2M`），**不是固件烧录接口**。

**判决**：❌ 推翻。F: 只是配置盘，芯片未被写入。已删除误拷的 hex。
**教训**：不熟悉的探针先读手册，别凭 U 盘存在推断功能。

---

### 2026-11-22 16:20 — ⏱️ OpenOCD 直连打通（正确烧录路径）

【AI】配置 `_horco_probe.cfg`：
```
adapter serial 90bbd2c8      # 关键：不指定会误认旁边的 PWLINK2 (0D28:0204)
source [find interface/cmsis-dap.cfg]
transport select swd
adapter speed 2000
reset_config none
source [find target/stm32h7x.cfg]
```
**识别结果**：`SWD DPIDR 0x6ba02477` / `Cortex-M7 r1p2` / flash 探测 `STM32H72x/73x` **1024KB 单 bank**。

**烧录命令**（唯一可靠形式）：
```bash
openocd -f _horco_probe.cfg -c "init; halt; program <hex> verify; exit"
```
- 必须带 `halt`，**不能带 `reset`**（DAP 无 nRESET，reset 必卡死）
- 偶发 `Unable to reset target`，重试 1-5 次必成

**判决**：✅ 闭环。烧录 Verified OK。

---

### 2026-11-22 16:35 — ⏱️ 【失败返工】误判 USART6 为调试口

【AI】按 MCU 型号选了 USART6（PC6/PC7）做调试串口，改 vofa 输出 + 编译烧录。

【用户】"我在它上面没有看到串口6的接口啊。它只有串口10、串口11、串口3、串口2。"

**判据**：管脚标注图（`CtrBoard-H7_V1.0-管脚标注图240127.pdf`）确认板子引出 USART1/2/3/UART7/UART10，**无 USART6**。且原厂 `vofa.c` 的 `vofa_transmit()` 本就是 `CDC_Transmit_HS`（板载 USB CDC）。

**判决**：❌ 推翻。`git checkout HEAD --` 回滚 4 个文件。
**教训**：**先读厂商例程的 printf/vofa 重定向确认输出口**，不要凭 MCU 型号假设板子引出。

---

### 2026-11-22 16:55 — ⏱️ ✅ BMI088 验收通过

【AI】原厂 hex + 板载 USB CDC，COM137 @115200 解析 VOFA 帧（帧尾 `00 00 80 7f`，每帧 16 字节 = 3×float + 帧尾）：

| 指标 | 实测 |
|---|---|
| 帧率 | **998.2 fps**（5s 内 3072 帧）|
| yaw | +0.1432 rad（8.2°），3s 漂移 **−0.0006 rad（−0.034°）** |
| pitch / roll | −0.019 / −0.020 rad，稳定 |
| 温控 | TIM3 CH4 PWM 工作 |

**判决**：✅ 闭环。BMI088 + SPI2 + Mahony + 温控全通。

---

### 2026-11-22 17:30 — ⏱️ ✅ CAN1↔CAN2 互收（根因=24V 供电）

【AI】前几轮 CAN ISR 恒 0。排查链：
1. 波特率核对：FDCAN 时钟 120MHz / prescaler 24 / (1+3+3) = **1Mbps** ✅ 正确
2. 滤波器：原厂 `FilterID1=FilterID2=0`（RANGE 0~0，只收 ID 0）→ 改 `0x07FF` 全开
3. 收发调用、中断使能：均正常
4. SWD 读寄存器：DAP 无 nRESET，`halt` 必卡 → 无法验证

【用户】接上 **24V 电源**后，CAN 端子旁的灯立刻变亮（40~50mA 板载功耗正常）。

**根因**：手册 §电源树明确"**使用 CAN 总线需要 USB 供电或者 24V 电源供电（CAN 芯片采用 5V 供电）**"。仅 USB 5V 带不动收发器。

【AI】24V 就绪后**复位**（先启动后接 24V 会让 FDCAN 进 bus-off）：
```
[CAN] isr1=7  isr2=7  isr3=0
[CAN] isr1=12 isr2=12 isr3=0     ← CAN1/CAN2 rx 计数 1:1 同步，每秒 +1
```
CAN3 未接线=0，反证计数真实。

**判决**：✅ 闭环。**教训：达妙板 CAN 必须 24V + 之后复位**。

---

### 2026-11-22 17:40 — ⏱️ 【用户纠偏 · 方向转折】原厂跑通 ≠ 移植完成

【用户】"你这个跑通的只是厂家的代码，没有移植到我们自己的代码上来跑通啊" + "把 BMI088 数据读取的全量优化...至少要保证 1000Hz 稳定的结算之后的陀螺仪数据...以及 CAN 在我们本身的算法上面的优化...全量的移植"

【AI】重新定义任务：任务1a（原厂验证）✅ / 任务1b（移植进主工程）❌ 未做。

---

### 2026-11-22 18:15 — ⏱️ 【失败返工】主工程 rm_main 烧录后 USB 不枚举

【AI】把 BMI088 BSP（SPI2 + 驱动 + Mahony + 1kHz 任务 + 队列 + imu_data_t）移植进 `rm_main`，编译 0 Error，但烧录后**COM137 恒 Unknown**。

【排查链】（走错的假设都留）：
1. 假设：IMU 任务抢占 CPU → `#if 0` 关掉全部 IMU 代码 + 注释 MX_SPI2_Init → **仍不枚举** ⇒ 与 IMU 代码无关
2. 假设：HSE 晶振错（主工程 16MHz，新板 24MHz）→ 改 `HSE_VALUE` 宏 + `SystemClock_Config` 换原厂 → 仍不枚举
3. 假设：PLL2 VCO 超范围（M=1 @24MHz = 24MHz 超 RANGE_3）→ 改 M=2 → 仍不枚举
4. 假设：PE2/PE3 被 8IR GPIO 占用 → 改回 USART10 → 仍不枚举
5. 假设：自定义 sct（DMA 优化版，DTCM/AXI 分散）→ **未验证**

**判决**：❌ 未闭环。系统对比发现主工程与原厂差异：HSE、PLL2、**自定义 sct**、初始化序列完全不同。

---

### 2026-11-22 19:00 — ⏱️ ✅ 【重大转折】全量迁移到原厂基座成功

【用户】"那就只是把我们主工程的业务代码做一个大迁移...建立一个新的基于官方例程改出来的框架"

【AI】新建 `D:\ZMS_Desktop\RobotWorkspace\taurus_h7_all\`：
- **基座** = 原厂 CtrBoard-H7_ALL（已验证能跑）
- **业务** = 主工程 1.2 万行（Application 21 / Protocol 32 / Comm 7 / UI 6 / Debug 2 / Math 20 文件）+ 旧 Bsp 4 文件
- uvprojx 合并：原厂构建骨架 + 业务源文件组

**15 轮编译迭代修复清单**（全部编译器驱动）：
1. `ARM_MATH_CM7` 宏缺失（23 错）→ 加 Define
2. `dbg_print` 走 huart6（板子未引出）→ 重定向
3. `VISION_HUART` huart6→huart10
4. `iwdg.h` 缺失 → 建占位头文件
5. `NEW_RC_HUART` huart9→huart7
6. include path 补 `Tool/debug`
7. **5 处链接冲突**：`fputc`(usart.c 禁用) / `HAL_UART_RxCpltCallback`(fun_test 改名) / `err`→`imu_err` / `abs_limit`+`ramp_calc`(user_lib.c 注释)
8. 18 个未定义符号 → Bsp/taurus 加入编译组
9. **XML 损坏事故**：Python 转义把 `\t`→TAB、`\b`→0x08，uvprojx 4 条 FilePath 损坏 → 按精确字节序列修复
10. 旧 can_bsp.o 残留冲突 → 移除 can_bsp.c + fun_test_task.c
11. `bsp_pwm.c` 重复定义 htim3 → 引用原厂 tim.h
12. `arm_task(void)` 签名 → `(void const *argu)`

**验收**：
| 项 | 结果 |
|---|---|
| 编译 | **0 Error / 0 Warning**，Code=99908 |
| USB CDC | COM137 枚举 OK |
| BMI088 | **1040 fps**，姿态正常 |
| 业务全链路 | `[SYS] imu yaw=-2.5 pit=4.7 cnt=80823 mode=0`（11 个任务全跑） |

**判决**：✅ 闭环（迁移完成，业务跑通）。

---

### 2026-11-22 19:35 — ⏱️ ✅ USB 单点发送队列架构（用户架构要求）

【用户】"所有USB的东西都应该用一个USB通讯任务来去封装...所有调用者都要都要走队列啊"

【AI】实现 `USB_tx_queue`（16×`usb_tx_msg_t{len,data[120]}`）：
- `usb_tx_send()` 非阻塞入队（满则丢帧，保 1kHz vofa 不断流）
- `Task_USB_Transmit` 是**全工程唯一** `CDC_Transmit_HS` 调用者
- vofa / dbg_print 全部改走队列
- 中途试过的互斥锁方案**已回滚删除**

**判决**：✅ 闭环（架构落地）。

---

### 2026-11-22 20:30 — ⏱️ DAP 调试通道确认（U10 @2M）

【AI】多波特率扫描 COM134（Horco DAP VCP）：

| 波特率 | 结果 |
|---|---|
| 115200 | 乱码 |
| 921600 | 乱码 |
| **2000000** | ✅ **完全可读** |

**原始数据**：
```
[SYS] imu yaw=0.0 pit=2.2 cnt=1057 mode=0
[LINE] st=0 pos=0 rx=0 a1..a4=0 0 0 0 d=0000
```
**结论**：DAP 的 VCP 波特率 = **2M**（与 `CONF.TXT rate=2M` 一致）。U10(PE3) = debug 通道。

---

### 2026-11-22 20:50 → 23:00 — ⏱️ 【长排查链】U7 寻线串口 rx=0

**走错的假设全部留档**：

| # | 假设 | 实验 | 判决 |
|---|---|---|---|
| 1 | PE7 无信号 | GPIO 电平采样 `[PE7] chg=2 lvl=0` | ❌ 有电平变化，推翻 |
| 2 | 中断优先级卡 FreeRTOS 边界 | 5→6 | ❌ 无效 |
| 3 | ORE 溢出锁死 HAL | 读 RDR+ICR 清 ORE | ❌ Err=8 持续 |
| 4 | 需要 HDSEL 单线半双工 | 设 CR1 bit3 | ❌ rx=0，已回滚 |
| 5 | HAL 中断状态机坏了 | 纯轮询读 RDR | ❌ rx=0，系统变慢，已回滚 |
| 6 | 模块是 I2C 不是 UART | 软 I2C(0x12/0x30) | ❌ 无响应，已回滚 |
| 7 | 模块 3.3V 供电 | 用户切 3.3V | ❌ **模块必须 5V**（3.3V 时 0 帧） |
| 8 | CH343 5V 电平不匹配 | 用户切 3.3V | ⚠️ 合理但仍无数据 |
| 9 | **dbg_print 抢占 U7** | 查 dbg_print 句柄 = huart7 | ✅ **真实 bug**，已改回 huart10 |
| 10 | **prot_8irtraking.c 文件损坏** | 文件 16KB（应为 8KB），函数重复两遍 | ✅ **真实 bug**（Python 注入所致），整文件重写 |
| 11 | **taurus UART7 波特率误设 921600** | 代码实读 | ✅ **真正根因** |

**关键中间结论（有数字）**：
- 模块接 COM95 直连（CH343 USB-TTL，115200）：**1204 对 A/D 帧每秒**，格式全对
  `$A,x1:219,x2:253,...,x8:220#` + `$D,x1:1,...,x8:1#`
- 模块 5V 供电时 TXD 空闲电压 = **4.4V**（标准 5V TTL）
- MCU UART7 寄存器：`PE7moder=2 AF=7 CR1=0xd ISR=6000d0`，`HAL_UART_Transmit` 返回 **HAL_OK**

---

### 2026-11-23 00:00 — ⏱️ ✅✅ 【决定性突破 · 用户判断正确】根因确认

【用户】"我不认同说有这个波特率问题.我觉得就是代码的问题，你直接在原厂那个历程，它只有hex没有工程吗？那上面改来去尝试跟本电脑通信"

**实验 D1**（用户提议 · AI 执行）：原厂工程 `MX_UART7_Init` 的 `921600` → `115200`，编译 0 Error，烧录：
```
COM95 @115200 → b'U7 Test!\nU7 Test!\nU7 Test!\nU7 Test!\n'   ← ✅ 完美
COM95 @921600 → 乱码                                            ← 预期
```
⇒ **硬件链路 100%，115200 完全可用**，"波特率不匹配"假设被彻底推翻。

**实验 D2**（AI）：全工程扫描 U7 操作源 → fun_test_task 未编译 / usb_comm 已注释 / bsp_usart 无调用者 ⇒ **排除多源互抢**。

**实验 D3**（AI · ★根因）：读 taurus `Core/Src/usart.c`：
```
huart7.Init.BaudRate = 921600;  /* temp: match vendor for link test */
```
⇒ **我追"原厂能跑"时把 taurus 的 UART7 临时改成 921600，忘记复原**；而所有读取实验都用 115200 ⇒ 发送/接收速率不匹配 ⇒ 必然 0 字节。**后续 6+ 轮排查全部建立在错误前提上。**

**实验 D4**（AI）：改回 115200 + 隔离（chassis 注释 diag，仅 status_task 发 `U7_FROM_STATUS`）：
```
COM95 bytes 2048 → b'U7_FROM_STATUS\r\n' × 反复   ← ✅ 闭环
```

**判决**：✅ 闭环（验证数字：COM95 收到 2048 字节连续数据）。

**三条教训**：
1. 排查多轮无果时，**回到最小可验证路径做对照实验**（原厂工程对照是决定性一刀），别在自己代码里叠加假设。
2. **改配置做对照实验后必须记录并复原**——921600 的遗忘导致 6+ 轮空转。
3. 用户对"就是代码问题"的现场直觉强于纸面推理；**对照实验 > 假设堆叠**。

---

### 2026-11-23 00:10 — ⏱️ 日志补档（用户令）

【用户】"已经很久没有压缩了，先执行一下全量的日志补充登记。所有日志，所有做过的实验，每一次烧录都一定要有日志。改了什么？为什么这么改？改完之后效果怎么样？"

【AI】补档内容：
1. `Develop_Log/20261122_全量基座移植_handoff.md` 第二十六节（D1-D4 逐条：目的/方法/现象/分析/改动/判决）
2. 同上第二十七节（**32 次烧录完整清单**：改动 → 动机 → 结果）
3. `handoffs/handoff_20261123_U7串口跑通.md`（状态/资产/已验结论/Dead Ends/没测清单/待办/环境假设清单）
4. `handoffs/README.md`（索引登记）
5. `Develop_Log/20261122_自迭代计划.md` 第五节「当前状态」
6. 本文件（daily_review.md）

**当前遗留（下一轮）**：
- **首要**：恢复 `chassis_task` 里被注释的 `prot_8ir_uart_diag()` → 看 `[LINE] rx` 是否增长
- 删除 status_task 的临时 `U7_FROM_STATUS` 测试代码
- 遮挡实验钉死通道映射（x1..x8 ↔ 物理探头）
- CAN 波特率重算（PLL2 改 M=2 后 FDCAN 时钟变 72MHz，`fdcan.c` prescaler 待校）
- BMI088 P0 优化（DMA burst / PE12 EXTI / 温控降频 ≤1Hz）

**终态**：🟡 半闭环（U7 链路已通，完整寻线功能待恢复）

---

### 2026-11-23 — 上一轮计划审计与新一轮计划改写（仅文档，未执行固件实验）

【用户】要求解释上一轮为何耗时、巡线为何一直未接入、舵机和实际电机为何未开始，并要求依据日志增补/改写/完善原自迭代计划。

【AI 审计结论】：
1. **主要失误是实验管理，不是模块本体**：taurus 的 U7 临时改为 921600 后未复原，PC 端持续按 115200 读取，后续多轮排查建立在错误前提上。
2. debug 一度走 huart7，与巡线协议争用同一串口；修复前模块会收到非协议文本。
3. 波特率、IRQ、ORE、HDSEL、轮询、I2C、供电和接线连续叠加，缺少单变量与固定拓扑，导致现象不可归因。
4. 多次把“HAL_OK/收到测试字符串”写成“真正根因/闭环”，混淆单向链路与端到端功能；D4 实际只达到 TX L1。
5. Python 修改 CRLF+中文 C 源码造成重复/截断，引入额外返工；超时后未及时退回原厂最小对照，直到用户提议 A/B 才突破。
6. 舵机/电源未启动是因为巡线验收门长期未过；此前 CAN1↔CAN2 互收不等于实际电机接入。
7. 日志质量亦有待复核项：烧录表题写 30 次但实为 32 行；部分时间顺序不一致；“1204 对/s@115200”与所记 ASCII 帧长带宽不符，暂不作为性能基准。

【计划改动】：原地改写 `Develop_Log/20261122_自迭代计划.md`，保留历史进展，新增/更新：
- 当前接口定案：U1 视觉、U7 巡线、U10 debug；舵机不得沿用旧计划的 U7。
- L0～L5 闭环等级，禁止用编译/TX 测试替代功能验收。
- U7 三阶段：T1 CH343 模拟器验证 MCU RX+解析 → T2 真实模块 → 遮挡与位置方向。
- 后续任务拆分：舵机+可调电源、实际电机接入、BMI088/CAN 优化。
- 止损规则：单假设最多 20 分钟或 2 次烧录；一次实验一个变量；配置台账、拓扑编号、每次烧录完整登记。

【结果】：✅ 计划文档已完成审计与改写；本轮未改 C 代码、未编译、未烧录、未抓串口。下一步仍需用户确认后才执行 U7-A。

---

### 2026-11-23 — U7-A0 当前串口基线（用户已确认开始执行）

【目标】：把 taurus 板 U7 与 COM95 双向跑通，达到 L3（MCU 收到并正确解析已知 A/D 帧）。持续迭代，一次一个变量，每次烧录登记。

【当前拓扑 T1】：U7 ↔ CH343（3.3V 模式）↔ COM95@115200；真实巡线模块不在此拓扑中。

【端口枚举结果】：当前存在 Horco `VID=FAED PID=4873 SN=90BBD2C8` 的 **COM134**；另有 mbed `0D28:0204` 的 COM136；132/135 当前未枚举。因此本轮 DAP 调试口使用 COM134@2M。

【基线读取】：COM134@2M 连续收到 1941 字节，包含：
```
[SYS] imu yaw=-126.8 pit=2.7 cnt=5031386 mode=0
[LINE] st=0 pos=0 rx=0 a1..a4=0 0 0 0 d=0000
[DIAG] PE7moder=2 AF=7 chg=0 lvl=1 ISR=6000d0 CR1=d poll=0
```

【判定】：✅ COM134 身份与 debug 通道闭环；板子在运行。❌ 当前 `[LINE] rx=0`，完整 U7 巡线仍为未闭环。当前 flash 含 `[DIAG]`，与磁盘源码“隔离版移除 DIAG”的状态不完全一致，后续以新编译 hex 的哈希和时间作为包身份。

---

### 2026-11-23 00:32 — U7-A1 恢复轮询接收（烧录 #33）

【唯一功能改动】：
1. `chassis_task.c`：恢复 `prot_8ir_uart_diag()`，使 U7 轮询接收重新运行。
2. `status_task.c`：删除 `U7_FROM_STATUS` 直发及无效冻结看门狗残留；`[LINE]` 增加 `pb=ir_poll_bytes`，区分“无原始字节”和“有字节但未成帧”。

【动机】：从隔离 TX 测试回到 CH343 模拟器双向验证；配置快照已核对 U7=115200、U10=2M、debug 只走 huart10。

【构建】：`build_u7a1.log`：Code=104928，**0 Error/0 Warning**。hex SHA256=`b35e66a46298e2fd9950d688cc73f2e34bcbec870bf1ef559a75196a349a0914`，大小 352537 字节。

【烧录命令】：
```
openocd -f _horco_probe.cfg -c "init; halt; program .../CtrBoard-H7_ALL.hex verify; exit"
```
【烧录结果】：✅ Programming Finished / Verified OK。功能效果待 COM95 已知帧测试，暂不判定 L3。

【U7-A1 已知帧测试】：COM95 发送 10 对已知 A/D 帧，共 1020 字节；预期 `state=1 position=-160 analog[0..3]=101/202/303/404`。
- 实测：6s 内 debug=893 字节，`[LINE] rx=0`，字段不变；COM95 从 MCU 收到 0 字节。
- **异常**：打印仍是不含 `pb=` 的旧格式，与本次源码/构建不符；不能据此判定 UART RX 失败。
- 当前假设：烧录后未运行新镜像（此前烧录命令不带 reset）。下一动作仅做 Cortex-M AIRCR 软件复位，再观察是否出现 `pb=`，不改代码、不烧录。

【软件复位对照】：
1. `init; halt; mww 0xE000ED0C 0x05FA0004; exit` 后，COM134@2M 读取 3s = 0 字节。
2. `init; halt; resume; exit` 后再读 3s 仍 = 0 字节。
3. Keil 工程路径复核：`status_task.c`/`chassis_task.c` 均指向本次修改的 `..\Application\` 文件；构建时间新于源码，hex 哈希已登记。
4. 判决：❌ 软件复位/恢复路线未让新固件输出，按止损规则停止继续堆 SWD 操作。需要用户按一次板载 RESET 或重新上电，再读取 COM134 验证包身份。

【用户纠正】：此前同样的 SWD 流程可运行，现场也不能再操作复位，不应把问题归因于缺少人工复位。

【纯 SWD 恢复（成功）】：在同一 openocd 会话执行：
```
init; halt;
mww 0xE000EDF0 0xA05F0000;   /* 清除调试停机控制 */
mww 0xE000ED0C 0x05FA0004;   /* AIRCR SYSRESETREQ */
sleep 200; exit
```
COM134 随即恢复，出现新格式 `[LINE] ... pb=0`，且 `imucnt` 从 0 附近重新增长。✅ 新镜像已运行，无需用户复位。后续烧录均使用该 SWD 放行尾段。

【U7-A1 有效复测】：在新镜像下，COM95 再发 10 对/1020 字节已知帧；6s 内 `pb=0 rx=0`，字段不变，COM95 从 MCU 收到 0 字节。说明既不是旧包假象，也没有观察到 U7 原始字节或周期发送。

【下一单变量实验 U7-A2】：只增加 `dt=ir_diag_ticks`（`prot_8ir_uart_diag()` 执行计数），区分 chassis 未执行到巡线函数 vs 已执行但 UART 无收发；不改接线、波特率或协议。

---

### 2026-11-23 — U7-A2 执行计数（烧录 #34）

【改动/动机】：新增 `ir_diag_ticks` 并在 `[LINE]` 打印 `dt`，其余配置不变。
【构建/包身份】：`build_u7a2.log` 0 Error/0 Warning；hex SHA256=`dc49bce3be3ac4cf43ceee150ed44c34d8ccab82e18af14f23f01dd488426fc0`。
【烧录】：Verified OK，并以“清 DHCSR 调试控制 + AIRCR 系统复位”纯 SWD 放行。
【结果】：新镜像下 `[LINE] ... pb=0 dt=0`，COM95 发 1020 字节仍无变化，MCU→COM95 也是 0 字节。
【判定】：✅ 收敛到 `prot_8ir_uart_diag()` 从未执行，不是解析器先失败。chassis 是最后创建的动态任务；FreeRTOS `configTOTAL_HEAP_SIZE=15360`，同时有 10 个动态业务任务与 5 个动态队列，存在堆耗尽导致末端任务创建失败的直接风险。

---

### 2026-11-23 — U7-A3 堆修正（烧录 #35，待构建）

【唯一改动】：`Core/Inc/FreeRTOSConfig.h` 的 `configTOTAL_HEAP_SIZE` **15360→32768**。
【动机】：保证全部动态任务和队列可创建；以 `dt` 是否从 0 增长作为直接验收，其他代码/接线/波特率不动。

【构建/包身份】：`build_u7a3.log` **0 Error/16 Warning**（增大配置头触发全量重建，暴露既有隐式声明/类型告警；本实验不夹带清理）。hex SHA256=`bba61bc8f48b84fee95ca02d4500425f0146b9546bb9c8ad91969f2ebe002c10`。
【烧录】：✅ Verified OK + 纯 SWD 复位放行。
【结果】：
- `dt` 从 0 增至 9557，chassis/巡线函数开始持续运行。
- COM95 收到 **153 字节**：多组 `MCU_ALIVE` 与 `$0,1,1#`，U7 TX 从任务层恢复。
- 但 COM95 发送 1020 字节后仍 `pb=0 rx=0`，RX 尚未闭环。

【判决】：✅ 单变量对照成立，**15KB FreeRTOS 堆导致末端 chassis 未运行**是本轮“恢复调用后仍无收发”的根因；32KB 后任务与 TX 已验证。U7 RX 进入下一层诊断。

---

### 2026-11-23 — U7-A4 RX 物理/外设观测（烧录 #36，待构建）

【唯一观测改动】：新增 `isr=UART7 ISR`、`ore=溢出次数`、`pc=PE7 电平变化次数`，不改变 UART 配置、接收方法、接线或发送内容。
【目的】：在 COM95 发送已知帧时区分：PE7 无信号 / 引脚有信号但 UART 无 RXNE / UART 已接收但解析失败。

【构建/包身份】：`build_u7a4.log` 0 Error/0 Warning；hex SHA256=`df3b403eecf3aef38a4b2c3130c49b383448b5d8f158709f4feeac6a3cca8482`。
【烧录】：✅ Verified OK + 纯 SWD 复位放行。
【结果】：发送前 `pc=0`；发送 1020 字节后出现 `pc=28 ore=53 pb=1 rx=0`，`isr` 在 `6000d0/6010d0` 间变化。COM95 同时收到 MCU 153 字节周期数据。
【判定】：✅ PE7 物理信号与 UART 接收活动均存在；❌ 2ms 轮询只读到 1 字节即持续 ORE。115200 8N1 每字节约 87µs，而轮询周期约 2ms，无法接收连续流。该实验推翻“无信号”，支持“接收服务方式不及时”。

---

### 2026-11-23 — U7-A5 切换中断接收（烧录 #37，待构建）

【唯一功能改动】：`prot_8irtraking_init()` 启动单字节 `HAL_UART_Receive_IT`；`prot_8ir_diag` 停止轮询 RDR，保留任务/周期命令。`[LINE]` 新增 `ib=ir_isr_bytes`。
【已核对接口链】：UART7 MspInit 配 PE7/PE8 AF7 + NVIC 优先级6；`UART7_IRQHandler→HAL_UART_IRQHandler`；UART7 完成回调进入 `prot_8ir_on_rx` 并重新挂接收。
【验收】：COM95 发 10 对/1020 字节后，预期 `ib≈1020 rx=20`，`101/202/303/404`、`d=1011`、`state=1 position=-160`。

【构建/包身份】：`build_u7a5.log` 0 Error/0 Warning；hex SHA256=`d7799f3b6cd6cb08864c09a6f124461ec4f9227524bb617d58f8a678ca41270c`。
【烧录】：✅ Verified OK + 纯 SWD 复位放行。
【实测结果】：
```
[LINE] st=1 pos=-160 rx=20 ib=1020 pb=0 dt=6545 isr=0 ore=0 pc=26 a1..a4=101 202 303 404 d=1011
```
- COM95→MCU：发送字节数 1020 = MCU 中断接收字节数 1020。
- 完整帧：10 对 = 20 帧，`rx=20`。
- 解析字段、数字帧和位置解算全部与预期一致；`ore=0`。
- MCU→COM95：同期收到 153 字节周期输出。

【判决】：✅✅ **T1 拓扑下 U7 双向串口与 A/D 协议解析达到 L3**。两个关键修复为：32KB FreeRTOS 堆让 chassis 真正运行；单字节中断替代 2ms 轮询，消除连续流 ORE。真实模块 T2 与遮挡 L4 不在本次 COM95 目标内，仍未测试。

---

### 2026-11-23 — U7-A6 清理测试文本并连续流复测（烧录 #38，待构建）

【唯一功能改动】：删除 U7 上每 500ms 的 `MCU_ALIVE` 测试文本；保留中断 RX 与每 2s `$0,1,1#` 合法协议命令。动机是避免以后真实模块收到非协议数据。
【复测口径】：COM95 连续发送 **100 对/10200 字节**，每对间隔 20ms（50 对/s），8s 采集；预期 `rx=200 ib=10200 ore=0`，字段仍为已知值。

【构建/包身份】：`build_u7a6.log` 0 Error/0 Warning；hex SHA256=`5b99be6107abd5013d8cca6779a4b6f5699300234e14a86e0ff4d2fb185bc7fe`。
【烧录】：✅ Verified OK + 纯 SWD 复位放行。
【连续流实测】：
```
sent_pairs=100 sent_bytes=10200
u7_from_mcu_bytes=28  = 4 × "$0,1,1#"
[LINE] st=1 pos=-160 rx=200 ib=10200 pb=0 dt=9561 isr=0 ore=0 pc=282 a1..a4=101 202 303 404 d=1011
```
【判决】：✅ **目标闭环（T1/L3）**：100 对帧无丢失，10200 字节数精确一致，200 帧全部解析，ORE=0；MCU→COM95 仅发送合法启动命令。临时 `U7_FROM_STATUS` 与 `MCU_ALIVE` 均已移除。

【本次未覆盖】：真实巡线模块直连（T2）、4.4V TXD→PE7 可靠性、遮挡映射与位置方向（L4），需后续接线后另验。

---

### 2026-11-23 — T2 真实模块直连 ✅（无烧录，纯接入验证）

【拓扑变更（用户操作）】：CH343 撤出 U7；真实八路巡线模块 VCC=5V、GND 共地、TXD→PE7、RXD→PE8；LA 并挂观察。固件仍为 #38（`5b99be61`），无任何改动。

【实测（COM134，模块上电稳定后）】：
```
[LINE] st=1 pos=0 rx=5569 ib=304677 pb=0 dt=16758561 isr=0 ore=0 pc=9231 a1..a4=4038 4057 4045 4053 d=0000
```
- `rx` 每 600ms 打印增 ~81 帧 → **≈135 帧/s（≈67 对 A/D/s）**；`ib` 增 ~4550 B/s，两者自洽（67 对 × 2 帧 × ~55B ≈ 7.4KB/s）。
- 总线负载 ≈ 7.6KB/s ÷ 11.52KB/s(115200 有效带宽) ≈ **66%**；与"此前 1204 对/s"旧记录矛盾——印证审计判断：**1204 对/s 是错误口径，真实速率 ≈67 对/s**。
- `ore=0`：中断接收在真实连续流下零溢出；4.4V TXD 直连 PE7 电平可靠（本轮 5 分钟内）。
- 模拟量真实量程 ~4037-4057（当前表面/高度）；数字量 d=0000（按手册 0=黑线/亮灯，当前 8 路全判黑——模块朝向/表面待遮挡定标）。

【判决】：✅ **T2 闭环（L3 于真实硬件成立）**。遗留：①遮挡实验钉 x1..x8 映射与 position 方向（L4）；②发现 `chassis_task` 把 digital=1 当黑线（`if (digital[i]) cnt_on_black++`），与手册/compute_position 的 0=黑线语义相反——线跟踪触发条件疑似反了，待 L4 定标后一并修。

---

### 2026-11-23 — 模拟量巡线框架（烧录 #39/#40）

【用户指令】：把模拟量接进去（数字阈值易误触发、边缘闪），用调研到的电赛算法搭软件框架；黑白校准语义暂不管；重点是解决"冲太快、转弯只出现半秒就冲过去、之后无纠正量"。

【参照算法】：`25-aiming-cart/chassis/User/Bsp/bsp_line.c` + `Tests/line_follow_sim.py`：数字质心 + 低通 + 丢线判定 + ω 限幅/角加速度限制。本工程升级为模拟量版。

【#39 改动（f7da4066，0 Error/0 Warning）】：
1. `prot_8irtraking.h`：结构体加 `on_cnt`/`contrast`；定义 `LINE_STATE_LOST/OK/ALLON`。
2. `prot_8irtraking.c` `compute_position()` 重写：自适应阈值 `thr=vmin+0.5*(vmax-vmin)`（逐帧相对，不依赖校准）+ 对比度门 `MIN_CONTRAST=150`（抑制边缘误触发）+ 模拟加权质心 + 丢线/全压线保持记忆 position。`LINE_BLACK_IS_LOW=1` 宏留待遮挡实验定极性。解析由 A 帧驱动。
3. `chassis_task.c` `CHASSIS_LINE_TRACK` 重写：OK=PID 纠偏 + 偏差越大越慢（|pos|=100→vx=700）；ALLON=直行过十字；LOST=700ms 内按记忆 position 方向给 ±900 转向、超时停车。彻底删掉旧的 digital=1 反极性逻辑 + 死循环隐患。
4. `[LINE]` 打印改 `on/ct/rx/ib/...a1..a4`。

【#40 改动（a6115e99，0 Error/0 Warning）】：#39 打印参数残留 3 个多余实参导致 a1 错位（打印出 dt 的值）；清理并扩展为 8 路全模拟量 + 8 位数字量打印。

【烧录波折（记为一条教训）】：#39 后连续 3 次复位序列未让新包运行（打印还是旧格式/计数器不归零），分别试了"单会话 program+复位"（芯片静默 0 字节）和"program、复位分两次会话"（放行但跑旧包）。最终有效序列：
```
openocd ... -c "init; halt; mww 0xE000EDF0 0xA05F0000; mww 0xE000ED0C 0x05FA0004; sleep 500; resume; exit"
```
（resume 报 not halted 但芯片已放行）→ 新格式出现。教训：**烧录后必须核对包身份（计数器归零/打印格式），不能只看 Verified OK。**

【验证状态】：新固件在跑（`st=0 pos=0 on=0 ct=0 a=0,...,0 d=00000000` 新格式正确）；但**真实模块此刻 ib=0、a 全 0 = PE7 无字节**——同一固件 #38 时 ib 曾涨到 36 万，故判定为模块侧停止输出（供电/校准/接线待用户确认），非软件故障。模拟量算法在真实数据下的质心/极性/丢线行为**尚未验**，待模块恢复输出或用户做遮挡实验。

【遗留（下一会话）】：①模块为何停输出（用户确认 5V/GND/接线）；②遮挡实验定 LINE_BLACK_IS_LOW 极性 + x1..x8 映射 + position 符号；③真模块恢复后核对模拟质心与弯道减速行为；④本会话到此准备压缩，切换下一任务（舵机+电源）。

---

### 2026-11-23 10:32 — ⏱️ 压缩审计（devlog-append + pre-compact）

【AI】按用户 `[$devlog-append]` + `[$pre-compact]` 执行压缩前审计：
- 新增 handoff `handoffs/handoff_20261123_U7闭环与模拟量巡线.md` 并在 `handoffs/README.md` 登记。
- 既有计划 `20261122_自迭代计划.md` 已更新当前状态（U7=L3、模拟量框架已烧录、模块停输出待查）。
- Git：master@e46f31d，30+ 未提交 commit 未推送，工作树大量源码改动；单 worktree 无异常。commit/push 需用户授权，本轮未执行。
- 跨会话环境对账：查 `rc_remote`(11-23 10:14)/`kmo_mc_rcv`(11-23 00:03) daily 尾部，均为 RF 地址族/桥固件/OTA 变更，与本主题无关；无环境差。
- 下一会话主题按计划取任务3A：串口总线舵机 + 可调电源（用户未指定，按“执行下一个任务”与计划顺序取）。

---

### 2026-11-24 23:10 — 舵机通信与无线 DAP 夜间续测（#41～#50）

【用户指令】用户离开实验室，要求 AI 持续监控 RD6030 电压/电流，并继续迭代 U10 无线调试串口与舵机通信；舵机法兰已卸，允许运动测试；不得改动其他端口。

【电源监控】RD6030 实际枚举为 **COM139 / CH340 / SN 6042 / FW V1.47**。复用桌面已运行的 hub（`127.0.0.1:8990`，`--allow-cutoff`），新增持续监控：采样 1s，电压窗 23.5~25V，电流上限 0.8A，持续 CC 3s、连续 5 次无响应、输出意外关闭均报警并尝试自动断电。记录：`_rd6030_servo_samples.csv`、`_rd6030_servo_events.jsonl`。23:09:58 稳定读数 **24.00V / 0.03A / CV / ON / prot=0**。监控过程产生的 VMIN/OUT_OFF 均来自主动断电重启，不是负载故障。

【烧录清单（工程 `taurus_h7_all`，全部经无线 DAP SN `8E74D081` SWD 烧录）】
| 编号 | 时间 | 唯一功能改动 | hex SHA256 | 结果 |
|---|---|---|---|---|
| #41 | 11-23 12:47 | UART1 921600→115200；PA7 常开；每 2s 发 P1200/P1800 大幅摆动 | `887ac8183f14afbb189a75bc615eec066af19d0a2c4e76d14a6543b25b0e5ae8` | 新包运行；用户观察舵机不动；后确认 PA7 不是本板 5V 灯使能，PC15 才是 5V_EN |
| #42 | 11-24 21:41 | 停止自动摆动，只发一次 `#000PVER!` 并打印 HAL 状态 | `4acb6d3f102a3a38d2f6f11e9b34593b8ce3c340c26c9dd2a23a1fc8054b5670` | COM141 0B；当时误以为 DTR/RTS 无关，实际 U10 仍为 2M |
| #43 | 11-24 21:47 | UART1 改 `HAL_HalfDuplex_Init`；PVER/PID 阻塞接收并打印原始字节 | `f14e03db71fa502ad69c8c44f0eeadaab12a690f16b9d575af54237465b25a3a` | COM141 0B；未获得协议证据 |
| #44 | 11-24 22:14 前后 | 单次 PRAD→位置+20→500ms→PRAD；无有效位置则禁止运动；完成后停止 | `68239c2b7992fa2d4baf5848d42a707529f110a503abbf55e4e3324870ccae99` | 已烧录；COM141 0B，无法判断是否执行 |
| #45 | 11-24 22:35 | **U10 2M→115200**；`SERVO_BENCH=0`，不发舵机命令；串口打开不强制 DTR/RTS=False | `57ef541e07db8e03ce6e9246f04fc1333c28dbc6ac556ed1df6c85735d1335da` | **COM141 收到 3400B**，`[SYS]/[LINE]/IMU` 连续增长；U10 无线链路确认打通 |
| #46 | 11-24 22:39 | 恢复 UART1 只读轮询 PVER/PID/PRAD；不发送运动帧 | `d3ffe2843862e254d1e41b1695e4a354cdad9f07f08291c09d070ef5443691f5` | 收到 1344B 系统日志，但无 `[SERVO]`；不能据此判断舵机 |
| #47 | 11-24 22:46 | arm 入口/初始化完成/空闲堆锚点 | `3f0ae8dc25d34203ffca75a6db7b4474527bfae29858006c25ae0721c95d6fba` | 0B；启动状态未确认 |
| #48 | 11-24 22:52 | 调度器前打印 arm/chassis 创建结果与空闲堆；尝试 `reset run` | `7253be538fb6682ddf9dddf580d2885ce12378a040d3fa0fd30990ec247b52b1` | 0B；仅 1 条历史 `__packed__` 警告；未获得任务创建证据 |
| #49 | 11-24 23:02 | 撤掉任务创建打印；`SERVO_BENCH=3`，不初始化 PWM、不发 UART1，只打印 arm 存活 | `ba3be3b22e66d1ecd6c443f09c0fed0534fe748c7445452e64db8dbb5e796eec` | **真实掉电到 0V 后重启成功**；COM141 2238B；`[SERVO] alive` tick 每秒增长，free_heap=16296B |
| #50 | 11-24 23:08 | `SERVO_BENCH=2`，跳过 PWM，UART1 轮询 ID 000/001/002 的 PVER/PRAD，只读 | `53b9b8cbcdba3fa45f08c7616a1c132179cc8707b70f186dfa582a2208afb02d` | 0B；断电仅 3s 后面板仍为 21.52V，板子未真正掉电，烧录后仍可能 halted；待按 Vout<0.5V 判据重做电源循环 |

【已纠正的错误结论】
1. “HDSEL 必须外部上拉 4.7kΩ / PA9 要开漏”：撤回。当前 PA9 仍为 AF_PP，手册未要求上拉；不得把 PA9 上拉到 5V。
2. “CH343 TXD 串 6.8kΩ 是必要接法”：手册无此要求；该土法未成功，后续按厂商单总线要求处理。
3. “U10 2M 是固定事实”：只适用于旧 Horco DAP；当前无线 DAP 实测 **115200** 才收到数据，DTR/RTS 保持默认。
4. “#47/#48 无日志=arm 任务没运行”：证据不足；真正掉电重启后的 #49 证明 arm 可运行。
5. “主动断电 3s 即完成掉电”：错误。RD6030 输出关闭后面板仍可能保持约 21.5V；必须以实测 Vout 接近 0V 为准。

【当前未决】#50 的舵机 PVER/PRAD 应答尚未取得；真实线路上 PA9 波形/电平未测；不把“HAL 发送成功”当成“舵机收到”。

---

### 2026-11-25 00:00 — ★舵机串口闭环突破（#51～#56）

【目标】通过 H7 UART1(PA9) 与 SP-30S 单线半双工通信，取得版本和角度；U10/COM141@115200 无线打印；RD6030 持续监控。

【关键纠错】
1. `gState=32 / RxState=32` 是 **HAL_UART_STATE_READY**，不是 BUSY；`st=3` 是 **HAL_TIMEOUT**。此前解释相反，已纠正。
2. #55 证明 BRR=`0x0412`、PCLK2=120MHz，对应 **115.16kbit/s**，与舵机 115200 匹配，排除波特率错误。
3. 逐字节 `dbg_print` 每字节耗时约 2.6ms，而 115200 字节间隔约 87µs，导致 ORE；改为 ISR 环形缓冲后收帧成功。
4. USART1 IRQ 原优先级 5 会被 FreeRTOS 临界区屏蔽；改优先级 6（高于系统调用边界 5），收包期间不进入临界区。

【构建/烧录】
| 编号 | hex SHA256 | 改动与结果 |
|---|---|---|
| #51 | `874ea322d23aedb965d5ef15018ceb593482c74fddf620a9330bfd2ecb2efab6` | 去掉发送前 Abort；日志停在 `before_tx`，首次误判 HAL 卡死 |
| #52 | `ec177494f390fd7867b63eb8d32c9cbbf33f5fd8a84e2487140982892db95ad6` | 关闭视觉 DMA；补读确认 HAL 立即 HAL_TIMEOUT，状态其实 READY，ISR 有 ORE |
| #53 | `1f804addb319a12215d2a61ec51b77472fe3662d734bd90982e95132122b9d9d` | DWT 直接寄存器发送：每命令 `sent=9`、TC=1、HAL_OK，接收 0B；证明 MCU 发送链路成功 |
| #54 | `47252aab60e1282f139c2d16f88cbc23fb6110bb5315ddf2733d8f5ae7060be0` | 接收窗扩至独立 5ms：首次收到 `0x23`（`#`）1B，后续 ORE |
| #55 | `1af212e0d717e3ffcbe7cb5828d4934dcefdd87aac9795f26fd7c8ed99218ccd` | 逐字节诊断确认每帧只收首字节、ORE；BRR/时钟匹配 |
| #56 | `059792738a7df928ea3d7454406e5138d52180e25dff0cb6201d4e592b55a85c` | **ISR 环形缓冲 + IRQ6 + 整帧打印，串口闭环成功** |

【#56 原始证据（COM141@115200，板上实测）】
```
[SERVO] tx=#000PVER! rc=0 rx_n=35 err=0 dt=4
HEX: 23 23 30 30 30 50 53 65 72 76 6F 2E 43 28 32 30 32 36 2D 30 34 2D 30 31 ...
[SERVO] tx=#000PRAD! rc=0 rx_n=11 err=0 dt=2
HEX: 23 23 30 30 30 50 31 35 30 36 21   => ##000P1506!
```
- 以上 PVER/PRAD 在监听窗内重复多轮，位置稳定 1506。
- ID 001/002 返回部分回显 `#`，无完整应答；当前总线只有 ID 000 一个舵机。
- 帧首出现两个 `#` 是当前物理链路的回显/响应拼接现象；完整帧和字段可稳定解析，不阻塞闭环。

【电源】00:00:07 RD6030：24.00V / 0.04A / CV / ON / prot=0；新版监控 `_rd6030_servo_samples_v2.csv`、`_rd6030_servo_events_v2.jsonl`，无过流/通信事件。

【验收等级】✅ **L3 协议闭环**：MCU 发送、舵机应答、版本识别、角度回读均通过。尚未做：单次微动和动作后角度回读（下一步）。

---

### 2026-11-25 00:07 — #57 状态机错误：意外连续微动，已断电

【烧录】#57，hex SHA256 `7395678a3f5ced4906b5ac01e8d3d004378a9b618fa0af0f0eafa5ba757a1962`，编译 0 Error/0 Warning。

【设计意图】读 PRAD → 发一次 `P=当前位置+20, T=500ms` → 等 700ms → PRAD 回读 → 停止。

【实际 bug】手册规定位置控制命令无返回；驱动按“超时但有部分回显”返回 `rc=3`。状态机把 `rc!=0` 当成失败并退回 step=0，随后再读位置、再发下一条 `+20`，形成循环。代码注释虽写“单次”，实现没有保证。

【原始证据】位置按每轮 +20 连续变化：
```
1511 -> 命令1531 -> 1523 -> 命令1543 -> 1543 -> 命令1563 -> 1563
-> 命令1583 -> 1583 -> 1603 -> 命令1623 -> 1623 -> 命令1643 -> 1643 -> 命令1663
```
即约 152 个脉宽单位（按 500~2500↔0~270° 映射约 **20.5°**），不是计划的 2.7°。

【保护动作】00:07:19 通过 RD6030 执行 `--cutoff --yes`，输出关闭，实测 `output=OFF, Iout=0.0A`（Vout 尚余 21.49V，随后按 <0.5V 判据确认放电）。持续监控未出现过流/OCP，但这不能证明机械上无碰撞；法兰已由用户卸下，当前只能确认无电气过流。

【修复】step=1 的运动指令无应答时也必须进入 step=2 回读；step=2 无论回读成功或失败都进入 step=3 停止，禁止回到 step=0/1。后续不再把 #57 当作单次微动验收。

【安全决策】在用户不在现场期间停止自动运动类实验；先烧无 UART1 自动发送的安全存活包，保持 RD6030 关闭，待用户回来后再决定是否单步回中。

---

### 2026-11-25 11:27～11:32 — #58 安全恢复 + #59 单次微动最终验收

【上午状态】RD6030 于 11:27:04 为 24.00V/0.06A/CV/ON；无线 DAP SN `8E74D081` 与 COM141 正常枚举。期间 #57 仍在循环，读取缓冲中的旧日志显示位置已从 1663 继续到 2210，随后 MCU 被 halt 并写入新固件。#57 的长时循环 Motion 由 AI 状态机错误造成，已单独记录，不作为成功实验。

【#58 安全包】hex SHA256 `58eb893d37ba68084082a32b1722d7e7b51750f94fa24c6483e854e789c2050b`。`SERVO_BENCH=3`：不初始化 PWM、不发 UART1，只打印 `[SERVO] alive`。烧录 Verified OK；COM141 确认持续 alive/tick 增长，FreeRTOS free_heap=16296B。说明新包已替换 #57，自动运动停止。

【#59 修复包】hex SHA256 `8f70b7cdc744c8eb8687cb44d56c0e5b5b38be25d720cb91d450485bec91e621`。`SERVO_BENCH=1`：PRAD→当前位置+20→T500ms→PRAD→step=3 永久停止。关键修复：step=1 即使运动命令无应答（rc=3/4）也只进入 step=2；step=2 无论回读成败都进入 step=3，绝不返回 step=0/1。

【#59 原始证据（COM141@115200）】
```
[SERVO] step=0 tx=#000PRAD! rc=3 rx_n=1 err=1 dt=172   # 首次上电抢先,未发运动
[SERVO] no_position: motion inhibited
[SERVO] step=0 tx=#000PRAD! rc=0 rx_n=11 err=1 dt=2
HEX: 23 23 30 30 30 50 31 35 30 37 21                => ##000P1507!
[SERVO] step=1 tx=#000P1527T0500! rc=3 rx_n=1 err=1 dt=12  # 手册:运动无返回
[SERVO] step=2 tx=#000PRAD! rc=0 rx_n=11 err=1 dt=2
HEX: 23 23 30 30 30 50 31 35 32 37 21                => ##000P1527!
[SERVO] move_done before=1507 after=1527
```
- 30s 监听窗内运动命令只出现 1 次，回读后无任何新的 step=0/1。
- 位置变化 +20，与命令目标完全一致；按 500~2500↔0~270° 映射约 2.7°。
- RD6030 11:32:21：24.00V / 0.05A / CV / ON / prot=0 / alarms=[]；全程无过流、OCP 或 CC_LONG。

【最终验收】✅ **L4 物理功能闭环**：UART1 发送、SP-30S 版本应答、PRAD 角度回读、单次位置控制、动作后角度一致均通过。U10/COM141@115200 无线调试串口同时打通。

【遗留】①#57 长时循环造成的位置从约 1511→2210 尚未人工回中；需用户现场确认机械安全后单步回中。②err 计数非零来自启动阶段/半双工切换的 ORE/FE/NE，完整帧仍稳定，后续应优化清错与方向切换。③驱动仍是台架诊断实现，业务化前需下沉到 Protocol 层、补参数检查和自动运动总开关。

---

### 2026-11-25 11:35 — #60 默认安全只读固件与最终状态

【目的】#59 已闭环，但源码默认 `SERVO_BENCH=1` 会在每次复位时自动微动；改为默认 `SERVO_BENCH=2`，只轮询 PVER/PRAD，不再自动发送位置命令。同步修正台架驱动头文件引用为 `prot_xb_servo_bench.h`。

【构建/烧录】0 Error/0 Warning；hex SHA256 `059792738a7df928ea3d7454406e5138d52180e25dff0cb6201d4e592b55a85c`（与 #56 功能镜像，默认模式改为只读）。无线 DAP 烧录 Verified OK，复位后 COM141 连续收到：
```
[SERVO] tx=#000PVER! rc=0 rx_n=35 err=0 dt=4
[SERVO] tx=#000PRAD! rc=0 rx_n=11 err=0 dt=2
HEX: 23 23 30 30 30 50 31 35 30 37 21  => ##000P1507!
```
15s 监听窗内 `T0500!` 运动命令 **0 条**。

【位置说明修正】#60 重新上电后 PRAD 为 1507，而不是 #57 断电前缓冲日志中的 2210；说明期间发生了掉电/复位，舵机回到上电初始位置。因此“仍停在 2210”的遗留判断撤回，当前实测位置为 **1507**，无需紧急人工回中；仍需用户后续确认机械零位定义。

【最终电源】11:35:43 RD6030：24.00V / 0.05A / CV / ON / prot=0 / alarms=[]。持续监控样本 `_rd6030_servo_samples_v2.csv` 已积累 42759 行；`_rd6030_servo_events_v2.jsonl` 无过流/OCP/通信事件。未 commit/push。

---

### 2026-11-25 12:10 — 实验代码清理 + 舵机控制回归机械臂业务

【用户指令】删干净实验代码，封装好，把舵机串口控制回归原本的机械臂控制。

【正式架构】
- `Protocol/prot_xb_servo.[ch]`：重写为正式 SP 系列单线半双工驱动，绑定 **USART1/PA9@115200**。发送使用 2ms DWT 硬件超时；RX ISR 仅写 128B 环形缓冲；`poll()` 解析完整 `#...!` 帧并维护在线超时。兼容台架实测的双 `#` 回显响应。
- `Protocol/prot_servo.[ch]`：保留原 `servo_*` 业务接口。肩/肘自动分派到 `xb_servo_set_pos()`，夹爪分派到 PWM；`servo_poll()` 每约 100ms 轮询肩/肘角度。相同目标不重复下发，避免 3ms 任务周期刷爆总线。
- `Application/arm_task.c`：恢复原遥控器手动/演示分支；保护模式或未解锁时**只断使能、不发位置命令**。业务层不再直接调用 UART、PWM 或 GPIO。
- `Bsp/taurus/bsp_pwm.[ch]`：只服务夹爪，使用达妙板实际引出的 **TIM2_CH3/PA2**，50Hz/20ms；不再重初始化 TIM3 干扰 IMU。
- `Bsp/taurus/bsp_gpio.[ch]`：供电使能由旧工程 PA7 纠正为达妙板 **PC15/Power_5V_EN，高电平开**。
- `Core/Src/stm32h7xx_it.c`：USART1 IRQ 改调正式 `xb_servo_rx_isr()`。
- Keil 工程移除 `prot_xb_servo_bench.c`；删除 bench `.c/.h`、台架探测脚本、bench 对象和 #41～#60 实验构建日志。全仓搜索无 `SERVO_BENCH/xb_servo_bench/T0500!/PVER!` 残留。

【构建】最终增量编译 **0 Error(s), 0 Warning(s)**；hex SHA256 `34e11986cfc17ffcae3c5eec2c92d18b98253bea64fb59acf84b3663159404ed`。全量重编译 0 Error，17 条历史警告来自 gimbal/CAN/vision/USB/ws2812 等未触碰模块；本次新增 0 条。

【未执行】本轮只清理、封装和编译，**未烧录、未让机械臂上电运动**。当前板上仍是上一轮 #60 只读诊断固件；正式业务固件已生成，待用户现场确认机械安全与接线后烧录。

【电源监控异常】12:10 检查发现 RD6030 已不在 COM 枚举中，hub `ws://127.0.0.1:8990` 不可达，`mon --list` 报 `RD6030 NOT FOUND`。因此此前持续监控已中断，不能声称当前仍受监控；最后有效读数为 11:35:43 的 24.00V/0.05A/CV/ON。需检查 RD6030 USB 连接或是否被其他程序占用。

---

### 2026-11-25 12:25 — U10 舵机测试口与 ID 分配

【用户要求】保留一个测试端口验证舵机在线/是否工作，并询问舵机 ID 分配是否已有。

【现状】正式驱动已硬编码肩=000、肘=001；此前没有独立测试命令或写 ID 接口。

【新增】
- `Protocol/prot_xb_servo.[ch]`：新增原始 ID 探测 `xb_servo_read_id_raw()`、原始 ID 修改 `xb_servo_write_id_raw(old,new)`、原始版本探测 `xb_servo_ping_raw()`、最后应答帧缓存。ID 修改仅在收到 `#newP!` 明确应答后更新肩/肘映射；不支持的广播改 ID 不开放。
- `Protocol/servo_test.[ch]`：在 **U10/COM141@115200** 增加命令行接收。ISR 只收整行放入 FreeRTOS 队列；`arm_task → servo_poll → servo_test_process` 顺序解析执行，不在中断中打印或访问舵机总线。
- 安全限制：测试命令只有 `ping/read/setid/map`，**无运动命令**；`setid` 在舵机供电 ON 时拒绝；保护模式仍处理测试命令，便于关机状态下分配 ID。

【命令】
```
servo ping 0       # 探测 ID 0 版本
servo ping 1       # 探测 ID 1 版本
servo read 0       # 已映射关节读角度; 未映射则读 ID
servo read 1
servo map          # 显示肩/肘当前 ID 映射
servo setid 0 3    # 仅关机状态: 将 ID 0 改为 3, 成功应答后更新映射
```

【构建】0 Error(s), 0 Warning(s)；hex SHA256 `8febbcc3d087d8d11496cd2afff2974b5ab9ec3b8a5a0c99fa54bfd1365ad7e7`。

【未执行】未烧录、未发送测试命令、未执行写 ID；当前板上仍为 #60 只读诊断固件。新固件需现场确认机械安全后烧录，再从 COM141 发送上述命令验收。

---

### 2026-11-25 12:35 — 手册 ID 分配规则核对与配置模式纠正

【手册事实】
- 每个 SP 系列舵机默认 ID=000，ID 范围 000~254，255 为广播地址。
- ID 存储在舵机内部，修改成功返回 `#newP!`；半恢复 `PCLE0` 保留 ID，全恢复 `PCLE` 才回 000。
- 多个舵机 ID 不能相同，否则串联控制时一起动作。
- **修改 ID 时必须一次只连接一台舵机**；逐台改完后再串联。
- 不知道旧 ID 时手册允许 `#255PIDxxx!` 广播改 ID，但本测试口为避免多机同时被改，不开放广播写。

【纠正】此前实现要求“舵机供电关闭才允许 setid”，这是错误的：舵机断电无法应答。改为 `servo config` 保护配置模式：
- `servo config`：保持舵机 5V 供电，机械臂配置状态锁定，禁止运动分支，仅允许测试/改 ID；
- `servo setid <old> <new>`：仅在配置模式允许，必须明确旧/新 ID，匹配新 ID 应答后才更新驱动映射；
- `servo bind shoulder|elbow <id>`：手动维护关节→ID 映射，拒绝两个关节使用同一 ID；
- `servo run`：退出配置模式、断舵机使能；
- `servo ping/read/map`：在线测试、角度读取和映射查询。

【注意】当前关节→ID 映射只在 RAM，重启恢复默认肩=000、肘=001；掉电保存尚未实现。逐台改 ID 后需用 `bind` 重设映射。

【构建】0 Error(s), 0 Warning(s)；hex SHA256 `d8fcfb65a5ca0f3d9862678879d160eeca725886fa129d0887342f6ad68b980b`。未烧录、未实际改 ID。

---

### 2026-11-25 12:55 — DBUS 遥控回归 + 纯拨杆拖动示教

【用户要求】参考 `https://gitee.com/full-smile/taurus_26_-aerial_-robot.git` 改回 DBUS，恢复原仓解锁/保护/巡线状态机；遥控器只有两个摇杆和两个拨杆，没有键盘。用户指定遥控串口为 U5。

【原仓审计】原仓 `prot_dr16.c` 使用 11bit 通道解析，`mode_switch_task.c` 的内八解锁条件为左拨杆中、右拨杆上，四摇杆内八；左拨杆上=遥控/视觉、中=保护、下=键鼠。原仓 DBUS 物理层为 UART1/PA9-RX，100000-8E1。当前工程 PA9 已固定给总线舵机，因此 DBUS 改走用户指定的 **UART5/PD2-RX**，不占用舵机 UART1。

【实现】
- 新增 `Protocol/prot_dbus.[ch]`：UART5 100000-8E1 接收，ISR 组 18B 帧入 FreeRTOS 队列，`mode_switch_task` 每 5ms 解析；100ms 无帧强制 PROTECT 并清解锁。
- `Core/Src/usart.c`：新增 UART5 初始化与 PD2/AF8 复用；`stm32h7xx_it.c` 增加 UART5 IRQ；Keil 工程登记新驱动。
- `Protocol/prot_sbus.[ch]`：改为 DBUS 兼容层，将 DR16 通道映射到既有 `wbus_rc`，保留底盘/机械臂原接口；DBUS 拨杆 1上/2下/3中映射为旧接口 2上/0下/1中。
- `Application/mode_switch_task.c`：恢复内八解锁、左拨杆中保护、上遥控、断联保护。因无键盘，左拨杆下改为 `ARM_TEACH_MODE`，不再进入键鼠模式。
- 右拨杆仍控制底盘：上=遥控、中=停止、下=巡线。
- 新增纯拨杆示教：左拨杆下进入示教，先给舵机上电并等待 500ms，再读取肩/肘位置、发送 PULK 释力；每 100ms 读取肩/肘位置，最多保存 64 个 `{shoulder,elbow}` 采样点。离开示教挡或进入保护立即发送 PULR 恢复力矩；释力失败会回滚锁力。示教期间 `arm_busy=1` 锁止底盘，不发位置控制命令。
- 当前只保存采样，不自动回放；回放需后续定义。

【诊断】`[SYS]` 打印新增 `dbus`（在线标志）、`rx`（DBUS 帧计数）、`lock`（解锁标志），便于现场确认 U5 链路。

【构建】最终增量编译 **0 Error(s), 7 Warning(s)**；7 条历史警告来自 gimbal 浮点转换、Math/status `__packed__`、prot_vision 未使用变量，本次新增 0 条。hex SHA256 `b43933fb45659adc38d660c3019b6f0b53d160ea933b9bf16226d35da099e8de`。

【未执行】未烧录、未接真实 DBUS 遥控器、未执行示教动作。需现场先验收 U5/PD2 收帧与 `[SYS] dbus/rx/lock`，再验证内八解锁和保护，最后空载验证示教。

---

### 2026-11-25 13:40～13:46 — 仅肘关节接入，分配 ID=001 + 最新 DBUS 固件上板

【用户指令】目前只接了肘关节，肩关节尚未接；先给当前肘关节分配 ID，肩关节留到下午。要求把最新固件换到板上。用户反馈修改慢，本轮因此收窄为单一舵机 ID 分配，不扩展遥控/示教功能。

【现场状态】COM141（无线 DAP SN `8E74D081`）恢复；板上原固件仍为 #60 只读诊断版。当前舵机 ID=000，版本 `Servo.C(2026-04-01 21:00:44)`，位置 1507。RD6030 USB 仍未枚举，无法读取实时电流；本轮不声称有电流监控。

【代码修正】
1. `prot_dbus.c`：恢复原仓缓冲口径，18B 帧包含起始字节，`prot_dr16` 从 `buff[0]` 开始解析；此前 `buff+1` 会让通道整体错位。
2. `prot_xb_servo.c`：写 ID 成功后只确认硬件应答，不再自动修改所有匹配旧 ID 的关节映射；关节归属必须显式 `servo bind`。此前自动映射把 shoulder 和 elbow 同时改成 1，导致 `bind rc=2` 冲突。

【烧录】无线 DAP SN `8E74D081`，OpenOCD `halt; program verify; exit`，Verified OK；复位后 `[SYS] ... dbus=0 rx=0 lock=0` 新格式确认运行。hex SHA256 `79cf419c056361e4c2d2a4df65fac8572d436817b27a6231e593a94324b43503`。

【ID 分配原始输出】
```
servo config       -> config mode ON: power held, motion inhibited, setid enabled
servo ping 0       -> rc=0 reply=#000PServo.C(2026-04-01 21:00:44)!
servo setid 0 1    -> rc=0 reply=#001POK!
servo bind elbow 1 -> 初次 rc=2（自动映射冲突）；修复固件后 rc=0
servo run          -> config mode OFF
servo ping 1       -> rc=0 reply=#001PServo.C(2026-04-01 21:00:44)!
servo read 1       -> rc=0 pulse=1507
servo map          -> shoulder=0 elbow=1
servo ping 0       -> rc=3（旧 ID 0 已无设备）
```

【验收】✅ 当前肘关节硬件 ID 已持久改为 **001**；版本/角度回读正常； shoulder=000、elbow=001 映射无冲突。未发送任何位置运动命令。

【下午待办】肩关节接入后单独上电，按 `servo config → servo ping 0 → servo setid 0 2 → servo bind shoulder 2 → servo run` 分配 ID=002，再与肘关节串联验证双 ID 独立寻址。当前肩关节未接，不做分配。

---

### 2026-11-25 13:55 — 肩关节接入并分配 ID=002

【用户指令】“下一步，换一个电机标 ID 吧？” 本轮只分配肩关节 ID，不进行运动测试。

【分配前检查】`servo map` 显示 shoulder=0、elbow=1；`ping 0` 有版本应答，`ping 1/2` 无应答，说明当前仅肩关节在线，肘关节已从总线撤下。肩关节当前 ID=000，版本 `Servo.C(2026-04-01 21:00:44)`。

【操作与原始输出】
```
servo config       -> config mode ON: power held, motion inhibited, setid enabled
servo setid 0 2    -> rc=0 reply=#002POK!
servo bind shoulder 2 -> rc=0
servo run          -> config mode OFF
servo ping 2       -> rc=0 reply=#002PServo.C(2026-04-01 21:00:44)!
servo read 2       -> rc=0 pulse=1505
servo map          -> shoulder=2 elbow=1
servo ping 0       -> rc=3（旧 ID 0 已无设备）
```

【验收】✅ 肩关节硬件 ID 持久改为 **002**，版本/位置回读正常；RAM 中关节映射为 shoulder=002、elbow=001。肘关节硬件仍为 001，未在本轮修改。

【限制】关节映射仍只在 RAM，MCU 重启后恢复源码默认 shoulder=0/elbow=1；需在 W25Q64 外部 Flash 跑通后增加映射持久化。本轮未发送位置控制命令，未验证双舵机同时在线独立寻址。

---

### 2026-11-25 14:23 — ⏱️ 开发日志补档审计（devlog-append）

【当前刀序】任务3A 舵机身份分配与业务接入（出处=`Develop_Log/20261122_自迭代计划.md` §五/§六）；本轮不再扩展功能，只补日志与计划状态。

【执行者】【AI】按用户 `[$devlog-append]` 审计 2026-11-25 会话；【用户】提供实物接线、供电与拨杆约束，并执行肩/肘逐个接入。

【本段动作枚举（12 步全录）】①重读 handoff/计划/daily；②审计 taurus_26 原仓 DBUS 与 mode switch；③核对 U5/PD2 引脚；④实现 UART5 100000-8E1 接收；⑤实现 DBUS→wbus_rc 兼容；⑥恢复内八解锁/断联保护；⑦按无键盘约束改纯拨杆示教；⑧修正 DBUS 缓冲偏移；⑨修复写 ID 自动改映射冲突；⑩肘关节 ID 000→001 并绑定/回读；⑪肩关节 ID 000→002 并绑定/回读；⑫核对 W25Q64 与示教存储状态。每步原始输出见本文件 12:55、13:40、13:55 段。

【W25Q64 结论】🟡 **未验证**：当前固件运行于 H7 内部 Flash；`Bsp/w25q64.c` 与 OCTOSPI2 配置存在，但 `Core/Src/main.c:251-253` 的初始化/读写测试仍被注释，日志无 JEDEC ID（器件身份号）、擦写或断电读回证据。示教采样仅存 RAM，断电丢失。曾误加载 `spi-diag` skill，该 skill 针对 BK3633/GD 链路，与本板无关，已停止使用。

【当前包身份】板上最新业务固件 hex SHA256 `79cf419c056361e4c2d2a4df65fac8572d436817b27a6231e593a94324b43503`；无线 DAP SN `8E74D081`，U10/COM141@115200。肩硬件 ID=002、肘硬件 ID=001；RAM 映射 shoulder=2/elbow=1，尚未验证双舵机同时在线独立寻址。

【仓库/环境】主仓 master@e46f31d，55 项工作树改动，单 worktree；`taurus_h7_all` 不在主仓 Git。RD6030 USB 未枚举，最后有效电流读数为 11:35:43 的 24.00V/0.05A，不能声称当前持续监控。未 commit/push。

【三问自检】数字均带板包/COM/时间条件；执行者与原始应答可追溯；终态为 ID 分配✅、DBUS 实物链路🟡、W25Q64🟡，无把编译成功写成物理闭环。

---

### 2026-11-25 15:17 — ⏱️ TF-mini/四路测距部署中途审计 + 压缩前补档

【当前刀序】比赛主线：明天回学校调底盘，后天早上巡线、简单避障；机械臂视觉可能只返回果子 x/y。TF-mini 属于避障传感器接入，出处=`Develop_Log/20261122_自迭代计划.md` §五/§六。

【用户指令】去查北醒 TF-mini 资料并部署四个前后左右测距，找简单避障逻辑；随后执行 `[$devlog-append]` + `[$pre-compact]`。用户提出可找子代理，但本会话长期硬规则为“无论如何都不开子代理”，【AI】因此继续前台执行，未派代理。

【协议事实】官方 `TFmini/TFminiPlus-Arduino` 示例确认：UART 115200-8N1；9 字节数据帧 `59 59 distL distH strengthL strengthH tempL tempH checksum`；距离/强度/原始温度均为小端，校验为前 8 字节求和模 256；温度 `raw/8-256`。配置命令帧头 `5A`，可设置输出频率/保存/复位。产品支持 UART、I/O、I²C；本地 `h723-rm-framework/modules/TFminiPlus` 使用 I²C 地址 `0x10`，因此四模块不能无复用器直接并到同一 I²C 总线。量程官方产品页为 0.1~12 m。

【板级接口核对】达妙 MC-Board02 的 USART2/3 内部仍是 TTL UART，但板上只引出 RS485 A/B；DE/RE# 仅控制收发方向，没有“恢复普通串口”模式脚，原理图未发现 TTL 旁路开关。裸接收脚为 PD6(USART2_RX)、PD9(USART3_RX)；用户无法飞线。24Pin 扩展 IO 明确引出 UART8_RX=PE0、UART9_RX=PD14、5V/GND，另含 RS485 A/B。曾短暂考虑 PA11/PA15/PB14，均未作为最终方案。

【本段动作枚举（18 步全录）】①读取比赛目标与当前计划；②查本地 TF-mini 资料；③获取官方 Arduino 仓库；④核对 9B 帧/校验/温度公式；⑤核对 I²C 多模块地址冲突；⑥核对 .ioc/管脚图/原理图；⑦确认 RS485 无兼容旁路；⑧确认扩展 IO 两路 TTL RX；⑨写 `Protocol/tfmini.[ch]` 四路解析与数据年龄；⑩改 USART2/3 为 115200 普通 UART；⑪声明 huart8/huart9；⑫写 UART8/USART9 初始化草案；⑬用户指出四路不够用；⑭讨论 I²C/PCA9548A 与额外开发板；⑮用户指出 A/B 排针不能用；⑯核对 RS485 芯片 DI/RO/DE/RE#/A/B；⑰确认 PA11 等飞线不可行；⑱核对半成品未接入构建。

【半成品状态（不得误报已部署）】`Protocol/tfmini.[ch]` 已创建，但 `main.c` 未调用 UART8/USART9 初始化，`stm32h7xx_it.c/.h` 未加 UART8/USART9 中断，MspInit 未配 PE0/PD14，Keil 工程未登记 tfmini 源文件，`tfmini_init()` 无调用点，避障逻辑未接入 chassis_task。USART2/3 草案虽然改为 115200，但若走扩展板仍应使用 PE0/PD14。**本轮未编译、未烧录、未接真实 TF-mini、未验证任何距离值。**

【原始锚点】官方示例 `Example-ReadData/ReadData/ReadData.ino:15-37`；本地参考 `_opensource_ref/h723-rm-framework/modules/TFminiPlus/tfminiplus.c:12-27`；扩展口定义见 DM-MC-Board02 说明书 §17 第 11~12 页；当前磁盘包仍为 SHA256 `79cf419c056361e4c2d2a4df65fac8572d436817b27a6231e593a94324b43503`。复跑协议核对：`py -I Tool/serial_probe/servo_probe.py` 不适用于 TF-mini；TF-mini 需另写 115200 原始帧读取器，当前未创建。

【赛道图理解与冲突】已确认参照工程为 100cm×100cm 黑色边线、逆时针、4 个左转 90°；但本项目早期记录另写“长方形掰 45°、线上 2~3 个障碍”。两者不是同一赛道，不能把参照工程地图当成本次课设地图。**果堆位置、果子数量/颜色、障碍物数量与位置、激光安装方向均未形成可执行地图。** 下一会话必须先让用户指出赛道照片/题面中的这些位置，再写死避障路径。

【避障候选（未实现）】最简可靠方案：前测距触发后减速/停车，比较左右距离选远侧；已知地图可写死绕行侧，使用陀螺转角闭环和编码器里程，随后重新找线。四路数据必须带 150ms 新鲜度；传感器失效不能当作距离 0，比赛策略需明确“失效停车”或“仅前向保守降速”。

【仓库/环境】主仓 master@e46f31d，55 项工作树改动，29 个未推送提交，单 worktree；`taurus_h7_all` 不在主仓 Git。未 commit/push。跨仓对账：rc_remote 14:36~14:45 为台架桥/遥控跳频复测，kmo_mc_rcv 13:40 记录 COM136 静默；均与达妙板 UART/TF-mini 接线无直接环境差，不借用其 COM 号/固件结论。

---

### 2026-11-25 16:25 — CAN 遗留电机清理：4×2006＋1×6020 全部收敛到 FDCAN1

【用户指令】仓库中与其他电机相关的遗留代码要删除；当前只有 4 个底盘 2006 和 1 个 6020。怀疑原 `trigger_motor` 是 2006 改名遗留，要求先看清再改；业务工程按本项目需要清理，不再保留原机器人外围代码。

【事实核对】`trigger_motor` 确为旧 2006 拨弹电机，反馈 ID=0x207、减速比 82.8；不是 6020。唯一 6020 为 `arm_yaw_motor`，反馈 ID=0x208。旧 `fric_motor[2]`、`yaw_motor`、`pit_motor`、DM/LK 协议、gimbal/shoot 任务及 CAN2/CAN3 电机任务均无本车用途。原 6020 仅注册和解析反馈，没有业务发送入口。

【连带 bug】`motor_msg[CAN_CHANNEL_NUM][5]` 的槽位 2 越界写 4 字节；旧 map 中 `motor_msg=0x2400DE6C`，0x208 对应槽位 2 写至 `0x2400DA6A`，落在 `rc=0x2400DA5C` 的 `mouse.y` 字段。`gimbal_task` 每 1ms 调用旧 `dji_send_current(&yaw_motor,...)`，还会向 CAN1 发 0x200 全零帧覆盖底盘控制。以上为静态代码/map 证据，不是运行时复现。

【过滤器阻断点】迁移后 `fdcan.c` 的 `StdFiltersNbr=1`，`bsp_can.c` 却配 5 条规则；硬件只认第 1 条 IMU 规则，其余规则落入接收 FIFO RAM，0x201~0x204/0x208 被硬件过滤。且原 BSP 按奇偶把规则分到 FIFO0/FIFO1，而 FIFO1 容量为 0，即使只改数量仍会漏一半 ID。

【执行者】【AI】先完成 46 个恢复文件备份（含当前烧录包 hex/axf/map），再用 Edit/Write 删除/改写。未使用 Python 修改 C 源码，未抓串口、未烧录、未改底盘控制算法。

【删除清单】16 个遗留源文件：`Application/gimbal_task.[ch]`、`Application/shoot_task.[ch]`、`Protocol/prot_FricMotor.[ch]`、`Protocol/prot_TriggerMotor.[ch]`、`Protocol/prot_dm_motor.[ch]`、`Protocol/prot_lk_motor.[ch]`，以及未参与构建的原厂 `Bsp/can_bsp.[ch]`、`App/fun_test_task.[ch]`。Keil 工程同步移除登记。

【实现】FDCAN/BSP/中断/CubeMX 仅保留 CAN1，Classic CAN 1Mbit/s，PD0/PD1；4 条过滤规则全部进 FIFO0（IMU 0x017~0x019、底盘 0x201~0x204、6020 0x208、空闲 0x205~0x207）。`prot_rm_motor` 仅保留 5 个电机对象，发送改为栈上局部 0x200/0x1FF 帧，删除共享缓冲区和越界写。依赖方 main/freertos/status/ui/vision/T_imu/remote_msg/judge_send 同步清理。

【编译】最终 `UV4 -b` 结果 **0 Error / 0 Warning**；Code=103440，RO-data=20304，RW-data=1092，ZI-data=57740；Total ROM Size=124068 字节（121.16 KiB）。最终 hex SHA256 `ddd114fa3f05af478f042bfc59114a3742809e1edff851496f7ea676819e0557`。最终 map 不含 trigger/fric/yaw/pit/DM/gimbal/shoot/hfdcan2/hfdcan3/motor_msg/CAN2/CAN3 队列或任务符号。

【构建阻塞记录】首次构建发现 TF-mini 草案 `MX_USART9_Init` 使用 H723 不存在的 USART9，已删除；UART8 草案保留但未接入构建。另修复本次 const 写入、回调声明和枚举类型警告。恢复点误建在主仓的子目录已核对后删除，原件在 `taurus_h7_all/_backup/20261125_can_cleanup/`。

【未测/下一步】新包未烧录，板上仍为 `79cf419c`；未接真实 5 电机，未用 USB-CAN 验证。6020 尚无零电流业务发送入口，下一轮先补 10Hz 的 0x1FF 零帧，再记录 5 路反馈与总线错误。禁止把本次 L0 构建写成 CAN L3/L4。

---

### 2026-11-25 20:17 — 6020 接 FDCAN2 实测：无 ACK；双舵机通信通过

【用户现场】两个总线舵机和一个 6020 已接；6020 绿灯每秒快闪4次。用户确认询问的“CAN2”是板上 FDCAN2（PB5/PB6），不是 USB-CAN 工具通道。

【灯语判读】按 6020 说明书：绿灯每1秒闪N次表示驱动器 ID=N。闪4次即 **ID=4**，映射反馈报文 **0x208**、控制报文 **0x1FF**；用户猜测 0x208 正确。该灯只证明 ID/供电，不证明 CAN 总线已通。

【代码改动（新包 28ab70b1）】
1. 恢复最小 FDCAN2：Classic CAN、1Mbit/s、PB5/PB6，只收 0x208，FIFO0；不恢复任何旧 gimbal/yaw 电机业务。
2. `dji_send_current` 按电机通道选择 CAN1/CAN2，使用局部 0x200/0x1FF 帧，保留共享缓冲区删除状态。
3. `arm_task` 每100ms给 6020 发 0x1FF 零电流，仅通信保活，不输出转矩。
4. `[CAN2] rx/tx/tec/rec/err` 加入 1Hz 状态日志；FreeRTOS 增加 CAN2 发送队列/任务。

【构建/烧录】增量构建 **0 Error / 0 Warning**；全量 `-r` 为 **0 Error / 7 Warning**（均为既有：vofa/dbg 的 `usb_tx_send` 隐式声明、status/arm_math 的 packed 警告、ws2812 末尾换行、bsp_user_key 的 #warning）。烧录使用 wireless DAP `8E74D081`，OpenOCD `halt; program verify` → **Verified OK**；纯 SWD 放行后运行日志出现 `[CAN2]`，确认运行包含本次改动。最终 hex SHA256 `28ab70b1c6701cd96b7732171d59d972c307a9bf832f7274b54852589ba02c9c`。

【CAN2 实测】连续多秒：`rx=0 tx=51 tec=248 rec=0 err=0x00000200`；`tx` 不再增长。结论：**无任何节点应答，物理层未闭环**，不是协议解析/rx 字段问题。按单变量原则，仅将 CAN2 采样点由 60%（24分频/2+2）改为 87.5%（60分频/TS1=6/TS2=1）重新编译烧录，结果完全相同，已停止继续改软件。24V、终端电阻、CANH/CANL 顺序为主板/6020侧待核项。系统未枚举到任何 USB-CAN 适配器，只有 CH343/FTDI/CH340/DAP 串口。

【舵机实测（只读，无运动）】`servo config` 后：
```
servo map             -> shoulder=0 elbow=1   （初始 RAM 映射）
servo bind shoulder 2 -> rc=0
servo bind elbow 1    -> rc=0
servo map             -> shoulder=2 elbow=1
servo ping 2          -> rc=0 #002PServo.C(2026-04-01 21:00:44)!
servo read 2          -> rc=0 pulse=1496
servo ping 1          -> rc=0 #001PServo.C(2026-04-01 21:00:44)!
servo read 1          -> rc=0 pulse=1491
servo run             -> config mode OFF
```
结论：两个总线舵机 UART1 通信正常，肩硬件ID=002、肘=001，角度回读正常。映射仅在 RAM，未持久化；本轮未发任何位置/运动指令。

【状态】CAN2 🟡 物理层阻断（L1 未达）；双舵机 ✅ L3 通信；未改底盘、未发非零电流、未 commit/push。

---

### 2026-11-25 20:20 — ⏱️ 压缩前审计（pre-compact）

【当前刀序】比赛主线：巡线/避障与机械臂示教；当前先解决 6020 的 CAN2 物理链路，之后回到片内 Flash 示教与 WS2812。出处=`Develop_Log/20261125_下午自动迭代计划.md`。

【代码/固件锚点】板上运行包 `28ab70b1c6701cd96b7732171d59d972c307a9bf832f7274b54852589ba02c9c`；回退包 `79cf419c...` 在 `taurus_h7_all/_backup/20261125_can_cleanup/build/`，恢复目录共46文件。源码锚点：fdcan `8b3639…`、can_comm `0f3d4f…`、bsp_can `cc3520…`、prot_rm_motor `97e20b…`、arm_task `2b3b24…`、status_task `8e4d8e…`。`taurus_h7_all` 不在 Git；主仓 master@e46f31d，57项工作树改动，29未推送，单 worktree。

【下一步已定】用户压缩后把 CAN1 与 CAN2 物理接起来（前两天已验证过该回环），下一会话先读 COM141 的 `[CAN2] tec/tx`：TEC 248→0 且 tx 增长即 ACK 恢复；若只做两路回环，CAN2 当前仅过滤 0x208，`rx=0` 不能单独判失败。TEC 不降只查 CAN_H/L、共地、24V、终端，不再改采样点/协议。

【Handoff】`Develop_Log/handoffs/handoff_20261125_CAN2与舵机实测.md`，已在 `handoffs/README.md` 登记。跨仓对账：rc_remote 18:37、kmo_mc_rcv 18:08、cadence 16:09 均为无线/桥/OTA 线，与本板 CAN2、UART1 舵机无共享环境变更。未 commit/push。

【本段动作枚举（13步全录）】①恢复 CAN2 最小代码与 0x208 过滤；②扩展 BSP 双通道与 FIFO0；③按电机通道发送 0x200/0x1FF；④arm_task 增加100ms零电流；⑤status_task 增加 CAN2 诊断；⑥Keil 全量构建；⑦无线DAP烧录验证；⑧纯SWD放行；⑨COM141读原始计数；⑩仅改CAN2采样点60→87.5%、重编译烧录；⑪复读计数无变化并停止；⑫舵机 config/map/bind/ping/read/run；⑬写 daily/plan/handoff。

【原始锚点与复跑】`MDK-ARM/flash_can2_6020.log`、`MDK-ARM/flash_can2_sp875.log`、`MDK-ARM/build_can2_6020_full.log`；串口原始行 `[CAN2] rx=0 tx=51 tec=248 rec=0 err=0x00000200`；舵机原始应答见上段代码块。复跑读串口：`py -I` 打开 COM141@115200，清空输入后读 ≥5s；复跑舵机只读：`servo config`→`servo ping/read`→`servo run`。CAN2 物理桥接后的判据已写入 handoff 与计划第八节。未 commit/push。


---

### 2026-11-25 21:01 — ✅ CAN2/6020 打通（根因=250kbps 位定时 + CAN_H/L 反接）

【用户口令】"6020 不是 FDCAN"＋"有能和 6020 通信的代码：达妙 demo、飞机云台 F4/H7"，并给出 `https://gitee.com/full-smile/taurus_26_-aerial_-robot.git` 与 GM6020 手册 PDF。

【参考工程核对】clone 到 `D:/ZMS_Desktop/RobotWorkspace/_ref_gimbal_6020/`。yaw=6020 在 `CAN_CHANNEL_2`，反馈 0x206 走 `User_CAN2_RxCpltCallback`，控制帧 0x1FF；FDCAN2 用 `RCC_FDCANCLKSOURCE_PLL2`=96MHz、prescaler 8、TS1=8、TS2=3 → 96/(8×12)=**1Mbps**，`AutoRetransmission=ENABLE`。

【手册核对】`_ref_manual_gm6020/gm6020_manual.pdf`：CAN 比特率 **1Mbps**；ID1–4 控制 0x1FF、ID5–7 控制 0x2FF；ID4 反馈 **0x208**；反馈 1kHz；CAN 线 A 红=CAN_H、B 黑=CAN_L；拨码第4位=终端电阻。

【代码修正（本轮唯一改动）】`Core/Src/fdcan.c` CAN2 位定时 `60/6/1`→`24/2/2`。原值 120MHz/(60×8)=**250kbps**，是把采样点改 87.5% 时误把 prescaler 从 24 改成 60 造成的；改后 120MHz/(24×5)=**1Mbps**，与 CAN1 一致。

【构建/烧录】Keil 增量 **0 Error / 0 Warning**；hex SHA256 `01d4b265b93e88711e11514809eb9a86fd73af46a4dbeccdb216f306ca543378`（与更早 1Mbps 包同哈希，互证位定时确为 24/2/2）；无线 DAP `8E74D081` OpenOCD `program verify` → **Verified OK**。

【第一次复测（H/L 未换）】仍 `[CAN2] rx=0 tx=51 tec=248 rec=0 err=0x200`，与旧 1Mbps 包一致 → 排除软件/位定时，指向 6020 侧接线。

【用户交换 CAN_H/L 后复测（决定性）】连续 ~35s：
```
[CAN2] rx=36508 tx=362 tec=0 rec=0 err=0x00000000
[CAN2] rx=48626 tx=481 tec=0 rec=0 err=0x00000000
[CAN2] rx=51179 tx=506 tec=0 rec=0 err=0x00000000
[CAN2] rx=77321 tx=762 tec=0 rec=0 err=0x00000000
```
rx 以约 1.1k/s 连续增长（6020 反馈 1kHz），`tec=0 rec=0 err=0`，tx 按 10Hz 保活稳定增长。结论：**CAN2 与 6020 双向通信闭环成立**；两个根因分别是位定时 250kbps、以及现场 CAN_H/CAN_L 与丝印相反。

【验收对照】计划原门槛"连续30秒 rx 增长且 TEC=0" ✅ 达到。尚未做：0x208 帧内编码器/温度/电流字段的数值核对（当前固件无该打印），未发非零转矩。

【Git】编译产物按用户令入库：`MDK-ARM/CtrBoard-H7_ALL/`（hex/axf/map/build_log/烧录日志）＋`history/20261125_28ab70b1_*.hex`、`history/20261125_01d4b265_*.hex`；旧包按 CRLF 精确恢复后哈希仍为 `28ab70b1…`。仅 `git add`，未 commit/push。

---

### 2026-11-25 21:40 — 用户裁决：架构重构另开会话 + 两份调研在跑

【用户原话要点】“另外一个K1就用来放四个底盘的2006，一直发500Hz应该足够了，就在底盘任务那里”；“架构上有问题的需要删减非常值得，没走队列的架构是一坨屎……从上往下，从任务看各文件，以及从BSP/中间层/业务层看，写handoff和任务书，我开新会话让模型做”。

【本会话只写文档不动手】新增 `Develop_Log/handoffs/handoff_20261125_架构重构与任务清理.md` 并在 `handoffs/README.md` 登记。含阶段0备份基线→阶段1需求到任务映射表→阶段2 CAN队列收口→阶段3删冗余→阶段4验证；验收门为 0 Error/0 Warning、无绕过队列的 `HAL_FDCAN_AddMessageToTxFifoQ`、已验证功能不回归、输出删除清单+回退路径。

【在跑调研】子代理 A：Ziegler–Nichols 临界比例度法 / relay feedback auto-tuning（Åström–Hägglund）在 GM6020 上的可行性与开源实现；子代理 B：开源大机械臂拖动示教状态机、单按钮映射、节点采样/断电保存格式。两者只读，不改代码。

【当前板上包】`51393bd4e37bb911a9021fdc2464e1396cb8342c1193341c90b1ddbe66774300`（5V常开 + 夹爪PE13/TIM1_CH3 + IMU加热关断 + yaw ECD限位 + `[HEAT]` 遥测）。未 commit/push。

---

### 2026-11-25 22:05 — 三份调研归档 + 巡线方案新标准

【用户裁决】调研结果必须保存；示教与自动调参不是可选项，必须装进去，否则车接上后不会动/不会轨迹。巡线避障方案不能由模型凭空生成“最小方案”，必须以电赛/智能车比赛完整开源工程为依据并逐文件核对；必要时让调研子代理增量调研。

【已落盘】`Develop_Log/20261125_调研归档_示教_自动调参_巡线避障.md`：三份调研完整记录（拖动示教状态机+单键映射+Flash格式+开源链接；Z-N/relay自动调参结论与开源代码；巡线避障临时方案+开源参考）。当前“巡线避障”章节明确标记为临时版，需按新标准增量调研后再定稿。

【已发增量任务】巡线避障子代理继续调研：优先电赛/智能车完整开源工程、逐文件核对偏差/PID/丢线/避障FSM，找最接近 3-5 个工程并说明可抄文件/函数；无法完全同构则明说。报告回来后追加进同一归档文件。其他调研不再重开。

【当前板上包】仍为 `51393bd4e37bb911a9021fdc2464e1396cb8342c1193341c90b1ddbe66774300`。未 commit/push。

---

### 2026-11-25 22:28 — ⏱️ 本地提交 + 今晚计划登记 + 补登 21:19~21:36 烧录清单 + 编译可复现性结论

【用户指令】"开始执行吧。当前的代码要本地提交一份登记好日志，把我们的计划登记好日志。然后，你这个模型只做最小的修改。修改完之后，你就回来停下来。"

【当前刀序】比赛主线：巡线/避障与机械臂示教（出处=`Develop_Log/20261125_下午自动迭代计划.md` §八）。今晚目标=四个关节各动一次并示教一次；CAN1 底盘诊断归架构重构会话。

【一、烧录补登】21:01 之后 4 次烧录，此前只登了 01d4b265。执行【AI】，判读依据为串口原文。

| 时间 | 包 | 唯一改动 | 动机 | 结果 |
|---|---|---|---|---|
| 21:19 | `2eb42604` | 新增 `arm_yaw_limit` 模块、`yaw monitor` 20Hz 采集、`servo_test_poll` | 采 yaw 左右极限 | ❌ 解析器要求 `servo` 前缀，`yaw monitor` 落入 usage，未采到数据 |
| 21:33 | `888c9950` | 解析器支持 `yaw` 前缀 | 同上 | ✅ `[YAW_LIMIT] ecd=7675 total=0 …`；1821 帧/112s：右极限 total=+2924（ecd 6478）停 6.4s，左极限 total=−2294（ecd 1260）停 10.2s，中间 −243（ecd 3310）；行程 5218 tick≈229° |
| 21:34 | `6ee07336` | 限位改用原始 ECD（默认 1260/6478）；5V 上电默认开；夹爪 PWM TIM2_CH3/PA2→TIM1_CH3/PE13；IMU 加热关 | 用户令：舵机上电、夹爪在 PE13、加热默认关 | ✅ CAN2 正常；用户确认舵机上电、夹爪自然关闭 |
| 21:36 | `51393bd4` | 增加 `[HEAT]` 1Hz 遥测 | 取加热关断证据 | ✅ `t=35.0→35.4 ccr4=0 cen=0 pb1=0 pc15=1`（约 26s）；**当前板上包** |

口径：`[YAW_LIMIT]` 20Hz、`[HEAT]` 1Hz，均为 COM141@115200。原始数据：`Develop_Log/raw/20261125_yaw_sweep.csv`（1821 行）。
未归档：以上 4 个 hex 均被下一次编译覆盖，磁盘无留存，只剩哈希与 Program Size；51393bd4 以 22:21 的 Flash 回读二进制保存（`MDK-ARM/CtrBoard-H7_ALL/flash_readback_20261125_board.bin`）。

【二、走错的假设与返工】
1. 假设 `total_ecd` 可作限位坐标 → 错。`offset_ecd` 每次上电重锁，total 是会话相对量。证据：采集结束到 `yaw set` 之间板子重启过一次（CAN2 rx 140914→39514，原点变为 ECD 4633）。→ 改用原始 ECD。重启原因未查。
2. `yaw monitor` 内调用 `arm_yaw_limit_init()` 把刚设的限位清零 → 已去掉。
3. 预期行程会过编码器零点 → 实测不过：3554+2924=6478、3554−2294=1260，`round` 全程 0。
4. 第一次采集 0 行（串口 0 字节，板子未在运行）→ OpenOCD 复位恢复后重采。
5. 21:56 把"比如画个长方形"当成指令，改了 `mode_switch_task.h`、`arm_task.h`、`servo_test.c` 共 3 处，未编译；22:13 全部撤回，`grep arm_rect|ARM_DEMO_MODE|is_rect_cmd` 无残留。
6. 验证回滚时直接增量编译，把磁盘上最后一份 51393bd4 hex 覆盖 → 改用 Flash 回读补救。

【三、编译可复现性（判决性实验）】
- 同一份源码三次编译：`51393bd4`（21:36 增量，板上）、`5b23fa0b`（22:14 增量，重编 15 个 .o）、`117e528e`（22:18 全量 `-r`，0 Error / 7 Warning）；Program Size 均为 `Code=106020 RO-data=20432 RW-data=1216 ZI-data=58008`。
- Flash 回读（`dump_image 0x08000000 0x20000`，未 halt）与全量包逐字节比对：126,796 字节中仅 4 字节不同，位于 `main.o(i.main)`：板上 `"20:08:17"`，全量包 `"22:18:05"`。来源 `Core/Src/main.c:259` 的 `dbg_print("… build %s %s …", __DATE__, __TIME__)`。
- 结论：✅ 板上固件与回滚后源码一致（差 4 字节编译时间戳）。hex 哈希只标识"一次构建"，不标识"一份源码"；凡重编 `main.c` 哈希必变。验证"源码=板上固件"用 Flash 回读比对。
- 本段未烧录。

【四、今晚计划（已同步到计划 §八）】
0–3min 回滚 ✅｜3–18min 四关节逐个动 ⏸｜18–45min 示教一次 ⏸｜45–50min 巡线模块回归 ⏸（当前 `[LINE] rx=0`）｜50–60min 归档 ⏸。
缺口：现有串口命令不能驱动单个关节运动（`servo_test` 仅 ping/read/setid/bind/config/run/map；`yaw` 仅 monitor/stop/set/clear），第 2 步需先加最小命令。不做：自动调参、矩形演示、CAN1 诊断、USB-CAN、架构重构。

【五、Git 锚点】
- 主仓 `Robot-major-practice`：`ce99f47d99d0764f373c35d370d1907240cd9db4`（51 文件：编译产物镜像、`.gitignore`、Develop_Log 全部改动与新增、rm_main 11-22 遗留 WIP 原样快照）。未提交：根目录 `_*.txt/_*.py/_*.png` 临时文件、`_ref_*` 参考目录。未 push（领先 origin 30 个提交）。
- 固件仓 `taurus_h7_all`：新建本地 Git（不 push），`da71dbc76f3ab0dfc9a45681827857425210e3c1`（1900 文件，约 107MiB），工作树干净。
- 构建产物：全量包 `117e528e6c375a55422ffa7be4ec8aca62900c946d1b803c64fe06227227fbdb`；Flash 回读 `93478d579c2747f0ac1c7a742a6d70490cd6c23ff519718e9f31125259f74d89`。

【六、调研】三份已归档到 `Develop_Log/20261125_调研归档_示教_自动调参_巡线避障.md`；巡线避障按"必须开源支撑"新标准做了增量，核实 5 个完整工程 + TFmini 官方驱动，无完全同构仓库。待纠正：6020 的 0x1FF 是电压帧、0x1FE 才是电流帧；M2006 配 C610 不是 C620。

【七、风险】IMU 加热永久关 → 陀螺零偏随温度漂；TIM1 被 `tim.c` 与 `bsp_pwm.c` 初始化两次；遥控器解锁后 `arm_task` 的肩肘演示脉冲（1333/1700/1500/2000）未在当前总线舵机上验证；CAN1 未收到过真实数据；巡线模块 `rx=0`。
