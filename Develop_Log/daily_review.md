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
