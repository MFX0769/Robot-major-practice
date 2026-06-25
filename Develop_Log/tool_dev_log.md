# 工具开发日志

记录项目相关工具的开发、配置和使用情况。

---

## 2026-06-22 nanoDLA 逻辑分析仪集成

### 工作内容

1. **克隆nanoDLA仓库**
   - 位置: `Source/nanoDLA`
   - 来源: https://github.com/wuxx/nanoDLA
   - 说明: 开源逻辑分析仪，最高24MHz采样率，8通道

2. **克隆固件源码**
   - 位置: `Source/sigrok-firmware-fx2lafw`
   - 来源: https://github.com/wuxx/sigrok-firmware-fx2lafw
   - 固件分析: `Source/sigrok-firmware-fx2lafw/develop_log/firmware_analysis.md`

3. **创建LA自动化工具**
   - 位置: `Tool/nanoDLA/`
   - 文件:
     - `la_reader.py` - 核心读取模块（标准化接口）
     - `requirements.txt` - 依赖
     - `README.md` - 使用说明

### 设备信息

| 参数 | 值 |
|------|-----|
| USB VID | 0x1D50 |
| USB PID | 0x608C |
| 通道数 | 8 (PB0-PB7) |
| 最高采样率 | 24MHz |
| 驱动 | WinUSB (Zadig安装) |

### 标准化接口

```python
from la_reader import capture, load, analyze

# 采集并保存
result = capture(sample_rate=1_000_000, duration_ms=100)

# 加载分析
result = load("data/capture_xxx.json")
stats = analyze(result)
```

### 待办

- [ ] 测试实际设备连接
- [ ] 验证采样数据正确性
- [ ] 集成到主控程序的自动调试流程

---

## 2026-06-22 sct文件分析（DMA优化）

### 提交历史

| 提交 | 日期 | 说明 |
|------|------|------|
| c2de50d | 2025-11-21 | 清理编译中间文件 |
| acfc92d | 2025-11-21 | can的配置 |
| f7b4ca1 | 2026-04-07 | 修改sct改变dma_buff存储地址 |

### 关键变更 (acfc92d → f7b4ca1)

**问题**: 串口DMA传输死机

**原因**: DTCM (0x20000000) 不支持DMA传输，usart_comm的缓冲区默认分配到DTCM导致DMA访问失败

**解决方案**: 修改scatter loading文件，将DMA缓冲区强制分配到AXI SRAM

### 内存布局对比

| 区域 | 旧版本 | 新版本 | 说明 |
|------|--------|--------|------|
| Flash | 1MB @ 0x08000000 | 不变 | 代码存储 |
| DTCM | 128KB @ 0x20000000 | 不变 | 普通全局变量 |
| AXI SRAM | 128KB @ 0x24000000 | **512KB** | DMA缓冲区 |

### 新版本sct关键点

```
RW_IRAM2 0x24000000 0x00080000 {   ; 扩大为 512KB
   ; 关键：usart_comm.o 的变量优先分配（含 dma_*_buf）
   usart_comm.o (+RW +ZI)
   
   ; 其他需要DMA的文件（Protocol 的结构体）
   .ANY (+RW +ZI)
}
```

### 教训

1. **STM32H7的DTCM不支持DMA** - 必须将DMA缓冲区放到AXI SRAM
2. **scatter loading可以精确控制变量分配** - 通过 `.o (+RW +ZI)` 语法
3. **调试串口问题时检查内存区域** - DMA死机很可能是内存区域错误

---

## 2026-06-24 USART6调试串口配置

### 改动记录

| 文件 | 改动内容 | 状态 |
|------|---------|------|
| `Core/Src/usart.c:234` | 波特率: 115200 → 921600 | ✅ 已完成 |
| `Core/Src/usart.c:233-235` | 添加TODO注释 | ✅ 已完成 |
| `Application/config.h` | 新建调试模式开关（VOFA/文本互斥） | ✅ 已完成 |
| `Application/debug_task.c` | 加config.h宏 + VOFA条件编译 | ✅ 已完成 |

### 配置确认

| 参数 | 值 | 说明 |
|------|---|------|
| 串口 | USART6 (烧录器虚拟串口) | 15cm无线连接 |
| 波特率 | 921600 | 从115200提升，支持VOFA+ |
| 模式 | UART_MODE_TX | 只发送，不接收 |
| FIFO | Disable | 轮询发送，无DMA |
| 发送方式 | HAL_UART_Transmit() | 阻塞轮询，超时10ms |

### 验证待办

- [ ] 921600波特率下无线15cm距离稳定性测试
- [ ] VOFA+ 8通道波形传输流畅度验证
- [ ] 是否有丢包（通过检查数据完整性）
- [ ] 如遇问题，考虑启用TX FIFO（删除DisableFifoMode调用）

### 风险评估

- **H7 FIFO坑**: 已禁用FIFO，避免H7 FIFO已知问题
- **无DMA**: 轮询发送，高波特率下可能阻塞主循环
- **无线环境**: 15cm距离应该稳定，但需实际测试

---

## 2026-06-22 机械臂代码框架搭建

### 共识确认

根据 Develop_Log 中的关键文档，确认以下方案：

| 项目 | 确认方案 |
|------|----------|
| 构型 | 4轴串联（yaw + 肩 + 肘 + 夹爪） |
| 舵机 | **PWM舵机**，非总线/串口舵机 |
| IK | 4轴几何闭式解 |
| 平滑 | 逆解→限位→步进平滑→PWM |

### 创建的文件

| 文件 | 说明 |
|------|------|
| `Protocol/prot_servo.h/c` | PWM舵机驱动（角度→脉宽映射、限幅、方向反转） |
| `Bsp/bsp_pwm.h/c` | PWM定时器输出封装 |
| `Bsp/bsp_gpio.h/c` | 舵机供电使能/失能控制 |
| `Application/arm_ik.h/c` | 逆运动学（几何法）+ 关节限位 + 平滑 |
| `Application/arm_task.h/c` | 机械臂主任务（状态机 + 视觉输入） |

### 接口分层

```
Application/arm_task.c  ← 状态机、流程编排、视觉输入
Application/arm_ik.c    ← 逆解、限位、平滑、目标修正
Protocol/prot_servo.c   ← 角度到脉宽映射、脉宽限幅、方向反转、目标下发
Bsp/bsp_pwm.c           ← PWM输出封装
Bsp/bsp_gpio.c          ← 舵机供电使能/失能
```

### 标定变量

舵机参数在 `prot_servo.c` 中定义，可在Keil Debug中修改：
- `pulse_min / pulse_max / pulse_zero` — 脉宽范围
- `direction` — 方向（1或-1）
- `angle_min / angle_max` — 角度范围

### 待办

- [ ] 根据实际硬件修改引脚和定时器配置
- [ ] 量测实际臂长（L1, L2）
- [ ] 单关节标定（Keil Debug修改变量）
- [ ] CAN ID分配（yaw用GM6020）
- [ ] 通信烟雾测试（UART、CAN、PWM）

---

## 2026-06-22 STM32H7 调试工具链配置

### 工具链组成

| 组件 | 用途 | 状态 |
|------|------|------|
| Keil MDK-ARM | 编译固件 | ✅ 已有 |
| OpenOCD 0.12.0 | 烧录+调试服务器 | ✅ 已安装 |
| Cortex-Debug (VS Code) | 调试 GUI | ✅ 已安装 |
| CMSIS-DAP | 调试器硬件接口 | ✅ 已有 |

### 安装记录

**OpenOCD 安装**
- 版本: xPack OpenOCD 0.12.0+dev (2025-02-05)
- 路径: `C:\OpenOCD\xpack-openocd-0.12.0-5\bin\openocd.exe`
- 下载源: https://github.com/xpack-dev-tools/openocd-xpack/releases

**VS Code 插件**
```
marus25.cortex-debug    # 调试主界面
mcu-debug.memory-view   # 内存查看器
mcu-debug.peripheral-viewer  # 外设寄存器查看
mcu-debug.rtos-views    # FreeRTOS 任务/队列可视化
mcu-debug.debug-tracker-vscode  # 调试跟踪
arm.vscode-cmsis-debugger  # CMSIS 调试器支持
```

### 配置文件

**OpenOCD 配置** (`Tool/debug/stm32h7.cfg`)
```
source [find interface/cmsis-dap.cfg]
adapter speed 4000
source [find target/stm32h7x.cfg]
```

**VS Code launch.json** (`.vscode/launch.json`)
```json
{
    "name": "Cortex-Debug: OpenOCD",
    "type": "cortex-debug",
    "servertype": "openocd",
    "executable": "${workspaceFolder}/MDK-ARM/rm_main/rm_main.axf",
    "device": "STM32H723VGTx",
    "configFiles": ["${workspaceFolder}/Tool/debug/stm32h7.cfg"],
    "runToEntryPoint": "main"
}
```

**VS Code settings.json** 添加
```json
{
    "cortex-debug.openocdPath": "C:\\OpenOCD\\xpack-openocd-0.12.0-5\\bin\\openocd.exe"
}
```

### 自动化脚本

**文件**: `Tool/debug/openocd_controller.py`
- 通过 Telnet (端口4444) 控制 OpenOCD
- 支持命令: reset, reg, mem, bp, continue, halt, flash

**文件**: `Tool/debug/stm32h7_tool.py`
- 整合编译+烧录+调试的完整流程
- 用法: `python stm32h7_tool.py build/flash/debug/full`

### 调试工作流

1. **启动调试**: VS Code 按 F5 → 选择 `Cortex-Debug: OpenOCD`
2. **AI 辅助**: OpenOCD 启动后，通过 Telnet (4444) 可远程控制
3. **GUI 观看**: 变量、寄存器、内存、RTOS 状态在 VS Code 中实时显示

### 调试器连接

| 调试器 | 接口 | 驱动 |
|--------|------|------|
| CMSIS-DAP | SWD | WinUSB (Zadig安装) |
| ST-Link | SWD | ST-Link驱动 |

### 待办

- [ ] 安装 CMSIS-DAP 的 WinUSB 驱动 (Zadig)
- [ ] 测试实际烧录
- [ ] 验证 RTOS Views 显示 FreeRTOS 任务
- [ ] 创建常用调试命令脚本

---

## 2026-06-22 无线调试器集成

### 硬件信息

| 参数 | 值 |
|------|-----|
| 型号 | 装甲版无线仿真器 |
| VID | 0xEF1A |
| PID | 0x74E5 |
| COM端口 | COM23 (HOST端) |
| 接口 | SWD + 无线串口 |
| 传输距离 | ≤30米 |

### 无线参数 (从DAP上位机提取)

| 参数 | 说明 | 范围 |
|------|------|------|
| Frequence | 无线频率 | 2400-2480 MHz |
| TX Power | 发射功率 | dB可调 |
| Sync Word | 同步字(配对) | 5字节Hex |
| Rate | 数据速率 | 可选 |
| Mode | 工作模式 | Central/Peripheral |
| TX Timeout | 发送超时 | ms |
| RX Timeout | 接收超时 | ms |
| Max Resend | 最大重发次数 | cnt |
| CDC After DAP | CDC延迟 | 0.1ms |
| DAP/CDC Timeout | DAP超时 | ms |

### 串口说明

| 串口 | 方向 | 功能 |
|------|------|------|
| VCP1 | STM32→PC | 调试串口（接收端）- 读取debug输出 |
| VCP2 | PC→STM32 | 透传串口（发送端）- 发送数据给H7 |

### 文件

| 文件 | 说明 |
|------|------|
| `Tool/debug/wireless_debug.py` | 无线串口独立工具 |
| `Tool/debug/stm32h7_tool.py` | 集成无线功能的主工具 |

### 使用方法

```bash
# 检测无线调试器
python wireless_debug.py detect

# 监听STM32输出
python wireless_debug.py monitor

# 发送数据给STM32
python wireless_debug.py send "Hello H7"

# 使用主工具
python stm32h7_tool.py wireless-monitor
python stm32h7_tool.py wireless-send "AI->STM32"
python stm32h7_tool.py wireless-test
```

### 集成到AI工作流

AI可以通过以下方式与STM32通信：

1. **发送指令**: 通过COM23向STM32发送控制命令
2. **读取输出**: 监听COM23接收STM32的debug打印
3. **自动测试**: 结合编译→烧录→无线收发的完整流程

### 测试结果

- COM23连接: ✅ 正常
- TX发送: ✅ 成功
- RX接收: ⚠️ 待STM32固件配合

### 待办

- [ ] 确认VCP1/VCP2的COM口分配
- [ ] 在STM32固件中添加串口回环测试
- [ ] 验证双向通信
- [ ] 创建AI自动调试脚本

---

## 2026-06-22 CubeMX配置 + 编译验证

### CubeMX配置 (TIM2 PWM)

在 `rm_main.ioc` 中添加：

| 外设 | 配置 |
|------|------|
| TIM2 | PWM Generation CH1/CH2/CH3 |
| PA0 | TIM2_CH1 (肩关节PWM) |
| PA1 | TIM2_CH2 (肘关节PWM) |
| PB0 | TIM2_CH3 (夹爪PWM) |
| PB1 | GPIO_Output (肩关节供电使能) |
| PB2 | GPIO_Output (肘关节供电使能) |
| PB3 | GPIO_Output (夹爪供电使能) |

**定时器参数**：
- Prescaler: 83 (84MHz/84 = 1MHz)
- Period: 19999 (1MHz/20000 = 50Hz, 20ms)
- Default Pulse: 1500us (中位)

### 代码修改

| 文件 | 改动 |
|------|------|
| `Core/Src/main.c` | 添加 htim2 + MX_TIM2_Init() |
| `Core/Inc/main.h` | 声明 extern TIM_HandleTypeDef htim2 |
| `Bsp/bsp_pwm.c` | TIM1→TIM2，使用全局 htim2 |
| `Bsp/bsp_gpio.h` | 引脚改为 PB1/2/3 |

### 编译验证

```
Keil5路径: D:\Users\Tommy0769\AppData\Local\Keil_v5\UV4\UV4.exe
编译命令: & "UV4.exe" -r "MDK-ARM\rm_main.uvprojx" -j0 -o "MDK-ARM\build.log"

编译结果:
Program Size: Code=85124 RO-data=2860 RW-data=1116 ZI-data=40660  
"rm_main\rm_main.axf" - 0 Error(s), 9 Warning(s).
Build Time Elapsed: 00:03:01
```

- ✅ 编译通过 (0 Error)
- ⚠️ 9个警告 (无害的 __packed 和类型转换警告)

### 下一步

- [ ] 在Keil中打开工程验证TIM2配置
- [ ] 烧录到板子测试PWM输出
- [ ] 用逻辑分析仪抓取PWM波形
- [ ] 标定舵机脉宽范围

---

## 2026-06-22 底盘电机全链路分析

### 旧代码参考 (RoboticMajor_Pratice_code)

**CAN ID定义**：
| 电机 | CAN ID | CAN总线 | 位置 |
|------|--------|---------|------|
| chassis_2006[0] | 0x204 | CAN1 | 左前 |
| chassis_2006[1] | 0x202 | CAN1 | 左后 |
| chassis_2006[2] | 0x203 | CAN2 | 右前 |
| chassis_2006[3] | 0x201 | CAN2 | 右后 |

**电机打包规则** (drv_dji_motor.c)：
```
CAN ID 0x201~0x204 → 控制帧 0x200
  data[0:1] = motor[0].tx_current
  data[2:3] = motor[1].tx_current
  data[4:5] = motor[2].tx_current
  data[6:7] = motor[3].tx_current
```

**全链路**：
1. `prot_rm_motor.c`: 声明 `chassis_2006[4]`
2. `can_comm.c`: 初始化 + 接收回调 + 发送
3. `chassis_task.c`: 运动学解算 + PID → tx_current

### 当前项目状态

- ✅ CAN队列架构已有 (CAN1_Tx_queue, CAN2_Tx_queue)
- ✅ 底盘电机声明已添加 (prot_rm_motor.c)
- ✅ 底盘初始化已添加 (can_comm.c)
- ✅ 底盘接收回调已添加 (can_comm.c)
- ✅ chassis_task.c 已创建
- ✅ 编译通过 (0 Error)
- ✅ 烧录成功

### 视觉限幅 (临时)

- 相机分辨率: 320x240
- 限幅: X ±160, Y ±120 (像素差)
- 后续与视觉联调时再调整

---

## 2026-06-24 机械臂任务代码完善

### 改动文件清单

| 文件 | 改动 | 状态 |
|------|------|------|
| `Application/arm_task.h` | 新增ARM_DBG宏、平滑模式枚举、yaw PID结构体 | ✅ |
| `Application/arm_task.c` | 完整重写：yaw ECD闭环PID、视觉xy→方向、调试输出 | ✅ |
| `Application/arm_ik.h` | 新增arm_ik_self_test声明 | ✅ |
| `Application/arm_ik.c` | 新增IK自检函数(正逆往返验证) | ✅ |
| `Application/config.h` | VOFA/文本打印互斥开关 | ✅ |
| `Application/debug_task.c` | config.h引入 + VOFA条件编译 | ✅ |
| `Core/Src/usart.c` | USART6波特率115200→921600 | ✅ |
| `Protocol/prot_servo.c/h` | 新增调试函数 | ✅ |
| `Develop_Log/串口指令协议TODO.md` | 新建，遥控器串口指令方案 | ✅ |

### 核心功能

**1. Yaw ECD闭环PID (参考gimbal_task.c)**
```
位置环: fdb=ecd→角度, Kp=40, Kd=5, MaxOutput=500
    ↓
速度环: fdb=speed_rpm, Kp=198, Kd=13, MaxOutput=25000
    ↓
电流限幅: ±5000 (ARM_YAW_CURRENT_LIMIT)
```

**2. 视觉xy→yaw方向**
```c
pix_x = (int16_t)vision_fruit.x_dis;
pix_y = (int16_t)vision_fruit.y_dis;
yaw_dir = atan2f(pix_y, pix_x) * 180/PI;
```

**3. 调试系统**
- ARM_DBG宏统一管理，config.h控制VOFA/文本互斥
- 启动时打印初始化信息
- 每200周期(600ms)打印状态+yaw PID参数

**4. 平滑模式**
- ARM_SMOOTH_DEBUG: 0.01°/周期 (每秒3.3°, 调试用)
- ARM_SMOOTH_NORMAL: 1.0°/周期 (每秒333°, 正常)
- 运行时可切换: `arm_set_smooth_mode()`

**5. IK自检**
- 启动时自动运行 (ARM_IK_SELFTEST_EN=1)
- 6个测试点往返验证，误差<1mm算通过

### 待办

- [ ] 视觉x_dis/y_dis类型改为int16_t (目前uint16_t有符号问题)
- [ ] PID参数需要实际调参 (目前置零)
- [ ] 视觉正反方向验证 (手动移动目标看输出)
- [ ] IK自检结果验证 (启动后看USART6输出)
- [ ] 后续接入距离数据用IK计算完整目标

### 与另一终端的分工

| 终端 | 负责范围 |
|------|---------|
| 本终端(MiMo) | arm_task, arm_ik, prot_servo, bsp_pwm, bsp_gpio, config.h |
| 另一终端 | 串口通信, CAN通信, prot_sbus, prot_8irtraking, prot_vision, freertos |

**冲突文件**: prot_vision.c/h, rm_main.uvprojx — 留到最后合并

---
