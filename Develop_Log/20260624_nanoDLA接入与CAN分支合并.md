# 2026-06-24 nanoDLA逻辑分析仪接入 + CAN分支合并

## nanoDLA 逻辑分析仪
- 硬件：fx2lafw固件，USB ID 1D50:608C，初始 Windows 报 Problem Code 28（无驱动）
- 用 PulseView 自带的 Zadig 工具绑定 WinUSB 驱动，`Status: OK, Service: WinUSB`
- `Tool/nanoDLA/la_reader.py`（AI写的pyusb封装）有两个真实bug，已修：
  1. `CMD_START` 控制传输payload多塞了一个字节（把bRequest本身又写进了data里），改成 `bytes([flags, delay_h, delay_l])`
  2. `raw_data` 解包前没reshape成二维，`np.unpackbits(..., axis=1, ...)` 报 AxisError，加 `.reshape(-1, 1)`
- pyusb 在 Windows 上找不到 libusb backend，需要显式 `backend=libusb_package.get_libusb1_backend()`
- 修完后实测采集成功：10000采样点×8通道，未接线时全部稳定高电平（toggle_count=0），验证采集链路打通
- CAN总线测试方案讨论：差分线不能直接用数字LA测（CANH常年在阈值之上、CANL在阈值附近抖动），需要外接收发器模块（如TJA1050）取RXD单端信号，且要拆掉模块自带的120Ω终端电阻（避免给总线多加一个终端点）；不能直接给CAN_H/CAN_L接地，会在总线工作时短路其他节点的驱动级

## D:\robot_practice 分支状态核查与修复
另一个AI终端在 `D:\robot_practice` 独立做了CAN栈+UART+任务恢复，期间发生过一次 `git stash` 失误导致11个文件改动永久丢失的事故。核查交接文档时发现：
- 交接文档引用的两份文件（代码恢复计划、代码丢失与恢复指南）当时只在 master 有，dev-ci 没有
- 分支状态描述跟实际不一致（写的是detached HEAD，实际在dev-ci分支）
- 第一轮"修复"把交接文档写出了编码损坏（大量乱码），且只改了部分矛盾描述
- 第二轮修复后核实通过：文档编码修好，三处分支描述统一，`rm_main.lnp`用`git rm --cached`真正取消跟踪（只加.gitignore对已跟踪文件无效）

## F:\ 与 D:\ 两个本地仓库的CAN合并
两边是同一个Gitee仓库的独立克隆，共同起点 `8c90f85`，都没推送到远程。各自独立做了CAN电机恢复，但方案不同：
- F:\ 未提交版本：DJI_2006_MOTOR，4电机分CAN1/CAN2两路，自定义ID
- D:\ `5c3384a`：C610电调，4电机全部CAN1，ID 0x201~0x204，已用硬件验证过

决定采用 D:\ 验证过的方案。Cherry-pick 因本地文件冲突失败，改用直接覆盖4个文件内容（`git show 5c3384a:<path> > <path>`）：
- `Comm/can_comm.c` / `can_comm.h`
- `Protocol/prot_rm_motor.c` / `prot_rm_motor.h`
关键修复随之带入：CAN ID 用 `uint8_t` 存储导致 0x201~0x204 被截断、全部错误路由到 0x201，改成 `uint32_t`。

## 编译修复
合并后编译报错，排查后确认与CAN合并无关，是Keil工程文件本身的遗留问题：
- `MDK-ARM/rm_main.uvprojx` 里 `arm_task.c/h`、`arm_ik.c/h` 在两个不同的 `<Group>` 里各重复出现一次，链接时 `multiply defined`。删掉重复的Application组里的4个`<File>`条目，只留`control_def.h`
- `arm_ik.c` 调用 `ARM_DBG` 宏但没 `#include "arm_task.h"`（宏定义在那里），导致编译器当成隐式函数声明，链接找不到符号。加上include
- `Protocol/prot_vision.c` 调用 `arm_vision_callback()` 但全项目没有实现（视觉联调本来就没接完，x/y/z坐标算出来后没传给任何地方）。加了个空函数占位解决链接，**没有实现真实逻辑**，TODO标注清楚

## 烧录验证
编译0错误后烧录，通过GDB读取 `chassis_2006[0..3]`：
- CAN ID 确认 0x201/0x202/0x203/0x204 各自独立，无截断
- 两次读取间隔5秒，4路 `receive_cnt` 增量几乎一致（约20700~20800），证明CAN总线健康，四电机同时正常接收，无掉线无卡死

## 已知遗留（未处理，仅记录）
- 机械臂视觉联调（`arm_vision_callback`真实逻辑）未实现
- F:\ 工作区仍有大量跟本次工作无关的未提交改动（Develop_Log删除、机械臂相关文件等），未触碰
- master分支（D:\侧）原有3个linker error未跟进确认是否已随这次合并解决

## 底盘速度闭环现状排查
检查 `chassis_task.c` 发现两处都还没接上：
1. `chassis_motor_pid[4]` 已声明+初始化，但运动学解算出来的 `chassis_motor.motor[i]` 直接赋给 `tx_current`，没有经过 `PID_calc`
2. 更基础的问题：所有控制模式下 `vx_ref/vy_ref/wz_ref` 都硬编码0（`REMOTER_MODE`等都写的是"TODO: 接入遥控器控制"），运动学解算结果永远是0，电机永远不会转

参考了 `F:\机器人工程实践小车\RoboticMajor_Pratice_code` 的 `chassis_task.c`，发现那份代码同样没有把 `chassis_motor_pid` 接入 `pid_calc`（同款遗留问题），但提供了`vx_ref = rc.ch3 * CHASSIS_SCALE_VX`这种遥控接入的可参考写法（用的是旧DR16协议的`rc`结构体，本项目要换成`wbus_rc.data.ch1/ch2/ch3`）。

## SBUS解析器bug修复
`Protocol/prot_sbus.c` 的 `sbus_parse_frame()` 把SBUS帧解码到本地数组 `ch[16]` 后，从未写回 `rc_frame->data.ch1~ch16`（联合体`wbus_frame_u`里`data`和`buf`共享内存，但SBUS原始11bit紧密打包格式跟`wbus_rc_data`的16bit对齐字段完全不是一回事，不能靠memcpy免解码——验证了用户提出的"是不是不用解码"的猜测，结论是除非改成精确匹配位排布的位域结构体，否则必须解码）。`wbus_rx_cnt`/`wbus_rx_freq`也只有声明初始化，没有递增/计算代码。

修复：解码完`ch[16]`之后，统一写回`rc_frame->data.ch1~ch16`（沿用`-1024`居中惯例，参考`RoboticMajor_Pratice_code/Math/remote_msg.c`里`rc_callback_handler`的同款做法），`sw1`拨杆状态改用刚解出来的`ch[4]`判断（原来的注释代码读的是可能过期的`rc_frame->data.ch5`），并加上`wbus_rx_cnt++`和基于`osKernelSysTick()`的频率计算。编译0错误0警告。

## 电源拓扑现场验证（舵机暂未接入）
用户实测：
- H7板载GND vs 外置降压模块输出GND：0V差，确认共地
- 外置降压GND vs 板载24→5V的5V轨：5V差，确认板载5V轨正常
- 外置降压模块带载电流约4mA（远低于文档里"PD30S空载~100mA"的估计，但没有触发降压模块的短路保护，反过来印证了文档里描述的NMOS接法（D极接电源正极、S极接GND，看起来像会短路）大概率没有照那样实接，否则不会是稳定4mA）
结论：信号线（PWM）接入条件已具备（共地确认，3.3V逻辑电平接5V舵机是标准做法），但用户决定先不接，舵机断电搁置。

## 烧录验证发现新异常
刷入SBUS修复后用GDB读取：
- **SBUS/USART1**：`wbus_rx_cnt`持续为0，所有通道值为0，等待9秒以上确认不是瞬时未捕获——当前完全没有收到SBUS帧，需检查遥控器/接收机状态（开头提到接收机指示灯是紫色，可能没有进入正确的绑定/SBUS模式）
- **CAN**：`chassis_2006[0/1/2]`（左前/左后/右前）健康，receive_cnt同步增长（约87000+），`chassis_2006[3]`（右后, 0x201）`receive_cnt=0`、`online=0`，完全离线。这台之前（合并验证那轮）四路全部正常，期间用户在做电源拓扑测量、可能碰动了线，需要物理检查右后电机CAN线/供电连接

## CAN异常排查的真实原因：BTB/GH1.25端子接触不良
继续排查右后电机离线问题时，发现情况逐步恶化：拔掉底盘电机分线板后IMU也归零，重插BTB连接器后CAN1+CAN2全部归零，GDB读到`pc=0x00000000`且OpenOCD报"not halted/context restore failed"——确认板子在调试过程中实际断电了。

排查到最后，用户确认：没有改任何线路接法，期间只反复插拔了BTB连接器（GH1.25规格）。怀疑其中某个GH1.25端子接触不可靠，插拔导致整板短暂掉电、CAN收发中断——但**这只是怀疑，没有100%确认是这一个端子的问题**，没有做过单独隔离测试（比如固定不动连续跑几分钟看是否还会无故掉电），不能排除是别的原因（其他连接点、线材本身、供电瞬态等）。可以确定的是：不是接线接法错误，不是代码问题。

最终重新插好后实测：CAN1（IMU+4个底盘电机）连续两次GDB读取（间隔5秒）全部同步爬升，5秒增量约18600~28000，确认当前是稳定的。CAN2当时未接（用户确认，不是故障）。

**待办**：怀疑GH1.25某端子接触不良导致瞬断电，但未确认。如果以后还出现同样"CAN/IMU突然全部归零"的情况，优先怀疑供电连接器接触问题，但也要留意排查其他可能性，不要先入为主。

## U1/SBUS排查记录
CAN1稳定之后单独查U1：`wbus_rx_cnt`和所有`wbus_rc.data.chN`持续为0，多次间隔读取（含接收机指示灯由紫变绿之后）依然全部是0，没有任何变化。
排查过程中用户又一次重新插拔了BTB连接器（这次是冲着U1/SBUS这条线路去查的）。插拔后重新读取，`wbus_rx_cnt`依然是0，排除了连接器问题，问题在代码里。

## SBUS真正的根因：DMA缓冲区长度配置错误（纯软件bug）
顺着"波特率/DMA是否配好"这个思路查代码（用户提问"115200？串口dma都注册号了？"带出来的），发现`Comm/usart_comm.h`里：
```c
#define DMA_DBUS_LEN  18
```
USART1波特率/校验/数据位配置本身是对的（100000, 9bit=8data+偶校验，符合SBUS协议要求），不是这个问题。
但SBUS一帧是25字节(`SBUS_FRAME_SIZE`)，DMA缓冲区只给了18字节。链路：DMA配置成只收18字节（非循环模式，收满就停），每帧后7个字节根本没被接住；IDLE中断触发后调用`sbus_callback_handler(dma_dbus_buf, DMA_DBUS_LEN)`传长度18，函数第一行`if (len < SBUS_FRAME_SIZE) return;`即`18<25`恒真，**每次调用直接返回，从未真正解析过一帧**。这就是`wbus_rx_cnt`和所有通道值永远为0的真正原因，跟BTB连接器、波特率都无关。

**修复**：没有改名`DMA_DBUS_LEN`/`dma_dbus_buf`（避免和D:\侧命名冲突——D:\的`DMA_DBUS_LEN=18`对应的是真正DBUS协议，rc_callback_handler，跟这边语义不同，不能混用同一个名字）。改为新增一组独立常量/缓冲区：
- `DMA_DBUS_LEN=18`（保留，给真DBUS协议用，跟D:\一致）
- 新增`DMA_SBUS_LEN=36`、`dma_sbus_buf[DMA_SBUS_LEN]`（SBUS实际用这个，36留余量，IDLE中断会提前abort，不需要精确等于25）
- 新增宏`SBUS_HUART`（=huart1，跟`DBUS_HUART`同一个物理口的另一个别名），`USER_UART_Init()`里SBUS初始化改用`SBUS_HUART`/`dma_sbus_buf`/`DMA_SBUS_LEN`，不再用DBUS命名的那一套
- 涉及文件：`Comm/usart_comm.h`、`Comm/usart_comm.c`

修复后烧录验证：`wbus_rx_cnt`从1240涨到1743（5秒+503，约100Hz，符合SBUS正常帧率），CAN1（imucnt/chassis_2006[0].receive_cnt）同步健康无回归。ch1/ch2/ch3读到0是因为遥控摇杆当时在中位（解码用`-1024`居中，中位摇杆算出来正好是0），不是bug。

## DEBUG_TEXT_ENABLE / DEBUG_VOFA_ENABLE 互斥修复 + VOFA加SBUS通道
`Application/config.h`注释说`DEBUG_TEXT_ENABLE`(dbg_print文本)和`DEBUG_VOFA_ENABLE`(VOFA波形)共用USART6二选一，但实际上这个互斥只在`#error`编译检查里生效——`dbg_print()`调用本身（main.c开机横幅、chassis_task.c周期打印）是无条件执行的，没有被这两个宏门控，两路输出会在物理线上打架。

修复：给所有`dbg_print`/`dbg_init`调用点加`#if DEBUG_TEXT_ENABLE ... #endif`门控（`Core/Src/main.c`、`Application/chassis_task.c`，两处都补了`#include "config.h"`），现在两个模式真正互斥。

然后把`DEBUG_TEXT_ENABLE`改成0，`DEBUG_VOFA_ENABLE`改成1，在`Application/debug_task.c`新增`case 11`：参考D:\侧已有的case 11(只有`chassis_2006[0..3].ecd`)，在此基础上加上4路SBUS通道(`wbus_rc.data.ch1~ch4`)，`debug_wave`默认值改成11。新增`#include "prot_sbus.h"`和本地`extern dji_motor_t chassis_2006[4];`(项目里chassis_2006没有统一头文件声明，各文件都是各自extern，沿用此惯例)。

编译0错误0警告，烧录验证：`wbus_rx_cnt`/`imucnt`/`chassis_2006[0].receive_cnt`全部按正常速率增长，`debug_wave`读出来确认是11。VOFA这条链路理论上已通，待用户实际用VOFA软件连USART6确认波形显示正常。

USART6(调试口)波特率确认为**921600**(不是115200)，`Core/Src/usart.c`里`MX_USART6_UART_Init`留了TODO注释"验证921600波特率稳定性，无线15cm距离下测试VOFA+波形传输"——这个高波特率本来就是配合VOFA+无线调试设计的，但稳定性从未实测验证过，如果VOFA波形丢包/乱码要回头看这条。

又加了`case 12`：专门看SBUS遥控器，`ch1~ch8` + `sw1` + `wbus_rx_cnt` + `wbus_rx_freq`，`debug_wave`改成12。编译0/0，重烧（第一次"failed erasing sectors 0 to 0"没看到wrote确认行，怀疑没写进去，重启OpenOCD后重烧成功，wrote 89408 bytes），验证：`wbus_rx_cnt`1094→1611(5秒+517≈103Hz)，`imucnt`/`chassis_2006[0].receive_cnt`同步健康，`debug_wave`确认是12。

**注意**：这一轮(case11/case12)两次flash都是先遇到"failed erasing sectors 0 to 0"且无wrote确认行，必须重启OpenOCD重试才成功写入——同一现象重复出现两次，值得记一下，以后看到这个报错不要假设烧录成功，必须用GDB读关键变量确认，不能只看telnet的输出表面正常。

## mode_switch_task发现的真问题：SBUS是个孤岛，没有任何路径能驱动ctrl_mode
检查`mode_switch_task.c`发现：主循环只有`if(remote_fusion.current_source==REMOTE_SOURCE_VTM)...else if(...==REMOTE_SOURCE_DR16)...`两个分支，`current_source`默认值是`REMOTE_SOURCE_NONE`，且全项目搜索`prot_remote_fusion.c/.h`没有任何SBUS相关代码——SBUS数据再正常，也没有路径能把`ctrl_mode`从初始的`PROTECT_MODE`切走。同时`chassis_task.c`里`REMOTER_MODE`分支本身也是空的(`vx_ref=vy_ref=wz_ref=0.0f`硬编码)，两边都要接。

用户明确表示"有一点点破坏性也没关系，这代码就用这两天"，要求直接参考老代码(`RoboticMajor_Pratice_code`)接入，不用顾虑remote_fusion那套复杂逻辑。

**改动**：
- `Application/control_def.h`：新增`CHASSIS_SCALE_VX/VY/WZ`(0.0020f/0.0020f/0.0030f)、`MECANUM_L`(1.0f)，跟老代码完全一致的数值
- `Application/mode_switch_task.c`：在主循环最前面加一个`if(wbus_rx_cnt > 0)`分支，绕开remote_fusion，直接用`wbus_rc.data.sw1`(0=下/1=中/2=上)映射`ctrl_mode`：上→`REMOTER_MODE`，中→`PROTECT_MODE`，下→`KEYBOARD_MODE`。原有VTM/DR16逻辑原样保留作为else分支（没删，只是现在用不上）
- `Application/chassis_task.c`：`REMOTER_MODE`分支改成`vx_ref=wbus_rc.data.ch3*CHASSIS_SCALE_VX; vy_ref=wbus_rc.data.ch2*CHASSIS_SCALE_VY; wz_ref=wbus_rc.data.ch1*CHASSIS_SCALE_WZ;`，跟老代码`rc.ch3/ch2/ch1`映射一致；`chassis_mecanum_resolve`最后一个参数从硬编码`1.0f`改成`MECANUM_L`

编译0错误(7个警告均为改动前已存在的无关警告：print_cnt未使用、arm_task的gpio隐式声明、vision的z未使用、gimbal单精度转双精度)。烧录(第一次又是"failed erasing sectors 0 to 0"无wrote确认行，重启OpenOCD后重烧成功，wrote 89536 bytes)。

验证：`wbus_rx_cnt`/`imucnt`/`chassis_2006[0].receive_cnt`全部正常增长。读到`ctrl_mode=KEYBOARD_MODE`、`wbus_rc.data.sw1=0`——拨杆当时在下位，正好对应`sw1==0→KEYBOARD_MODE`的映射，**证明SBUS驱动ctrl_mode这条链路已经真正跑通**。要测底盘摇杆响应，需要把拨杆拨到上位进入`REMOTER_MODE`才会真正用ch1/2/3驱动电机（目前KEYBOARD_MODE分支还是硬编码0）。

**待办**：chassis_motor_pid[4]速度环PID仍未接入闭环（tx_current直接等于运动学解算输出），用户说"落地之后Kp给大一点点就行"，等实际能跑起来再调。
