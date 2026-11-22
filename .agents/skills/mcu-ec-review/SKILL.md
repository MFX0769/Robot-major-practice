---
name: mcu-ec-review
description: MCU/RTOS 电控代码规范审查（STM32/FreeRTOS/RoboMaster 小车/机械臂）。写完或改完任何嵌入式代码后、commit 前、或用户说"跑一下规范/检查代码/别堆石山/代码清洗"时必用。覆盖分层、命名、任务写法、调试打印、注释、安全检查、编译纪律。规范源自上交 RoboMaster 电控部代码规范 + 湖大跃鹿 basic_framework 铁律 + 中科大 RoboWalker 实践 + 本工程 Develop_Log/20261120 调研。
---

# MCU 电控代码规范审查

## 使命

每次改动当场改好，不堆石山：注释尸骸不留下、调试打印必挂宏、本次改动引入的警告清零、一个逻辑一个 commit。参考仓库里没人留着整段注释掉的旧代码——git 历史就是存档。

## 触发时机

- 改完任何 `.c/.h` 之后（写/改/删）
- commit 之前
- 用户说"检查一下 / 规范化 / 别堆石山 / 清洗代码"

## 执行流程（按序）

1. `git status && git diff --stat` 圈出本次改动面，只审本次动过的文件
2. 逐条过下方【红线】，违反当场修
3. 命名拿不准时查【命名速查】；要示例/细则时读工程的 `Develop_Log/20261120_开源电控代码规范调研.md`（八维度完整版，含出处）
4. 编译验证（见【构建】），以日志 `0 Error(s)` 为准
5. 本次改动引入的警告必须清零；历史遗留警告单独列出，不混进本次修复
6. 汇报格式：改了什么 → 编译结果（错误/警告数）→ 剩余风险

## 红线（违反必须当场修）

1. **调试打印必须挂总开关宏**：`dbg_print`/`printf` 一律包在 `DEBUG_TEXT_ENABLE` 类宏内；整段注释掉的打印/旧逻辑直接删（git 历史里都有）。VOFA 波形通道同理收进 debug_task 集中管理。
2. **临界区内禁止延时**：`taskENTER_CRITICAL()`~`taskEXIT_CRITICAL()` 之间不得出现 `osDelay`/`HAL_Delay`（关中断会卡死系统时基）；确需短延时就地用 DWT 计数差值。
3. **安全态优先**：新任务必须有 PROTECT/STOP 零输出路径（参考 chassis_task 的 `force_zero`）；PID 输出必配 maxout 限幅；外部输入做范围钳位。
4. **应用层零外设代码**：Application/ 不得直接摸寄存器/句柄（`__HAL_TIM_SET_COMPARE` 这类只许出现在 Bsp//Protocol/ 层封装里）；任务只调模块接口。湖大铁律："应用层不应出现任何与片上外设相关的代码"。
5. **全局变量集中登记**：extern 声明放在所属模块的头文件里；`.c` 里散装 `extern` 视为违规（新 extern 必须说明归属）。目标值由拥有者模块赋值（"读取而非写入"：遥控器层不直写底盘目标）。
6. **编译零警告（本次引入的）**：隐式声明 = 缺 include，必须补齐；unused variable = 死代码，删。
7. **注释尸骸清理**：`//` 掉的整段旧实现、`#if 0` 块、复制粘贴的重复注释——全删。
8. **CubeMX 生成文件只动 USER CODE 块**（Core/Src 下 main.c/freertos.c/gpio.c 等），否则重新生成即丢。

## 命名速查（上交规范 + 湖大变体）

- 变量：写全单词、名能达意；小驼峰 `targetAngle` 或下划线小写 `gimbal_recv_cmd`，**工程内保持一种**
- 函数：动宾短语大驼峰 ≤4 词：`SetMotorControl` / `ChassisInit`；static 私有函数加 `_` 前缀
- 宏：全大写下划线；能用 `const`/`inline` 就不用宏（宏无类型检查、有副作用、难断点——华为规范）
- 类型：结构体 `_t`、枚举 `_e` 后缀（如 `chassis_mode_e`）
- 公认缩写白名单：arg buff clk cmd cmp cfg dev err inc init max msg min para prev reg sem stat sync tmp
- 文件头：`@file/@brief/@note` + 日期；接口注释写 `.h`（Doxygen `@brief/@param/@return`），static 函数注释写 `.c`

## 构建（taurus_26 工程）

```
python Tool/debug/stm32h7_tool.py build
```
- 约 2 分钟；UV4 退出码 0=干净 1=仅警告 2+=错误，以 `MDK-ARM/rm_main/build_output.log` 里 `0 Error(s)` 为准
- 源码 **GBK+CRLF**：中文注释改动用 `LC_ALL=C sed`（字节透明）或 Write 后 `iconv -f UTF-8 -t GBK` 转回 + `unix2dos`；本机 `grep` 被 ugrep 包装，查 GBK 文件加 `command grep`
- 调试宏互斥：`DEBUG_TEXT_ENABLE` / `DEBUG_VOFA_ENABLE`（config.h），同时开编译报错

## 完成标准

- [ ] 红线 1-8 全过
- [ ] 编译 0 Error，本次改动 0 新增警告
- [ ] diff 只含本次逻辑变更（无顺手重构混入）
- [ ] commit message 一句话说清一个逻辑变更
