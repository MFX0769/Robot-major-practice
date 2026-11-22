---
name: g24-debug
description: BK3633 厂家 2.4G/基带测试总线调试口（rf_debug_gpio_init，引出 P20~P27 或 P30~P37 共 8 根 test_signal）——把芯片内部 RF/BB 测试信号复用到 GPIO 供逻辑分析仪抓取。当用户说"2.4G 调试引脚/RF debug 引脚/引脚20~28/调试口输出/test_gpio_sel/BB_TEST_PIN/LA 抓 2.4G 内部信号"时使用。覆盖：寄存器机制、三仓函数位置、启用一行改动、V3 板脚位冲突表、能读什么（证据分级）、FAE 待确认清单、LA 配合流程。
---

# /g24-debug — BK3633 厂家 2.4G 测试总线调试口（P2x/P3x test_signal）

## 一句话

`rf_debug_gpio_init(mode)` 一行调用，把芯片内部测试总线复用到 8 根 GPIO（P2x 或 P3x 口），逻辑分析仪直接抓 2.4G 内部硬件信号——与软件打点探针（PROBE_LA/P00-P01）互补：**打点=软件事件，本口=硬件真实时序**。

## 机制（全部源码+厂家寄存器表实锤）

| 项 | 值 | 证据 |
|---|---|---|
| 函数 | `rf_debug_gpio_init(uint8_t GPIO_C_D)` | rc_remote `SDK/src/driver/rf/rf_xvr.c:1290`（三仓同源：kmo_mc_rcv rf_xvr.c:1307、kmo_cadence rf_xvr_24.c:1331） |
| mode=1 | **P20~P27**（=GPIO16~23=厂家命名 GPIOC[0..7]），sel=7 | rf_xvr.c:1294-1307 |
| mode=0 | **P30~P37**（=GPIO24~31=GPIOD[0..7]），sel=7 | rf_xvr.c:1308-1321 |
| mode=2 | P20~P27，sel=4（另一组信号） | rf_xvr.c:1322-1332 |
| 总开关 | `SYS_Reg0xc` bit4=`BB_TEST_PIN`（基带测试脚使能）、bit5=`ana_test_pin`（模拟测试脚使能） | 厂家寄存器表 `APP_Notes/BK3633_ADDR Mapping.xlsx`（提取版=xlsx_out.txt:285-288） |
| 信号选择 | `SYS_Reg0xc[23:0]` `test_gpio_sel` = "test signal output control" | 同上 :171 |
| 引脚复用 | 每脚写 `Config=0x40` = `GPIO_Fun_Ena`（第二功能使能） | gpio.h:42；AON_GPIO 表 0x0[6] |
| 厂家意图 | 桥固件遗留被注释调用 `// rf_debug_gpio_init(2);`（厂家 rf_init 末尾） | kmo_mc_rcv rf_xvr.c:873 |

参数名 `GPIO_C_D` 即"GPIOC 口或 GPIOD 口"——厂家 GPIO 分配表里 P2x=GPIOC、P3x=GPIOD。

## 能读什么（按证据强度分级，勿越级写判据）

**实锤（厂家寄存器表白纸黑字）**：
- GPIOC/GPIOD 全部 16 根第二功能 = `test_signal`（DIAG MODE 列，xlsx_out.txt:65-80）——8 位内部测试总线引出
- **IF-ADC 测试总线**：GPIOC[2]=`ADC CLK`、GPIOC[3..7]=`IF-ADC In[0..4]`、GPIOD[0..2]=`IF-ADC In[5..7]`——模拟测试源可输出 8 位 ADC 采样流+时钟
- `SYS_Reg0x17[8]` `adc_q_sel` = "测试模式下，控制送到 port 口的 adc 输出 i 或 q 信号"——**ADC 采样可分 I/Q 两路引出**
- `SYS_Reg0x6[7:0]` `sw_dbg_gpio`——纯软件调试输出口（备用）
- `SYS_Reg0x4[24:26]` `xtal_out_en/sel/div`——时钟可引出观测

**推断（合理但未验证）**：BB 源（bit4）输出基带状态机事件（RX/TX 窗、sync 检出、demod 时钟/数据一类）；ANA 源（bit5）输出 IF-ADC 采样流。依据=名字+vendor 代码两处开关的组合。

**未知（本地无文档，FAE 挂账）**：`test_gpio_sel` 各取值（至少 4/7）对应每脚输出什么信号的 mux 对照表。数据手册/BK3633_ADDR Mapping/reglist/BLE 开发指南/Controller V4 全查过，**只有名字没有表**；公网无此函数文档。**信号表没拿到之前，只做"有无翻转/周期"级判读，不做事件级定量结论。**

## 启用方法（1 行改动 + 全量重编）

```c
#include "rf.h"                    // 声明已在 rf.h:54/63
rf_debug_gpio_init(1);             // RF init 完成后任意时刻调用；P20~P27 出 test_signal(sel=7)
```

- 编译走 /build（全量重编，BUILD 戳核对）
- 恢复=不调用（寄存器复位值全 0）或清 `SYS_Reg0xc` bit4/bit5
- LA 侧联动 /kingst-la：`la_direct.py capture --srate 16e6` 起步（内部总线沿快于 UART，别用 1MHz 档），通道接 P2x 排针/飞线

## V3 板（QFN48）脚位冲突表——启用前必读（pinmap.h 实测）

**mode=1 组 P20~P27：8 根全被占，启用即同时失去屏幕+充电状态+电源键**：

| 脚 | V3 现役占用 | 启用后后果 |
|---|---|---|
| P20 | LCD_R1 | 屏幕红数据 bit1 丢失 |
| P21 | CHG_CHRG | 充电中标志丢失 |
| P22 | LCD_BCK | 屏幕移位时钟丢失（屏花） |
| P23 | CHG_PGOOD | 电源好标志丢失 |
| P24 | LCD_BSP | 屏幕行起始丢失 |
| P25 | LCD_VB | VCOM B 极丢失 |
| P26 | LCD_G1 | 绿数据 bit1 丢失 |
| P27 | PWR 电源键 | 开机靠 P11 自保持可存活，但按键失灵 |

**mode=0 组 P30~P37**：P30=LCD_B1、P31/P32=CW2017 I2C、P33=VBAT ADC（已弃用可牺牲）、P34=**霍尔 ADC（动不了）**、P35=背光、P36=空闲但未出线、P37=分压使能（已弃用）——同样不可整机启用。

**结论：产品板不可用。只在测试台架/专用调试板（或接受"RF 调试会话期间熄屏+断充电检测"）上做。** 建议下次打板把 P2x 组中 2~4 根经 0Ω 电阻引到测试点。

## FAE 问题清单（让 mentor 问 Beken，拿到前本 skill 停留在"机制可用"级）

1. `test_gpio_sel` 取值→每脚信号对照表（至少 sel=4 与 sel=7 两组）
2. `BB_TEST_PIN` 与 `ana_test_pin` 两组输出的完整信号清单（是否含 RX/TX 切换、sync 检出、RSSI、demod 时钟/数据）
3. IF-ADC In[0..7] 是否即解调 IQ 采样？`adc_q_sel` 与 `test_gpio_sel` 怎么配合输出完整 I/Q 流？
4. `sw_dbg_gpio`（SYS_Reg0x6）的用法与引出脚

## 铁律

1. **信号表未确认前不做事件级判读**（可看周期/有无，不写"sync 在第 X µs"类结论）——防瞎解读进台账
2. **产品板带屏状态禁启用**（冲突表见上）；台架实验前声明本次牺牲哪些外设
3. **与 PROBE_LA(P00/P01) 互不冲突可同抓**，但 P00/P01 是串口/打点复用，别把 test_signal 接到串口脚上混淆
4. 时序测量纪律照旧：片上量时基、LA 时间戳只用于相对间隔（/kingst-la 铁律 1）
5. 证据源版本：寄存器表=APP_Notes/BK3633_ADDR Mapping.xlsx（xlsx_out.txt 为提取版）；本地 PDF 手册无 mux 表，别再重复翻
