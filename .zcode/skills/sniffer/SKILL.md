---
name: sniffer
description: BK3633 嗅探器测试机（rc_sniffer 工程）操作手册——烧录（merge_crc 铁律/假砖排障）、被动模式与 ACK 模式切换（0xC7）、SNIF 槽位选择（0xC8）、串口命令面与输出判读。当用户说"嗅探器/sniffer/嗅探板烧录/嗅探器没反应/假砖/被动模式/零发射/SNIF 槽位/0xC7/听槽位"，或要把嗅探器烧入/还原、排查"嗅探器串口全静默"、验证"嗅探器有没有在发包"时使用。代码全景正本=rc_remote/develop_log/knowledge/code_docs/codebase_sniffer.md。
---

# /sniffer — BK3633 嗅探器测试机操作

## 1. 这块板是什么

BK3633 开发板（与遥控器/电池同芯片），**台架专用测试机**：纯 AUX 从机——听踏频 SNIF+槽位地址（`53:4E:49:46:xx`），ACK 载荷下行命令（0xA4），收踏频上行 0xA6/0xA5/0xA7/0x42。**纯 2.4G、无 BLE、无配对、无 flash 存储、无低功耗**；80MHz、串口 1M、WDT 常态关。默认停靠 **ch20**、pipe1 专用地址。跨仓编译 rc_remote 的 8 个共享文件（改 `rf_handler.c` 连锁本工程与电池固件构建）。

## 2. 烧录（★镜像铁律，2026-10-08 假砖案）

```bash
# 唯一正确姿势：merge_crc 合包（BIM+app+CRC）
python ../Tools/self_flasher/self_flasher.py --port <N> --mode rts --bin ../rc_sniffer/output/app/rc_sniffer_merge_crc.bin
# 跑我方固件时 wdt 通道也可（0xB4 停喂狗）：
python ../Tools/self_flasher/self_flasher.py --port <N> --wdt --app-baud 1000000 --bin ../rc_sniffer/output/app/rc_sniffer_merge_crc.bin
```

- **★裸 `rc_sniffer.bin` 直烧 0x0 = 假砖**：ROM 找不到合法镜像（无 BIM）→ 落 UART 下载模式 → 串口全静默。**POR 复电无效**（内容不变永远拒）。fcc-bench skill 曾写错成裸 bin（已修正 2026-10-08）。
- **假砖指纹**（三查定谳）：①app 串口零输出 ②停喂狗(0xB4)无回显 ③`probe_rom_alive`@1M 应答（=芯片滞留 ROM loader，不是 app 崩）。判别脚本思路：`import self_flasher; sf.probe_rom_alive(ser)`。
- 烧录后芯片留在 ROM 会话（REBOOT 命令在本芯片=no-op，DEVLOG 8-11 实测）——**复位一次才见 app**（RTS 或断电）。
- 二进制取证（比对镜像）**禁 `git show`/管道导出**：CRLF 污染+丢首字节（本仓 HEAD blob 已实证损坏）。只用工作区 Keil 产物或芯片侧 `CMD_CHECKCRC` 仲裁。
- HCI 帧读回格式：每 4K 块应答=5B 头+4096 数据（勿按 4B 解析，2026-10-08 踩过）。

## 3. 行为模式（2026-10-08 新增）

| 模式 | 行为 | 用途 |
|---|---|---|
| **passive（boot 默认）** | pipe1 auto-ACK 关，**空中零发射**；0xA4 下行不可用 | 台架共存（不给 FHSS 网添发射污染） |
| ack | 经典从机：auto-ACK+ACK 载荷（0xA4 下行可用） | 需要下行命令的实验 |

- 切换：`0xC7 DATA[0]=0/1`（缺省=0）。**RAM 态，复位回 passive**（0xCC autarch 同款教训：RAM 态勿当持久）。
- 零发射验收：fcc-bench DTM 收板驻嗅探器信道（ch20）→ 数包=0。参考命令：`python tools/fcc_dtm.py --tx <DTM口> --rx <DTM口> rx 20` → 等 ≥10s → `stop --both` 看 count。
- boot 横幅与 `[STAT]` 行自带 `mode=/ack=` 字段，判读先看这两个。

## 4. 串口命令面（AA 55 帧，1M 波特；构造前过 serial-cmd skill 校验 chk/CRC）

| CMD | 功能 | 备注 |
|---|---|---|
| 0xB4 | 停喂狗→WDT 硬复位进 ROM boot | self_flasher `--wdt` 依赖此通道 |
| 0xB5 | IO 翻转耗时测试（LA） | P27 主测/P21 对照 |
| 0xC5 / 0xC6 | 下行命令装/卸 ACK 载荷 | 仅 ack 模式有意义 |
| **0xC7** | **行为模式切换**：DATA[0]=0 被动/1 ACK | 2026-10-08 从单向诊断升级 |
| 0xC8 | 选听槽位：DATA=1B 槽位或 5B 全地址 | 与踏频同码同语义 |
| 0xC9 | HT 钉道+判决收包（E-A 实验） | DATA[0]=信道 DATA[1]=arm |
| 0xCA | HT 换台接收聋窗测量（E-E） | 阻塞跑完打 [HER] |

## 5. 输出判读

- `[STAT] ok= bad= aux= raw= dbg= ack=`：5s 窗计数+当前模式；`ack=0`=被动。
- **`[HEAL]` 两级自愈是预期行为不是 bug**：5s 窗计数零增量→一级 RF 重 init→再零→WDT 复位（~3s 回来）。**安静空气下周期复位循环正常**（台架无踏频发射时必见）。
- `[CAD]`(0xA6 gz×10 还原)/`[RAW]`(0xA7)/`[ACK]`(0xA5)/`[DBG][DBG2]`(0x42 两行)——0x42 打印拆两行是 uart_printf 128B 栈缓冲红线，勿合并。
- boot 横幅：`=== AUX slave sniffer: ch20 1Mbps pipe1 addr ... mode=passive ===`。

## 6. 坑清单（历史血案浓缩）

- **地址化石**：头注释 A3:BB:CC:DD:03 已废——踏频默认发射地址=AA:BB:CC:DD:EE，SNIF 槽位地址见 0xC8 回执。
- 嗅探器**固定道不跟跳**：FHSS 网跑跳频时它失聪（对自家网零感知是设计，不是故障）。
- 0xB5/0x42 相关打印语义变更看 git log，勿按头注释（多处过期注释，正本=codebase_sniffer.md）。
- COM 非恒久映射：今天 COM22/明天换号（2026-10-08 实证 COM92↔COM22 洗牌）；先枚举再指纹（1M 静默+115200 HCI 无回响=嗅探器候选）。

## 7. 纪律

- 结论/测量当场 append rc_sniffer `develop_log/daily_review.md`（实测时间戳）。
- 与 fcc-bench/zs406 共用台架：串口先实测占用；嗅探器纯接收无发射压制需求（ack 模式下有 ACK 发射，用完切回 passive）。
- 镜像/产物入 git 不可删；恢复走 `git checkout`。
