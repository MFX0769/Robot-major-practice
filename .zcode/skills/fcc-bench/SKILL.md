---
name: fcc-bench
description: 遥控器 RF 双板测试台维护与操作（厂家定频固件 + fcc_dtm.py 测量脚本 + fcc_bench.py 网页测试台）。当用户说"测试台/丢包测试/信道体检/全信道扫描/干扰雷达/测丢包率/厂家固件/DTM/定频包"，或要烧入/烧出厂家测试固件、把板子还原成正式包/嗅探器固件，或排查"厂家固件没反应/上位机抓不到命令"时使用。外场干扰普查（"外场测试/环境干扰/点位普查/带板子出去测"）也走本 skill：采样律+频谱仪协同标定。覆盖：启动网页测试台、两块板对测 PER/信道地图、功率档（0x206B）、烧录归位、协议排错、外场点位普查。
---

# /fcc-bench — 遥控器 RF 双板测试台

两块 BK3633 板（0611 即可）烧厂家定频 bin，一块发一块收，测 BLE 物理层丢包/信道占用。**不需要厂家上位机、不需要 com0com 桥**——纯 Python 直连 HCI 二进制协议。

## 快速上手

```bash
python tools/fcc_bench.py          # 启动网页测试台，浏览器自动打开 127.0.0.1:8765
# 或双击 tools/fcc_bench.bat（人类入口）
```

网页里点「🔍 自动找板子」→ 自动认出测试固件的板 → 「▶ 全信道体检」出柱状图+人话结论。
命令行版（脚本/自动化用）：

```bash
python tools/fcc_dtm.py probe                    # 双板探活
python tools/fcc_dtm.py per 10 30                # ch10 测 30s 丢包率
python tools/fcc_dtm.py scan --dwell 5 --csv out.csv   # 40 信道地图
python tools/fcc_dtm.py watch 30 -i 10 -n 30     # 单信道时间序列（干扰占用监测）
python tools/fcc_dtm.py raw tx "01 6B 20 01 0C"  # 任意 HCI 命令
```

## 文件地图

| 文件 | 作用 |
|------|------|
| `tools/fcc_dtm.py` | 测量内核（HCI 驱动 + probe/per/scan/watch/tx/rx/stop/raw） |
| `tools/fcc_bench.py` | 网页测试台（内嵌 HTML，后端跑任务、刷新不丢结果） |
| `tools/fcc_bench.bat` | 双击启动器 |
| `temp/fcc_fixed_P_C.bin` | 厂家定频 bin 的 ASCII 路径副本（md5 3f266f3d43ed4bba712688984f372b93） |
| 厂家原件 | `../kmo_mc_rcv/Tools/fcc_test/bk3633_定频程序_210510(P=C).bin`（**只读，禁改 mentor 目录**；P=7 是低功率档，两 bin 只差 3 字节） |

## 协议要点（2026-09-24 实测定案）

- **厂家 bin 说 HCI 二进制（H:4 @115200），不理文本**：`01 <opcode> <len> <params>` 命令，`04 0E ...` 回包。之前"全静默"是因为拿文本 CMD+ 去问——别再犯。
- 标准 DTM：`0x201D` RX 测试、`0x201E` TX 测试、`0x201F` Test_End（回包带 2 字节收包计数）。
- **0x206B(power) = 运行时功率档**（0..255 全透传，P=C 的 C=12）。改功率不用换 bin。
- **0x2067() = 恢复键**：任何异常状态一发复活（5/5 实测）。
- **0x2065 = 禁用键**：中途发必杀 DTM 发射，别碰（它是 CW/载波域命令）。
- 厂家 exe（BK32xx RF Test）按钮在 IC类型=BK3633 时发的就是这些二进制命令；选 Others 才发文本 `CMD+...`（板子不理）。

## 测量口径与判读

- TX 侧不回报发送数（Test_End 对 TX 返回 0）→ PER = 相对**最佳信道归一**，ref 速率 ≈1430 pps（干净信道收 8400+ 包/5s）。
- 计数 16 位（65535 封顶）→ dwell 别超 ~40s；fcc_dtm.py 有溢出守卫。
- 台架近距离链路饱和：功率档差异在包计数上看不出来（差 <0.5%），要量 dB 差需 RSSI（RF_SCAN_EN=1 探针固件）或拉远。
- 零信道=被占：如隔壁 FHSS 网驻留（ch22/ch8/ch26 实例）；**FHSS 网在跳频，干扰信道逐轮漂移**，静态避让不够。

## 外场点位普查（field survey · 2026-09-28 增补）

测产品日常环境的干扰。两件仪器分工，**不可互替**：频谱仪=能量地图（安静带/漂移常数/认干扰源），双板 `scan`=杀伤地图（PER，含"能量→丢包"传递函数）；理想编队=频谱仪主力 + 双板在 1~2 个点做**同点同时标定**（频谱图↔死道对账一次，此后纯频谱仪点位有换算底气）。

**采样律（判例=09-25 洞漂移：21 轮 16 坏点无一重复）**：
- 单点单次=对非稳态过程抽一个样本，**不算数**。每点位 ≥3 轮、轮间隔 ≥2min。
- 漂移时间常数：选 1 个干扰最活的点驻留 30min——有瀑布图开着录；没有则每 5min 一轮 max-hold/scan 快照 ×6。板侧对应工具=`watch`（单信道时间序列）。
- 每轮记录时刻+点位+环境一句话（WiFi 密度/微波炉/人流），否则回来无法归因。

**点位操作卡（每点 ~10min）**：①记录三行 ②频谱仪全带 max-hold 拍照 ③average 拍照（设置拍进画面）③ 板标定点加跑 `scan --dwell 5 --csv <点名>.csv` ④歇 2min 重复，共 ≥3 轮。

**外场坑**：COM 非恒久映射，换电脑必实测端口；笔记本+充电宝供电先试；纯观测日不烧板不改固件；板子测完必压制（Test_End/断电）；绝对 dBm 仅参考、同天线相对比较为准；频谱图上细沙状低占空比=自家跳频网，勿认成干扰。

**产出物（预注册，回来判读）**：①各场景坏道占比分布 ②最坏场景画像（max_gap 预算输入）③安静带（ch24/28/46-54/68-72）外场有效性→回港信道依据 ④坏点漂移时间常数→AFH 更新周期依据。

## 板子烧入/烧出

```bash
# 烧入厂家 bin（板子跑我方固件时用 wdt；跑厂家 bin 时用 rts 撞窗）
python ../Tools/self_flasher/self_flasher.py --port <N> --wdt --app-baud 1000000 --bin temp/fcc_fixed_P_C.bin
python ../Tools/self_flasher/self_flasher.py --port <N> --mode rts --bin temp/fcc_fixed_P_C.bin
# 烧回正式认可包（output/app/rc_remote_merge_crc.bin，BUILD 戳核对身份）
python ../Tools/self_flasher/self_flasher.py --port <N> --mode rts
# COM22 原本是嗅探器板，还原（★必须用 merge_crc 带版——裸 rc_sniffer.bin 烧 0x0 无合法 BIM，ROM 校验失败会落下载模式=全静默"假砖"，2026-10-08 实战教训）：
python ../Tools/self_flasher/self_flasher.py --port 22 --mode rts --bin ../rc_sniffer/output/app/rc_sniffer_merge_crc.bin
```

- 厂家 bin 无 0xB4 停喂狗 → 复位只能靠 RTS 脉冲或断电；`--mode rts` 撞窗对已跑厂家 bin 的板可用（boot ROM 应答）。
- 烧录完核对 `Writing ... OK` + 逐块 CRC；交付前必须烧回认可包（一码一包）。

## 排错速查

| 现象 | 处置 |
|------|------|
| 端口全静默 | 波特率 115200；发的是二进制不是文本；厂家 bin 不主动打印 |
| /api/ports 卡死 | 蓝牙虚拟口（BTHENUM）写入永久阻塞——candidate_ports() 已排除；新环境注意同类坑 |
| 扫出多个零信道 | 先看是不是 FHSS 网在跳（watch 盯一条看时间序列）；再查板子接触 |
| exe 的 CMD+ 无回包 | 正常——板子只应二进制；要抓二进制命令就设 IC类型=BK3633 再点按钮 |
| COM 口号变了 | COM 非恒久映射，烧录/监听前必须实测（枚举+识别） |

## 不用时的压制（共存纪律 · 隔壁 FHSS 线正在猛开发跳频）

厂家 bin 的设计=**没有命令就没有射频**——不发测试命令就永远静默，不存在放着不管自己发射。但台架离 FHSS 开发环境近，双向干扰都要管：

| 场景 | 动作 | 结果 |
|------|------|------|
| 短暂离开（随时复测） | 发 `01 1F 20 00`（Test_End） | 射频关、空转待命；fcc_dtm.py 每次测量收尾自动做 |
| 隔夜/收工 | 断电或拔 USB | 零功耗零发射；再上电自动回 idle（厂家 bin 不自启动测试） |
| 长期不用/板子还活 | 烧回正式认可包（COM22 原本是嗅探器就还嗅探器固件） | 板子恢复干活身份 |

纪律三条：
1. **测完必压制**：任何一轮测量/抓包结束，确认双板 status=0C 或断电，不留发射态。
2. **测前先扫**：FHSS 网在跳频、干扰信道逐轮漂移（实测 ch22→ch8/ch26）——长测前先 `scan` 看一眼当前占用，别盲测把人家的跳频撞了还说链路差。
3. **我方 TX 也是干扰源**：1430pps 满功率发包对隔壁 FHSS 接收机就是 jammer——能用低功率档（0x206B）就不用高档，测完即停。

## 维护规则

- 改 fcc_dtm.py/fcc_bench.py 后必须重跑 `probe` + `per 10 5` 验证双板链路。
- 厂家 bin 与工具说明在 mentor 的 `kmo_mc_rcv/Tools/` 下，**只读**；temp 副本丢了从那里复制。
- 测试完按用户指令归位板子（正式包/嗅探器固件），不留厂家 bin 过夜除非在测。
- 测量结论带证据落 daily_review.md（条件/步骤/原始数据/结论，实证推断分栏）。
