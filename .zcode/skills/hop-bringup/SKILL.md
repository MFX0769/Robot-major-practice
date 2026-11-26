---
name: hop-bringup
description: 跳频三机开台架/换套（5 分钟版）。覆盖：端口认板→选族（C0/C1/C2 合法 hex）→一键配三址→台架态(0xB0→0xA7)→踏频重锚→档位→验收；**§0c 串口触发配对（含自动同步地址、80% 成功率参数、不钉 RF_ONLY）**。当用户说"这套跳频配一下/换一套/烧包配址/串台了/收不到数"时使用。帧一律 frame.js 生成，禁手算 CRC。2026-11-04 按红板套 C1 实战修订。
---

# hop-bringup — 跳频三机开台架（5 分钟）

> 正本流程 = `develop_log/sop/跳频三机写地址SOP.md`；登记 = 桌面 `跳频地址管制看板_20261030.md`。
> 本 skill 是**可执行清单**：照顺序做，每步有判据，不通过就停。

## 0. 一键部署（首选，2026-11-04 起）

```bash
cd projects/rc_remote
py tools/hop_deploy.py --dry-run --family C1                 # 自检+看帧，不碰串口
py tools/hop_deploy.py --family C1 --bridge 43 --cadence 99 --remote 22 --tier HIGH
py tools/hop_deploy.py --family C1 --bridge 43 --cadence 99 --remote 22 --tier HIGH --verify-only
```

脚本做完全部流程：认板 → 桥三址 → `0xA7`(motor_ignore) → 踏频三址+`0xA0` 重锚 → 遥控两址 → 档位 → 读桥 12s 验收（PASS/FAIL + 退出码）。**`0xA5` 不预发**：只在「症状成立」时钉（见 §0d）。
帧由脚本生成，启动时用 6 条已知好帧自检 CRC（不符直接退出）——无需手工拼 hex。
**唯一前提**：桥 app 不能是挂死状态（串口无输出）；桥烧录仍需人工（见 §6）。

手工分步流程见下（脚本失败或需要单步排查时用）。

## 0b. 认板（端口每次插拔/换板都变，禁凭记忆）

4 秒读串口特征定身份：

| 设备 | 特征行 |
|---|---|
| 桥 | `[HB] … svcmax=… rfst=…` |
| 遥控 | `[SBt] anch=… r=… slot=…` 或 `[TH ` |
| 踏频 | `[CSl] dt=…` 或 `[H] mode=G24`（1024 线）/ `[H0]`（主线） |

```bash
py - <<'EOF'
import serial,time
def sig(p,d=4):
    try:
        s=serial.Serial(); s.port=p; s.baudrate=1000000; s.timeout=0.2
        s.rts=False; s.dtr=False; s.open()
        t0=time.time(); b=b""
        while time.time()-t0<d: b+=s.read(4096)
        s.close(); t=b.decode('utf-8','replace')
        return "BRIDGE" if "[HB]" in t else "REMOTE" if "[SBt]" in t or "[TH " in t else "CADENCE" if "[CSl]" in t or "[H0]" in t else f"silent({len(b)}B)"
    except Exception as e: return f"BUSY/{e.__class__.__name__}"
for p in ["COM43","COM99","COM22"]: print(p, sig(p))
EOF
```

> 端口被占 = `PermissionError(13)`：**先排除自己上一轮刚关的句柄**（Windows 秒级释放延迟，同进程紧跟重开必失败；隔 6~10s 再开即好），再考虑用户开着串口助手。

> **★2026-11-10 血案：认板窗口必须 ≥8s，且 `PermissionError(13)` 要重试。**
> ① 1024 线踏频的 `[H] mode=G24` 是 **5s 周期**打印，4s 窗口只抓到 `[TX]` 行 → 判 `UNKNOWN` → 必须加 `--force` 才能继续，白跑一轮（当天连撞两次）。`hop_deploy.py` 的 `identify()` 已改默认 `dur=8.0`。
> ② `PermissionError(13)` 多为**上一轮句柄未释放的瞬时占用**（同秒重开就成功），不是设备问题、不是"让别人关串口助手"。现已内置 3 次×1.5s 重试；手工脚本也要照做。

## 0d. ★★ 快速配方（2026-11-11 实测，一车 40~60 秒）

今天连配 4 套车能这么快，靠的是这四条，**与"发够 15 次就当成功"的老习惯相反**：

1. **写→回读闭环**，不信"发过了"。实测 108 的 `slot1` 第一次被吞（回读仍是 `7F:BE:11`），补发一次即正。
   回读用 `0xAF`（槽）+ `0xCF` len=0（信标），最多 3 轮；**桥 `0xCF` 写入常静默落地**，只能靠回读判。
2. **一次性生成整套帧**（frame.js 一次出 5~8 条 / 或 `hop_deploy.py --dry-run`），别一条条算。
3. **两机套（桥+遥控）就是 5 条帧**，比三机少一半动作；没接踏频时 `p1=0` 属正常，不要当故障查。
4. **不预发仪式命令**：`0xA0/0xA5/0xA7` 一律等症状。判据见下表。

| 症状 | 处置 |
|---|---|
| `bm=0/1` + 15s 内零 `[BEACON]` + `p0=0` | 发 `0xA5` 钉 RF_ONLY（**今天 108 实测：钉前 p0=0、钉后 23.0/s**） |
| `bm` 正常、`[BEACON]` 在发、`p0` 低但非零 | 不钉，先查信标族是否与跟随者一致 |
| `DEAF` 持续刷 + `p0` 只剩突发 | `0xA0` ×10，10s 后复测；再不行断电 5s |
| 遥控 `anch=0` + `txds` 冻结/`send-stall` 长期挂 | 遥控侧 RF 停摆：先 `0xA0`，无效则断电 |

> **`0xA7`(motor_ignore) 同理**：只在桥报 `rs=3`（MOTOR_RUNNING 误判）时才需要，不是每次必发。

## 0e. 烧完起不来（2026-11-11 103 实录）

正常 `--wdt` 烧完出现 `bm=255`、`ch=` 冻结、`rxdr=0`、不发包 → **半擦残留混血**。处置：

```bash
py .../self_flasher.py --port <数字COM> --wdt --app-baud 1000000 --bin <xxx_merge_crc.bin> --erase-all
```

`--erase-all` 抹 `0x51000` 配置块 + `0x51FF0` OTA 头 ⇒ **烧完必须重配址**（`0xB6`+`0xCF`，桥则 `0xAC`×2+`0xCF`），并按 §3 重锚。
仍零字节 → 断电 POR；再不行是这颗包在该板跑不起来，换板或用已知好板对照。

## 0c. 串口触发配对（2026-11-09 实测两次复现，正本=SOP §八）

> 适用：地址配好了但两端没绑定；或换了板/整擦过要重新绑。
> **不要手工写 `0xAC slot1`** —— 配对成功会自动同步两端地址（`main.c:412` `memcpy(g_paired_tx_addr, …)`），手写反而会把已对齐的地址改歪。

**顺序（缺一不可）**

1. **不发 0xA5**（2026-11-09 用户令）：配对会话本身会把桥切 RF_ONLY（`bridge_mode_mgr.c:1088` `REASON_PAIRING`），无需手工钉。
   - 「BLE 广播饿死 UART」是**错误归因**：UART 硬件接收，主循环只延迟处理；1Mbps 下 9 字节帧 ≈90µs，TDM 阻塞只延后不丢。
   - 回显命中率约 50% = 回显被 `[WIND]`/yield 吞 → **重发到见回显**即可，不要用钉住去"治"它。
   - 判据：发 `0xC9` 能见 `[HOPS]`（只证明 UART 活，不再是"必须先钉"的理由）
2. **0xCF 写信标**（同族，见 §1）
3. **桥 0xAA + expect**：遥控 `DATA=0`、踏频 `DATA=1`；重发到见
   `ble_pair req pair3_listen cnt=N expect=0/1`
   → `ble_pair classic_rx_enter ch=20` → `enter classic_v2 ch=20 addr=AA,BB,CC,DD,EE`
   - ⚠️ `ble_pair init classic_v2_ack=1` 是**开机打印**，不是进配对，不能当判据
4. **跟随者 0xAA**（帧 `AA55AA0000A9F5BB`，空 DATA），重发到见
   - 遥控：`Host: ACK received` → `PAIR_OK RF_ADDR=xx:xx:xx:xx:xx`
   - 踏频：`Host: paired, addr=xx:xx:xx:xx:xx` → `Host: paired ok, bind window open`
   - 桥侧：`classic_v2 saved link addr=…` → `ble_pair success`
5. **对账**：桥 `saved link addr` 必须等于跟随者打印的 addr（实测四组逐对相同）
6. **等 60s**：`post_pair_ble_hold=60000ms` / `bind window open`，期间别复位、别判失败
7. **★配对后必须按「新族」重对齐（2026-11-17 实测，缺这步 = 配对成功但链路死）**：串口配对会在桥侧**生成一个全新族**——实测 C7 套配对后 `PAIR_DUMP slot=1 … addr=7F:4A:11:11:11`（新族 4A），`slot=3` 被写成**蓝牙链路址**（如 `2A:A9:24:A3:1E`，不是 `7F:xx:22:22:22`），而三端 `0xCF` 信标**仍停在旧族**。直接后果：`p0=p1=0`、遥控 `anch=0 bm=255`。
   处置：以配对给出的新族为准，跑一次
   `py tools/hop_deploy.py --beacon 88:<新族>:82:07:84 --bridge <桥> --cadence <踏频> --remote <遥控> --tier HIGH`
   重对齐三端，再把新族登记到桌面管制台。**别反过来**（保留旧族手工补 `0xAC slot1`）——那正是本节开头「手写会把已对齐地址改歪」的情形。

**帧**：一律 `frame.js`/脚本生成，禁手算 CRC。

> **★深睡遥控（Nov15+ 构建）配址可靠配方（2026-11-18 实战）**：深睡态命令时窗性失聪 → `0xA0` 软重启 + boot 窗内轮灌 `0xD2(不睡)`+`0xB6`+`0xCF` 各 ×20（D2 先落=保持清醒，B6/CF 趁窗落 flash），一次全中。**此构建写回显关键字=`set&live:`（信标）而查询才是 `cur=`；esc_addr 回显=`esc_addr set:`，且配完必打 `[CFG] ESC addr reloaded:`**——只盯 `cur=` 会把已落地的写误判 FAIL。RAM 档位（D1/D2）重启即丢，boot 窗后必须补发。
>
> **★★深睡构建 E2E/验收全程禁发 0xA0（2026-11-18 事故定谳）**：`0xA0` 重启清 RAM D2 → 板子秒睡且 UART 聋（实测重启后 45s×100+ 唤醒帧 0 字节，物理按键才能醒）；此时若用户/工具随后烧录，`0xB4` 握手零应答 → 强写=半擦混血坏启动。**重锚的正确动作=补发 `0xD2`→`0xD1` 后干等锚定，不重启**。遥控醒着时发出的配址，flash 已持久化，无需靠重启"生效"。

**反复配对的经验值**：轮隔 100s + 每次复位后静置 60s → 成功率 80%（40%→60%→80% 的实测递进）。
间隔 70s/静置 3s 只有 40%，失败几乎都落在"刚复位/刚配对完"的窗口。

**验完想给 APP 用**：新流程**不发 0xA5**，无需释放。只有沿用了旧配方（发过 0xA5）时才需要 `0xA6` 解钉。

## 1. 选族（多套并行必读）

| 族 | 信标 | A（遥控→桥） | B（踏频→桥） | 归属 |
|---|---|---|---|---|
| BB | `88:0D:82:07:84` | `7F:BB:11:11:11` | `7F:BB:22:22:22` | 旧包（<10-23 无 0xCF）专用 |
| BD/BE | `88:0F` / `88:10` | `7F:BD/BE:11…` | `7F:BD/BE:22…` | 外场套件 |
| BF | `88:11:82:07:84` | `7F:BF:11:11:11` | `7F:BF:22:22:22` | 旧台架（95/54/74） |
| **C0** | `88:12:82:07:84` | `7F:C0:11:11:11` | `7F:C0:22:22:22` | 新台架 |
| **C1** | `88:13:82:07:84` | `7F:C1:11:11:11` | `7F:C1:22:22:22` | 红板套（43/99/22） |
| **C7** | `88:C7:82:07:84` | `7F:C7:11:11:11` | `7F:C7:22:22:22` | 第二套（11-06） |
| **C8** | `88:C8:82:07:84` | `7F:C8:11:11:11` | `7F:C8:22:22:22` | 红色车（11-11） |
| **C9** | `88:C9:82:07:84` | `7F:C9:11:11:11` | `7F:C9:22:22:22` | 灰色车（11-11） |
| **CA** | `88:CA:82:07:84` | `7F:CA:11:11:11` | `7F:CA:22:22:22` | MT 车（11-11） |
| **CB** | `88:CB:82:07:84` | `7F:CB:11:11:11` | `7F:CB:22:22:22` | 外出车（11-11） |

**族名 = 信标第三字节**（`CB` ↔ `88:CB`），C4~CF 全部合法，`hop_deploy.py --family` 已内置 C4..CF。
**分配规则**：以桌面 `跳频地址管制看板_20261030.md`（只有「车 —— 地址」一张表）为准，取未占用的下一个。
临时族可直接 `--beacon 88:XX:82:07:84`，A/B 由 XX 自动派生（非法 hex 如 `BG` 会被拒）。

**规则**：三端信标必须相同；A/B 必须成对；**只用合法 hex 字节**——`BG` 的 G 不是十六进制，frame.js 直接拒收；下一个新族从 **C2** 起（信标 88:14）。
两套同场务必先查桌面看板确认在用的族，撞族 = 双方 p0/p1 塌方。

> **★现场族与脚本族不一致时（2026-11-10）**：桥可能是 C7 族（`88 C7 11 11 11`），而 `hop_deploy.py --family` 只到 C3。**直接报 `invalid choice: 'C7'` 就说明该族没进脚本**——此时跑 C1 会把桥整个改写族（本次实测：跑 C1 后桥 slot1/slot3/信标全部变成 C1 值）。**这是覆盖性写入，不是查询**，跑前先 `0xCF` 只读确认当前族，并明确"我要换族"还是"我要沿用"。

## 2. 一键配址（核心：发到有回显为止）

```bash
# 帧一律现生成（别抄死 hex，家族/地址不同就要重生成）
node ~/.agents/skills/serial-cmd/frame.js --quiet bridge  SET_ADDR 1 7F:C1:11:11:11
node ~/.agents/skills/serial-cmd/frame.js --quiet bridge  SET_ADDR 3 7F:C1:22:22:22
node ~/.agents/skills/serial-cmd/frame.js --quiet bridge  SET_BEACON_ADDR 88:13:82:07:84
node ~/.agents/skills/serial-cmd/frame.js --quiet cadence SET_PAIR_ADDR 1 7F:C1:22:22:22
node ~/.agents/skills/serial-cmd/frame.js --quiet cadence SET_PAIR_ADDR 3 7F:C1:22:22:22
node ~/.agents/skills/serial-cmd/frame.js --quiet cadence SET_BEACON_ADDR 88:13:82:07:84
node ~/.agents/skills/serial-cmd/frame.js --quiet remote  SET_ESC_ADDR 7F:C1:11:11:11
node ~/.agents/skills/serial-cmd/frame.js --quiet remote  SET_BEACON_ADDR 88:13:82:07:84
```

**顺序不能换**（2026-11-04 实战）：
1. **桥**：`0xAC slot1` → `0xAC slot3` → `0xCF`（各连发 15~20 次）
2. **桥台架态**：`0xA7`（motor_ignore）×25 —— 只压幽灵电机，**不钉 RF_ONLY**（2026-11-09 用户令）
3. **踏频**：`0xAC slot1`（= TX 目标！）→ `0xAC slot3` → `0xCF`
4. **遥控**：`0xB6 esc_addr` → `0xCF`

**连发规程（踩过三次）**：桥/踏频 UART 单帧回显命中率约 50%；**混合态（bm=0/2 交替）下回显会被 188ms TDM 阻塞段/yield 整段吞掉**（命令本身仍被接收），6~10 次全无回显是常态。做法：每帧连发到有回显，最多 4 轮×12 次；**不要用 `0xA5` 钉住去"治"它**。

**配址免重启**（代码为准）：桥 `uart_cmd.c:250`（09-17）置一次性 reinit、踏频 `uart_cmd.c:229`（09-24）`g24_apply_normal_rf_state()` 立即生效；SOP §九「必须重启」是过期条款，仅在混合态 reinit 悬置时用 `0xA0`。

### 2b. ★★ 命令零回显时先分「链路 / 白名单 / 构建」三档（2026-11-10 定谳，别再绕一小时）

连发多轮全无回显时，**必须先用一条同一白名单里的只读命令做对照**，不要先怀疑链路、更不要先翻源码下「没有 handler」的结论：

| 现象 | 结论 | 动作 |
|---|---|---|
| 只读命令（`0xCF` len=0 / `0xAF` len=0）**有**回显，写入命令（`0xAC`/`0xCF` 带 DATA）**无**回显 | 不是链路、不是 CRC、不是白名单——白名单是**命令字节级**的，同字节同 white-list 不可能一半通一半不通 | **板子跑的是旧构建**，该命令的 `case` 与白名单项都是后加的 → **重烧含该 handler 的包** |
| 只读命令也全无回显，但 `[H]`/`[TX]` 等周期打印正常 | 串口链路活；命令被白名单丢弃或 CRC 不匹配 | 先验 CRC（frame.js 生成），再查白名单行 |
| 全静默零字节 | 供电/接线/睡眠 | 查供电，禁判设备死 |

**踩坑实证（1024 踏频 COM22）**：`0xAF` 有回显、`0xAC`/`0xCF` 零回显，实际是板跑 11-04 前构建，旧白名单 `uart_cmd.c:105-106` 缺这两个命令字节 → 命中 `queue_remove(q,1)` 逐字节静默丢弃，**连副作用都没有**（不是"收到了但没打印"）。重烧 `kmo_cadence/kamingo_v21_1024/.../output/app/bk3633_app_merge_crc.bin`（11-09 22:22，287,712B，md5 `47ee75f5efa64d99c1a9fdfade091a7d`）后五条命令回显全出。

**★版本号不能当构建判据**：1024 树上 `ver=1.18` 同时存在于新旧两个构建（`app/api/app_ota.h:37-39` 的 `FW_VERSION_MINOR 18` 未随 0xAC/0xCF 加入而递增）。判构建只能：
1. `grep -a -o "set_pair_addr slot=" <bin>` / `grep -a -o "BCNAD" <bin>` 有没有这两个字符串；
2. 或发该命令看回显。

**重烧后必做**：`0xAC slot1/slot3` + `0xCF` 三条重发（重烧会清 flash 里的配置尾块范围内的地址？——不会，但**同版本号包可能覆盖配置区**，实读确认最省事），然后 `0xA0` 重锚，再验 `p1`。

## 3. 换信标族后：踏频必须重锚

改 `0xCF` 后踏频会失锚且不自愈（实测 `mxage=12278ms`、`drift≈6.8万`、`gate=0`）：

```bash
# 发 0xA0 软复位 ×8，等 8 秒
# 判据：RF Config: Loaded from Flash + pair=OK + gate=1
```

踏频地址自 2026-11-03 起为 **flash 持久化**（修复 `calc_checksum` 用 `offsetof(checksum)` 前，断电必丢）；重锚后无需重配。**重烧固件**才需要重配一轮。

## 4. 档位（RAM 态，任何一机重新上电都要重下）

> **★★2026-11-10 血案：踏频 HIGH 能不能打进去，取决于烧的是哪棵树。** 本机有**三棵**踏频树，
> 同一块板烧不同树，能力完全不同——选错的表现是「HIGH 三连零回显、p1 卡在 15/s 上不去」：

| 树 | 路径 | `0xAC`/`0xCF` | `0xD1`/`0xD3`/`0xD4`（HIGH） | IMU |
|---|---|---|---|---|
| `kmo_cadence/kamingo` | `kmo_cadence/kamingo/projects/.../output/app/` | ❌ 白名单只 9 条 | ❌ 无 case | ✅ |
| `kmo_cadence/kamingo_v21_1024` | `kmo_cadence/kamingo_v21_1024/projects/.../output/app/` | ✅ | ❌ 无 case | ❌ **六轴全零**（SPI 引脚不匹配） |
| **`cadence/`（主线）** | `cadence/output/app/` | ✅ | ✅ `CMD_TX_RATE:285`/`CMD_TX_PHASE:302`/`CMD_TX_TICK:325`，白名单 `uart_cmd.c:123` | ✅ |

**判据速查**（2026-11-11 核实行号后修正）：
1. **`[H] mode=G24 … ver=1.18` 只有 `kamingo`/`1024` 打**（`kamingo` `main.c:1722`、`1024` `main.c:1126`）——**看到它就说明不是 `cadence/`，HIGH 必然打不进去**。`cadence/` 主线打 `[CSl]`/`[CD]`/`[CDg]`/`[PHOu]`/`[RTC]`，不打 `[H] mode=`。
2. `0xCF` len=0 有回显 ⇒ 有 `0xAC`/`0xCF`；`0xD1` 回 `[CMD] tx_gap=10ms` ⇒ 有 HIGH
3. `[TX] gx=0 … az=0` 全零 ⇒ 烧到 1024 树了（IMU 引脚不对）
4. 版本号 `ver=1.18` **不能**区分 kamingo 与 1024（两者都 MINOR=18），只能靠 IMU 是否全零

**实测差**：`kamingo` 树 `txds=14.4/s`、`p1=15.0/s`；换 `cadence/` 树 + HIGH 后 `txds=40.07/s`、`p1=43.20/s`（p99=50，零值秒 0/30）。

| 档 | 遥控（COM99） | 踏频（COM22） |
|---|---|---|
| LOW 默认 | `0xD1 0` `AA55D10002D200007CC9` | `0xD1 0`（回默认 80ms） |
| **HIGH** | `0xD1 25ms` `AA55D10002D219007759` + `0xD2 1` `AA55D20001D20169BD` | `0xD1 gap10` `AA55D10002D20A007A69` + `0xD4 tick5` `AA55D40002D50500CE0D` + `0xD3 [5,8]` `AA55D30002D405089FBC` |

> `0xD1` 带 2 字节 = 设置；**2 字节全 0 = 回默认**（不是查询！）；查询用 len=0 裸帧 `AA55D10000D02CBD`。

## 5. 验收（不达标就停，别继续加步骤）

| 项 | 判据 |
|---|---|
| 桥 | `DEAF=0`；`bm` 由数据决定（0=混合常态，2=电机/巡航帧自然进入），不许 0↔2 抖动；持续刷 `DEAF` = RX 楔死，发 `0xA0`，再不行断电 5s |
| p0 | HIGH ≥20/s（目标 23~26）、默认 ≥7/s |
| p1 | HIGH ≥38/s（目标 42~45）、默认 ≥7/s。**达不到先查烧的是哪棵树**（§4）与 HIGH 是否重发，**不是**因为曲柄没转（§5d） |
| 遥控 | `anch=1`、ch 每秒变、无长期 `send-stall defer=1` |
| 踏频 | `pair=OK`、`gate=1`、`r∈[90,113]`、`xi≈0` |
| 回读 | 桥 `0xAF` 两槽 valid=1；三端 `0xCF` 同址 |

> **★判死前先量桥信标率（2026-11-17）**：遥控 `anch=0 bm=255 ch=20 rxdr=0` 持续几十秒 = **一次信标都没收到**，不是板坏，`0xA0` 也救不回来；先数桥 12s 内 `[BEACON]` 行（正常每秒数次，实测坏态只有 1 次/12s），信标稀疏就去查族/是否配对后未对齐，别先给遥控判死。
> **★踏频跑 IMU 数据流固件时 `p1` 会暴涨到 1000+/s**（`[TX] gx=… ay=…` 每行带六轴），此时 p1 阈值判据失效，改看 `[TX]` 行判断它在发什么。
> **`--tier LOW` 只配址会必然验收 FAIL**（实测 p0=0.7 / p1=0.8）：档位段不是可选优化，是链路生效条件；嫌 HIGH 副作用大就先配址再单独评估，别把它当失败信号。

### 5b. ★★ `bmode tkw … rdy=0` **不是故障**（2026-11-10 定谳，别再当硬阻塞）

`bridge_mode_rf_takeover_ready()`（`bridge_mode_mgr.c:1719-1732`）要求

```
mode==RF_ONLY && rf_session_active==0 && app_env.connected==0 &&
(adv_state ∈ {IDLE, CREATED(4), STARTING(5), STOPPING(7)+complete锁存})
&& (adv_state==IDLE || adv_stop_complete_latched)
```

**`adv=6 (APP_ADV_STATE_STARTED)` 不在允许集合内**。`bmode tkw adv=6 … rdy=0` 是**广播真正在跑那一瞬的采样**，属正常有害噪声；真正的接管发生在 `adv=4`。

判「射频到底有没有接管」只看这三条，不看 `rdy`：

1. `[WIND] take init=… total=…ms` —— **`total` 应是几百 ms 量级**（正常实测 108200ms 的一次 take 属窗口内累计，健康）；若 `total` 只有 **20~40ms** 且 `p0=p1=0` → 窗口坍缩，真问题在这。
2. `[HB] … p0=/p1=` 是否非零（>7/s）。
3. `yld=` 是否持续非零暴涨 + `[MODE] rf-svc yield` 循环。

**窗口坍缩的唯一已验证解法 = 桥断电重上电**（2026-11-10 实证：坍缩态 `[WIND] take total=30ms`、`bm=2 rs=3`、`p0=p1=0`；断电重上电后 `total` 正常、`p0` 立刻 18~26/s、`rs` 变 11）。**禁**用改地址 / `0xAB` / `0xA5` 去"治"它。

### 5c. `p1` 达不到 HIGH 目标 ≠ 配置错（先分「树 / 档位 / 曲柄」三档）

`p1` 是踏频**业务帧**上桥计数。达不到 38~45 时按序排除：

1. **烧错树**（最常见，占 2026-11-10 两次实战全部）：`0xD1` 无回显 ⇒ 该树没有 HIGH case ⇒ 换 `cadence/` 树重烧。见 §4 的三棵树对照。
2. **HIGH 没重发**：`0xD1/D3/D4` 是 RAM 态，任何复位/重烧后必重下。判据是三条 `[CMD] tx_gap / tx_tick / tx_phase` 回显。
3. **曲柄没转**：`[TX] rpm=0` 时业务帧里的踏频值恒 0，但**帧照发**（见 §5d），所以这一条**不会**让 p1 掉到心跳级——若 p1 真的只有 7~13/s，病因在 1 或 2，不在曲柄。

### 5d. ★「rpm=0 不发业务帧」是**假结论**（2026-11-10 证伪）

`main.c:564` 那句注释「静止时业务帧沉默, 靠它保活防误判离线」来自 `cd2744a` initial import，
**代码从未实现该行为**：

- `rf_tx_task()`（`APPLICATION_MODE_TEST.C:907`）唯一早退是 `cad_hop_tx_prepare()`（未入锚/学习期/配对窗内），**不检查 rpm**。
- `app_g24_service_tx()`（`main.c:556`）每 30ms 无条件调 `rf_tx_task()`。
- **实测反证**：rpm 全程 0 的 30 秒窗内，踏频 `txds=14.43/s`、桥 `p1=15.77/s`。

**该注释是过期注释，不可作为「p1 低」的解释**（handoff 1412 的「22 静止不发业务帧 → 桥 p1=0 是必然」据此作废；
当时 p1=0 的真因是踏频板跑旧构建 + 地址未对齐）。

## 6. 烧包

```bash
# 遥控/踏频：wdt（默认，零按键）
py <Tools>/self_flasher/self_flasher.py --port <数字COM> --wdt --app-baud 1000000 --bin <xxx_merge_crc.bin>
# 只认 *_merge_crc.bin（裸 app bin 直烧 = 假砖）
```

> **`--port` 只吃数字**（`--port 22` 而非 `--port COM22`），传 `COM22` 会报 `invalid int value` 白跑一次。

**桥（COM43）在本机烧不了**：`--mode dap-rst` 撞窗 10/10 未中（DAP 已按 serial 配对、复位确曾生效，但 LinkCheck 无应答——疑 RST 未接 RSTN 或原生 USB 复位重枚举）；`--wdt` 在混合态不回 `wdt_feed=0`。**桥烧录需人工/换通道，别反复空撞。**
桥无 BUILD/版本行，只能凭 `[HB] svcmax/rfst` 判跳频线；确切 bin 需离线读 flash 比对。

## 7. 坑（按出现频率）

1. **混合态回显命中率约 50%** → 重发到见回显即可。**`0xA5` 不预发、但症状成立时必须发**：`bm=0/1` + 零 `[BEACON]` + `p0=0` 三条同时成立才钉（2026-11-11 实测 108：钉前 p0=0，钉后 23.0/s）；无这三条而钉 = 干扰正常模式机（详见 §0d 表）。
2. **换信标族后踏频失锚** → `0xA0` 重锚（不自愈）。
3. **同族双套同场互踩** → 指纹：p0 塌到 2~3、踏频 2s 间隙、drift 暴涨、`[CSl] ds` 反向跳。分族解决。
4. **踏频 `0xAC` 用 slot3 当 TX 目标** → p1=0；必须 slot1。
5. **遥控 `PAIR_ADDR_NOT_READY`** → esc_addr 没设，发 `0xB6`。**注意：遥控 `0xAF` 查不到 flash 里的 esc_addr**（只回 RAM 最近配地址），只能用 `0xB6` 回显 `esc_addr set:` 判定。
6. **配完不补 `0xA7`/档位** → 桥重启后 motor 分支误判、速率回默认。
7. **端口被占（PermissionError 13）** → 先怀疑**自己上一轮刚关的句柄**：Windows 释放有秒级延迟，同进程 `close()` 后立刻 `open()` 必失败（2026-11-17 连撞 3 次；隔 6~10s 再开即好）。规程：**一个端口一次只留一个句柄**（发+读同一句柄），跨脚本间隔 ≥5s，重试 ≥6×1.5s；仍被占再问用户关串口助手，不要换口猜。
8. **`bm` 是结果不是目标** → 不许为了拿到 `bm=2` 去发 `0xA5`；`bm` 由注入/配对/巡航数据自然决定，验收只看它是否稳定（不 0↔2 抖动）。

## 8. 2026-11-10 夜实测修订（本套 108/74/95，覆盖上文相关判据）

> 场景：本夜按「核 C1 三端址 → 补 HIGH → 真实 APP N=10」的接续注入开工，实测把上文若干条推翻。**下文与上文冲突时以下文为准。**

### 8.1 本任务套的族 = BE（不是 C1）

- `108/74/95` 这套实测：遥控与踏频 `0xCF` 各自独立回读均 `cur=88 10 82 07 84`；踏频 `0xAF` = `PAIR_OK RF_ADDR=7F:BE:22:22:22`。
- 接续注入写的「C1 三端同址 `88:13`」属 **43/22/103** 那套（已作废端口）。**照注入跑 `--family C1` = 把桥 slot1/slot3/信标整族改写**（§1 的覆盖性写入）。
- 核族只信两处：`0xCF` 回读（遥控/踏频）+ 踏频 `0xAF`。桥无回读，别猜。

### 8.2 ★ 失败配对会把桥打进 BLE 死态（软件救不回）

- 症状：`[HB] adv=6→7`、`yld≈900~1000`、`sess=1→0`、`p0=p1=0`、**`[BEACON]` 行消失**、`up` 持续增长。
- 判据：`0xA0`×8 后 **`up` 不归零**（实测 10382s→10515s 继续涨）⇒ **桥的 CMD_REBOOT 无效**；`0xA6`/`0xAB` 禁用 ⇒ **软件无出口**。
- 唯一解：**桥断电重上电（pw-rst，人工）**，与 §5b 的窗口坍缩同解但成因不同。
- 推论：**无人值守不能跑配对压测**。过去 767/790 能跑完 = 失败率低 + 有人守台架。
- 工具侧硬要求（已实现在 `tools/app_battle/n2_pair_baseline.py`）：①开跑前 `p0≥7` 否则 `exit=3` 不空跑；②失败轮自动 `0xA0`+10s 重验；③仍 `p0=0` 立即 abort 并记 `bridge_no_rf_abort`。

### 8.3 ★ `txds` 不是上空气帧数；本套 `p1` 天花板 ≈7/s

- 3×60s 同配置：踏频 `txds = 61.3 / 10.5 / 81.9`（差 8 倍），而桥 `p0 = 21.65/21.94/20.77`、`p1 = 6.97/7.17/6.25`（稳）。
- `swtx/txds ≈ 3.2` ⇒ **`txds` 含重试**，不能当发送速率。
- 结论：**§4「HIGH 后 p1=43.20/s」与 §5「p1 HIGH ≥38」在本套不成立**；把 `txds` 拉到 38 只让 `p1` 从 6.4 到 9.0。
- 本套验收门改为：`p1 ≥ 6/s` + `p0 ≥ 20/s` + `bm` 含 2 + `DEAF=0`。**`txds` 不再作为 HIGH 判据**。

### 8.4 树身份判据表（§4）失真，别再用来定性

同一块 95 同时呈现：`[H] mode=G24 … ver=1.18`（§4 说是 1024 特征）+ `0xD1` 改变 txds（§4 说只有主线有 HIGH）+ IMU `gx=56 gy=-56 gz=-77` 非零（§4 说 1024 六轴全零）。→ **三树判别法不可靠**；只能看 bin 的 md5 或"命令效应"，不能看启动打印。

### 8.5 `0xAF` 看不见 slot1 的写入

- 实测：`0xAC slot1` 写 `7F:EE:22:22:22`（连发 30 帧）后，`0xAF` 仍回 `7F:BE:22:22:22`。
- 原因（代码侧）：`0xAF` 走 `rf_config_read_device_addr(DEV_TYPE_ESC)`，而本构建不打 `[H0]`（含 `addr=` 的那行），所以 **TX 目标只能靠桥侧 `p1` 反证**，不能靠 `0xAF`。
- 推论：`0xAF` 回 `PAIR_OK` **不能证明 TX 目标正确**；失锚态下它照样回 `PAIR_OK` 而 `txds=0`（本夜 D0 样本）。

### 8.6 配对成功会自动改写族地址

配对成功后两端地址被同步成派生值（本夜实测 `B4:37:BA:3D:A0`）。⇒ 任何「先核三端同址、再配对」的流程顺序要倒过来；也解释了为何"核地址"在配对之后总要重做。

### 8.7 踏频 beacon 写 flash 在复位后保持

`0xCF` 写 `88 10 82 07 85` → 立即回读 85 → `0xA0`×8 → 仍 85 → 还原 84 成功。⇒ §3 的「换族必重锚」只针对"失锚"，**不针对"没写进 flash"**。

### 8.8 工具侧两个坑（已修）

- `tools/app_battle/pair_recipe.py` 原把 `rts/dtr=False` 放在 `open()` **之后** → 开串口瞬间先复位一次被配端。已改为 open 之前。
- `tools/app_battle/pair_bench.py` 的 `default` 档缺 `pair_ok` 键 → `KeyError`。已补。

### 8.2 ?????2026-11-10 22:49 ??????????
- ??????20:18 `p0=0`????? 22:49 `p0=24 / p1=12`?? `up=19,655,685ms`?5.46h?? 17:15 ?????? **??????**?? BLE ?????????
- ??? `0xA0`?8 ? `0xB4`?12 **????**?`[HB] yld?900~1000` ?????????????????"?????"??? **"???????"**???????? 1~2.5h ????????????????
- ??????????`develop_log/reference/workflow/commands.md` ????? ? ??????????**?=DAP-rst?RTS ?????10-29?**???=RTS+?????=wdt??? `0xA0` ???????app+???????
