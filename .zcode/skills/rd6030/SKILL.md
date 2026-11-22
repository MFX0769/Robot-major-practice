---
name: rd6030
description: RIDEN RD6030 数控直流电源（60V/30A）操作、共享监控、悬浮窗与急停。当用户说"电源/RD6030/可调电源/盯电源/监控电源/电源报警/断电/急停/悬浮窗"或要做供电值守、报警联动时使用。铁律=默认只读；写（输出开/关）须 hub --allow-cutoff+明确确认。本地资料中心=SDK/projects/Tools/rd6030/（勿再上网搜文档）。
---

# /rd6030 — RD6030 电源操作与值守

**资料中心（先看这里，勿再搜网）**：`D:\Kamingo\bk3633_main_my\SDK\projects\Tools\rd6030\`
- `rd6030_hub.py` 共享集线器（独占串口自轮询，WS 状态流 `ws://127.0.0.1:8990`，默认只听回环，`--lan` 才开放）
- `rd6030_mon.py` 监控/报警器（代理 A 主入口；**完整手册=`--help`，自足勿读源码**）
- `rd6030_panel.py` 悬浮窗（人用：右下角常顶实时 V/I/P + 急停/开启按钮，hub 不在会自动拉起）
- `rd6030_read.py` 只读单发工具（早期版，日常用 mon）
- `README.md` 设备事实卡（SN 6042 / FW V1.47 / 定标 ×0.01）｜ SOP=`rc_remote/develop_log/sop/rd6030_power_supply_sop.md`

## 快速上手

**人（最快）**：桌面「RD6030悬浮窗.bat」双击——自动起 hub（带 --allow-cutoff），显示实时电压电流，红=急停 绿=开启。桌面「RD6030上位机」=官方 GUI（需停 hub 独占串口，详见 SOP §三B）。

**代理 A（三步）**：
```bash
# ① 起集线器（若面板已拉起可跳过；端口 8990，8989=示波器桥勿占）
python Tools/rd6030/rd6030_hub.py --port COM106            # 找口先跑 mon --list
# ② 单次体检（退出码即结论）
python Tools/rd6030/rd6030_mon.py --hub ws://127.0.0.1:8990 --oneshot --json   # 0 正常/3 报警/2 hub 不在
# ③ 后台值守（阈值按任务定；值守自动防 PC 睡眠）
python Tools/rd6030/rd6030_mon.py --hub ws://127.0.0.1:8990 -w 2 \
    --vmax 32 --vmin 5 --require-on --events events.jsonl
```

## 报警与输出控制

- 事件：VMAX/VMIN/IMAX/IMIN/PMAX/OUT_OFF/OUT_ON/CC_LONG/PROT_OVP/PROT_OCP/NO_RESPONSE/PORT_LOST；防抖+冷却内建
- 通知：`--events` jsonl / `--csv` 全样本 / `--beep` / `--exec CMD`（事件 JSON 作末参）/ `--exit-on-alarm`
- **输出开/关=唯一写**（reg18）：hub `--allow-cutoff` 闸门 → mon `--cutoff --yes` / `--on --yes` / `--auto-cutoff`，或面板两按钮。**写路径双向拒绝已实测，真实写入未实测**（首测选空载窗口，或直接点面板急停看输出灯灭）

## 铁律与边界

- **默认 100% 只读（FC03）**；电压/电流设定值任何脚本都写不了（工具链未实现）
- **负载电流判读下限（2026-10-28 用户口述）**：GD 板静态电流 ≈44 mA，RD6030 非低功耗分析仪、此量级读不出——`i_out=0.00` **≠** 断载/没电；判板子供电只看 `v_out` 正常 + 终端在线，禁止用 i_out 反推接线状态
- hub 默认只听 127.0.0.1；`--lan` 开放前想清楚（远程可断电）
- 端口共享：多脚本=hub 真共享；官方 GUI=物理独占，交接=停 hub 再开 GUI
- 找口唯一判据=FC03 回 ID 60301（`mon --list`），口号非恒久；hub 失联自动重扫口（换号自愈）；扫口并行限时防驱动挂死
- 联机后面板锁键 3s 自解；上位机校准菜单（168168）**禁动**
- 值守防睡眠 mon/panel 已内建（SetThreadExecutionState）

## 维护

- 换口/换固件后：`mon --list` 重认口 + 更新 README 事实卡
- 改 hub/mon/panel 后：重跑 SOP §三B/§三C 测试清单（T1-T17 口径）
- 本 skill=正本；四仓（rc_remote/kmo_mc_rcv/kamingo_gd32/kmo_cadence）`.zcode|.agents/skills/rd6030/` 有分发副本，改正本后机械复制 5 处（清单=SOP §八）
