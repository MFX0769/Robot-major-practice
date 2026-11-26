---
name: sa-spectrum-analysis
description: 频谱仪路测 CSV（sa_*.csv，ZS-406/tinySA 全带扫描帧表）离线分析——从原始数据一键得到环境指纹（平均场/瞬时场/异常门/综合判读）。当用户说"分析频谱数据/路测频谱/噪声指纹/sa.csv/频谱仪数据/桥底车流噪声/环境干扰指纹"，或拿到链路台视图 F 录制的 sa_*.csv 要做环境判读时使用。**只做离线分析，不操作仪器**（仪器操作用 skill zs406）。输出 JSON + 人读报告；后续 agent 跑一条命令即可，不必手看原始数据。
---

# /sa-spectrum-analysis — 频谱 CSV 环境指纹分析

> 正本：SOP = `develop_log/sop/SA频谱数据解析SOP_20261120.md`（格式/口径/检查清单/坑全量）；脚本 = `tools/sa_spectrum_analyze.py`。本 skill 只给最短路径，细节查 SOP。
> 适用数据：链路台（oscilloscope）视图 F `Recorder.register('sa',...)` 录制的 `sa_*.csv`，一帧一行宽表（列格式正本=`cadence/tools/oscilloscope/web/solver.html` `[CSV-REG] 'sa'`）。

## 快速上手（一条命令）

```bash
python tools/sa_spectrum_analyze.py <sa.csv>
# 默认 --expect-band 2.4G --segments 6 --strong-db 8 --tz +8
# 产出 <csvstem>.sa.json（机器可读）+ <csvstem>.sa.md（人读报告）+ stdout 摘要
```

常见变体：

```bash
# 怀疑仪器频段设错（如 100–200MHz）→ 自定义门，错配会 FAIL 拦下
python tools/sa_spectrum_analyze.py <sa.csv> --expect-band none --band-low 100 --band-high 200
# 改分段/时区/强点门/输出目录
python tools/sa_spectrum_analyze.py <sa.csv> --segments 10 --tz 0 --strong-db 10 --out-dir D:\out
```

Windows 真 python（本仓）：`C:/Users/Administrator/AppData/Local/Python/bin/python.exe`（用 `-I` 隔离防脚本目录 planted 模块）。

## Agent 工作流（拿到 sa.csv 后照做）

1. **先读 SOP** `develop_log/sop/SA频谱数据解析SOP_20261120.md` §二（列格式）+ §五（检查清单）——不要直接写临时脚本（2026-11-19 首轮的教训）。
2. 跑上面的命令 → **查退出码**：0=全绿 / 1=WARN（可用但降级解读）/ 2=FAIL（数据作废，按门动作修数据源重跑，禁止解读）/ 3=用法错。
3. exit 0/1 → 读 `.sa.md` 的 §1（结论）+ §7（给 A 君判读）。
4. 对话交付：先给 1–3 句结论（平均场/瞬时场/定性），再给数据身份（帧数/频段/时段）——**文件是存档，结论完整写在对话里**。给 A 君的完整判读引用 §7。
5. 机器消费读 `.sa.json`（字段见 SOP §4.3）。

## 判读红线（脚本已落实，agent 别绕过）

- 主判据=median/MAD/modified-z（0.6745/门3.5），**不用 mean/std 判环境**；大平均只作辅助对照。
- 强点=本底中位+8dB（zs406 skill 口径）；离群=|修正z|>3.5；两者独立，**离群≠坏数据**（强占用环境里离群多为真实突发）。
- 时序先分段（默认 6 段）再判；全程平均会抹掉"拥挤→安静→拥挤"。
- 参数从数据自识别（n/f0/df），**禁止写死 2400MHz/145 点**；频段错配由 `--expect-band` 门 FAIL 拦。
- 无场景标签的录制**不能指认"桥底/车流旁"**，只能给沿线混合指纹；SA 无 Mac 层解析，不归属具体设备。
- 每段结论带数字（median/MAD/峰值/强点%/串数），不写"吵/干净"类形容词代替数字（全局输出契约）。

## 校准锚点（回归基准，改脚本后必跑）

`C:\Users\Administrator\Desktop\rec_20261115_002355\sa.csv`：3,583 帧 / 39 点 / 2402–2442MHz / 本地 11-15 00:23:54–00:54:51；底噪中位 −89.3dBm（MAD 0.40）、峰值 −42.3dBm、强点 1.57%、421 串、异常门 FAIL0/WARN3/PASS7。复现不出这些数字=口径变了，先查脚本常量。

## 已知坑（速查，全量见 SOP §六）

- solver.html 注释的 145 点/2400MHz 是默认列宽，**不是本次真值**；本次实扫 39 点/2402MHz，后 106 列全空。
- manifest 用 UTC，报告默认 UTC+8；`16:23Z` = 本地 `00:23`，用 `--tz` 对齐目录名。
- 空文件/坏路径/错配频段分别 exit 2/3/2，不要强行解读 WARN 以下数据。
