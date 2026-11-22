# AGENTS.md — taurus 课设小车工程手册

> 2026-11-22 自 AI_Harness 全量包 v3 吸纳。通用协作习惯正本；Kamingo 工程专属内容未搬。
> 工程硬件规则正本仍是 `CLAUDE.md`（主控平台/日志策略/排查清单）——两文件分工：本文件管"AI 怎么协作"，CLAUDE.md 管"这个工程是什么"。

## 协作双模式（全仓适用）

- **模式① 人在电脑旁**（工作日白天常规）：用户在线盯屏。AI **不擅自抓数据、不擅自运行指令**；只做明确指令的事，快响应、快恢复，**严禁发散思考**（除非用户明说"分析/想想/查原因"），不要 hmm/but wait 式犹豫。
- **模式② 通宵/远程实验**（用户不在场）：AI 自主迭代执行实验；判据/停止条件仍须预注册。
- **模式判定时间默认**：工作日上班时段至 20:00 默认模式①；此外时段（20:00 后/周末/节假日）默认用户不在场——抓数据/开长任务前先确认在场；用户明确指示永远覆盖时间默认。
- **串口抓取红线（模式①）**：要抓数据必须用户明确下令；抓取前说明：执行时间不固定、可能占用串口/阻塞实验窗口。

## 输出契约

全局 `~/.zcode/AGENTS.md`（每条回复生效）：先结论后支撑、上限用数字不用形容词、不写客套/复述/自辩、长指令末尾重申关键约束。

## 硬规则（踩过的坑，勿犯）

1. **黄金排查原则**：开环能转闭环不动 → 先查 PID 限幅参数，不先动螺丝刀（6 月右前轮 3 小时弯路的教训）。
2. **改 UART 分配后**：grep 全局 `huart` 确认所有引用点同步。
3. **Keil 增量编译**不认外部编辑改动：编译日志确认有 `compiling xxx.c...`，否则 `-r` 全量重编。
4. **OpenOCD 烧录**必须确认 `wrote xxx bytes`；"failed erasing sectors" 不算成功。
5. **一个逻辑一个 commit**；不用 `git add -A`，明确列路径。
6. **CubeMX 生成文件只动 USER CODE 块**。
7. 引脚事实来源=官方例程 `.ioc` 文本解析（勿信视觉读图）；本工程引脚表见 `Develop_Log/20261122_MC02移植方案.md` §6。

## develop_log 习惯（v2 双轨说明）

- **`Develop_Log/`**（大写，进 git）：本工程调车日志正本，按日期命名（`2026xxxx_主题.md`），append-only。
- **`develop_log/`**（小写，**不进 git**）：AI Harness 包带来的完整习惯结构（README 分类规则 / daily / handoffs / knowledge/INDEX / sop / decisions / lessons / reference）。**注意：其中历史内容来自 rc_remote/Kamingo 工程，是结构样例与习惯参照——查规则读它，查本车事实读 Develop_Log/ 与 CLAUDE.md。**
- 每会话收尾用 `pre-compact` skill 补日志；重要决策一决策一文件进 decisions/。

## 烧录与验证铁律

- MC02 板烧录走 Keil Download（板载调试口），确认烧录日志成功
- taurus 旧板烧录：`python Tool/debug/stm32h7_tool.py flash`（OpenOCD，见 wote 字节确认）
- 串口健康打印：启动横幅 + 1Hz 健康包（设计见 `Develop_Log/20261122_MC02移植方案.md` §7）
