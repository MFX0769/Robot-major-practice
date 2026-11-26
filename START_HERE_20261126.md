# 明天上车：只从这一个仓库恢复全部内容

远程：`https://github.com/MFX0769/Robot-major-practice.git`

- 主分支 `master`：全部日志、交接、3235 行代码架构与调试说明书、任务书、调研归档。
- 分支 `taurus-h7-all`：完整固件仓（源码、Keil 工程、hex/axf/map、主机测试台、构建日志）。

## 新电脑两条命令

```bash
git clone https://github.com/MFX0769/Robot-major-practice.git Robot-major-practice
git clone -b taurus-h7-all https://github.com/MFX0769/Robot-major-practice.git taurus_h7_all
```

## 立即核对

```bash
git -C Robot-major-practice log --oneline -1
git -C taurus_h7_all log --oneline -1
sha256sum taurus_h7_all/MDK-ARM/CtrBoard-H7_ALL/CtrBoard-H7_ALL.hex
```

固件应为：

- 分支 `taurus-h7-all`
- HEAD `c1ceee7`
- hex SHA256 `d197a46fe5c5f08a2bcf5464daeef9a16fb4ee5b3890319a3102522e37348153`

## 先读哪两份

1. `Robot-major-practice/Develop_Log/20261126_代码架构与两天调试说明书.md`
   - 第 0 章：总顺序和六条命令。
   - 第 3 章：所有门控及拒绝码。
   - 第 17 章：下午逐阶段调试剧本。
   - 第 18 章：故障树。
   - 第 24 章：源码行号索引。
2. `Robot-major-practice/Develop_Log/handoffs/handoff_20261126_明天上车_固件交付与应急.md`
   - 分步上电与应急表。

## 上板前先跑

```bash
cd taurus_h7_all
py Tool/hosttest/run.py
```

期望 `[RESULT] ALL PASS`，630 项 0 失败。

## 上板后第一条命令

```text
diag
```

它显示模式、最后拒绝原因、舵机/yaw 在线、视觉年龄、巡线八路原值与门限、DBUS 帧统计、日志和 USB 丢弃。
任何“设备不动”先看 `diag`，不要连续试命令。

## 当前硬件状态

所有硬件项未实测：CAN1/CAN2 实收实发、四轮方向、6020 PID、舵机带载、夹爪、巡线、TF-mini、避障距离、视觉精度、Flash 写入停顿。
未实测不写“通过”。

## 约束

- 不 push 到未确认的远程。
- 不改 CAN 位定时和 0x1FF/0x208 协议。
- 一次只改一个变量。
- 上车第一件事按上面的 hex 哈希确认烧的是这一版。
