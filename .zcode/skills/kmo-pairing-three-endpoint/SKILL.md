---
name: kmo-pairing-three-endpoint
description: 在任意台架主机上用真实 Android Kamingo App、BK3633 桥和遥控器重复执行三方解绑与配对，并保存三端原始日志。适用于手机 App 配对复测；PC 扮 App 的 BLE 压测使用仓库内 pair-stress skill。
---

# 真实手机 App 三方配对

本 skill 随桥接仓库分发。路径一律相对仓库根目录；不要使用某台电脑的用户目录、绝对路径、固定 COM 口或历史 ADB 序列号。脚本在 `scripts/pair_cycle.py`，依赖见 `requirements.txt`。

## 准备

需要 Python 3.10+、Android platform-tools (`adb` 在 PATH，或用 `--adb` 指定)、两路 3.3 V USB-UART、已授权 ADB 的 Android 手机，以及桥和遥控器。**这是跨电脑可运行，不是脱离台架主机运行。**脚本不烧录、不复位、不发电机、减速器或标定命令；解绑与配对会改变手机和设备的绑定关系，只在本次测试获授权时运行。

从仓库根目录安装依赖：`python -m pip install -r .agents/skills/kmo-pairing-three-endpoint/requirements.txt`。用 `python -m serial.tools.list_ports -v` 和 `adb devices` 识别设备。将 `bench.example.json` 复制到仓库外或未跟踪的 `bench.json`，填入本次台架的桥与遥控器 **USB 序列号**。脚本据此自动发现 COM 或 `/dev/tty*`；串口没有唯一序列号时，才用 `--bridge-port` / `--remote-port` 显式选口，并人工核对物理连接。COM 号、BLE 显示名和旧证据不能单独证明板卡身份。不要结束其他程序来抢串口。

```text
python .agents/skills/kmo-pairing-three-endpoint/scripts/pair_cycle.py --bench bench.json --preflight
python .agents/skills/kmo-pairing-three-endpoint/scripts/pair_cycle.py --bench bench.json --rounds 10 --out pair_evidence/run01
```

`--preflight` 只枚举串口、ADB 手机并检查 App 进程；不打开串口、不解绑。运行前在手机上确认目标桥的实际 BLE 地址、App 页面和设备状态；遥控器油门松开、电机静止。当前 UI 适配器按中文 Kamingo App 的“添加设备 控制器”“开始绑定”“知道了”“解绑”等控件文字查找，App 改版或翻译后先核对页面，再修改适配器；不能盲点固定坐标。

## 触发、判定和证据

仅当**选定桥串口**报告 `app_cfg_rx: cmd=0x00E0` 时，脚本才向选定遥控器 UART 发配对帧。手机收到 `0x0160` 不能代替本桥收帧证据。遥控器每次触发先发送唤醒字节；0.8 秒内无 `[CMD] pair triggered by uart` 回显时最多补发两次。主机 `write()` 成功不等于遥控器收到。

每轮分别检查：手机向目标桥发请求、桥和遥控器 2.4G 配对成功、遥控器 BLE 窗口与模式、手机连接正确遥控器并完成绑定。脚本保存 `identities.json`、手机/桥/遥控器原始 `.bin`、同步时间线、`round_XX.json` 和 `SHA256SUMS.txt`。按 `round_XX.json` 和三端原始日志报告，不把页面显示成功或单端 RF 成功当作三方通过。

页面无法识别、设备身份歧义、串口掉线或安全恢复失败时停止并保留部分证据；后续用新输出目录和 `--start-round` 续测，不能覆盖失败轮。手机选错桥、遥控器无串口回显、BLE 窗口被断连取消应分开归因。要只读分析时不要运行配对脚本。
