# VS Code OpenOCD GUI界面使用指南

## 工作流程

**AI脚本控制OpenOCD → VS Code GUI显示 → 您观看**

## 步骤1：安装VS Code插件

1. 打开VS Code
2. 转到扩展视图 (Ctrl+Shift+X)
3. 安装以下插件：
   - **Cortex-Debug** (推荐)
   - **C/C++** (Microsoft)
   - **ARM Assembly** (可选)

## 步骤2：配置OpenOCD

### 方法A：使用Cortex-Debug插件（推荐）

1. 使用我生成的 `.vscode/launch.json`
2. 按 F5 启动调试
3. VS Code会自动启动OpenOCD并连接到目标板

### 方法B：手动启动OpenOCD

1. 打开命令行
2. 运行：
   ```bash
   openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg
   ```
3. OpenOCD会启动GDB服务器 (端口3333) 和 Telnet服务器 (端口4444)

## 步骤3：AI控制OpenOCD

### 方式1：使用控制脚本（推荐）

```bash
# 在项目目录下运行
python tools/debug/openocd_controller.py
```

**可用命令**：
- `interactive` - 交互式控制模式
- `reset` - 复位设备
- `demo` - 运行示例演示

### 方式2：交互式控制

```bash
python tools/debug/openocd_controller.py
> interactive
```

**控制命令**：
- `reset` - 复位设备
- `reg pc` - 读取PC寄存器
- `mem 0x08001000` - 读取内存
- `bp 0x08001000` - 设置断点
- `continue` - 继续执行
- `halt` - 暂停执行
- `flash firmware.bin` - 烧录固件

## 步骤4：在VS Code中观看

### GUI界面元素：

1. **调试视图** (侧边栏)：
   - 变量监视
   - 调用栈
   - 断点列表
   - 寄存器视图
   - 内存视图

2. **调试控制面板** (顶部)：
   - 继续/暂停
   - 单步执行
   - 重启/停止

3. **外设寄存器视图**：
   - 实时查看STM32外设状态

## 工作示例

### 示例1：查看寄存器

1. **AI执行**：`python openocd_controller.py` → `interactive` → `reg pc`
2. **您观看**：VS Code调试视图中的寄存器面板

### 示例2：设置断点

1. **AI执行**：`python openocd_controller.py` → `interactive` → `bp 0x08001000`
2. **您观看**：VS Code断点列表中显示的断点

### 示例3：单步调试

1. **AI执行**：继续执行命令
2. **您观看**：VS Code中代码执行位置高亮

## 日志记录

所有AI操作都会记录到：
```
D:\RM2026\homework\RoboticMajor_Pratice_code\develop_log\openocd_control.log
```

## 优势

1. **AI控制**：自动化调试过程
2. **GUI观看**：直观的可视化界面
3. **随时干预**：可以随时在VS Code中手动操作
4. **历史记录**：所有操作都有日志记录

## 注意事项

1. **确保OpenOCD已启动**：先在VS Code中启动调试会话
2. **连接调试器**：确保DAP调试器已正确连接
3. **权限问题**：确保有权限访问调试器和串口

## 故障排除

### 问题1：无法连接OpenOCD
**解决**：确保VS Code中的OpenOCD已启动，且Telnet端口4444可用

### 问题2：调试器未连接
**解决**：检查DAP调试器USB连接和驱动

### 问题3：VS Code GUI不更新
**解决**：刷新调试视图或重启VS Code

---

## 无线调试器使用指南

### 硬件连接

1. 将无线调试器HOST端通过USB连接电脑
2. 检查设备管理器，确认COM端口（如COM23）
3. 在DAP上位机软件中配置无线参数

### 串口测试

```bash
# 检测无线调试器
python Tool/debug/wireless_debug.py detect

# 发送数据给STM32
python Tool/debug/wireless_debug.py send "Hello H7"

# 监听STM32输出
python Tool/debug/wireless_debug.py monitor

# 使用主工具
python Tool/debug/stm32h7_tool.py wireless-test
```

### AI自动化工作流

AI可以通过无线串口与STM32交互：

```
AI (COM23 TX) → 无线 → STM32 (RX)
AI (COM23 RX) ← 无线 ← STM32 (TX)
```

典型流程：
1. 编译固件 → `stm32h7_tool.py build`
2. 烧录固件 → `stm32h7_tool.py flash`
3. 发送测试命令 → `stm32h7_tool.py wireless-send "TEST"`
4. 监听响应 → `stm32h7_tool.py wireless-monitor`
