# 串口指令协议 TODO

记录主控侧串口指令方案，暂不实现。

---

## 参考协议（遥控器端 uart_cmd.c）

**帧格式：**
```
AA 55 [CMD] [ADDR] [LEN] [CHKSUM] [DATA...] [CRC16-Modbus]
```

- 帧头：0xAA 0x55
- CMD：命令码（1字节）
- ADDR：地址（1字节，保留）
- LEN：数据长度（1字节）
- CHKSUM：校验和（帧头+CMD+ADDR+LEN）
- DATA：数据（0~N字节）
- CRC16-Modbus：2字节（低字节在前）

**现有命令（遥控器→接收端）：**
| CMD | 功能 | DATA |
|-----|------|------|
| 0xA0 | 软复位 | 空 |
| 0xAB | RF模式切换 | 1字节(1=BLE, 0=2.4G) |
| 0xAC | 设置配对地址 | 5字节 |
| 0xAA | 触发配对 | 空 |
| 0xAD | 恢复默认地址 | 空 |
| 0xAF | 查询配对地址 | 空 |
| 0xB0 | 设置UI模式 | 1字节(0~5) |
| 0xB1 | 设置油门覆盖 | 1字节(0~100, 0xFF=禁用) |
| 0xB2 | 设置档位 | 1字节(1~4) |

---

## 主控侧扩展命令（待实现）

### 接收命令（遥控器→主控）

| CMD | 功能 | DATA | 说明 |
|-----|------|------|------|
| 0xC0 | 机械臂回零 | 空 | 触发ARM_HOME状态 |
| 0xC1 | 夹取 | 空 | 触发ARM_GRIP状态 |
| 0xC2 | 释放 | 空 | 触发ARM_RELEASE状态 |
| 0xC3 | 设置yaw角度 | 2字节 | int16, 0.1度精度, 有符号 |
| 0xC4 | 设置目标位置 | 6字节 | x,y,z各int16, mm单位, 有符号 |
| 0xC5 | 查询状态 | 空 | 请求主控回传状态 |
| 0xC6 | 设置关节角度 | 8字节 | yaw,shoulder,elbow,gripper各int16, 0.1度 |

### 发送反馈（主控→遥控器）

| CMD | 功能 | DATA | 说明 |
|-----|------|------|------|
| 0xD0 | 状态反馈 | 1字节 | arm_state_e枚举值 |
| 0xD1 | 位置反馈 | 6字节 | x,y,z各int16 (mm) |
| 0xD2 | 关节角度反馈 | 8字节 | yaw,shoulder,elbow,gripper各int16 (0.1度) |
| 0xD3 | IK错误 | 1字节 | 0=成功, 1=超出工作空间 |

---

## 实现方案

### 文件结构

```
Application/
  uart_cmd.h        — 命令码定义、帧结构
  uart_cmd.c        — 帧解析、命令分发
Protocol/
  prot_uart_cmd.c/h — 与遥控器通信协议（可选，或放在Application）
```

### 核心函数

```c
/* 初始化 */
void uart_cmd_init(void);

/* 周期调用（在arm_task或单独任务中） */
void uart_cmd_process(void);

/* 发送反馈 */
void uart_cmd_send_state(uint8_t state);
void uart_cmd_send_position(float x, float y, float z);

/* 命令回调（用户实现） */
void on_cmd_arm_home(void);
void on_cmd_arm_grip(void);
void on_cmd_arm_release(void);
void on_cmd_set_yaw(float yaw_deg);
void on_cmd_set_pos(float x, float y, float z);
```

### 集成方式

1. 在 `freertos.c` 中注册任务（或在arm_task中轮询）
2. 使用 USART6（921600波特率）接收遥控器数据
3. 解析帧后调用对应机械臂函数

---

## 待办

- [ ] 确认遥控器端是否需要添加新命令码
- [ ] 确认USART6是否分配给遥控器通信
- [ ] 实现帧解析函数
- [ ] 实现命令回调函数
- [ ] 测试双向通信
- [ ] 添加错误处理和超时机制

---

## 参考文件

- `F:\RM2026\kamingo_ui\projects\rc_remote\app\uart_cmd.c` — 遥控器端协议实现
- `Application/arm_task.c` — 机械臂主任务
- `Application/config.h` — 调试配置