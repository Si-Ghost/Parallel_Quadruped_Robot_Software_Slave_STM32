# 05 - 通信协议

## 总体架构

```
Web Browser ←WebSocket→ ESP32 ←UART6→ STM32 ←RS485×4→ 8x GO-M8010-6 motors
```

## ESP32 ↔ WebSocket ↔ STM32

### 握手
1. ESP32 启动，等待 STM32 UART6 就绪
2. ESP32 发送 `ESP32_HELLO`
3. STM32 收到后回复 `STM32_ACK\n`，设置 `handshake_done=1`
4. 每 500ms 重试直到握手完成

### STM32 → ESP32（日志/状态）
- 文本格式：`COMMAND args...\r\n`
- 编码：ASCII
- 发送方式：`Communication_SendString()` → `HAL_UART_Transmit_IT()` (UART6 中断发送)
- 共用 `tx_buf[256]`，忙碌时丢弃

### ESP32 → STM32（命令）
- 遥控帧：固定 16 字节 `FE EE ... FF` 格式，含 ch0~ch3(16-bit) + s1/s2(8-bit) + CRC
- 文本命令：`COMMAND <args>\n`，由 `handle_motor_debug_command()` 逐字节匹配

### 文本命令列表

| 命令 | 参数 | 功能 |
|------|------|------|
| `MOTOR_RESCAN` | 无 | 重新握手扫描 |
| `MOTOR_STOP_ALL` | 无 | 清空所有 Kp/Kw |
| `MOTOR_HOLD_CURRENT` | 无 | 当前角度 Hold |
| `MOTOR_SET <i> <angle>` | 电机索引, 转子角(rad) | 单电机角度测试 |
| `LEG_NUDGE_MM <l> <dx> <dy>` | 腿号, dx(mm), dy(mm) | 单腿足端微动 |
| `LEG_TRACE <l>` | 腿号 | 单腿四点轨迹 |
| `LEG_SNAPSHOT` | 无 | 四腿足端快照 |
| `LEG_ALL_MICRO` | 无 | 四腿微动 |
| `LEG_PREP_POSE` | 无 | 四腿预备姿态 |
| `LEG_SINE <l> <amp> <freq>` | 腿号, 幅值(mm), 频率(Hz) | 单腿正弦测试 |
| `LEG_TROT` | 无 | 对角小跑步态 |

### WebSocket JSON 命令

ESP32 将 Web UI 操作转为文本命令：

| JSON | 等效命令 |
|------|---------|
| `{motorScan:1}` | `MOTOR_RESCAN\n` |
| `{motorStopAll:1}` | `MOTOR_STOP_ALL\n` |
| `{motorHoldCurrent:1}` | `MOTOR_HOLD_CURRENT\n` |
| `{legNudge:1, ...}` | `LEG_NUDGE_MM <l> <dx> <dy>\n` |
| `{legAllMicro:1}` | `LEG_ALL_MICRO\n` |
| `{legPrepPose:1}` | `LEG_PREP_POSE\n` |
| `{legSine:1, sineLeg:0, sineAmp:3, sineFreq:0.5}` | `LEG_SINE 0 3.00 0.50\n` |
| `{legTrot:1}` | `LEG_TROT\n` |

## STM32 ↔ 电机 (RS485 / MIT 协议)

### 命令帧 (17 字节)

| 偏移 | 大小 | 字段 | 说明 |
|------|------|------|------|
| 0 | 2 | head | `FE EE` |
| 2 | 1 | mode.id | 电机 ID (0/1) |
| 3 | 1 | mode.status | 模式: 0=锁定, 1=FOC, 2=校准 |
| 4 | 2 | comd.tor_des | 目标扭矩 (Nm × 256, Q8) |
| 6 | 2 | comd.spd_des | 目标速度 (rad/s / 2π × 256) |
| 8 | 4 | comd.pos_des | 目标位置 (rad / 2π × 32768, Q15) |
| 12 | 2 | comd.k_pos | 刚度系数 (Kp / 25.6 × 32768) |
| 14 | 2 | comd.k_spd | 阻尼系数 (Kw / 25.6 × 32768) |
| 16 | 1 | — | 未使用 |

### 反馈帧 (16 字节)

| 偏移 | 大小 | 字段 | 说明 |
|------|------|------|------|
| 0 | 2 | head | `FD EE` |
| 2 | 1 | mode.id | 电机 ID |
| 3 | 1 | mode.status | 模式 |
| 4 | 2 | fbk.torque | 实际扭矩 (Nm × 256, Q8) |
| 6 | 2 | fbk.speed | 实际速度 (rad/s / 2π × 256) |
| 8 | 4 | fbk.pos | 实际位置 (rad / 2π × 32768, Q15) |
| 12 | 1 | fbk.temp | 温度 (°C) |
| 13 | 1 | fbk.MError | 错误码 |
| 14 | 2 | CRC16 | CRC-CCITT 校验 |

### RS485 收发时序

```
1. 配置 DMA RX buffer (RAM_D2)
2. HAL_UART_Receive_DMA() 预启动接收
3. GPIO 拉高 (RS485 发送模式)
4. HAL_UART_Transmit() 发送 17 字节命令帧
5. GPIO 拉低 (RS485 接收模式)
6. 轮询 DMA 计数器等待 16 字节接收完成（超时 20ms）
7. CRC 校验 → extract_data() 解析
```

关键细节：
- 包头实际为 `FD EE`（非 `FE EE`），已兼容
- FIFO threshold 1/8
- DMA buffer 在 `.dma_buffer` 段（RAM_D2），避免 H7 DMA 访问 DTCMRAM 失败
- transient IO 失败立即重试 1 次，重试失败才累计错误
- 连续 5 次错误 → 标记 offline
