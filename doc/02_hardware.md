# 02 - 硬件资源分配

## MCU

| 芯片 | 职责 |
|------|------|
| STM32H743VGTx (Cortex-M7, 480MHz) | 电机控制、运动学、RS485 通信 |
| ESP32 | Wi-Fi、WebSocket、Web 服务器 |

## STM32 UART 分配

| UART | 用途 | TX | RX | 方向控制 |
|------|------|-----|-----|---------|
| UART1 | 电脑调试（保留） | — | — | — |
| UART2 | LF 腿 (RS485) | PA2 | PA3 | PA4 |
| UART8 | RF 腿 (RS485) | PE1 | PE0 | PB9 |
| UART7 | LB 腿 (RS485) | PE8 | PE7 | PE9 |
| UART5 | RB 腿 (RS485) | PB13 | PB12 | PB15 |
| UART6 | ESP32 桥接 | PC6 | PC7 | 无 |
| UART4 | IMU（暂未使用） | — | — | — |

## ESP32 引脚

| 引脚 | 方向 | 连接 |
|------|------|------|
| GPIO17 (TXD) | → | STM32 UART6 RX (PC7) |
| GPIO18 (RXD) | ← | STM32 UART6 TX (PC6) |

## RS485 方向控制

每条腿的 RS485 收发器有方向控制引脚：
- LF: PA4
- RF: PB9
- LB: PE9
- RB: PB15

发送时拉高，接收时拉低。`SERVO_Send_recv()` 在 `GO-M8010-6.c` 中管理这一时序。

## 电机

- 型号: Unitree GO-M8010-6
- 通信: TTL-RS485，波特率由电机固件决定
- 协议: MIT 协议（17 字节命令帧 / 16 字节反馈帧）
- 每条腿一个 RS485 总线，上面挂 2 个电机（ID0 和 ID1）

## 供电

- 电机供电：直流电源（电压取决于电机规格，约 24V）
- STM32/ESP32：USB 或独立供电

## 关键硬件注意事项

1. STM32H7 的 DMA 无法访问 DTCMRAM，DMA 缓冲区必须放在 RAM_D2（`.dma_buffer` 段）
2. 电机反馈包头实际为 `FD EE`（非文档标称的 `FE EE`），已兼容
3. UART FIFO TX/RX threshold 设为 1/8，避免 FIFO 溢出导致的断联
4. 每个电机轮询流程：预启动 RX DMA → 发送 → 切 RS485 接收 → 等待 DMA 完成
