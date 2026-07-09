# 03 - 软件架构

## 目录结构

```
Software/STM32/
├── APP/                          # 应用层（用户代码）
│   ├── Inc/
│   │   ├── Leg_Control/          # 电机管理 + 编排层
│   │   │   └── Leg_Control.h
│   │   ├── Leg_Kinematics/       # 纯运动学数学
│   │   │   └── Leg_Kinematics.h
│   │   ├── Leg_Gait/             # 步态/轨迹生成
│   │   │   └── Leg_Gait.h
│   │   ├── PID/                  # PID 控制器
│   │   │   └── pid.h
│   │   └── Communication/        # ESP32 通信模块
│   │       └── communication.h
│   └── Src/
│       ├── Leg_Control/
│       │   └── Leg_Control.c      # ~800 行：握手、轮询、调试目标、编排
│       ├── Leg_Kinematics/
│       │   └── Leg_Kinematics.c   # ~90 行：FK/IK、clamp/wrap/delta
│       ├── Leg_Gait/
│       │   └── Leg_Gait.c         # ~510 行：Trace/AllMicro/PrepPose/Sine/Trot
│       ├── PID/
│       │   └── pid.c              # ~85 行：位置式 + 增量式 PID
│       └── Communication/
│           └── communication.c    # ~830 行：ESP32 帧/文本解析、遥控器
├── BSP/                          # 板级支持包
│   ├── Inc/GO-M8010-6/
│   │   ├── GO-M8010-6.h          # 电机驱动数据结构
│   │   └── ris_protocol.h        # RIS/MIT 协议帧定义
│   └── Src/GO-M8010-6/
│       └── GO-M8010-6.c          # modify_data()/extract_data()/SERVO_Send_recv()
├── Utils/
│   └── crc_ccitt.h               # CRC-CCITT 校验
├── Src/                          # CubeMX 生成
│   ├── main.c                    # 入口、外设初始化、主循环
│   ├── stm32h7xx_it.c            # 中断服务函数
│   └── stm32h7xx_hal_msp.c       # HAL 外设 MSP 配置
├── Drivers/                      # HAL + CMSIS
├── CMakeLists.txt                # 顶层 CMake
└── cmake/stm32cubemx/            # CubeMX 生成的 CMake 子项目

Software/ESP32/
├── components/
│   ├── esp_stm32_comm/           # 与 STM32 UART6 通信
│   └── web_server/               # WebSocket 服务器 + Web UI
└── main/
```

## 模块依赖关系

```
communication.c
  ├── Leg_Control.h      (Hold/Stop/Rescan/Snapshot/SetAngle/Nudge)
  └── Leg_Gait.h         (StartTrace/StartAllMicro/StartPrepPose/StartSine/StartTrot)

Leg_Gait.c
  ├── Leg_Control.h      (电机状态、目标控制、足端读取)
  ├── Leg_Kinematics.h   (IK 解算)
  └── communication.h    (日志输出)

Leg_Control.c
  ├── Leg_Kinematics.h   (FK/IK、clamp/wrap)
  ├── Leg_Gait.h         (编排——调用 gait 服务函数)
  ├── communication.h    (日志输出)
  └── GO-M8010-6.h      (电机协议)
```

## 数据流

```
main.c 主循环 (~50ms 周期)
  │
  ├── Communication_RxCallback  (ESP32 → UART6 中断)
  │     └── handle_motor_debug_command()
  │           ├── Leg_Control_* (Hold/Stop/Rescan)
  │           ├── Leg_Gait_Start* (Trace/AllMicro/PrepPose/Sine/Trot)
  │           └── Leg_Control_SetDebugFootOffset() (Nudge)
  │
  ├── Communication_Task()  (握手维护、看门狗)
  │
  └── Leg_Control_Service(now_ms)  [50ms 周期]
        ├── update_debug_targets()    (对 active 目标做 ramp + stall 检测)
        ├── Leg_Gait_ServiceSine()    (正弦轨迹更新)
        ├── Leg_Gait_ServiceTrot()    (步态轨迹更新)
        ├── Leg_Control_Start()       (轮询所有在线电机)
        │     └── poll_online_motor() (SERVO_Send_recv 收发)
        ├── Leg_Gait_ServiceDebugTrace()
        ├── Leg_Gait_ServiceAllMicro()
        └── Leg_Gait_ServicePrepPose()

中断上下文:
  ├── UARTx_DMA_TX_IRQ  → Leg_Tx_Handler()  (RS485 方向切换 + RX DMA 启动)
  └── UARTx_DMA_RX_IRQ  → Leg_Rx_Handler()  (解析反馈、更新 angle/speed)
```

## 构建

- **STM32**: `cmake --build build/Debug` (CMake 3.22+, arm-none-eabi-gcc)
- **ESP32**: 用户在 ESP-IDF 环境下自行构建 (idf.py build)

## 关键常量速查

| 常量 | 文件 | 值 | 说明 |
|------|------|-----|------|
| `LEG_LINK_L1_MM` | Leg_Kinematics.h | 130 | 上连杆长度 |
| `LEG_LINK_L2_MM` | Leg_Kinematics.h | 260 | 下连杆长度 |
| `LEG_REDUCTION_RATIO` | Leg_Kinematics.h | 6.33 | 减速比 |
| `LEG_WEB_KP` | Leg_Control.h | 2.0 | 默认位置增益 |
| `LEG_WEB_KW` | Leg_Control.h | 0.15 | 默认速度阻尼 |
| `LEG_HOLD_KP` | Leg_Control.h | 1.0 | Hold 状态位置增益 |
| `LEG_TROT_KP` | Leg_Gait.c | 6.0 | Trot 步态位置增益 |
| `LEG_TROT_KW` | Leg_Gait.c | 0.35 | Trot 步态速度阻尼 |
| `LEG_SERVICE_PERIOD_MS` | Leg_Control.c | 50 | 主服务循环周期 |
| `LEG_TARGET_RAMP_MS` | Leg_Gait.c | 1200 | 目标斜坡时间 |
