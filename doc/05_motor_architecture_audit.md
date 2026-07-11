# 05 - 电机数据流与第一阶段边界重构

## 1. 本阶段安全边界

- 机器人必须架空。
- `Motor_Transport.c` 中的 `MOTOR_TRANSPORT_ZERO_OUTPUT_ONLY` 保持为 `1`。
- 发送到电机的 `T/W/Pos/K_P/K_W` 仍在 transport 最后一层被强制清零。
- `Leg_Control_Service()` 继续不调用调试目标、PID 或步态服务。
- 本阶段不修改 ESP32 协议解析，也不恢复 Hold、Nudge、Prep Pose、Stand、Trot 等动作。

## 2. 重构前的实际调用链

```text
TIM7 1 ms
  -> Motor_Transport_Tick()

main while
  -> Motor_Transport_Service()
     -> 读取 Legs[leg]->motor_cmd[motor]
     -> UART TX DMA / TC 释放 DE
     -> circular RX DMA ring 解析
     -> 直接写 motor_data / motor_state
     -> 在 transport 内做跨圈累计、online 和 handshake 状态更新

Rescan
  -> Leg_Control_RequestHandshake()
  -> Leg_Control_Service()
  -> Leg_Control_Handshake()
  -> Motor_Transport_Stop()
  -> SERVO_Send_recv() 阻塞握手
  -> Motor_Transport_Start()
```

主要问题是 transport 同时知道 UART、腿对象、零位和显示角；`Motor_State` 没有独立边界，原始量、单圈量、累计量和关节量也没有统一命名。

## 3. 第一阶段后的数据流

```text
Leg_Control command provider
  -> Motor_Transport
     -> UART/DMA/RS485
     -> ID0 -> ID1 调度
     -> CRC / ID / 帧解析
     -> raw feedback callback
  -> Leg_Control 保持 motor_data 兼容副本
  -> Motor_State_UpdateRawFeedback()
     -> raw_position / raw_velocity / raw_torque
     -> single_turn_angle
     -> accumulated_angle
     -> zero_offset / direction / joint_angle
     -> online / zero_checked / timestamp / error_count
  -> Leg_Control / Kinematics / Telemetry
```

`Motor_Transport` 不再包含 `Leg_Control.h`，也不再直接访问全局 `Legs`。它只通过注册的回调：

1. 获取待发送命令；
2. 交付校验通过的原始反馈；
3. 报告反馈超时；
4. 报告 UART 错误。

调度节拍、四路 UART 映射、ID0 收到有效反馈后立即发送 ID1、RX circular DMA、TX DMA 和 UART TC 后释放 DE 的时序均未改变。

## 4. Motor_State 语义

| 字段 | 含义 |
|---|---|
| `raw_position` | 协议解包后的原始位置反馈 |
| `raw_velocity` | 协议解包后的原始速度反馈 |
| `raw_torque` | 协议解包后的原始力矩反馈 |
| `single_turn_angle` | 当前单圈转子角；本阶段与 `raw_position` 相同 |
| `accumulated_angle` | 在 1 kHz 有效反馈处按最短角差累计的连续转子角 |
| `zero_offset` | 当前腿、电机对应的转子零位 |
| `direction` | 关节方向，归一化为 `+1/-1` |
| `joint_angle` | `direction * wrap(single_turn-zero_offset) / 6.33` |
| `online` | transport 最近 100 ms 内是否持续收到有效反馈 |
| `zero_checked` | 在线、角度有效且单圈零位误差不超过安全阈值 |
| `timestamp` | 最近一帧有效原始反馈的 `HAL_GetTick()` |
| `error_count` | Motor_State 层累计观察到的握手、超时或 UART/IO 错误次数 |

为保持现有源代码兼容：

- `angle` 是 `single_turn_angle` 的兼容别名；
- `display_angle` 是 `accumulated_angle` 的兼容别名；
- `speed` 是 `raw_velocity` 的兼容别名；
- `Legs[leg]->motor_cmd[motor]`、`motor_data[motor]`、`motor_state[motor]` 均保持不变。

调试目标相关字段暂时保留在 `Motor_StateTypeDef` 尾部，仅作为第一阶段兼容层；`Motor_State.c` 不访问这些字段。下一阶段再把 Command/Mode 运行时状态移出 Motor_State，避免一次性改动 Leg_Gait 和所有命令路径。

## 5. Handshake / Rescan 顺序

当前顺序为：

1. `transport_running=0`，停止产生新的 ID0 周期；
2. 最多等待 5 ms，让正在发送的四路 UART 进入 TC 回调并释放 DE；
3. 若超时则中止 TX，保持安全停止并返回错误；
4. abort UART，清 ORE/NE/PE/FE、flush RX、清 RX ring/read index；
5. RX DMA 切回 normal；
6. `SERVO_Send_recv()` 以全零输出逐台阻塞握手；
7. 更新 `motor_data`、Motor_State 和在线状态；
8. RX DMA 切回 circular，重新预置四路常驻 ring；
9. 重启高速 transport。

## 6. 可观测性与兼容格式

原有固定遥测前缀保持不变：

```text
MOTOR_ANGLES <8个累计角> <8个valid>
```

现在每帧轮转附带一台电机的状态后缀：

```text
S<index>=<raw_position>,<single_turn_angle>,<accumulated_angle>,<joint_angle>
```

完整示例：

```text
MOTOR_ANGLES ... 1,1,1,1,1,1,1,1 S0=4.4221,4.4221,4.4221,0.0000
```

ESP32 现有 `sscanf` 只读取前面的 8 个角和 8 个 valid，会忽略后缀，因此接口保持兼容。固件内部也提供 `Leg_Control_GetMotorStateSnapshot()`，可读取速度、力矩、时间戳、online、zero_checked 和 error_count。让 PC UI 原生显示全部新字段需要后续明确修改 ESP32 的 JSON 转换；本阶段不越过“不修改 ESP32”的约束。

## 7. 下一阶段建议

1. 用零输出实机日志验证四路 `tx/rx/miss/crc/id/restart` 统计未退化，同时人工缓慢转动每个关节，核对 raw、single、accumulated、joint 的关系。
2. 增加主机侧 Motor_State 与 FK/IK 录制数据测试，不接实时硬件动作。
3. 将调试目标、Hold/Release 和模式状态从 Motor_State 兼容尾部迁到独立 Command/Mode 模块。
4. 经用户明确授权后再修改 ESP32/PC，使新状态作为结构化 telemetry 而不是诊断文本进入 UI。
