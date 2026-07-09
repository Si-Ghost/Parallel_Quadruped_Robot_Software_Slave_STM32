# Parallel Quadruped Robot 文档索引

## 项目简介

8-DOF 并联腿四足机器人。每条腿由 2 个直流无刷电机（Unitree GO-M8010-6）驱动，通过 RS485（MIT 协议）通信。STM32H7 负责底层电机控制和运动学解算，ESP32 提供 Wi-Fi WebSocket 桥接用于网页端控制与监控。

## 快速入门

### 构建
- **STM32**: `cd Software/STM32 && cmake --build build/Debug`
- **ESP32**: 用户在 ESP-IDF 环境下自行构建

### 上电测试流程
1. 将四条腿掰到零位/趴姿附近（单圈绝对编码器，断电后会折返）
2. 给 STM32 + ESP32 上电
3. 打开 Web 页面，等待 `ESP32_HELLO` → `STM32_ACK` 握手完成
4. 点击 **Rescan** 扫描电机在线状态
5. 点击 **Snapshot** 确认 8 个电机 online=1、zero=1
6. 点击 **Hold Current** 保持当前姿态
7. 勾选 **Leg Arm**，开始测试

### Web UI 按钮说明
| 按钮 | 功能 | 安全要求 |
|------|------|---------|
| Rescan | 重新扫描 8 个电机在线状态 | 任意 |
| Snapshot | 打印四腿足端坐标与状态 | 任意 |
| Hold Current | 以 Kp=1.0 保持当前位置 | 任意 |
| Release All | 清空 Kp/Kw，卸掉所有力矩 | **机身必须有支撑** |
| Nudge | 单腿微动（dx/dy 限幅 ±15mm） | Leg Arm |
| Trace | 单腿四步小轨迹 | Leg Arm |
| All Micro | 四腿微动 +3mm 自动返回 | Leg Arm |
| Prep Pose | 四腿下压 +5mm 后 Hold | Leg Arm |
| Sine Test | 单腿正弦波摆动（调试用） | Leg Arm |
| Trot | 对角小跑步态 | Leg Arm |

## 安全准则（必读）

1. **上电前腿必须放在零位/趴姿附近**（单圈绝对编码器）
2. 每次测试前必须先 `Rescan`，确认 online/zero 全 OK
3. 所有腿部动作必须勾选 `Leg Arm`
4. 触地/承重测试时必须有架子或手能托住
5. 出现 stall、zero out、异响、抖动、方向异常 → 立即停止
6. STM32 无回复时**不要连续点按钮**
7. 承重阶段只用 Loaded Step / Trot，不要用普通 Stand Step
8. `Release All` 会卸掉所有力矩，**必须在机身被托住时使用**

## 文档目录

| 文档 | 内容 |
|------|------|
| [01 - 机械装配与坐标系](01_mechanical.md) | 腿结构、连杆长度、电机编号、零位表、坐标定义 |
| [02 - 硬件资源分配](02_hardware.md) | STM32/ESP32 引脚、UART 分配、RS485 接线 |
| [03 - 软件架构](03_software.md) | 代码模块结构、构建系统、关键文件 |
| [04 - 运动学与控制](04_kinematics_control.md) | FK/IK 推导、MIT 协议、PID、步态生成 |
| [05 - 通信协议](05_communication.md) | UART/RS485/MIT 帧格式、WebSocket JSON |
| [06 - 开发日志](06_devlog.md) | 开发历程、测试结果、已知问题 |
| [07 - 未来规划](07_roadmap.md) | 后续开发计划、待优化项 |
