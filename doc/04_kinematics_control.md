# 04 - 运动学与控制

## 正运动学 (FK)

输入：关节角 `(θ1, θ2)`，输出：足端坐标 `(x, y)`，坐标系原点在髋关节 A。

```
d² = L1² × (2 + 2×cos(θ1+θ2))
h² = L2² - d²/4
x_e = L1/2 × (cos(θ2) - cos(θ1))
y_e = L1/2 × (sin(θ1) + sin(θ2))
x = x_e + (h/d) × L1 × (sin(θ1) - sin(θ2))
y = y_e + (h/d) × L1 × (cos(θ1) + cos(θ2))
```

实现: `Leg_Kinematics_Forward()` in `Leg_Kinematics.c`

## 逆运动学 (IK)

输入：足端坐标 `(x, y)`，输出：关节角 `(θ1, θ2)`。

```
r² = x² + y²
φ = atan2(y, x)
α = acos((L1² + r² - L2²) / (2×L1×r))
θ2 = φ - α
θ1 = π - α - φ
```

限制: `|L2-L1| < r < L1+L2` (130mm < r < 390mm)

实现: `Leg_Kinematics_Inverse()` in `Leg_Kinematics.c`

## 关节角 ↔ 转子角转换

电机反馈的是转子侧角度（编码器直读），需要转换为关节角用于运动学：

```
joint_angle = direction × (rotor_angle - rotor_zero_offset) / 6.33
rotor_target = rotor_zero_offset + direction × joint_angle × 6.33
```

- `direction`: 1 或 -1，当前全部为 1
- `rotor_zero_offset`: 零位表（见 01_mechanical.md）
- `6.33`: 减速比

## MIT 协议控制律

```
τ = τ_ff + Kp × (q_des - q) + Kd × (dq_des - dq)
```

- `Kp`: 位置刚度系数 (0~25.6)
- `Kd` (代码中 Kw): 速度阻尼系数 (0~25.6)
- `q_des`: 目标转子角 (rad)
- `τ_ff`: 前馈扭矩 (Nm)

标准模式（MIT Position）:
- Kp=2.0, Kw=0.15, T=0 → 普通位置跟踪
- Kp=6.0, Kw=0.35, T=0 → Trot 步态

扭矩模式（T=non-zero, Kp=Kw=0）→ 用于串级 PID（实验阶段）

## PID 控制器

`APP/PID/pid.c` — 支持两种模式：
- **位置式 PID**: `out = Kp×e + Ki×Σe + Kd×(e-e_prev)`，带积分限幅和输出限幅
- **增量式 PID**: `Δout = Kp×(e-e_prev) + Ki×e + Kd×(e-2e_prev+e_prev2)`

当前未在步态中使用（实验阶段），预留用于串级 PID 控制。

## Trot 步态生成

`Leg_Gait.c` 中的 `Leg_Gait_ServiceTrot()` 实现对角小跑步态。

### 步态参数

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `LEG_TROT_LIFT_HEIGHT_MM` | 80 | 摆动相抬腿高度 |
| `LEG_TROT_START_POINT_MM` | 50 | 半步长（总步长=100mm） |
| `LEG_TROT_STEP_RATE_MS` | 400 | 每半步持续时间 |
| `LEG_TROT_STEP_CYCLE_MS` | 800 | 完整步态周期 |

### 相位调度（对角小跑）

对角线两组：A(LF+RB) 和 B(RF+LB)。

| 时段 | LF | RF | LB | RB |
|------|-----|-----|-----|-----|
| 0~400ms | 支撑 G | 摆动 S | 摆动 S | 支撑 G |
| 400~800ms | 摆动 S | 支撑 G | 支撑 G | 摆动 S |

### 轨迹形状

**摆动相**（参考工程同款）：
- X: 修正摆线 `x(t) = -start + 2×start × (t - sin(2πt)/(2π))`
  - t=0 时 x=-start（后），t=1 时 x=+start（前）
  - 零速度起点和终点，减少冲击
- Y: 升余弦 `y(t) = leg_high - lift × (0.5 - 0.5×cos(2πt))`
  - t=0 和 t=1 时 y=leg_high（着地），t=0.5 时 y=leg_high-lift（抬腿最高点）

**支撑相**：
- X: 线性 `x(t) = +start - 2×start × t`
  - 从 +start(前) 到 -start(后)，往后蹬地面
- Y: 恒定 `y(t) = leg_high`

## 调试目标引擎

`Leg_Control.c` 中的调试目标系统用于 Nudge/Trace/AllMicro/PrepPose 等非周期性动作：

1. `start_debug_offset()` → 设置 `target_offset`，标记 `target_active=Active`
2. `apply_debug_target()` → 每周期计算斜坡 + 发送 MIT 命令
3. 完成条件: ramp=1.0 且 `|error| ≤ stop_error`
4. 堵转检测: grace 500ms 后，4000ms 无进度(0.005rad) → stall
5. 超时: 30000ms → timeout
6. `stop_debug_target()` → 完成时 Hold(Kp=1.0) 或失败时释放(Kp=0)
