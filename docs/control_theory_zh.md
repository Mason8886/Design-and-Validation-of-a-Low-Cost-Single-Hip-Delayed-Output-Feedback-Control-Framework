# 控制理论说明

## 1. 为什么不用 FSM

有限状态机控制会把步态切成几个阶段，例如支撑相、摆动相，然后在不同阶段输出不同转矩。这个方法直观，但它依赖相位检测。一旦检测晚了、误判了，转矩就可能在状态切换处突然跳变。

你的项目书选择 DOFC 的原因正是为了避开这个问题：控制器不显式判断步态相位，而是直接利用髋关节运动本身。

## 2. DOFC 的基本思想

设髋关节测量信号为：

```text
y(t) = theta_hip(t)
```

其中 `theta_hip` 是髋关节角度。也可以选择：

```text
y(t) = theta_dot_hip(t)
```

也就是髋关节角速度。

最基本的延迟输出反馈可以写成：

```text
tau_raw(t) = K * y(t - T_d)
```

含义很简单：

- `tau_raw(t)`：当前希望输出的原始助力转矩
- `K`：反馈增益，决定助力强弱
- `T_d`：延迟时间，决定助力相对于人体运动晚多少
- `y(t - T_d)`：过去某一时刻的髋关节信号

代码里的默认控制律就是这个形式。

项目也保留了 Pyragas delayed feedback 的差分形式：

```text
tau_raw(t) = K * (y(t) - y(t - T_d))
```

这更接近经典延迟反馈控制文献。你后面做论文时，可以把默认形式作为工程实现主线，把差分形式作为对照实验。

## 3. 为什么增益和延迟要扫描

DOFC 的两个最关键参数是 `K` 和 `T_d`。

`K` 太小，助力不明显。`K` 太大，系统可能振荡，甚至让人感觉被拉扯。

`T_d` 太短，转矩可能跟人体自然节奏对不上。`T_d` 太长，转矩可能在错误时间出现，尤其是在开始走、停止走、重新开始走的时候。

所以项目书里 Phase 1 的参数扫描非常重要。代码中的 `parameter_sweep` 会自动尝试多组 `K` 和 `T_d`，输出每组参数的稳定性指标。

## 4. 安全层不是可选项

真实外骨骼控制不能直接把 `tau_raw` 发给电机。代码中实际输出的是：

```text
tau_cmd = safety_limit(motion_gate * tau_raw)
```

这里有三层保护：

1. `MotionStateDetector`：如果检测到停止走路，就把助力逐渐降到 0。
2. `TorqueLimiter` 的转矩限幅：防止超过最大允许转矩。
3. `TorqueLimiter` 的变化率限制：防止转矩突然跳变。

这对应项目书里的 torque saturation, torque-rate limiting, walk-stop-walk transition behaviour。

## 5. 串联弹性转矩估计

项目书提出用双磁编码器和串联弹性元件估计输出转矩。代码中的公式是：

```text
theta_motor_out = (theta_motor - theta_motor_zero) / gear_ratio
theta_output_rel = theta_output - theta_output_zero
spring_deflection = theta_motor_out - theta_output_rel
tau_est = k_spring * spring_deflection
```

直观理解：

- 电机侧编码器告诉你“电机通过减速后希望输出到哪里”
- 输出侧编码器告诉你“人体侧真实输出到哪里”
- 两者的差就是弹性元件被扭了多少
- 弹性元件刚度乘以扭转角，就是估计转矩

实际硬件中需要做静态加载标定，确认 `k_spring` 和零点。

## 6. 单髋仿真模型

仿真模型使用简化二阶系统：

```text
J * theta_ddot + b * theta_dot + k * theta = tau_human + tau_exo
```

它不是完整人体模型，但足够用于 Phase 1：

- 看控制器会不会发散
- 看转矩是否平滑
- 看 walk-stop-walk 时是否有不该出现的大转矩
- 初步筛选安全的 `K` 和 `T_d`

毕业论文里要强调：这是 preliminary simulation，不代表最终人体实验结果，但它能降低硬件调试风险。
