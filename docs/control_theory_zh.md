# 控制理论说明

## 1. 为什么不用 FSM

有限状态机控制会把步态切成几个阶段，例如支撑相、摆动相，然后在不同阶段输出不同转矩。这个方法直观，但它依赖相位检测。一旦检测晚了、误判了，转矩就可能在状态切换处突然跳变。

本项目选择 DOFC 的原因正是为了避开这个问题：控制器不显式判断步态相位，而是直接利用髋关节运动本身。

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

含义是：

- `tau_raw(t)`：当前希望输出的原始助力转矩
- `K`：反馈增益，决定助力强弱
- `T_d`：延迟时间，决定助力相对于人体运动晚多少
- `y(t - T_d)`：过去某一时刻的髋关节信号

代码里的默认控制律就是这个形式。

项目也保留了 Pyragas delayed feedback 的差分形式：

```text
tau_raw(t) = K * (y(t) - y(t - T_d))
```

这更接近经典延迟反馈控制文献。后续论文中可以把默认形式作为工程实现主线，把差分形式作为对照实验。

## 3. 全尺度人体转矩与部分外骨骼助力

当前仿真把人体髋关节力矩和外骨骼执行器力矩分开处理：

```text
tau_human_peak = 60 Nm
tau_exo_limit  = 6 Nm
```

`60 Nm` 表示仿真中的 biological human hip torque amplitude，也就是人体走路时主要由肌肉和身体动力学产生的髋关节转矩幅值。

`6 Nm` 表示低成本单髋外骨骼原型机的保守 actuator torque limit。它不是完整人体髋关节力矩，也不是要替代人的髋关节。它只代表大约 10% 峰值比例的部分助力。

所以仿真关系应该理解为：

```text
human hip torque:        main driving torque, around 60 Nm peak
exoskeleton assistance:  limited partial assistance, up to 6 Nm peak
```

这更符合低成本、低助力单髋外骨骼的项目定位。

## 4. 为什么增益和延迟要扫描

DOFC 的两个最关键参数是 `K` 和 `T_d`。

`K` 太小，助力不明显。`K` 太大，输出容易碰到 6 Nm 执行器上限，甚至在过渡阶段产生不必要的转矩。

`T_d` 太短，转矩可能跟人体自然节奏对不上。`T_d` 太长，转矩可能在错误时间出现，尤其是在开始走、停止走、重新开始走的时候。

因此 `parameter_sweep` 会扫描不同 `K` 和 `T_d`，并用新的全尺度稳定性指标筛选参数：

- 最大髋角约小于 `1.0 rad`
- RMS 转矩变化率不超过默认 `20 Nm/s` 限制
- 饱和比例小于约 `0.20`
- 外骨骼峰值转矩明显小于人体峰值转矩

参数扫描输出三个更清楚的分类：

- `safe`：角度、转矩变化率、饱和比例和最大助力比例均在限制内
- `useful`：峰值助力比例至少达到 `0.03`，避免把零增益误认为推荐参数
- `recommended`：同时满足 `safe` 和 `useful`

这些指标服务于当前唯一支持的全尺度部分助力仿真假设。

## 5. 安全层不是可选项

真实外骨骼控制不能直接把 `tau_raw` 发给电机。代码中实际输出的是：

```text
tau_cmd = safety_limit(motion_gate * tau_raw)
```

这里有三层保护：

1. `MotionStateDetector`：如果检测到停止走路，就把助力逐渐降到 0。
2. `TorqueLimiter` 的转矩限幅：默认最大只允许输出 `±6 Nm`。
3. `TorqueLimiter` 的变化率限制：默认每秒最多变化 `20 Nm`。

这对应项目书里的 torque saturation, torque-rate limiting, walk-stop-walk transition behaviour。

## 6. 串联弹性转矩估计

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

## 7. 单髋仿真模型

仿真模型使用简化二阶系统：

```text
J * theta_ddot + b * theta_dot + k * theta = tau_human + tau_exo
```

当前默认全尺度配置为：

```text
J = 1.2 kg*m^2
b = 12 Nm*s/rad
k = 100 Nm/rad
```

这不是完整人体肌骨模型，但适合用于 60 Nm 人体髋转矩输入下的低助力控制初步验证。它的作用是：

- 检查 DOFC 控制器会不会发散
- 检查外骨骼转矩是否平滑
- 检查 walk-stop-walk 时是否有不该出现的大转矩
- 初步筛选安全的 `K` 和 `T_d`

毕业论文里要强调：这是 preliminary simulation，不代表最终人体实验结果，但它能降低硬件调试风险。
