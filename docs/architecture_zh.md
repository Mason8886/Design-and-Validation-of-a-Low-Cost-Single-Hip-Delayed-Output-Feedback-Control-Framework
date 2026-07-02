# 代码结构说明

## 1. DelayBuffer

位置：

```text
include/dofc/core/DelayBuffer.hpp
src/core/DelayBuffer.cpp
```

作用：保存过去一段时间的测量信号，比如髋关节角度。DOFC 需要 `y(t - T_d)`，所以必须能根据时间查到“过去某一刻”的值。

它不是简单按数组下标取值，而是按时间插值。这样即使采样间隔有轻微变化，也能得到合理的延迟信号。

## 2. DofcController

位置：

```text
include/dofc/control/DofcController.hpp
src/control/DofcController.cpp
```

作用：实现 DOFC 控制律。

输入：

- 当前时间
- 髋关节角度
- 髋关节角速度

输出：

- 原始转矩 `raw_torque_nm`
- 控制器是否已经有足够历史数据 `ready`
- 当前信号和延迟信号，便于调试和画图

一开始控制器不会立刻输出，因为还没有足够的历史数据。例如延迟是 0.25 s，那至少要等 0.25 s 后才能查到过去的信号。

## 3. MotionStateDetector

位置：

```text
include/dofc/core/MotionStateDetector.hpp
src/core/MotionStateDetector.cpp
```

作用：处理 walk-stop-walk 过渡。

它用髋关节角速度判断当前是否像是在走路。如果速度长期很小，就认为停止走路，并把助力比例慢慢降到 0。重新开始走路后，助力比例再慢慢升上去。

这样做是为了避免人已经停下来了，但延迟缓冲里还存着刚才走路的数据，导致控制器继续输出不合适的转矩。

## 4. TorqueLimiter

位置：

```text
include/dofc/core/SafetyLimiter.hpp
src/core/SafetyLimiter.cpp
```

作用：最后一道软件安全保护。

它做两件事：

- 转矩限幅：默认最大只允许输出 `±6 Nm`
- 转矩变化率限制：默认每秒最多变化 `20 Nm`

这里的 `6 Nm` 是外骨骼执行器的保守部分助力上限，不是人体完整髋关节力矩。当前仿真中的人体髋关节转矩幅值默认是 `60 Nm`，所以外骨骼只提供约 10% 峰值比例的低助力。

## 5. SeriesElasticEstimator

位置：

```text
include/dofc/estimation/SeriesElasticEstimator.hpp
src/estimation/SeriesElasticEstimator.cpp
```

作用：用两个编码器估计输出转矩。

这个模块以后会接真实硬件传感器。现在它先作为独立算法存在，便于单元测试和论文解释。

## 6. SingleHipModel

位置：

```text
include/dofc/model/SingleHipModel.hpp
src/model/SingleHipModel.cpp
```

作用：单髋关节简化动力学模型。

当前默认配置是全尺度步行模型：

```text
inertia = 1.2 kg*m^2
damping = 12 Nm*s/rad
passive stiffness = 100 Nm/rad
```

这个模型用于仿真，不直接代表真实人体。它的价值是让你在没有硬件前先检查控制器逻辑，并保证 60 Nm 人体髋转矩输入下模型仍然稳定。

代码中不再保留旧的小尺度模型预设；默认模型就是当前唯一支持的全尺度部分助力仿真模型。

## 7. simulate_dofc

位置：

```text
apps/simulate_dofc.cpp
```

作用：跑一次完整仿真。

流程是：

```text
人体步行转矩 -> 单髋模型 -> 髋角度/角速度 -> DOFC -> 运动状态门控 -> 安全限幅 -> 外骨骼转矩 -> 单髋模型
```

默认输入关系是：

```text
human biological hip torque amplitude = 60 Nm
exoskeleton actuator torque limit     = 6 Nm
```

输出 CSV 后，可以画：

- 髋关节角度
- 人体生物髋转矩
- 原始 DOFC 转矩
- 安全限制后的实际外骨骼转矩
- walk-stop-walk 时的助力比例

## 8. parameter_sweep

位置：

```text
apps/parameter_sweep.cpp
```

作用：扫描多组 `K` 和 `T_d`。

输出文件：

```text
data/parameter_sweep.csv
```

当前扫描采用 60 Nm 人体输入和 6 Nm 外骨骼上限，并记录：

- 最大髋角
- 外骨骼峰值转矩
- 外骨骼峰值/人体峰值比例
- RMS 转矩变化率
- 转矩饱和比例
- 转矩变化率受限比例
- `safe`：角度、转矩变化率、饱和比例和最大助力比例均在限制内
- `useful`：峰值助力比例至少达到 3%，避免把零增益误判成推荐参数
- `recommended`：同时满足 `safe` 和 `useful`

这个文件可以帮助你在论文中说明：为什么选择某个增益和延迟作为硬件初始参数。

## 9. tests

位置：

```text
tests/
```

作用：自动检查核心模块。

现在测试覆盖：

- 延迟缓冲插值
- DOFC 延迟输出
- 全尺度 60 Nm 人体髋转矩下的单髋模型响应
- 转矩限幅和变化率限制
- 串联弹性转矩估计

以后每加一个新模块，都应该先加测试。
