# Low-Cost Single-Hip DOFC Control Framework

这是一个以 C++ 为主的低成本单髋外骨骼控制系统工程，面向毕设题目：

**Design and Validation of a Low-Cost Single-Hip Delayed Output Feedback Control Framework**

项目核心是 Delayed Output Feedback Control (DOFC)：控制器不显式判断支撑相/摆动相，而是把髋关节运动信号延迟一小段时间，再乘以反馈增益生成连续助力转矩。

## Simulation Scale

当前仿真采用“人体主驱动、外骨骼小比例助力”的全尺度设定：

- biological human hip torque amplitude: `60 Nm`
- conservative exoskeleton actuator torque limit: `6 Nm`
- nominal peak assistance ratio: about `10%`

这意味着 `6 Nm` 不是用来复现完整人体髋关节力矩，也不是要替代人的髋关节。它表示低成本原型机的保守部分助力上限。人体生物髋转矩仍然是主要驱动，外骨骼只提供小比例辅助。

单髋模型也已经重新定标到更接近全尺度步行响应：

- inertia: `1.2 kg·m²`
- damping: `12 Nm·s/rad`
- passive stiffness: `100 Nm/rad`

在默认 60 Nm 正弦人体髋转矩输入下，仿真髋角保持在合理步行范围内，而不会像旧 toy model 那样因为输入放大而数值发散。

## What Is Included

- C++17 DOFC 控制核心
- 延迟缓冲器：按时间保存髋角度/速度，并查询过去某个时刻的信号
- 安全限制：转矩限幅、转矩变化率限制
- walk-stop-walk 保护：走路时逐渐打开助力，停止时逐渐关掉助力
- 串联弹性转矩估计：用电机侧编码器和输出侧编码器估计弹性元件转矩
- 全尺度单髋关节仿真：用于参数扫描和稳定性检查
- CSV 数据输出：用于论文图表和数据分析
- 单元测试：验证核心模块和全尺度模型响应
- GitHub Actions：推到 GitHub 后自动编译、测试、跑一次仿真

## Project Structure

```text
include/dofc/      C++ 头文件，定义控制器、模型、估计器接口
src/               C++ 实现文件
apps/              可运行程序：仿真和参数扫描
tests/             单元测试
tools/             Python 数据画图工具
docs/              中文解释文档
.github/workflows/ GitHub 自动测试配置
data/              仿真输出数据目录
```

## Build And Run

需要先安装 CMake 和 C++ 编译器。Windows 推荐安装 Visual Studio Build Tools，macOS/Linux 推荐使用 clang 或 g++。

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build --output-on-failure
```

运行默认全尺度仿真：

```powershell
.\build\Release\simulate_dofc.exe --output data/simulation_output.csv
```

默认值等价于：

```powershell
.\build\Release\simulate_dofc.exe --human-torque 60 --max-torque 6 --gain 10 --delay 0.25
```

在 Linux 或 GitHub Actions 中通常是：

```bash
./build/simulate_dofc --output data/simulation_output.csv
```

做参数扫描：

```powershell
.\build\Release\parameter_sweep.exe
```

画图：

```powershell
python -m pip install -r requirements.txt
python tools/plot_results.py data/simulation_output.csv --out data/simulation_plot.png
```

## Recommended Reading Order

1. `docs/control_theory_zh.md`：先理解控制理论、全尺度仿真假设和安全限制
2. `docs/architecture_zh.md`：再看每个代码模块负责什么
3. `docs/github_debugging_zh.md`：最后看怎么用 GitHub 自动检查项目

## Mapping To The Project Plan

- Phase 1 modelling and simulation: `apps/simulate_dofc.cpp`, `apps/parameter_sweep.cpp`
- Real-time DOFC implementation: `DofcController`, `DelayBuffer`
- Torque saturation and torque-rate limiting: `TorqueLimiter`
- Dual magnetic encoders torque estimation: `SeriesElasticEstimator`
- walk-stop-walk transition behaviour: `MotionStateDetector`
- Experimental dataset: `CsvLogger` 输出 CSV

## License

This repository uses the Apache License 2.0.
