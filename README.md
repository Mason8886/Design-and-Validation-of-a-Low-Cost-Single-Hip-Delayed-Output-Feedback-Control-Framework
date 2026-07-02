# Low-Cost Single-Hip DOFC Control Framework

这是一个从零开始的 C++ 控制系统工程，面向毕设题目：

**Design and Validation of a Low-Cost Single-Hip Delayed Output Feedback Control Framework**

项目书里的核心思想是：不用有限状态机去判断“现在是支撑相还是摆动相”，而是直接把髋关节运动信号延迟一小段时间，再乘以反馈增益生成助力转矩。这样控制器可以连续输出转矩，并随步频变化自然调整。

## 这个代码现在包含什么

- C++17 控制核心：Delayed Output Feedback Control, DOFC
- 延迟缓冲器：按时间保存髋角度/速度，并查询过去某个时刻的信号
- 安全限制：转矩限幅、转矩变化率限制
- walk-stop-walk 保护：走路时逐渐打开助力，停止时逐渐关掉助力
- 串联弹性转矩估计：用电机侧编码器和输出侧编码器估计弹性元件转矩
- 单髋关节仿真：用于 Phase 1 的参数扫描和稳定性检查
- CSV 数据输出：用于论文里的图表和数据分析
- 单元测试：验证核心模块没有明显逻辑错误
- GitHub Actions：推到 GitHub 后自动编译、测试、跑一次仿真

## 文件结构

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

## 编译和运行

需要先安装 CMake 和 C++ 编译器。Windows 推荐安装 Visual Studio Build Tools，macOS/Linux 推荐使用 clang 或 g++。

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build --output-on-failure
```

运行一次默认仿真：

```powershell
.\build\Release\simulate_dofc.exe --output data/simulation_output.csv
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

## 推荐阅读顺序

1. `docs/control_theory_zh.md`：先理解控制理论和公式
2. `docs/architecture_zh.md`：再看每个代码模块负责什么
3. `docs/github_debugging_zh.md`：最后看怎么用 GitHub 自动检查项目

## 和项目书的对应关系

- Phase 1 Modelling and simulation：`apps/simulate_dofc.cpp`, `apps/parameter_sweep.cpp`
- Real-time DOFC implementation：`DofcController`, `DelayBuffer`
- Torque saturation and torque-rate limiting：`TorqueLimiter`
- Dual magnetic encoders torque estimation：`SeriesElasticEstimator`
- walk-stop-walk transition behaviour：`MotionStateDetector`
- Experimental dataset：`CsvLogger` 输出 CSV

## License

This repository uses the Apache License 2.0, preserved from the GitHub repository.
