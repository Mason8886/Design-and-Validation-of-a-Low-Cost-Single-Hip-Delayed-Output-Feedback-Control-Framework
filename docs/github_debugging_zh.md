# GitHub 调试流程

当前本地文件夹一开始不是 Git 仓库，也没有远程 GitHub 地址。因此这里先给项目加入 GitHub Actions 配置；等你把代码推到 GitHub 后，GitHub 会自动帮你编译、测试、跑一次仿真。

## 1. 第一次建立仓库

在本地安装 Git 后，可以在项目根目录运行：

```bash
git init
git add .
git commit -m "Initial DOFC control framework"
```

然后在 GitHub 网站新建一个空仓库，再按网页提示添加远程地址：

```bash
git remote add origin https://github.com/YOUR_NAME/YOUR_REPO.git
git branch -M main
git push -u origin main
```

## 2. GitHub 会自动做什么

文件：

```text
.github/workflows/ci.yml
```

会让 GitHub 在每次 push 或 pull request 时自动执行：

1. 配置 CMake
2. 编译 C++ 项目
3. 运行单元测试
4. 运行一次短仿真，确认程序能输出 CSV

这相当于一个自动助教：每次你改代码，它都会帮你检查有没有把项目改坏。

## 3. 常见失败怎么读

### 编译失败

通常说明 C++ 代码语法错了，或者某个头文件路径不对。

优先看 GitHub Actions 红色日志里第一条真正的 compiler error。后面的错误经常只是连锁反应。

### 测试失败

说明代码能编译，但行为不符合预期。

例如延迟缓冲测试失败，可能是插值逻辑错了；转矩限幅测试失败，可能是安全逻辑被改坏了。

### 仿真失败

说明核心程序能编译、测试也过了，但运行时出错。常见原因是输出路径不存在、参数非法、或者控制参数导致数值不稳定。

## 4. 推荐开发方式

每次只做一小步：

```text
改一个模块 -> 本地编译测试 -> commit -> push -> 看 GitHub Actions
```

不要一次性改很多文件。否则 CI 报错时，很难判断是哪一步造成的。

## 5. 和论文的关系

GitHub Actions 可以写进 dissertation 的 software validation 小节：

```text
The control software was maintained under version control, and automated CI checks were used to compile the C++ project, run unit tests, and execute a simulation smoke test after each update.
```

这会让你的软件开发过程看起来更专业，也更符合工程项目的验证逻辑。
