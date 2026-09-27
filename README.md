# PMPC / SUV DLC Simulink Model

## 快速开始

在 MATLAB 中把当前目录设为项目根目录，然后运行：

```matlab
startup_pmpc
setup_pmpc
```

## 目录

| 目录 | 内容 |
|---|---|
| `controller/` | PMPC 控制算法、预测模型、QP 和执行器分配 |
| `data/` | 轮胎、路径、车辆参数和端口映射数据 |
| `scripts/` | 构建、仿真、回归检查和绘图脚本 |
| `docs/` | 改进记录、运行说明、HIL 文档和图片 |
| `simulation_results/current/` | 当前有效仿真结果 |
| `simulation_results/archive/` | 历史基准和参数扫描结果 |
| `simulation_results/output/` | 新仿真的默认输出 |
| `tests/` | MATLAB 自动化测试 |

三个 `.slx` 模型、`simfile.sim` 和 `setup_pmpc.m` 保留在根目录，以兼容 CarSim/Simulink 的相对路径。

当前基准：`simulation_results/current/erd_0927_base/`。

详细运行配置见 [`docs/README_运行配置.md`](docs/README_运行配置.md)，改进记录见 [`docs/改进.md`](docs/改进.md)。
