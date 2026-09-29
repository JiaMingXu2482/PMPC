# PMPC / SUV DLC Simulink Model

## 快速开始

在 MATLAB 中把当前目录设为项目根目录，然后运行：

```matlab
run_current_carsim
```

运行前在 CarSim 中选定 Run Control，修改 Procedure 的初速度并点击 **Send to Simulink**。
`JT72_R69_mu0.85_{MPC,ZENG,PMPC}` 共用一个 Procedure；车速只需改一次，但三个
Run 要逐个 Send、逐个执行上述命令。详细步骤见[运行配置](docs/README_运行配置.md)。

MPC、ZENG 和 PMPC 现在共用 `nlcsnn/` 中的 CDC 减振器模型。默认电流范围
`0~1.6 A`、温度 `42.5 degC`，旧的一阶执行器延迟已旁路。CarSim Export
必须是 54 路，其中 51–54 依次为 `CmpD_L1/L2/R1/R2`。

## 目录

| 目录 | 内容 |
|---|---|
| `controller/` | PMPC 核心算法、预测模型、QP 和执行器分配 |
| `nlcsnn/` | NLCSNN 权重、MATLAB 前向/反演模型与黄金向量 |
| `data/` | 轮胎、路径和车辆参数数据 |
| `scripts/` | 构建、仿真、回归检查和绘图脚本 |
| `scripts/lib/` | CarSim、结果读取和项目路径等公共工具 |
| `docs/` | 改进记录、运行说明和图片 |
| `simulation_results/current/` | 当前有效仿真结果 |
| `simulation_results/output/` | 新仿真的默认输出 |
| `tests/` | MATLAB 自动化测试 |

唯一模型 `pmpc_mil.slx`、`simfile.sim` 和 `setup_pmpc.m` 保留在根目录，以兼容 CarSim/Simulink 的相对路径。

当前基准：`simulation_results/current/erd_0927_base/`。

详细运行配置见 [`docs/README_运行配置.md`](docs/README_运行配置.md)，改进记录见 [`docs/改进.md`](docs/改进.md)。
