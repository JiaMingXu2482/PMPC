# PMPC / SUV DLC Simulink Model

## 快速开始

在 MATLAB 中把当前目录设为项目根目录，然后运行：

```matlab
run_current_carsim('pmpc_mil')
```

运行前在 CarSim 中选定 Run Control，修改 Procedure 的初速度并点击 **Send to Simulink**。
`JT72_R69_mu0.85_{MPC,ZENG,PMPC}` 共用一个 Procedure；车速只需改一次，但三个
Run 要逐个 Send、逐个执行上述命令。详细步骤见[运行配置](docs/README_运行配置.md)。

目标手工 COM 配置（CarSim 道路需手动同步）：DLC 至 `X=200 m`，恢复直线延长到 `X=330 m` 开始
R80、90° 左转；`X=230 m` 切换到 `mu=0.85`，三控制器共用 90→80 km/h
目标速度。分别在 CarSim 修改并 Send 三个 Run 后，用 `run_current_carsim`
运行对应模型。控制器从 CarSim 展开的道路参数读取实际入弯位置和半径。

下列命令是历史 R70、80 m 恢复直线的隔离试验，不代表当前手工 COM 道路：

```matlab
run_combined_carsim('mpc_mil')
run_combined_carsim('zeng_mil')
run_combined_carsim('pmpc_mil')
```

该历史工况以 90 km/h 起步：DLC 路段 `mu=0.5`，DLC 后有 80 m 直线；
三个控制器在直线起点共用平滑的 90→80 km/h 车速目标，在
`X=230 m`（路径里程约 230.65 m）统一把 CarSim 路面和控制器轮胎模型切到
`mu=0.85`，从 `X=280 m` 开始 R70 左转。原 CarSim Run 与 `simfile.sim`
不会被修改；输入、ERD 输出和分段误差保存在 `simulation_results/combined/`。命令行会打印
DLC 和 J-turn 两段的 `e_y` 峰值/RMS 及 `Vx` RMS。R70 弯道和 80 km/h 目标均不保证
三个控制器具有相同实测速度或稳定跟踪，必须核对进弯速度、车身角点是否
越界及 J-turn 段误差后，才能把它用于公平比较。手动建立三个 CarSim Run 的
共用道路时，见[联合工况 R70 建路参数](docs/联合工况_R70_建路.md)。

2026-10-01 实测（R69、80 m 直线、三个控制器共用 80 km/h 进弯目标）：
MPC / ZENG / PMPC 的实际进弯速度分别为 77.97 / 78.00 / 76.07 km/h；
J-turn 段 `e_y` 峰值为 3.56 / 2.99 / 1.38 m，RMS 为 1.65 / 1.51 / 0.76 m。
虽然 PMPC 误差最小，但三者按控制器的 3.5 m 车道角点判据都越界，且
PMPC 没达到 80 km/h 进弯，因此**此结果不满足联合工况验收条件**。

2026-10-02 实测（R75、其余设置不变）：MPC / ZENG / PMPC 的实际进弯
速度为 77.90 / 77.93 / 78.16 km/h；J-turn 段 `e_y` 峰值为
0.337 / 0.128 / 0.175 m，RMS 为 0.152 / 0.034 / 0.068 m，
三个控制器在 J-turn 段的角点越界率均为 0%。不过 DLC 段角点越界率
仍为 17.3% / 4.9% / 5.6%，且 PMPC 在 J-turn 段不优于 ZENG；
因此不能把这组数据描述为“PMPC 全程不越界且 J-turn 最优”。

2026-10-02 实测（当前 R70、其余设置不变）：MPC / ZENG / PMPC 的实际进弯
速度为 77.96 / 78.10 / 76.54 km/h；J-turn 段 `e_y` 峰值为
2.673 / 2.194 / 1.323 m，RMS 为 1.224 / 1.074 / 0.725 m，
角点越界率为 46.0% / 46.6% / 40.4%。PMPC 相对最小，
但**三个控制器均越界，R70 不满足联合工况验收条件**。

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
| `simulation_results/archive/` | 历史试验和诊断结果（不纳入 Git） |
| `simulation_results/combined/` | 联合工况历史样例和测试夹具 |
| `tests/` | MATLAB 自动化测试 |

`simulation_results/` 是本机仿真输出，已被 Git 忽略；需要上传的历史道路坐标表保存在 `data/`。

根目录保留四个固定控制器模型和 CarSim 句柄文件：`mpc_mil.slx`（6 状态、3 控制量）、
`zeng_mil.slx`（7 状态、3 控制量）、`pmpc_mil.slx`（7 状态、3 控制量，含减振器时延）
和 `pmpc_nodelay_mil.slx`（6 状态、3 控制量）。它们共用 NLCSNN、车辆参数、
状态估计和 54 路 CarSim 接口。8×4 `Vx/Fx` 版本保留在历史 Git 标签
`pmpc-vxfx-8x4-experiment`，不属于当前主线。

当前基准：`simulation_results/current/erd_0927_base/`。

详细运行配置见 [`docs/README_运行配置.md`](docs/README_运行配置.md)，改进记录见 [`docs/改进.md`](docs/改进.md)。
