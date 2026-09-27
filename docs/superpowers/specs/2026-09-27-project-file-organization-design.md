# PMPC 项目文件整理设计

> 后续已进一步简化为单模型结构：只保留 `pmpc_mil.slx`。原设计中关于
> `cq3_2019.slx`、`pmpc_hil.slx`、部署目录和历史结果归档的内容已废止；
> 当前结构以项目根目录的 `README.md` 为准。

## 目标

把当前根目录中的控制算法、数据、工具脚本、说明文档和仿真结果分开存放，使常用文件容易查找，同时不破坏 CarSim、Simulink、MATLAB Coder 和现有 Git 管理方式。

整理完成后，项目根目录只保留必须从根目录访问的模型、入口文件和项目说明。移动源文件时使用 Git 记录重命名；生成结果仍不纳入版本控制。

## 约束

- `cq3_2019.slx`、`pmpc_mil.slx`、`pmpc_hil.slx` 和 `simfile.sim` 保留在根目录，避免破坏 CarSim 的相对路径和 Send to Simulink 工作流。
- `setup_pmpc.m` 保留在根目录，作为统一配置入口。
- 新增 `startup_pmpc.m`，通过项目根目录的绝对路径加入算法、数据和脚本目录；不得依赖用户机器上的固定绝对路径。
- `MIL_deploy/`、`HIL_deploy/` 和 `deploy/` 是生成副本，本次不整理、不修改。
- `Results/` 是 CarSim 运行目录，本次不移动；继续由 `.gitignore` 排除。
- 删除对象只限确认无用的模型备份和一次性诊断输出；当前基准、源代码、配置数据和文档不得删除。
- Windows 下 `Results` 与 `results` 大小写不区分，因此统一使用 `simulation_results/` 存放控制器仿真输出。

## 目标目录

```text
Simulink_Model/
├─ README.md
├─ startup_pmpc.m
├─ setup_pmpc.m
├─ cq3_2019.slx
├─ pmpc_mil.slx
├─ pmpc_hil.slx
├─ simfile.sim
├─ controller/
├─ data/
├─ scripts/
├─ docs/
│  ├─ figures/
│  └─ superpowers/
├─ simulation_results/
│  ├─ current/
│  └─ archive/
├─ tests/
├─ TruckSim_Model/
├─ Results/
├─ MIL_deploy/
├─ HIL_deploy/
└─ deploy/
```

## 文件归类

### `controller/`

存放运行时控制算法和其直接依赖：

- `pmpc_step.m`、`pmpc_block.m`；
- 所有 `func_*.m` 控制、预测、约束、代价、求解、状态估计、执行器分配及路径处理函数；
- `wsget.m`。

`func_ErdDir.m` 移入该目录后必须改为从父目录定位项目根目录，并把默认输出改到 `simulation_results/output/`。读取 `simfile.sim` 的函数必须使用项目根目录绝对路径，不能继续依赖当前工作目录。

### `data/`

存放受版本控制的输入数据：

- 轮胎、车辆、路径和鞍点边界 `.mat` 文件；
- `portmap*.csv` 端口映射文件。

`tests/WayPoints_Type1.mat` 是测试夹具，继续留在 `tests/`。

### `scripts/`

存放开发和分析入口，不参与控制器逐拍运行：

- `build_*.m`、`make_*.m`、`cg_*.m`；
- `run*.m`、`chk_*.m`；
- `cert_replay.m`、`plot_week.m`、`fh_theory.m`；
- `mil_init.m`、`hil50.m`。

脚本中的相对文件路径改为基于项目根目录或 `startup_pmpc.m` 配置的搜索路径。

### `docs/`

存放现有 Markdown 文档和图片：

- `改进.md`、HIL/端口映射/运行配置/审查文档；
- 原 `figs/` 移为 `docs/figures/`；
- 设计和实施计划继续放在 `docs/superpowers/`。

根目录新增简短 `README.md`，只说明目录结构、首次启动命令和当前有效基准位置。

### `simulation_results/`

- `current/` 只保存当前有效基准 `erd_0927_base/`；
- `archive/` 收纳仍需保留的旧基准和参数扫描结果；
- `output/` 作为新仿真的默认输出目录。

这些目录整体由 `.gitignore` 排除。整理阶段不把大型仿真结果提交到 Git。

## 启动与路径管理

`startup_pmpc.m` 以自身位置确定项目根目录，并加入：

- `controller/`
- `data/`
- `scripts/`

它不使用 `genpath`，避免把结果、部署副本和临时目录加入 MATLAB 路径。`setup_pmpc.m` 开头调用一次 `startup_pmpc`，保证从项目根目录运行模型时自动找到依赖。

README 中统一使用以下启动方式：

```matlab
cd('<项目根目录>')
startup_pmpc
setup_pmpc
```

## 清理规则

- 删除未跟踪的 `*.bak_*`、`*.original` 等旧模型备份；当前 `.slx` 模型保留。
- `step_*_result.mat` 等一次性诊断结果在确认结论已写入 `改进.md` 后删除或放入回收站。
- 已被判定为过时但仍有历史参考价值的结果移入 `simulation_results/archive/`，不留在根目录。
- 不删除当前基准 `erd_0927_base/`。

## 验证标准

整理完成必须同时满足：

1. 根目录只剩模型、入口、项目说明及必要的外部工具目录。
2. `git status` 只显示预期的重命名、路径修正和新增说明文件。
3. MATLAB 执行 `startup_pmpc` 后，`which pmpc_step`、`which func_SolveMPCQP`、`which func_TireTable` 和 `which run_ds` 均指向新目录。
4. `setup_pmpc` 能从项目根目录执行，所需 MAT 数据能够找到。
5. 三个 Simulink 模型可以加载，且 `simfile.sim` 仍从根目录解析。
6. 文件移动前后的受版本控制文件数量一致，清理清单中的生成文件除外。
7. 当前基准 `simulation_results/current/erd_0927_base/` 文件数量与移动前一致。

## 非目标

- 不修改控制算法、权重、约束、采样周期或仿真参数。
- 不同步或重建 MIL/HIL 部署包。
- 不重跑控制器对比实验。
- 不借文件整理修复现有的算法测试或论文内容问题。
