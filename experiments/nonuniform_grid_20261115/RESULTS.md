> **⚠ 本文件是 TASK-006 的旧结果，其中「Md 跟踪误差减半」已被证明是实现错误的产物，以 `RESULTS_corrected.md` 为准。**

# 非均匀预测网格 · 实现与最小闭环对比

目录：`MAIN/experiments/nonuniform_grid_20261115/`
生产 `pmpc_mil.slx` / 生产 `pmpc_core` 未改动；本目录是独立副本，未提交 Git。

## 0. 结果（DLC、mu=0.4、70 km/h、ctrl=3、同一快照、同一 CarSim 数据集）

| 指标 | A 均匀 10×100 ms | B 非均匀 19 步 | 变化 |
|---|---|---|---|
| ey_max [m] | 0.6359 | **0.6293** | −1.0% |
| ey_rms [m] | 0.2036 | **0.2004** | −1.6% |
| LTR_max | 0.330 | 0.324 | −1.8% |
| φ_max [°] | 1.94 | 1.89 | −2.6% |
| p_max [°/s] | 7.6 | 7.6 | 0 |
| SW_max [°] | 67.8 | **62.4** | **−8.0%** |
| SW 变化率峰值 [°/s] | 540 | 540 | 0（两者都到限幅） |
| **Md 跟踪 RMS [N·m]** | 113.9 | **57.3** | **−49.7%** |
| QP 成功率 | 100% | 100% | — |
| t_end [s] | 10.64 | 10.63 | — |

**结论：本工况有可见收益，主要是转向动作更小（−8%）与 Md 跟踪误差减半；
ey/LTR/φ 改善 1–3%。没有出现变差或失稳。** 单工况、单速度、未调权重，
不外推为普遍结论。

## 1. 网格与控制分块

```
dt = [0.005×10, 0.05×3, 0.1×4, 0.2×2]      % 19 节点，总和 1.000 s
控制变化时刻仍为 [0, 0.1, 0.2, 0.3, 0.4] s，Nc=5，最后一块保持到 1 s
节点→控制块映射 blk = [1(×11), 2(×2), 3, 4, 5(×4)]
```

映射由**节点区间起点**的累计时间决定：`blk(j)=min(Nc, floor((Σdt_{1..j-1})/Ts)+1)`。
均匀网格时退化为 `[1,2,3,4,5,5,5,5,5,5]`，与旧版 `if j<=K` 完全一致。

## 2. 实际修改（实验副本内）

| 文件 | 改动 |
|---|---|
| `pmpc_core/build_prediction.m` | 每节点用 `E0=expm(A_aug*dtmin)` 的矩阵幂得到 `Ad_j/Bd_j/Dd_j`；按 `blk(j)` 在块切换节点施加 `Baug`；首节点即施加首个决策 |
| `pmpc_core/prepare_step.m` | 计算 `dtg/blk/wnode/dtmin/kmul`；`reference()` 的站点推进改为 `Vx*dtg(j)` |
| `pmpc_core/pmpc_step.m` | 状态积分代价 `Q=kron(diag(c.wnode),Q)`（`wnode=dt/Ts`，均匀时为 1）；控制增量惩罚与约束仍按 5 个控制块 |
| `pmpc_core/pmpc_preview.m` | 预瞄时刻改为「节点区间起点 + `preview_L`」，按真实累计时间在两节点间插值 |
| `pmpc_core/pmpc_init.m` | 新增 `M.dt_grid`，`M.Np=numel(dt_grid)` |
| `pmpc_mil_nu.slx` | 模型副本；`PreLoadFcn` 的硬编码 `root='...\Code\Jturn'` 改为本目录，且不再在回调里默认 `pmpc_init(1)` |
| `run_nu.m` | runner：`gridMode='uniform'|'nu'`，`ctrl`、`speed`、`stopTime` 可传 |

**没有**新增诊断接口、没有二阶模型、没有改逆模型、没有改权重、没有动 Nc。

## 3. 一致性检查

- `sum(dt)=1.000000`、`Np=19`、`Nc=5`、`active_main=298`（均匀时 190）。
- 均匀档 A 与生产基线（`PMPC_Results/.../cp_dlc_70`）对比：
  ey 0.636 vs 0.637、SW 67.8 vs 68.1、Mz_avg 674 vs 666、LTR 0.33 vs 0.32。
  **不是逐位相同**：差异来自模型副本不再在 `PreLoadFcn` 里用默认参数初始化，
  以及预瞄取点改为按累计时间插值。量级在 1% 以内，可作为回归基线。
- 两个工况 QP 均 100%，无失败、无候选启用。

## 4. 运行与复现

```bat
cd /d "...\experiments\nonuniform_grid_20261115"
matlab -batch "worker_boot(pwd); run_nu('uniform',3,70,struct('stopTime',15));"
matlab -batch "worker_boot(pwd); run_nu('nu',     3,70,struct('stopTime',15));"
```

结果在 `results/nu_uniform_c3_70.mat`、`results/nu_nu_c3_70.mat`（含 `.vs/.vsb`）。

## 5. 已知限制与注意

1. 只跑了 DLC 70 km/h 一个工况；未扫速度/附着/权重。
2. `Md 跟踪 RMS` 用的是 `Md_cmd - Md_act`（27 行版布局，6/7 行），
   与整车日志那一轮的口径一致，但**不等于**整车上层指令已最优。
3. SW 变化率两个配置都顶到 540°/s 限幅，说明该限幅在此工况饱和，
   非均匀网格没有改变这一点。
4. 计算耗时：两次运行在同一台主机、单次墙钟约 2–3 min（含模型编译）。
   **这只是主机仿真墙钟，不代表 ECU 10 ms 实时性。**
   QP 规模：变量 `n = 3·Nc + 5 + 1 = 21`（含 5 个松弛/gamma），
   约束行 `6·Nc + 12·Np + 2·(3·Nc+5) = 30+228+40 = 298`。
5. 模型副本的 `PreLoadFcn` 被改写；若把该模型拷回生产目录，必须同步改回。

## 6. 判断

- 本工况下，非均匀网格**没有变差**，且在 Md 跟踪与转向幅度上有可见收益；
- 收益是否来自"看得见 5 ms 尺度的近期响应"还是"代价积分权重变化"，
  本轮未分离（两者同时改了）。若需要归因，下一步可只改其一做对照；
- 未验证 ECU 实时性，也未验证其他工况。
