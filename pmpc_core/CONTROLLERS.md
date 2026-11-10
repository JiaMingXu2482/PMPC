# 三种控制器的切换说明

## 1. 只改一个参数

```matlab
M.controller = 3;   % 1=MPC, 2=ZENG, 3=Proposed
```

`pmpc_init.m` 里默认是 1（MPC）。做对比时也可以直接传第二个参数：

```matlab
[params,Reftraj,state0,meas0] = pmpc_init(3,1);   % J-turn + MPC
[params,Reftraj,state0,meas0] = pmpc_init(3,2);   % J-turn + ZENG
[params,Reftraj,state0,meas0] = pmpc_init(3,3);   % J-turn + Proposed
```

`pmpc_step.m` / `pmpc_preview.m` 不需要改，它们只读 `params`。

## 2. 控制器层字段

```matlab
M.controller = 3;   % 1=MPC, 2=ZENG, 3=Proposed
M.Q_norm     = [0;0;5;0;50;5;0];   % [Vy;r;phi;p;ey;epsi;Md]
M.R_norm     = [0.1;500;0.1];      % [dDelta;dMz;dMd]
M.slack_w    = [5;5;5;5];          % [r;alpha;LTR;env]
```

这三个权重现在按工况在 `switch maneuver` 里预设：

| 工况 | Q_norm | R_norm | slack_w | 目标车速 |
|---|---|---|---|---|
| DLC (mu=0.5) | `[0;0;10;0;100;20;0]` | `[100;200;0.1]` | `[1e4;1e4;1e4;1e4]` | 60/70/80 km/h 对比 |
| Slalom | 暂用 J-turn | 暂用 J-turn | 暂用 J-turn | 待定 |
| J-turn (mu=0.85) | `[0;0;5;0;50;5;0]` | `[0.1;500;0.1]` | `[5;5;5;5]` | 70 km/h |

固定参考尺度保持全局，不随工况变。

DLC 使用原 tanh 路径。该路径峰值曲率约 `0.036 1/m`：
- 60 km/h 峰值横向加速度需求约 `1.0g`；
- 70/80/90 km/h 分别需要约 `1.4g/1.8g/2.3g`。

`mu=0.5` 时三档都远超附着极限，车辆主要表现为推头、`ey` 为米级；
`mu=0.85` 时 60 km/h 接近可行，70/80 仍超限，但 Zeng 稳定域有效，
适合比较 MPC/ZENG 的稳定性调度差异。

其余行为由 `M.controller` 派生，不再单独设开关：

| 行为 | MPC | ZENG | Proposed |
|---|---|---|---|
| 松弛权重 | `M.slack_w` 固定 | `M.slack_w` 固定，`w_r`、`w_alpha` 都乘 `rho` | 余量在线算 `w1/w2` |
| `gamma` | 固定 1 | 固定 1 | 自由，`W_b=100`（待调） |
| 绝对 Mz 二次罚 | 无 | 无 | 无 |
| 约束 | 全软 | 全软 | 全软 |

## 3. 固定归一化尺度

在 `pmpc_step.m` 的 `build_qp` 里写死，与工况无关：

```matlab
VY_REF=1.0; R_REF=0.2; PHI_REF=deg2rad(2); P_REF=deg2rad(10);
EY_REF=0.3; EPSI_REF=deg2rad(1);
DFYF_REF=deg2rad(1); MFX_REF=500; MD_REF=500;
EPS_R_REF=deg2rad(1); EPS_A_REF=deg2rad(1);
EPS_LTR_REF=0.1; EPS_E_REF=0.1;
```

- 状态：`Q = diag(Q_norm ./ X_REF.^2)`，`X_REF=[VY_REF;R_REF;PHI_REF;P_REF;EY_REF;EPSI_REF;1]`
- 输入：`R = diag(R_norm ./ [DFYF_REF;MFX_REF;MD_REF].^2)`
- 松弛：4 个松弛都按对应的 `EPS_*_REF` 归一化，是无量纲量

## 4. 4 松弛 QP

```text
z = [DeltaU; s_r; s_alpha; s_LTR; s_env; gamma]
n = nd + 5
```

| 行 | 约束 | 松弛 |
|---|---|---|
| `1:6K` | 输入盒/速率 | 硬 |
| `6K+1:6K+2N` | 横摆 `±r` | `s_r` |
| `6K+2N+1:6K+4N` | 后轴侧偏 `±alpha_r` | `s_alpha` |
| `6K+4N+1:6K+6N` | LTR `±` | `s_LTR` |
| `6K+6N+1:6K+10N` | 道路四角 | `s_env` |
| `6K+10N+1:6K+12N` | Md 盒 | 硬 |

松弛罚为线性：

```text
J_slack = w_r*s_r + w_alpha*s_alpha + w_LTR*s_LTR + w_env*s_env
```

## 5. 稳定性约束（三种控制器共用）

```matlab
r_max = mu*g/Vx;
Fzr_st = m*g*lf/L;
Cr_cur = max(abs(CbarR),1e3);
alpha_r_sat = min(atan(mu*Fzr_st/Cr_cur), atan(3*mu*Fzr_st/abs(CarHat)));
Gsh = [r_max; r_max; alpha_r_sat; alpha_r_sat];
```

## 6. ZENG 2025 的 rho

`func_ZengRho.m` 实现 Zeng 2025 Eq.(12)-(14)：先查 SUV 鞍点库得到
`beta_saddle_l/r`，再算 `I_beta`、`I_r`，取 `I_ind=max(I_beta,I_r)`：

```text
rho = 0                              , I_ind <= I0
rho = 0.5*(1-cos(pi*(I_ind-I0)/dI))  , I0 < I_ind < I0+dI
rho = 1                              , I_ind >= I0+dI
```

当前 `I0=0.7`、`dI=0.3`。`rho` 乘在稳定性松弛权重上：

- `w_r     = w_r0     * rho`
- `w_alpha = w_alpha0 * rho`

稳定时 `rho=0`，放松稳定性、优先跟踪；接近/超过稳定域时
`rho→1`，收紧稳定性。LTR/道路松弛不乘 `rho`。

鞍点库 `ZengSaddleDB_SUV.mat` 与 `func_ZengRho.m` 为控制器 2 的正式实现。
## 7. 改完必须重新初始化

4 个松弛改变了 QP 维数，`state0.active_main` 长度从
`6K+12N+2*(3K+3)` 变成 `6K+12N+2*(3K+5)`。改完代码后必须：

1. 重新运行 `pmpc_init`（或让模型 `InitFcn` 重跑）；
2. 重新生成 `PMPC_StateBus`；
3. 重新打开模型，让 MATLAB Function 块重新编译。

否则会出现状态总线维度不匹配。

## 8. 当前状态

- DLC 已恢复为原 tanh 路径，没有改成 60 m 正弦路径；
- DLC 权重定为 `Q=[0;0;10;0;100;20;0]`、`R=[100;200;0.1]`、`slack_w=[1e4;1e4;1e4;1e4]`；
- MPC/ZENG 已在 60/70/80 km/h 下对比，Proposed 暂不参与；
- Slalom 权重暂用 J-turn，后续单独整定；
- Proposed 的 `W_b` 目前固定 100，写在 `build_qp` 里，后续再调。

## 9. DLC MPC vs ZENG（mu=0.5，10 s，按同一 station 截取）

**注意：本节表格是旧 ZENG 版本（只乘 `w_r`，且有 `rho_min=0.1`）的历史结果；
当前代码已改为 `w_r`/`w_alpha` 都乘 `rho`、无下限，数值会变。**

**2026-11-08 修正**：之前表里的 `ey max = 1.9/6.9/7.1 m` 是
`Cf_floor = 0.2` 的结果。轮胎饱和时预测侧偏刚度的下限取得太高，
控制器误以为轮胎还有 20% 的侧偏刚度，实际已经饱和，导致推头、
`ey` 被放大。把 `prepare_step.m` 里的下限硬编码为 `0.04` 后，三档 `ey max`
降到 `0.62/0.67/0.78 m`。同时在 `pmpc_step.m` 里给 ZENG 的 rho 加 0.1 下限，
防止稳定时 `w_r = rho*w_r0` 把横摆约束完全关掉（80 km/h 会因此丢路径）。

按公共 station `0–165.6 m` 截取：

| 目标 | 控制器 | ey max | ey RMS | epsi max | beta max | beta_r max | r max | Mz max | QP |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 60 | MPC | 0.617 | 0.165 | 5.56° | 1.248° | 6.500° | 16.82°/s | 3059 | 100% |
| 60 | ZENG | 0.617 | 0.165 | 5.58° | 1.246° | **6.381°** | 16.82°/s | 3162 | 100% |
| 70 | MPC | 0.669 | 0.254 | 6.01° | 1.633° | 8.475° | 15.40°/s | 3218 | 100% |
| 70 | ZENG | **0.667** | 0.255 | 6.00° | 1.636° | 8.503° | 15.40°/s | 3224 | 100% |
| 80 | MPC | 0.779 | 0.361 | 5.94° | 1.717° | 9.253° | 13.05°/s | 2861 | 100% |
| 80 | ZENG | **0.776** | **0.360** | 5.96° | 1.716° | 9.203° | 13.06°/s | 2861 | 100% |

结论：

- MPC 与 ZENG 的 `ey max` / `ey RMS` 差异都在 0.5% 以内，曲线基本重合；
- ZENG 的 `beta_r max` 在 60 km/h 低 1.8%，70/80 差异小于 0.5%；
- `Mz max` 差异小于 3.4%；
- 修正轮胎饱和模型后，mu=0.5 DLC 下 ZENG 的 rho 调度不再带来明显差异。
  之前的“ZENG 明显更好/更差”主要是 `Cf_floor=0.2` 造成的数值假象。

`rho` 诊断：修正后 70 km/h 的原始 rho 均值约 0.35，`rho_min=0.1`
把有效调度下限钳在 0.1；因此 ZENG 的横摆约束不会完全消失，但也不会
比 MPC 更严格。DLC 不是持续接近稳定边界的工况，ZENG 与 MPC 接近是合理的。

## 10. DLC MPC vs ZENG（mu=0.85，10 s，旧轮胎下限）

**注意：以下结果基于旧的 `Cf_floor=0.2`，只作历史对照；修正后需重跑。**


按同一 station `0–165.8 m` 截取：

| 目标 | 控制器 | ey max | ey RMS | epsi max | beta max | beta_r max | r max | Mz max | QP |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 60 | MPC | 0.278 | 0.074 | 2.42° | 1.685° | 7.814° | 25.28°/s | 2488 | 100% |
| 60 | ZENG | 0.278 | 0.074 | 2.42° | 1.683° | 7.810° | 25.28°/s | 2488 | 100% |
| 70 | MPC | **0.403** | **0.115** | 3.67° | 2.196° | 11.045° | 24.27°/s | 2903 | 100% |
| 70 | ZENG | 0.424 | 0.119 | 3.59° | 2.207° | **10.803°** | 24.24°/s | 2960 | 100% |
| 80 | MPC | **0.510** | **0.156** | 4.58° | 2.402° | **10.971°** | 22.12°/s | 3522 | 100% |
| 80 | ZENG | 0.531 | 0.163 | 4.50° | 2.402° | 11.789° | 22.29°/s | 3785 | 100% |

补充高车速（同一数据集，目标 90/100 km/h）：

| 目标 | 控制器 | ey max | ey RMS | beta_r max | Mz max |
|---:|---|---:|---:|---:|---:|
| 90 | MPC | 0.626 | 0.203 | 10.93° | 5294 |
| 90 | ZENG | **0.597** | 0.203 | 12.26° | **5016** |
| 100 | MPC | **0.629** | **0.229** | 14.26° | 5262 |
| 100 | ZENG | 0.678 | 0.252 | 16.42° | **4743** |

`rho` 诊断（mu=0.85）：

| 目标 | rho 均值（MPC/ZENG） | rho>0.7 占比 | I_beta 主导占比 |
|---:|---:|---:|---:|
| 60 | 0.01 / 0.01 | 0% | 42% |
| 70 | 0.07 / 0.08 | 4.5–4.8% | 40–41% |
| 80 | 0.11 / 0.12 | 8.9–9.7% | 42% |
| 90 | 0.14 / 0.17 | 11.6–14.4% | 45–51% |
| 100 | 0.22 / 0.29 | 19.4–28.0% | 65–68% |

结论：

- 60 km/h：MPC 与 ZENG 完全一致，`rho=0`，稳定域未触发；
- 70/80 km/h：MPC 的 `ey` 略好，ZENG 的 `beta_r` 在 70 略好、80 略差；
- 90 km/h：ZENG 的 `ey max` 略好（-4.7%），但 `beta_r` 高 12.2%；
- 100 km/h：MPC 的 `ey` 和 `beta_r` 都更好，ZENG 只胜在 `Mz` 低 9.9%。

原因：mu=0.85 下 DLC 的 `Ay max` 只有 0.59–0.60g，远低于 0.85；Zeng 稳定域
（`r_max` 21.5–28.7°/s、`beta` 约 ±10°）大部分时间未触发（rho>0.7 仅 0–28%）。
ZENG 在常规工况下基本退化为 MPC；它的“稳定时衰减、失稳时收紧”机制需要
持续接近稳定边界的工况才能体现。

注意：QP 的后轴侧偏约束仍可能被违反（`beta_r` 10–12° > `alpha_r_sat` 10.4°），
但 `func_ZengRho` 用的是 CG `beta`（约 2.4°），看不到这个违反。这是 rho 调度
变量（CG beta）与 QP 稳定性约束变量（后轴 alpha_r）不一致的地方，也是
DLC 下 ZENG 优势不明显的原因之一。

## 11. 轮胎侧偏刚度核对（TireFy CSV）

数据：`TireFy_4a6a64d4-89b0-4460-b52b-6b8720461ac6.csv`
（265/75 R16，轮胎数据 mu=1.0，Fz=2819–22555 N/胎）。

由 CSV 的 0/0.5/1° 三点拟合初始斜率，与 `TireF.cq` 对比：

| Fz/胎 (N) | C_CSV (kN/rad/胎) | C_cq (kN/rad/胎) | 比值 |
|---:|---:|---:|---:|
| 2819 | 38.4 | 37.8 | 0.985 |
| 5638 | 73.8 | 72.5 | 0.983 |
| 8458 | 105.8 | 104.1 | 0.983 |
| 11278 | 134.3 | 132.3 | 0.985 |
| 14097 | 159.2 | 157.0 | 0.986 |
| 16916 | 180.4 | 178.2 | 0.988 |
| 19736 | 198.1 | 195.6 | 0.988 |
| 22555 | 212.2 | 209.3 | 0.986 |

静态载荷处：

| 位置 | Fz/胎 (N) | C_CSV (kN/rad/胎) | C_CSV 轴值 (kN/rad) | 名义值 (kN/rad) |
|---|---:|---:|---:|---:|
| 前 | 5268 | 69.2 | 138.3 | 115.5（-16.5%） |
| 后 | 3855 | 51.4 | 102.8 | 107.0（+4.1%） |

结论：

- `cq` 在 2819–22555 N 范围内与 CSV 误差 1.2–1.7%，`mq` 峰值误差 <0.1%，
  主轮胎模型是正确的；
- 名义 `CafHat=-115.5 kN/rad`、`CarHat=-107.0 kN/rad` 只是近似值；
- `CafHat/CarHat` 只用于 `Cf` 下限和 `alpha_r_sat`，不参与主轮胎力计算；
  当前 `Cf_floor=0.04` 与 `CafHat=115.5k` 配套（绝对下限 4.62 kN/rad）。
  若把名义值改成 CSV 值（138.4/102.8 kN/rad），应把 floor 改成约 0.033，
  保持同样的绝对下限；否则 80 km/h 会因下限抬高而变差（实测 2.37 m）。
- 之前的 7 m 误差来自 `Cf_floor=0.2`，不是 `cq` 或名义值。

## 12. 转向抖动整定（mu=0.9，DLC）

原 DLC 的 `R_norm(1)=0.1`、`delta_sw_rate_max=720°/s`，80 km/h 时方向盘
命令在轮胎饱和边界附近来回切，高频抖动明显（5–6.5 s 处约 ±10°）。

整定后：

```matlab
M.R_norm = [30; 200; 0.1];          % 60 km/h：R1=30；80 km/h 建议改回 10
V.delta_sw_rate_max = deg2rad(540); % 最大方向盘角速度
```

60 km/h 下 R1 的对比：

| R1 | ey max / RMS | dSW RMS | 高频残差 RMS | 换向次数 |
|---:|---|---:|---:|---:|
| 10 | 0.507 / 0.153 | 178°/s | 0.90° | 39 |
| 20 | 0.497 / 0.145 | 139°/s | 0.82° | 16 |
| **30** | **0.504 / 0.150** | **125°/s** | **0.36°** | **11** |

80 km/h 下 R1=30 会把 `ey max` 推到 2.14 m（R1=10 是 1.10 m），
所以 80 km/h 工况建议保持 R1=10。

## 12. 方向盘抖动与转向速率

DLC 80 km/h、mu=0.9、MPC 下，原 `R1=0.1` 的方向盘命令有明显高频抖动。
把 DLC 的 `M.R_norm(1)` 从 0.1 提到 10 后：

| R1 | ey max | ey RMS | dSW RMS | 高频残差 RMS | 换向次数 |
|---:|---:|---:|---:|---:|---:|
| 0.1 | 0.490 | 0.151 | 272°/s | 1.58° | 56 |
| 5 | 0.481 | 0.164 | 214°/s | 0.70° | 23 |
| **10** | **0.489** | **0.173** | **187°/s** | **0.41°** | **15** |
| 30 | 0.527 | 0.191 | 139°/s | 0.21° | 7 |

R1=10 把高频抖动从 1.58° 降到 0.41°（-74%），`ey max` 基本不变。

转向速率上限：

- 720°/s 属于高性能/F1 EPS 的量级；
- 乘用车 EPS 常用 **540°/s**；
- 实测 540°/s 与 720°/s 的 `ey` 几乎相同（80 km/h: 0.487 vs 0.489），
  但命令速率被限制在更合理的 540°/s。

当前 DLC 设置：`M.R_norm = [10; 200; 0.1]`，`V.delta_sw_rate_max = 540°/s`。

## 12. 方向盘速率上限

`V.delta_sw_rate_max` 是**方向盘**角速度上限，前轮速率 = 它 / `isw`（18.57）。

- 720°/s 是 NHTSA 鱼钩（fishhook）试验用的极限值，不是 DLC 的常规值；
- 一般乘用车前轮转角速率取 **0.35–0.5 rad/s**，对应方向盘
  **370–540°/s**；
- 旧工程 DLC 用过 `dlt_rate_max = 0.5 rad/s`（约 487°/s 方向盘）；
  也有文献取 300°/s（Ataei）；
- 当前 DLC 取 **480°/s**（前轮约 0.5 rad/s），J-turn/fishhook 保留 720°/s。

DLC 的 `M.R_norm(1)` 从 0.1 提到 10，进一步抑制 QP 在轮胎饱和附近的高频切换。
80 km/h、mu=0.9 下：`ey max 0.495 m`，方向盘高频残差从 1.58° 降到 0.44°，
换向次数从 56 降到 17。


## 13. Proposed 整定（DLC 80 km/h，mu=0.5）

> 以下结果基于旧车辆模型；2026-11-10 更新车辆参数和转向柔度后，
> 结果见第 15 节。

原始 Proposed 的在线权重上限是 `w2_max=1e6`、`w1_max=1e5`。在
`margin→0` 的极限附近，`w2=3F/margin` 会冲到上限，QP 活动集在相邻拍
切换，表现为 `Mz` 抖动、`SW` 过大，`ey max=1.556 m`、`QP=98.9%`。

整定后（已写入 `pmpc_init.m` 默认值）：

```matlab
M.Wb      = 100;
M.w2_max  = 2e3;
M.w2_gain = 3;
M.w1_max  = 2e3;
M.w1_gain = 3;
% DLC + controller 3
M.Q_norm(6) = 10;      % epsi 权重 20 -> 10
M.R_norm(2) = 200;
```

结果（同一 CarSim 数据 `Run_4d669775`，DLC80_mu0.5，80 km/h）：

| 控制器 | ey max | ey RMS | SW max | Mz max | Md max | beta max | yaw max | LTR max | QP |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| MPC-NPC (`Q_epsi=20`) | 0.805 | 0.363 | 396.1° | 2778 | 1325 | 1.72° | 13.1°/s | 0.48 | 100% |
| ZENG (`Q_epsi=10`, `I0=1.0`) | 0.689 | 0.316 | 320.5° | 2815 | 1710 | 1.79° | 14.9°/s | 0.48 | 100% |
| **Proposed 整定** | **0.674** | **0.272** | **88.6°** | 2729 | **1152** | **1.40°** | 13.1°/s | **0.44** | **100%** |

相对 MPC：`ey max -16.3%`、`ey RMS -25.1%`、`SW max -77.6%`、
`Md max -13.1%`、`beta max -18.6%`，QP 从 98.9% 恢复到 100%。

关键现象：Proposed 整定后用很小的前轮转角（`Steer_L1 max ≈ 4.6°`）
配合 `Mz/Md` 完成 DLC，`LTR` 反而更低（0.44 vs 0.48），说明不是靠
放松侧倾约束换跟踪，而是把执行器从方向盘转移到横摆力矩。

结果文件：`PMPC_Results/DLC_mu0.5_80_compare/prop_tuned_vs_mpc_zeng.png`。


## 14. 车辆模型参数更新（2026-11-10，来自 Predict_Model_Verify）

按 `Thesis/Code/Predict_Model_Verify/verification/README.md` 的最终版，
把 Jturn 控制器的车辆模型同步到当前 CarSim 定制车：

```matlab
V.m    = 1830;            % 原 1860
V.Ix   = 1070.177651;     % 原 1038
V.Iz   = 3378.9735;       % 原 3438
V.h_TL = 0.6760655738;    % 原 0.671694
V.lf   = 1.218688525;     % 原 1.246613
V.lr   = V.L - V.lf;      % 1.731311475

V.Kt   = 81.5e3;          % 稳态圆周辨识值（原公式 78.97e3）
V.cs_fy = -3.2e-6;        % eclass2 CS_FY_COEFFICIENT [rad/N]

V.CafHat = -145.4e3;      % 轮胎模型静态值
V.CarHat = -103.8e3;

% TireF.cq = cq0*1.029; TireR.cq = cq0*1.044; 保留载荷相关和 Fiala
```

`prepare_step.m`：

- `Cf/Cr` 下限从 `0.04*|CafHat|` 改为 `0.032*|CafHat|`，
  保持绝对下限约 4.65 kN/rad；
- `c.Fyf = sum(Fy([1,3]))`，供转向柔度前馈使用。

`pmpc_step.m`：

```matlab
Fyf_ff = sum(meas.Fy([1,3]));
SW_raw = rad2deg(isw*(delta_d + u(1) - V.cs_fy*Fyf_ff));
SW = sw_prev + clip(SW_raw - sw_prev, ±delta_sw_rate_max*Ts_exec);
stateNext.sw_prev = SW;
```

`state0.sw_prev = 0`，`PMPC_StateBus` 由 `pmpc_bus` 自动增加该字段。
`V.isw` 仍保持 18.57，不把有效转向比塞回传动比。

注意：

- 这是**预测模型**的参数/执行器柔度修正，不改变 CarSim 被控对象；
- 修改后 DLC/J-turn/Slalom 的 `ey/LTR/Mz` 会变，需要重新跑基线；
- 当前 J-turn 80 km/h/mu=0.85 仍是不可行工况，不能用来评价这次修正；
- 初步对照见 `Predict_Model_Verify/verification/README.md` 第 3.3(e) 节：
  DLC 80 km/h 加入转向柔度前馈后 `ey RMS` 约 0.236 m、`ey max` 约 0.597 m。


## 15. 更新车辆模型后的 DLC 三控制器（2026-11-10）

车辆模型已按 `Predict_Model_Verify/verification/README.md` 更新：
`m/Ix/Iz/lf/h_TL`、`Kt=81.5e3`、`CafHat/CarHat`、轮胎 `cq` 幅值修正，
以及转向柔度前馈 `V.cs_fy=-3.2e-6` + `sw_prev` 540°/s 速率限制。

工况：`Run_4d669775`，DLC 80 km/h，mu=0.5。
MPC：`Q_epsi=20`、`slack=0.5e4`；ZENG：`Q_epsi=10`、`I0=1.0`、`slack=1e4`；
Proposed：`Q_epsi=10`、`w2_max=w1_max=2e3`（旧模型整定值）。

| 控制器 | ey max | ey RMS | ey final | SW max | Mz max | Md max | beta max | yaw max | LTR max | QP |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| MPC-NPC | **0.597** | **0.236** | 0.036 | **96.5°** | 2729 | 1423 | **1.58°** | 13.4°/s | **0.46** | 100% |
| ZENG | 0.611 | 0.242 | 0.034 | 112.5° | 2807 | 1837 | 1.66° | 14.7°/s | 0.47 | 100% |
| Proposed（整定前） | 0.641 | 0.249 | **0.025** | 104.2° | **2715** | **1359** | 1.66° | **13.3°/s** | 0.47 | 100% |

结论：

- 更新模型/转向柔度后，MPC 从旧模型的 `0.805/0.363` 降到
  **`0.597/0.236`**，与 `Predict_Model_Verify` 的 DLC + steer FF 结果
  （0.597/0.236）一致，说明移植正确；
- 现在 **MPC 的 ey/beta 最好**；Proposed 的 ey 比 MPC 差约 7%，
  但 `Mz 均值` 更低（1063 vs 1364）、`Md max` 更低（1359 vs 1423）、
  终点 ey 更小（0.025 vs 0.036）；
- Proposed 的 `2e3/2e3`、`Q_epsi=10` 是在旧模型上整定的；新模型上的
  重新整定见第 16 节；
- ZENG 在这组仍然略差于 MPC，`rho` 调度没有带来路径收益。

图：`PMPC_Results/DLC_mu0.5_80_compare/three_controllers_updated_model.png`。


## 16. Proposed 在新车辆模型上的最终整定（DLC 80 km/h，mu=0.5）

新模型/转向柔度前馈生效后，Proposed 原来的 `Q_epsi=10`、`Wb=100` 不再合适。
重新扫描 `Q_epsi`、`Wb`、`w2/w1` caps 和 `R2`，最终选择：

```matlab
% pmpc_init.m, DLC + controller 3
M.Q_norm(6) = 5;      % epsi 权重 20 -> 5
M.Wb        = 10;     % DB gamma 罚 100 -> 10
M.w2_max    = 2e3;
M.w1_max    = 2e3;
M.w2_gain   = 3;
M.w1_gain   = 3;
M.R_norm(2) = 200;
```

整定趋势（`ey max / RMS`）：

| 设置 | ey max | ey RMS | 说明 |
|---|---:|---:|---|
| `Q_epsi=10`、`Wb=100`、caps `2e3` | 0.641 | 0.249 | 新模型初始值 |
| `Q_epsi=5`、`Wb=100`、caps `2e3` | 0.623 | 0.244 | 降 Q_epsi 有帮助 |
| `Q_epsi=10`、`Wb=100`、caps `1e4` | 0.601 | 0.257 | max 好，RMS/SW 变差 |
| `Q_epsi=5`、`Wb=100`、caps `1e4` | 0.607 | 0.262 | max 好，RMS 差 |
| **`Q_epsi=5`、`Wb=10`、caps `2e3`** | **0.598** | **0.229** | **最终选择** |
| `Q_epsi=5`、`Wb=20`、caps `2e3` | 0.598 | 0.231 | SW/Mz 更小，beta 略高 |
| `Q_epsi=5`、`Wb=1000`、caps `2e3` | 0.663 | 0.248 | gamma 罚太大，DB 几乎不用 |

最终三控制器：

| 控制器 | ey max | ey RMS | ey final | SW max | Mz max | Md max | beta max | yaw max | LTR max | QP |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| MPC-NPC | **0.597** | 0.236 | 0.036 | 96.5° | 2729 | 1423 | **1.58°** | 13.4°/s | 0.46 | 100% |
| ZENG | 0.611 | 0.242 | 0.034 | 112.5° | 2807 | 1837 | 1.66° | 14.7°/s | 0.47 | 100% |
| **Proposed 最终** | **0.598** | **0.229** | **0.024** | **77.0°** | **2717** | **1289** | 1.59° | **13.3°/s** | **0.45** | **100%** |

结论：

- Proposed 的 `ey max` 与 MPC 基本持平（0.598 vs 0.597），`ey RMS`
  好 3.0%，`SW max` 小 20.2%，`Md max` 小 9.4%，`beta` 只差 0.01°；
- `Wb=10` 是关键：新模型下 MPC/ZENG 的转向柔度前馈已经把方向盘压到
  96–112°，Proposed 不能再用 `Wb=100` 去过度惩罚 DB；`Wb=10` 允许
  gamma 正常参与，从而用更小的方向盘和 Md 达到同等 ey；
- `Wb=1000` 会几乎关闭 DB（`Mz max` 只有 247 N·m），ey 退化到 0.663；
- `Wb=20` 是更平滑的备选（SW 74.5°、Mz max 2710、HF 13），但 beta 1.60°。

图：`PMPC_Results/DLC_mu0.5_80_compare/three_controllers_new_model_tuned.png`。
默认参数复现：`dlc_one(1e4,10,'prop_new_default_check',3,80)` →
`ey 0.598/0.229`，QP 100%。


## 17. DLC mu=0.5 速度扫描：初始速度 = 目标速度

CarSim 数据集 `Run_4d669775`，通过 CarSim 的 procedure/run-control
`*SPEED` 设置初速，并在 `Run_all.par` 中确认 `SV_VXS` 与目标一致；
Simulink `v_target` 同步设置。每档实际 `Vx` 从目标车速开始。

| 初速=目标 | MPC ey max/RMS | ZENG ey max/RMS | Proposed ey max/RMS |
|---:|---:|---:|---:|
| 80 | 0.597 / 0.236 | 0.611 / 0.242 | **0.598 / 0.229** |
| 82 | 0.604 / 0.273 | 0.603 / 0.269 | **0.609 / 0.239** |
| 83 | 0.697 / 0.324 | 0.673 / 0.311 | **0.615 / 0.246** |
| 84 | 0.848 / 0.387 | 0.687 / 0.325 | **0.622 / 0.257** |
| **85** | **3.244 / 1.480** | **8.416 / 3.552** | **0.631 / 0.277** |
| 90 | 9.457 / 3.583 | 9.668 / 3.675 | **3.951 / 1.804** |
| 100 | 7.690 / 2.670 | 7.592 / 2.641 | 7.562 / 2.730 |

全部 QP 成功率 100%。

### 85 km/h 是否刻意挑选？

不是单点挑出来的。80–82 三车都还受控；83–84 Proposed 逐步占优
（84: 0.622 vs MPC 0.848 / ZENG 0.687）；**85 是临界边界**：
MPC/ZENG 丢路径（3.24/8.42 m），Proposed 仍保持 0.631 m、SW 170°、
beta 1.82°、QP 100%。90 三车都丢路径，100 三车接近且都失败。

论文建议：

- **84 km/h 作为干净的主对比**：三车都还受控，Proposed
  `0.622/0.257`，MPC `0.848/0.387`，ZENG `0.687/0.325`；
  Proposed 相对 MPC `ey max -26.6%`、`ey RMS -33.6%`、`SW -73%`，
  且没有方向盘饱和。**不要把这组 84 km/h 数据标成 85 km/h**；
- **85 km/h 作为“可控包络边界”**：MPC/ZENG 已丢路径，Proposed 仍受控；
  同时给出 80–100 扫描，说明 85 不是任意挑的；
- 90/100 只作为失败极限参考，不作为跟踪精度主结果。

图/指标：`PMPC_Results/DLC_mu0.5_speed_compare/`：
`summary_ey.png`、`three_controllers_85.png`、
`three_controllers_90.png`、`three_controllers_100.png`、
`metrics_speed_sweep.csv`。


## 18. DLC mu=0.4 速度扫描（初速=目标速度）

CarSim DLC 路面从 0.5 改为 0.4（split-mu friction map 的
`#DiagramTwo0` 表和 `Run_all.par` 内联表都改为 0.4），并确认 `Vx` 从目标
车速开始。扫描 60/70/75/80 km/h：

| 初速=目标 | MPC ey max/RMS | ZENG ey max/RMS | Proposed ey max/RMS |
|---:|---:|---:|---:|
| 60 | 0.600 / 0.180 | 0.594 / 0.174 | **0.558 / 0.169** |
| **70** | **0.750 / 0.345** | **0.814 / 0.367** | **0.631 / 0.252** |
| 75 | 2.463 / 1.095 | 1.163 / 0.548 | **0.743 / 0.293** |
| 80 | 6.031 / 2.585 | **0.906 / 0.398** | 1.010 / 0.349 |

全部 QP 100%。

**推荐 70 km/h 作为 mu=0.4 主对比**：这是三车都还受控的最高速度，
Proposed 相对 MPC `ey max -15.9%`、`ey RMS -27.0%`、`SW -79.8%`、
`beta -23.8%`、`Md -10.6%`；相对 ZENG `ey max -22.5%`、`ey RMS -31.3%`。
75 时 MPC 已饱和（2.46 m、720°），只能作为边界工况；80 时 MPC 失稳
（6.03 m），ZENG 的 max 略好于 Proposed，但 Proposed 的 RMS/SW/beta 更好。
60 太容易，三车差距小，且 10 s 只跑了 160 m，未到完整路径终点。

图/指标：`PMPC_Results/DLC_mu0.4_speed_compare/`：
`summary_ey.png`、`three_controllers_70.png`、
`three_controllers_75.png`、`three_controllers_80.png`、
`metrics_speed_sweep.csv`。
