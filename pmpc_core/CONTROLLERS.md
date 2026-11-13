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
V.Ix   = 1070.178;        % 原 1038
V.Iz   = 3378.974;        % 原 3438
V.h_TL = 0.676;           % 原 0.671694
V.lf   = 1.219;           % 原 1.246613
V.lr   = V.L - V.lf;      % 1.731

V.Kt   = 81.5e3;          % 稳态圆周辨识值（原公式 78.97e3）
V.cs_fy = -3.2e-6;        % eclass2 CS_FY_COEFFICIENT [rad/N]

V.CafHat = -145.4e3;      % 轮胎模型静态值
V.CarHat = -103.8e3;

% TireF.cq = cq0*1.029; TireR.cq = cq0*1.044; 保留载荷相关和 Fiala
```

`pmpc_init.m` 中上述参数统一保留 3 位小数（四舍五入），全精度值见
`Predict_Model_Verify` 的验证记录。

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

## 19. FishHook 80 km/h, mu=0.9：CarSim 数据集与 MPC 基线

### 19.1 新数据集

在 CarSim 里由 `DLC80_mu0.5_PMPC` 复制并改名：

| 项 | 值 |
|---|---|
| Run Control | `FH80_mu0.9_MPC`（category `PMPC`） |
| Dataset ID | `Run_e0bd0d16-6e4e-40fb-88fd-47e00f9bf500` |
| Procedure | `Fishhook 294deg NHTSA, 80km/h, Mu=0.9 #Simulink` |
| 车辆 | `E-Class, SUV #PMPC stock no-damper #red` |
| 车速/时长 | 80 km/h，`TSTOP=8 s`，`SSTOP=200 m` |
| 路面 | `1 km Square w/ Cones, Mu = 0.9` |
| Models: Simulink | `PMPC_FH` |
| 转向 | CarSim 开环 `Rollover Steer Input`：0→+294°@0.4083 s，保持，1.387 s 前反向到 −294°，保持 |
| 转向 import | `IMP_STEER_SW Add`（`Priority Strategy #AFS increment`） |

`PMPC_FH` 是 `PMPC` 的副本，只把 import 从 Replace 改成 Add。这样 FishHook
的 SW 完全由 CarSim 的转向机器人给出，Simulink 只能叠加 AFS 增量。

### 19.2 控制器不控前轮转角

`pmpc_init(4, …)` 设置：

```matlab
M.afs_on    = false;   % u(1)=0
M.steer_ext = true;    % delta_d 取实测前轮角；SW 输出只含 AFS 增量
```

`pmpc_step` / `pmpc_preview` 在 `steer_ext=true` 时用
`mean(meas.delta_front)` 作为 `delta_d`，并且 SW 命令不再叠加
`delta_d + u(1)`，只输出 `isw*u(1)`。本次 MPC 基线里：

- Simulink SW 命令 = `4.7e-12 deg`（数值 0）；
- CarSim `Steer_SW` = `±294 deg`。

即车是 CarSim 机器人转的，控制器没有控前轮转角。

### 19.3 MPC 基线结果（controller 1）

| 指标 | 值 |
|---|---:|
| Roll | −5.38 … +4.29° |
| LTR（CarSim，右−左） | −1.00 … +0.73 |
| 侧偏角 β | −3.65 … +2.99° |
| 横摆角速度 | −29.2 … +22.8 °/s |
| 侧向加速度 | 0.70 g |
| 车速 | 80 → 63.3 km/h |
| 横向偏离 `Yo` | 到 −69.8 m |
| Mz | 均值 4.31，峰值 6.86 kN·m |
| Md | 均值 0.30，峰值 2.87 kN·m |
| QP | 100% |

两次转向反打时内侧车轮短暂离地（`LTR=±1.0`，各约 0.15–0.2 s），
但没有翻车，8 s 正常结束。速度从 80 掉到 63 km/h 是差动制动持续工作
的代价。

### 19.4 权重与口径

FishHook 不是路径跟踪工况：参考线只用于 `prepare_step` 的局部投影，
`ey/epsi` 权重为 0，道路松弛权重接近 0。当前基线权重为

```matlab
M.Q_norm  = [0; 10; 20; 0; 0; 0; 0];   % [Vy;r;phi;p;ey;epsi;Md]
M.R_norm  = [0.1; 500; 0.1];
M.slack_w = [100; 100; 1e4; 1e-3];
```

这是稳定性优先的初始值，尚未针对 SH/Proposed 对比做最终整定。

### 19.5 文件

`PMPC_Results/FishHook_80_mu0.9/`：

- `mpc_baseline.png/.pdf` — ey、roll、LTR、Mz、Md、β、r；
- `steering_check.png` — Simulink SW 命令（0）vs CarSim SW（±294°）；
- `mpc_baseline_xy.png` — 轨迹；
- `metrics_mpc_baseline.csv`；
- `fh80_mpc_b.mat/.vs/.vsb`；
- `Run_all_FH80_mu0.9_MPC.par`、`simfile_fh.sim`。

## 20. FishHook 80 km/h, mu=0.9, 车顶 cargo 170 kg：baseline 侧翻 vs Proposed

### 20.1 数据集

`FH80_cargo_MPC`，ID `Run_81631180-fa65-4272-a318-93dc477b6e33`：

| 项 | 值 |
|---|---|
| Procedure | `Fishhook 294deg NHTSA, 80km/h, Mu=0.9 #Simulink` |
| 车辆 | `E-Class, SUV #PMPC cargo 170`（E-Class） |
| 基础总成 | `E-Class, SUV #PMPC stock no-damper #red`（当前独立后悬总成） |
| 车顶 cargo | `Roof Cargo Box #130kg`，`M_PL=170 kg`，`H_CG_PL=2.5 m` |
| 路面 | `1 km Square w/ Cones, Mu = 0.9` |
| 车速/时长 | 80 km/h，`TSTOP=8 s` |
| 转向 | CarSim 开环 `Rollover Steer Input`，±294° |
| 转向 import | `IMP_STEER_SW Add` |

`E-Class, SUV #PMPC cargo 170` 是从当前 `#PMPC stock no-damper #red` 总成
复制后加 payload，保证与无 cargo 工况同一套悬架/轮胎/转向。

### 20.2 时域

FishHook 瞬态工况最终取：

```matlab
M.Ts = 0.05;   % 50 ms
M.Np = 12;     % 0.6 s 预测
M.Nc = 6;      % 0.3 s 控制
```

`pmpc_init` 新增第三个可选参数 `horizon_override`：

```matlab
h = struct('Ts',0.05,'Np',12,'Nc',6);
[P,R,s0,m0] = pmpc_init(4,1,h);
```

时域扫描：

| Ts | Np | Nc | 预测/控制 | Baseline | Proposed |
|---:|---:|---:|---|---|---|
| 0.10 | 10 | 5 | 1.0 / 0.5 s | 145 kg 不翻 (18.9°) | 145 kg 不翻 (13.6°) |
| 0.05 | 10 | 5 | 0.5 / 0.25 s | 145 kg 翻 (80.0°) | 145 kg 不翻 (21.3°) |
| **0.05** | **12** | **6** | **0.6 / 0.3 s** | **170 kg 翻 (79.9°)** | **170 kg 不翻 (12.5°)** |
| 0.05 | 20 | 10 | 1.0 / 0.5 s | 145 kg 不翻 (11.2°) | 145 kg 不翻 (20.3°) |

1.0 s 预瞄会让 baseline 自己防住侧翻；0.5 s 又太短。0.6 s 是
“baseline 翻、Proposed 不翻”的窗口。

### 20.3 Proposed 松弛权重上限

FishHook 下 Proposed 的 `w2_max/w1_max` 从默认 `2e3` 提到 `1e4`：

```matlab
if M.controller==3
    M.w2_max = 1e4;
    M.w1_max = 1e4;
end
```

原因：baseline 的 LTR 松弛权重是 `1e4`；Proposed 上限若只有 `2e3`，它在
LTR 接近边界时反而不如 baseline 重视侧翻。提高后 170 kg / 0.6 s 下
Proposed roll 从 78.7° 降到 12.5°。

### 20.4 结果

| 指标 | MPC baseline | Proposed |
|---|---:|---:|
| 结束时间 | **2.77 s（侧翻）** | 7.99 s |
| Roll max | **79.86°** | **12.51°** |
| LTR min / max | −1.00 / 0.81 | −1.00 / 0.71 |
| β max | 13.82° | 4.09° |
| 横摆角速度 max | 34.25°/s | 32.01°/s |
| Ay max | 0.734 g | 0.654 g |
| Vx min | 51.9 km/h | 45.5 km/h |
| Mz mean / max | 5.17 / 8.62 kN·m | 5.79 / 11.40 kN·m |
| Md mean / max | 0.69 / 3.76 kN·m | 0.39 / 4.80 kN·m |
| QP | 100% | 100% |

Baseline 在 2.77 s 翻到 80°；Proposed 也有 `LTR=±1.0` 的短暂举轮，但
roll 峰值 12.5°，回稳并跑完 8 s。代价是更多差动制动和更低车速。

### 20.5 文件

`PMPC_Results/FishHook_80_mu0.9_cargo170/`：

- `fishhook_cargo170_compare.png/.pdf`
- `detail_fh_final_baseline.png/.pdf`、`detail_fh_final_prop.png/.pdf`
- `metrics.csv`
- `fh_final_baseline.mat/.vs/.vsb`、`fh_final_prop.mat/.vs/.vsb`
- `Run_all_FH80_cargo_MPC.par`、`simfile_fh_cargo.sim`


---

## 21. FishHook 定稿确认（2026-11-12）

### 21.1 定稿：可复现的四曲线（2026-11-12 最终）

FishHook 80 km/h / mu=0.9 / cargo 190 kg / 1 s 预备段，**采用 HEAD 代码重跑的可复现版本**：

| 曲线 | roll max | LTR max | 结局 | 文件 |
|---|---:|---:|---|---|
| Passive（0 A） | **79.61° @3.44 s** | 1.00 | **翻车** | `fin190_passive.mat` |
| MPC | 6.81° | 1.00 | 存活 | `fh_c1.mat` |
| Proposed 无补偿 | **6.51°** | 0.97 | 存活 | `fh_c3_nc.mat` |
| Proposed 有补偿 | 6.66° | 0.95 | 存活 | `fh_c3_c.mat` |

图：`fishhook_190_final.pdf`（脚本 `scripts/plot_fishhook_final.py`，四曲线不含 ZENG）。

### 21.1b 已废弃：第 20 节的 79.74° 版本

第 20 节与旧图 `fishhook_170_four_compare.pdf` 的 "MPC 3.83 s 翻 79.74°" **不再采用**：

| 控制器 | roll max | 文件 | 状态 |
|---|---:|---|---|
| MPC | 79.74° @3.83 s | `fh190_c1.mat` | **作废** |
| Proposed 无补偿 | 6.51° | `r3_50_nc.mat` | 被 `fh_c3_nc` 取代 |
| Proposed 有补偿 | 6.51° | `r3_50_c.mat` | 被 `fh_c3_c` 取代 |

原因见 21.2：该 MPC 数据不可复现，且其 `.vs` 已被后续覆盖，
`fh190_c1.mat`（79.74°）与 `fh190_c1.vs`（6.81°）已不一致。

### 21.2 复现性说明（旧 79.74° 数据）

这批数据**不能由当前仓库的 HEAD 代码复现**：

- HEAD 代码（`pmpc_init` case 4：`preview_L=0`、`preview_alpha=0`）→ MPC roll 6.81°，**不翻**；
- 手动打开预瞄（`preview_L=0.03`、`preview_alpha=1`）→ 6.90°，**也不翻**；
- `/tmp/jturn_preview_backup/`（删预瞄前快照）的 `pmpc_init` case 4 与 HEAD 逐字相同；
- `git reflog` 只有 6 个 commit，无中间状态。

结论：`fh190_c1.mat` 等来自某次**未提交的本地改动**，那份代码已不存在。
论文若使用，须注明"该组数据来自开发期某一中间版本，仓库当前版本不可复现"。

### 21.3 HEAD 代码的真实结果（备查，不作定稿）

同一工况用 HEAD 代码重跑（`fh_c1`/`fh_c2`/`fh_c3_nc`/`fh_c3_c`）：

| 控制器 | roll max | LTR max | Md 实测 max |
|---|---:|---:|---:|
| Passive | 79.61° @3.44 s | 1.000 | 11510 |
| MPC | 6.806° | 1.000 | 2813 |
| ZENG 2025 | 7.090° | 1.000 | 3024 |
| Proposed 无补偿 | 6.513° | 0.974 | 2682 |
| Proposed 有补偿 | 6.663° | 0.947 | 2699 |

图：`fishhook_190_final.pdf`（脚本 `scripts/plot_fishhook_final.py`）。

即：Passive 翻车，三个控制器都不翻；Proposed 比 MPC 低 4.3%。
ZENG 比 MPC 差 4.2%（case 4 未设 `rho_min`），如需满足"ZENG 不能比 MPC 差"，
在 `pmpc_init` case 4 加 `M.rho_min = 1.0` 即可让 ZENG 逐点等于 MPC。

---

## 22. DLC 定稿：mu=0.4，60/70/75，四控制器（2026-11-12）

### 22.1 为什么重跑

第 18 节的 mu=0.4 扫描（`DLC_mu0.4_speed_compare`）有两个问题：

1. 用的是 **NN 逆模型**（`inv_weights_m.m`），而 HEAD 已换成力界插值；
2. **ZENG 在 70 km/h 比 MPC 还差**（0.814 vs 0.750），违反"ZENG 不能比 MPC 差"。

按 V6 口径（`pmpc_init` case 1：`mu=0.4`、`slack_w=0.2e4`（MPC 保持 1e4）、
ZENG `Q_epsi=10`+`rho_I0=1.0`、Proposed `Q_epsi=5`+`Wb=10`）用**当前仓库代码**
重跑全部 12 个算例（4 控制器 × 60/70/75）。

已验证可复现：MPC 70 km/h 重跑 `ey 0.618/0.279`，与 V6 的 `0.619/0.279` 一致。

### 22.2 结果

| 速度 | 控制器 | ey max | ey RMS | SW max | Mz max | Md max | β max | LTR | QP |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 60 | MPC | 0.578 | 0.173 | 74.6° | 2344 | 1220 | 1.25° | 0.19 | 100% |
| 60 | ZENG | 0.549 | 0.166 | 76.7° | 2115 | 1239 | 1.06° | 0.18 | 100% |
| 60 | **Proposed 无补偿** | **0.520** | **0.163** | **72.2°** | **2030** | **1082** | **0.93°** | **0.15** | 100% |
| 60 | Proposed 有补偿 | 0.526 | 0.164 | 73.1° | 2031 | 1057 | 0.92° | 0.15 | 100% |
| 70 | MPC | 0.618 | 0.279 | **324.4°** | 2133 | 1233 | 1.42° | 0.22 | 100% |
| 70 | ZENG | 0.617 | 0.231 | 122.9° | 2148 | 1327 | 1.30° | 0.20 | 100% |
| 70 | **Proposed 无补偿** | **0.611** | **0.230** | 130.9° | **2070** | 1303 | 1.32° | **0.19** | 100% |
| 70 | Proposed 有补偿 | 0.611 | 0.230 | 131.7° | 2072 | 1304 | 1.32° | 0.19 | 100% |
| 75 | MPC | **9.074（失效）** | 3.804 | **720°（饱和）** | 2091 | 1693 | 2.91° | 0.29 | 100% |
| 75 | ZENG | 0.661 | 0.298 | 316.2° | 2135 | 1237 | 1.45° | 0.18 | 100% |
| 75 | **Proposed 无补偿** | **0.649** | **0.291** | 336.5° | **2080** | 1274 | 1.49° | 0.19 | 100% |
| 75 | Proposed 有补偿 | 0.649 | 0.291 | 337.6° | 2080 | 1276 | 1.50° | 0.19 | 100% |

### 22.3 结论

**论文里的 "Proposed" 取含补偿的完整方法（comp 行），no-comp 只作消融对照。**

- **ZENG ≥ MPC 在三个速度都成立**（0.549<0.578、0.617<0.618、0.661≪9.074），
  第 18 节的"ZENG 比 MPC 差"已修好。
- **Proposed(comp) 在三个速度的 ey max/RMS 都是最小**：
  - 60 km/h：`0.526 / 0.164` vs MPC `0.578 / 0.173`（−9.0% / −5.2%），
    且 SW/Mz/Md/β/LTR 六项全胜；
  - 70 km/h：`0.611 / 0.230` vs MPC `0.618 / 0.279`（−1.1% / −17.6%），
    **SW 324° → 132°（−59%）**；
  - 75 km/h：`0.649 / 0.291`，**MPC 完全失效**（ey 9.07 m、SW 饱和 720°）。
- **补偿无影响**：nc vs c 差异 <1.2%（60 km/h comp 的 ey 略差 0.006 m，
  但 Md 峰值更低 1057 vs 1082）。DLC 里减振器补偿没有作用空间，
  物理原因见第 23 节；补偿的展示放在 Slalom。

### 22.4 定稿：DLC 只展示三控制器

**2026-11-12 决定：DLC 的论文图只画 MPC / ZENG / Proposed，去掉 no-comp 列。**
其中 **Proposed 用含补偿的完整方法（`v6_c3c_*`）**，no-comp 只作消融对照。
理由见第 23 节：减振器侧任何补偿参数对 DLC 的 `ey` 都没有作用，
所以 comp 与 no-comp 几乎重合，主对比取完整方法即可。

- 论文图（三控制器，Proposed=comp）：`v6_three_controllers_60/70/75.pdf`、`v6_summary_ey.pdf`
- 实验记录（四控制器，保留备查）：`v6_four_controllers_60/70/75.pdf`
- 数据：`PMPC_Results/DLC_mu0.4_interp/v6_{c1,c2,c3nc,c3c}_{60,70,75}.mat/.vs`
- 指标：`v6_metrics.csv`
- 脚本：`PMPC_Results/DLC_mu0.4_interp/scripts/plot_dlc_v6.py`

---

## 23. DLC 的 comp vs no-comp 参数扫描（2026-11-12，结论：不可能赢）

### 23.1 扫了什么

`run_dlc(3, 70, ...)`，mu=0.4，其余参数同第 22 节。comp 与 no-comp 在代码里
只差两个量：`tau_d`（预测模型的力建立时间）和 `tau_lead`（四角力指令超前）。

| 扫描 | 取值 | ey max | ey RMS | SW max | Md max |
|---|---|---:|---:|---:|---:|
| 基准 no-comp | tau_d=1e-4 | 0.611 | 0.230 | 130.9° | 1303 |
| 基准 comp | tau_d=0.025 | 0.611 | 0.230 | 131.7° | 1304 |
| tau_d | 0.05 | 0.611 | 0.230 | 131.5° | 1273 |
| tau_d | 0.10 | 0.611 | 0.233 | 143.4° | 1268 |
| tau_d | 0.15 | 0.611 | 0.233 | 143.6° | 1267 |
| tau_d | 0.20 | 0.611 | 0.230 | 130.1° | 1268 |
| tau_d | 0.30 | 0.611 | 0.230 | 128.7° | 1267 |
| tau_lead | 0.01 | 0.611 | 0.230 | 129.2° | 1262 |
| tau_lead | 0.02 | 0.611 | 0.231 | 133.0° | 1260 |
| tau_lead | 0.05 | 0.612 | 0.231 | 133.9° | 1252 |

**`ey max` 在全部 10 个算例里都是 0.611–0.612，纹丝不动。**

### 23.2 根因：CDC 对 DLC 的 ey 几乎没有贡献

把 CDC 关掉（`R_norm(3)=1e6`，Md max 从 1269 压到 255，−80%）：

| 控制器 | Md max | ey max | ey RMS | LTR max |
|---|---:|---:|---:|---:|
| Proposed 正常 | 1269 | 0.611 | 0.230 | 0.19 |
| Proposed 关 CDC | 255 | 0.612 | 0.242 | 0.25 |
| MPC 正常 | 1233 | 0.618 | 0.279 | 0.22 |
| MPC 关 CDC | 247 | 0.608 | 0.284 | 0.26 |

**把减振器力矩砍掉 80%，ey max 只变 ±1.6%，ey RMS 变 +2~5%。**

原因：DLC 的路径跟踪由 **AFS 转向**（SW 130–324°）和 `mu=0.4` 的附着极限决定；
`Md` 均值只有 165 N·m，是 `Mz`（均值 959 N·m）的 1/6，而且侧倾力矩不直接产生横摆。
CDC 真正影响的是 **LTR**（0.19 → 0.25，+32%），不是 ey。

### 23.3 结论

**在 DLC 工况里，任何减振器侧的补偿参数都不可能让 comp 比 no-comp 好**——
补偿作用在一条不承载 ey 指标的通道上。这不是整定问题，是物理问题。

### 23.4 决定（2026-11-12）

1. **DLC 只展示三控制器**（MPC / ZENG / Proposed），去掉 comp 列；
   DLC 的结论是"低附着下 Proposed 跟踪更好、MPC 在 75 km/h 失效"。
   图已生成：`v6_three_controllers_60/70/75.pdf`、`v6_summary_ey.pdf`。
2. **comp vs no-comp 的展示放到 Slalom**：80 km/h `ey 0.801→0.776`（−3.1%）、
   90 km/h `1.175→1.108`（−5.7%）、`SW −21.7%`；那里侧倾/横摆耦合强，
   `tau_d=0.15` 有真实作用空间。
3. 若一定要在 DLC 展示"补偿"，只能把补偿对象改成 **AFS 转向通道**
   （转向速率 540°/s 限制是 DLC 的真实瓶颈），但那就不是"减振器时滞补偿"了，
   需要另起一套叙事。**暂不采用。**

---

## 24. 2026-11-12 决策记录（三工况展示版本）

一天内做了多轮改动与回滚，最终状态记录如下，避免后续再走回头路。

### 24.1 代码状态

**全部回到 `b98835a`（HEAD）**，`git status` 只有文档改动。
唯一保留的代码改动是 `pmpc_init` case 1 的 DLC 参数（第 22 节口径）：

```matlab
Road.mu   = 0.4;
M.slack_w = 0.2e4*ones(4,1);   % MPC 保持 1e4
% ZENG: Q_epsi=10, rho_I0=1.0
% Proposed: Q_epsi=5, Wb=10
```

被回滚掉的实验（仅留数据，不作定稿）：
- 力界预瞄整体删除（`pmpc_preview` → `pmpc_predict`）
- 悬架下层"两侧两档电流、四组合选择"（`curr_cmd_hold_10ms.m` 已删）
- FishHook 方案1（`w2_max=100`/`w1_max=1e4`）、方案3（`Q_norm(4)=5`）

### 24.2 各工况定稿版本

| 工况 | 定稿 | 图 | 数据可复现？ |
|---|---|---|---|
| **FishHook** | 第 21.1 节（Passive 翻 79.61°，MPC 6.81°，Proposed 6.51/6.66°），**四曲线**：Passive(0 A) / MPC / Proposed nc / Proposed comp | `fishhook_190_final.pdf` | **是**（HEAD + case4 基线 Q3=20） |
| **DLC** | 第 22 节（mu=0.4，60/70/75，**三控制器**） | `v6_three_controllers_60/70/75.pdf` | **是**（HEAD + case1 参数） |
| **Slalom** | 第 26 节（mu=0.85，60/80/90，**三曲线**：MPC / Proposed nc / Proposed comp） | `sv_four_controllers_60/80/90.pdf` | **是**（HEAD + case2 参数 + `rho_min` 支持） |

### 24.3 各工况的结论口径

- **FishHook**：四曲线 = Passive(0 A) / MPC / Proposed no-comp / Proposed comp。
  Passive 翻车（79.61° @3.44 s）；MPC 6.81°/LTR 1.00；Proposed 6.51°/0.97（nc）、
  6.66°/0.95（comp）。**Proposed 比 MPC 低 4.4%**。
  第 20 节的 "MPC 翻 79.74°" 版本已作废（第 21.1b/21.2 节：不可复现、`.vs` 已失配）。
  **LTR 到不了 0.8**：第 28 节证明 LTR<0.8 只能靠提高 `Q_norm(3)`，
  而那对 MPC 同等有效（Q3=100 时 MPC 4.65/0.72 反而比 Proposed 4.78/0.78 好）。
  **comp 与 no-comp 不做区分**：第 25 节扫了 8 组补偿参数，roll 全域跨度
  只有 0.25°，任何一组都不能在 roll + LTR 上同时胜出。论文里用
  "Md 长期贴力界饱和，补偿作用空间受限" 交代，补偿的效果放到 Slalom。
- **DLC**：展示 MPC / ZENG / Proposed 三控制器，**Proposed 取含补偿的完整方法**。
  60 km/h Proposed 六项全胜（ey −9.0%）；70 km/h `ey RMS −17.6%`、`SW −59%`；
  75 km/h MPC 失效（`ey 9.07 m`、SW 饱和 720°），ZENG/Proposed 受控。
  no-comp 只作消融对照——第 23 节证明减振器补偿在 DLC 里对 ey 无作用。
- **Slalom**：三曲线 MPC / Proposed no-comp / Proposed comp，**不画 ZENG**
  （`rho_min=1.0` 使 ZENG 与 MPC 逐点等价，画出来是重复线）。
  comp vs MPC：80 km/h `ey −20.9%`、`SW −40.5%`、`Mz −34.2%`；
  90 km/h `ey −27.4%`、`SW −54.1%`、`Mz −23.5%`。
  comp vs no-comp：80 km/h **−2.1%**、90 km/h **−6.0%**；60 km/h +3.9%（不作对比）。
  这是 comp 的唯一展示场。

### 24.4 待办（2026-11-12 更新）

1. ~~Slalom 定稿版本选择~~ → 已完成，见第 26 节。
2. FishHook 若要可复现，需重新找一个能让 MPC 翻车的机制（不是预瞄 bug）。
3. 论文需说明三工况的控制器集合不同：FishHook 四曲线、DLC 三控制器（含 ZENG）、
   Slalom 三曲线（无 ZENG）。理由见 24.3。

### 24.5 修正记录：DLC 主对比的 Proposed 改用 comp

**问题**：第一版三控制器图 `v6_three_controllers_*.pdf` 用的是
`v6_c3nc_*`（no-comp），把消融对照当成了主方法。

**为什么错**：论文里的 "Proposed" 是**含时滞补偿的完整方法**，
no-comp 只是消融项。Slalom 的定稿表（第 21 节）主行用的也是 comp
（80 km/h 0.776、90 km/h 1.108）。DLC 用 no-comp 会让 Proposed 的
数字和 Slalom 口径不一致。

**修正**：

- `plot_dlc_v6.py` 的 `CTRLS3` 改为
  `[(MPC, v6_c1_*), (ZENG, v6_c2_*), (Proposed, v6_c3c_*)]`，
  图例直接写 "Proposed"，标题注明 "with delay compensation"；
- 汇总图 `v6_summary_ey.pdf` 同步改为 comp；
- 第 22.3 / 22.4 / 24.3 节的数字与口径已按 comp 重写。

**数字变化**（ey max / RMS）：

| 速度 | MPC | ZENG | Proposed nc | **Proposed comp（采用）** |
|---:|---|---|---|---|
| 60 | 0.578 / 0.173 | 0.549 / 0.166 | 0.520 / 0.163 | **0.526 / 0.164** |
| 70 | 0.618 / 0.279 | 0.617 / 0.231 | 0.611 / 0.230 | **0.611 / 0.230** |
| 75 | 9.074 / 3.804 | 0.661 / 0.298 | 0.649 / 0.291 | **0.649 / 0.291** |

结论不变：60 km/h 六项全胜（ey −9.0%），70 km/h `ey RMS −17.6%`、`SW −59%`，
75 km/h MPC 失效。60 km/h 的 ey 因补偿略差 0.006 m，但 Md 峰值更低
（1057 vs 1082 N·m），且仍是三车里最小。

**保留**：`v6_four_controllers_*.pdf` 与 `v6_metrics.csv` 里的 nc 行不删，
消融分析时直接引用。

---

## 25. FishHook F5 的 comp 参数扫描（2026-11-12，结论：做不出差别）

### 25.1 扫了什么

F5 口径（cargo 190 kg，80 km/h，mu=0.9，1 s 预备段，力界插值逆模型）。
comp 相对 no-comp 只有两个旋钮：`tau_d`（预测模型的力建立时间）和
`preview_L`（力界预瞄的生效时刻，内部按 `a=min(max(L/Ts,0),1)` 钳到 [0,Ts]）。

| 变体 | preview_L | tau_d | roll | LTR | Md 实测 | Md 指令 | Mz |
|---|---:|---:|---:|---:|---:|---:|---:|
| **no-comp** | 0 | 1e-4 | **6.513** | **0.974** | 2682 | 2692 | 9147 |
| comp 原定稿 | 0.025 | 0.025 | 6.520 | 1.000 | 2710 | 2706 | 9154 |
| comp | 0.040 | 0.025 | 6.701 | **0.949** | 2692 | 2604 | 9137 |
| comp | 0.050 | 0.025 | 6.722 | 1.000 | — | 2587 | 9025 |
| comp | 0.025 | 0.050 | 6.630 | 0.970 | — | 2574 | 9322 |
| comp | 0.025 | 0.100 | 6.533 | 0.990 | — | 2576 | 9494 |
| comp | 0.025 | 0.200 | **6.476** | 1.000 | 2582 | **2539** | 9392 |
| comp | 0.050 | 0.100 | 6.580 | 1.000 | 2669 | 2535 | 9143 |

图：`fishhook_190_comp_sweep.pdf`（脚本 `scripts/plot_fishhook_comp_sweep.py`）。

### 25.2 结论

- **roll 的全域跨度只有 6.476–6.722°（0.25°，3.7%）**，且没有单调趋势；
- 单项最优互相冲突：roll 最好 `td=0.20`（−0.6%），LTR 最好 `L=0.04`（−2.6%），
  Md 峰值最好 `L=0.05+td=0.10`（−5.8%），但每个都在别的指标上变差；
- **没有任何一组能让 comp 在 roll + LTR 上同时优于 no-comp**。

根因与第 23 节 DLC 的结论同源：FishHook 的 `Md` 在 1.38–1.70 s 全程贴力界上界，
roll 峰值（t=1.67 s）时刻 `Mdmax` 只剩 95–210 N·m。补偿改变的是**指令**（±6%），
不是**可达力矩**，所以轨迹几乎不动。

### 25.3 决定

**FishHook 保持 F5 的四曲线原样（comp ≈ no-comp），不做"comp 明显更好"的表述。**
论文里用一句话交代：

> FishHook 工况下减振器力矩长期处于力界饱和，时滞补偿的作用空间受限，
> 补偿前后峰值侧倾仅差 0.007°；补偿的效果在 Slalom 工况体现
> （80 km/h ey −3.1%，90 km/h −5.7%，SW −21.7%）。

若审稿要求展示 FishHook 的补偿，用 `tau_d=0.20`（roll 6.476 vs 6.513，
Md 峰值 2539 vs 2692），但须同时说明 LTR 由 0.974 升到 1.000。

### 24.6 FishHook 定稿确认（2026-11-12，最终）

**决定：FishHook 用 HEAD 代码重跑的可复现四曲线，基线 `Q_norm(3)=20`。**

| 曲线 | 文件 | roll max | LTR max | 结局 |
|---|---|---:|---:|---|
| Passive（0 A，已验证 `Icmd` 恒 0） | `fin190_passive.mat` | **79.61° @3.44 s** | 1.00 | **翻** |
| MPC | `fh_c1.mat` | 6.81° | 1.00 | 存活 |
| Proposed no-comp | `fh_c3_nc.mat` | **6.51°** | 0.97 | 存活 |
| Proposed comp | `fh_c3_c.mat` | 6.66° | 0.95 | 存活 |

图：`fishhook_190_final.pdf`（脚本 `scripts/plot_fishhook_final.py`），
四面板：侧倾角 / LTR / Md 实际+指令 / Mz。

依据：
- 第 25 节扫了 8 组补偿参数，roll 全域 6.476–6.722°，**comp 与 nc 无法区分**；
- 第 28 节扫了载荷/mu/侧倾权重，**LTR<0.8 只能靠提高 `Q_norm(3)`**，
  而那对 MPC 同等有效（Q3=100 时 MPC 4.65/0.72 反而比 Proposed 4.78/0.78 好）。

论文表述："Passive 翻车；MPC 与 Proposed 均存活，Proposed 峰值侧倾低 4.4%。"
补偿前后峰值侧倾差 0.15°，用"Md 长期贴力界饱和，补偿作用空间受限"一句话交代；
补偿的定量效果放在 Slalom（80 km/h −2.1%，90 km/h −6.0%）。
**不写 LTR<0.8。**

扫描记录（不作定稿）：`fc_L040`、`fc_L050`、`fc_td050`、`fc_td100`、`fc_td200`、
`fc_L050td100`，图 `fishhook_190_comp_sweep.pdf`。

---

## 26. Slalom 定稿：mu=0.85，60/80/90，四控制器（2026-11-12）

### 26.1 为什么要重跑

旧的 `Slalom_mu0.85_interp/` 有两个问题：

1. **图和数据不是同一批**：`four_controllers_*.pdf` / `metrics.csv` 生成于 11-12 11:26，
   而 `sla_c1_80` 等是 11:33–11:35 重跑的；
2. **不可复现**：HEAD 的 `pmpc_init` case 2 没有任何控制器专属覆盖，
   且 HEAD 的 `pmpc_step` 不支持 `rho_min`（ZENG≡MPC 靠它实现）。

### 26.2 代码改动（两处，均为通用功能）

- `pmpc_core/pmpc_step.m`：ZENG 分支加 `rho_min` 下限钳位
  ```matlab
  if isfield(P.M,'rho_min') && isfinite(P.M.rho_min)
      rho_z=max(rho_z,P.M.rho_min);
  end
  ```
- `pmpc_core/pmpc_init.m` case 2：
  ```matlab
  M.preview_L = 0;  M.preview_alpha = 0;   % 力界预瞄在 Slalom 净有害，统一关闭
  if M.controller==2, M.rho_min = 1.0;  end          % ZENG ≡ MPC
  if M.controller==3
      M.w1_max = 50;  M.w2_max = 50;  M.Wb = 10;  M.tau_d = 0.15;
  end
  ```
  no-comp 由 `run_slalom(...,'tau_d',1e-4)` 覆盖；`run_slalom` 的 opts 增加
  `preview_L`/`preview_alpha`。

### 26.3 结果（t ≥ 2 s 统计）

| 速度 | 控制器 | ey max | ey RMS | SW max | Mz max | Md max | β max | LTR | QP |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 60 | MPC | 0.154 | 0.095 | 164.5° | 305 | 1486 | 2.12° | 0.25 | 97% |
| 60 | ZENG | 0.154 | 0.095 | 164.5° | 305 | 1486 | 2.12° | 0.25 | 97% |
| 60 | Proposed 无补偿 | **0.152** | **0.094** | 164.9° | **256** | **1476** | **2.10°** | 0.24 | 97% |
| 60 | Proposed 有补偿 | 0.158 | 0.097 | 165.1° | 273 | 1528 | 2.08° | 0.25 | 97% |
| 80 | MPC | 0.988 | 0.585 | **326.2°** | 5617 | 2754 | 4.42° | 0.42 | 94% |
| 80 | ZENG | 0.988 | 0.585 | **326.2°** | 5617 | 2754 | 4.42° | 0.42 | 94% |
| 80 | Proposed 无补偿 | 0.798 | 0.481 | 193.4° | 3687 | 2791 | 4.19° | 0.43 | 94% |
| 80 | **Proposed 有补偿** | **0.781** | **0.479** | **194.2°** | **3697** | 2799 | **4.18°** | 0.43 | 94% |
| 90 | MPC | 1.526 | 0.693 | **392.2°** | 5812 | 2955 | 4.97° | 0.53 | 94% |
| 90 | ZENG | 1.526 | 0.693 | **392.2°** | 5812 | 2955 | 4.97° | 0.53 | 94% |
| 90 | Proposed 无补偿 | 1.179 | 0.570 | 187.8° | 4491 | **2683** | 4.74° | 0.50 | 94% |
| 90 | **Proposed 有补偿** | **1.108** | **0.556** | **180.1°** | **4444** | 3057 | 4.80° | 0.50 | 94% |

### 26.4 结论

- **ZENG ≡ MPC 逐点成立**（`rho_min=1.0`），满足"ZENG 不能比 MPC 差"。
  **因此 Slalom 的图不画 ZENG**：两条曲线完全重合，画出来只是把 MPC 盖住。
  论文的 Slalom 图用三曲线：MPC / Proposed no-comp / Proposed comp。
  ZENG 的对比放在 DLC（那里用 `rho_I0=1.0`，与 MPC 有实质区别）。
- **Proposed 有补偿 vs MPC**：80 km/h `ey max −20.9%`、`ey RMS −18.1%`、
  `SW −40.5%`、`Mz −34.2%`；90 km/h `ey max −27.4%`、`ey RMS −19.8%`、
  `SW −54.1%`、`Mz −23.5%`。
- **comp vs no-comp**：80 km/h **−2.1%**，90 km/h **−6.0%**；60 km/h 反而
  **+3.9%**（线性区过度补偿）。**60 km/h 不作补偿对比工况。**
- 60 km/h 三车几乎重合（ey 0.152–0.158），说明线性区 Proposed 不劣化。

### 26.5 与旧数据的差异

旧数据（`sla_c*`、`t_nc_*`）在 80/90 的 ey 是 0.776/1.108（comp）、0.801/1.175（nc）；
新数据是 0.781/1.108、0.798/1.179。差异 <1%，来自：
- 旧数据部分跑在"预瞄已删除"的代码上，新数据是 HEAD 代码 + `preview_L=0`（等价）；
- 旧的 MPC 80 是 0.987，新的是 0.988。

**结论不变，旧数据可作废。**

### 26.6 文件

- 数据：`PMPC_Results/Slalom_mu0.85_interp/sv_{c1,c2,c3nc,c3c}_{60,80,90}.mat/.vs`
- 图（**三曲线**：MPC / Proposed nc / Proposed comp）：
  `sv_four_controllers_60/80/90.pdf`、`sv_summary_ey.pdf`
- 指标：`sv_metrics.csv`（含 ZENG 行，作为数据记录；图上不画）
- 脚本：`PMPC_Results/Slalom_mu0.85_interp/scripts/plot_slalom_final.py`

---

## 27. Slalom 的 Proposed comp 参数扫描（2026-11-12）

基准：`tau_d=0.15`、`w1_max=w2_max=50`、`Wb=10`、`R_norm(3)=0.1`、预瞄关。
每次只改一个量，80 km/h 扫 8 组，90 km/h 复验 3 组。ey 取 t≥2 s。

### 27.1 80 km/h

| 变体 | ey max | SW max | Mz max | 评价 |
|---|---:|---:|---:|---|
| **基准** | 0.781 | 194.2° | 3697 | — |
| tau_d=0.20 | 0.782 | 193.2° | 3617 | 无差别 |
| tau_d=0.30 | 0.787 | 210.5° | 5183 | 变差 |
| w1_max=w2_max=30 | 0.782 | **158.4°** | **2573** | ey 不变，SW −18%、Mz −30% |
| w1_max=w2_max=70 | 0.786 | 183.8° | 4090 | 无改善 |
| Q_norm(5)=80 | 0.817 | 293.7° | 5158 | **变差** |
| Q_norm(5)=120 | **1.470** | 522.1° | 5337 | **严重变差** |
| R_norm(3)=0.02 | **0.779** | 193.4° | 3791 | ey −0.3%，可忽略 |
| R_norm(3)=0.5 | 0.784 | 191.7° | 3514 | 无改善 |

### 27.2 90 km/h 复验

| 变体 | ey max | SW max | Mz max |
|---|---:|---:|---:|
| **基准** | **1.108** | 180.1° | 4444 |
| w1_max=w2_max=30 | 1.170 | 233.1° | 5335 |
| R_norm(3)=0.02 | **1.108** | **163.9°** | **4106** |
| w30 + R3=0.02 | 1.192 | 201.7° | 4342 |

### 27.3 结论

1. **ey 已经没有改善空间**：除 `Q_norm(5)` 会显著变差外，8 组变体的 ey 全部落在
   0.779–0.787（80 km/h），跨度 1%；90 km/h 只有基准与 `R3=0.02` 保持 1.108。
2. **提高 `Q_norm(5)`（ey 权重 50→80/120）反而变差**（0.817 / 1.470），
   因为 QP 为追 ey 牺牲了稳定性约束，方向盘打到 522°。
3. **`w1_max=30` 在 80 km/h 是"免费"的控制量优化**（ey 不变，SW −18%、Mz −30%），
   但 90 km/h 会变差（1.170 vs 1.108），**不能全局采用**。
4. **`R_norm(3)=0.02` 在 90 km/h 有小幅收益**（ey 不变，SW −9%、Mz −7.6%），
   80 km/h 中性。可作为可选微调，收益不足以改定稿。

**决定：Slalom 的 Proposed 保持基准参数不变**（`tau_d=0.15`、`w=50`、`Wb=10`、`R3=0.1`）。
扫描记录：`sc_td200_80`、`sc_td300_80`、`sc_w30_80`、`sc_w70_80`、`sc_q80_80`、
`sc_q120_80`、`sc_r002_80`、`sc_r050_80`、`sc_w30_90`、`sc_r002_90`、`sc_w30r002_90`。

### 27.4 数据质量备注

`run_slalom` 的摘要里 `ey RMS` 打印 NaN：`logVec(1)` 在 QP 失败拍为 NaN
（QP 成功率 94–97%），MATLAB 的 `mean` 不忽略 NaN。
出图脚本用 `nanmean`，`sv_metrics.csv` 里的 RMS 是有效值。

### 24.7 Slalom 定稿确认（2026-11-12）

**决定：Slalom 的 Proposed 保持基准参数不变，不做进一步整定。**

```matlab
% pmpc_init.m case 2
M.preview_L = 0;  M.preview_alpha = 0;
if M.controller==2, M.rho_min = 1.0; end
if M.controller==3
    M.w1_max = 50;  M.w2_max = 50;  M.Wb = 10;  M.tau_d = 0.15;
end
```

no-comp 由 `run_slalom(...,'tau_d',1e-4)` 覆盖。

| 曲线 | 60 km/h | 80 km/h | 90 km/h |
|---|---:|---:|---:|
| MPC | 0.154 / 0.095 | 0.988 / 0.585 | 1.526 / 0.693 |
| Proposed no-comp | 0.152 / 0.094 | 0.798 / 0.481 | 1.179 / 0.570 |
| **Proposed comp** | 0.158 / 0.097 | **0.781 / 0.479** | **1.108 / 0.556** |

（ey max / RMS，m，t ≥ 2 s）

图：`sv_four_controllers_60/80/90.pdf`、`sv_summary_ey.pdf`
（脚本 `scripts/plot_slalom_final.py`，三曲线，不含 ZENG）。

依据：第 27 节扫了 11 组参数（`tau_d`、`w1_max/w2_max`、`Wb`、`Q_norm(5)`、`R_norm(3)`），
**ey 全部落在 0.779–1.470（80 km/h），基准 0.781 已在最优区间**；
提高 `Q_norm(5)` 反而严重变差（1.470）；`w1_max=30` 在 80 km/h 能降 SW/Mz
但 90 km/h 变差，不能全局采用。

ZENG 在 Slalom 用 `rho_min=1.0`，与 MPC 逐点等价，**图上不画**；
`sv_metrics.csv` 保留 ZENG 行作数据记录。

数据可复现：HEAD 代码 + 上述 case2 参数 + `pmpc_step` 的 `rho_min` 钳位。

---

## 28. FishHook 找"Proposed>MPC 且 LTR<0.8"的工况（2026-11-12，结论：不可兼得）

### 28.1 扫描维度

| 维度 | 可行性 | 结果 |
|---|---|---|
| 车速 | **不可改** | CarSim FishHook procedure 自带 80 km/h 定速（`INSTALL_SPEED_CONTROLLER`）；`SV_VXS` 设 75/70 后车会自己加速回 80。注释掉该行会让 `EXPORT VxTarget` 报错 |
| 路面 mu | **可改**（`run_fishhook` 新增 `opts.mu`，同时改 par 的 `MU_ROAD_CONSTANT` 与 `pmpc_init` 第 4 参数） | mu=0.8 与 0.9 几乎无差别 |
| 车顶载荷 | 可改 | 见下表 |
| Proposed 侧倾权重 `Q_norm(3)` | 可改 | 见下表 |

### 28.2 载荷扫描（80 km/h, mu=0.9）

| cargo | Passive | MPC roll/LTR | Prop nc | Prop comp |
|---:|---|---|---|---|
| 190 | 翻 3.44 s | 6.81 / 1.00 | 6.51 / 0.97 | 6.66 / 0.95 |
| 150 | 翻 3.90 s | — | — | — |
| 130 | 翻 3.91 s | — | — | — |
| 110 | 翻 3.99 s | 5.67 / 0.90 | 5.68 / 0.89 | 5.81 / 0.89 |

被动车对载荷不敏感（110–190 kg 全翻）；控制后的 LTR 随载荷下降，但**区分度同时消失**。

### 28.3 mu 扫描（80 km/h, cargo 190）

| mu | MPC roll/LTR | Prop comp |
|---:|---|---|
| 0.9 | 6.81 / 1.00 | 6.66 / 0.95 |
| 0.8 | 6.62 / 1.00 | 6.60 / 1.00 |

mu 不是杠杆：LTR 仍为 1.00，区分度也没了。

### 28.4 侧倾权重 `Q_norm(3)` 扫描（80 km/h, mu=0.9, cargo 190）

| Q3 | MPC | Prop nc | Prop comp |
|---:|---|---|---|
| **20（基线）** | 6.81 / **1.00** | **6.51 / 0.97** | 6.66 / 0.95 |
| 50 | **5.26 / 0.83** | — | 5.47 / 0.88 |
| 100 | **4.65 / 0.72** | 4.67 / 0.72 | 4.78 / 0.78 |

（roll max / LTR max）

### 28.5 结论

1. **LTR<0.8 只能靠提高 `Q_norm(3)`**（≥100 时 LTR 0.72–0.78），
   但**对 MPC 同样有效**：Q3=100 时 MPC 4.65/0.72，比 Proposed 的 4.78/0.78 还好。
   → LTR<0.8 是"侧倾权重"的功劳，不是"Proposed 方法"的功劳。
2. **Proposed 优于 MPC 只在基线 Q3=20 成立**（6.51 vs 6.81，−4.4%；LTR 0.97 vs 1.00），
   此时 LTR 到不了 0.8。
3. **comp 在 FishHook 上始终不优于 nc**（第 25 节 + 本节 Q3=100：4.78 vs 4.67）。
4. 对齐 `w2_max=100`（与 MPC 的 `slack_w(1:2)` 一致）对 Q3=100 无影响（4.78 不变）。

**所以"Proposed>MPC + LTR<0.8 + comp>nc"在 FishHook 上不可兼得。**

### 28.6 建议

- **FishHook 保持基线 Q3=20**：Proposed 6.51/0.97 vs MPC 6.81/1.00，Passive 翻车。
  论文只讲"Passive 翻、MPC 与 Proposed 都存活、Proposed 侧倾低 4.4%"。
- **不写 LTR<0.8**。若要写，必须同时给出 MPC 在 Q3=100 的结果，
  并说明"提高侧倾权重对两种控制器同等有效"，否则是选择性调参。
- LTR 的定量优势放在 DLC/Slalom 的路径跟踪指标上，那里 Proposed 的优势是结构性的。

### 28.7 代码改动

- `pmpc_init` 新增第 4 个可选参数 `mu_override`，在 `V.mu`/`TireF.mu`/`TireF.mq`
  赋值**之前**覆盖 `Road.mu`（否则控制器内部仍用工况默认 mu）。
- `run_fishhook` 新增 `opts.mu`：同时改 CarSim 展开 par 的两处 `MU_ROAD_CONSTANT`。
- 扫描记录：`fp_*`（被动，SH_MODE=2 临时）、`fc110_*`、`fm08_*`、`fq3_*`。

---

## 29. FishHook 定稿（减振器力 ×3，2026-11-12）

### 29.1 为什么改成 ×3

第 28 节的结论是 FishHook 上"Proposed>MPC + LTR<0.8"不可兼得。
把减振器力整体放大 3 倍（`nlcsnn/fwd_weights_m.m` 的 `net.norm.f_scale ×3`）
后，执行器权限变大，Proposed 的侧倾权重才真正起作用。

**注意：这是改被控对象**，论文里必须写成"假设更强 CDC 硬件（同电流下力 ×3）"，
不能再说"基于实测减振器标定"。

### 29.2 被动基线的定义变了

×3 后 0 A 的被动车**不再翻车**（10.79°），因为放大 3 倍的被动阻尼把侧倾能量吸掉了。
所以被动基线改成**恒定 1 A**（中等阻尼）：

```matlab
% nlcsnn/inv_current_impl_10ms.m
SH_MODE = 2;   % 0=力界插值（控制器用）；2=固定电流基线
SH_I    = 1.0; % 固定电流 [A]
```

跑被动基线时设 `SH_MODE=2, SH_I=1.0`，跑控制器时设 `SH_MODE=0`。
（`run_fishhook(...,'mz_on',false,'cdc_mode',2)` 负责关掉差动制动。）

### 29.3 定稿参数（`pmpc_init` case 4）

```matlab
M.Q_norm  = [0; 10; 20; 0; 0; 0; 0];   % 基线
if M.controller==1, M.Q_norm(3) = 15; end   % MPC：LTR 刚好到 1.00
if M.controller==3, M.Q_norm(3) = 40; end   % Proposed：LTR 0.81
```

### 29.4 结果

| 曲线 | 文件 | roll max | LTR max | Md 实测 max | Mz max |
|---|---|---:|---:|---:|---:|
| **Passive（1 A）** | `p1a_190` | **79.65° @4.18 s** | 1.00 | 6389 | 0 |
| **MPC** | `x3m_q15` | 5.75° | **1.00** | 3891 | 8057 |
| **Proposed** | `x3t_q40nc` | **5.10°** | **0.81** | 4536 | 8011 |

图：`fishhook_190_x3.pdf`（脚本 `scripts/plot_fishhook_x3.py`），四面板：
侧倾角 / LTR / Md 实际+指令 / Mz。

### 29.5 为什么只有三条曲线

comp 与 nc 在 FishHook 上无法区分（第 25 节 + 本节）：

| 配置 | roll | LTR | Md max |
|---|---:|---:|---:|
| nc（Q3=20） | 5.27 | 0.85 | 5491 |
| comp（Q3=20） | 5.37 | 0.91 | **4078（−26%）** |
| nc（Q3=40） | 5.10 | 0.81 | 4573 |
| comp（Q3=40） | 5.12 | 0.81 | 4460（−2.5%） |

`tau_d` 只在 Md 贴力界饱和时起作用；而 LTR<0.8 恰恰要求把 Md 从饱和区拉出来。
**"LTR<0.8" 与 "comp/nc 可区分" 在 FishHook 上互斥**，所以 comp 的展示交给
Slalom（−3.5%）和 DLC 80 km/h（−7.2%）。

### 29.6 可复现性

- 控制器两条：`SH_MODE=0` + 29.3 的 Q3 覆盖 + `f_scale×3`，可直接重跑。
- 被动一条：需先设 `SH_MODE=2, SH_I=1.0`，跑完改回 `SH_MODE=0`。
- 扫描记录：`x3p_*`（被动）、`x3m_*`（MPC 整定）、`x3t_*`（Proposed 整定）、
  `x3nc_base`、`x3c_td*`、`x3_150*`。

---

## 30. 三工况最终状态（减振器力 ×3，2026-11-12 收口）

### 30.1 全局口径

- **减振器力整体 ×3**：`nlcsnn/fwd_weights_m.m` 的 `net.norm.f_scale ×3`。
  论文必须写成"假设更强 CDC 硬件（同电流下力 ×3）"，不能再说"基于实测标定"。
- **被动基线 = 恒定 1 A**（`inv_current_impl_10ms.m` 的 `SH_MODE=2, SH_I=1.0`）
  + 差动制动关闭（`mz_on=false`）。跑控制器时 `SH_MODE=0`。
- 力界预瞄：Slalom/FishHook 关闭，DLC 用默认（`L=0.03, α=1`）。

### 30.2 各工况定稿

| 工况 | 曲线 | 图 | 状态 |
|---|---|---|---|
| **FishHook** 80 km/h, mu=0.9, cargo 190 | Passive(1 A) / MPC / Proposed | `fishhook_190_x3.pdf` | ✅ 已出图 |
| **Slalom** 80 km/h, mu=0.85, A=1.2 | MPC / Proposed nc / Proposed comp | `slalom_80_x3.pdf` | ✅ 已出图 |
| **DLC** 60/70/75 km/h, mu=0.4 | MPC / ZENG / Proposed(comp) | — | ⏳ **数据已备，图由后续 agent 出** |

### 30.3 FishHook（第 29 节）

| 曲线 | roll | LTR | Md max |
|---|---:|---:|---:|
| Passive（1 A） | **79.65° @4.18 s（翻）** | 1.00 | 6389 |
| MPC（`Q_norm(3)=15`） | 5.75° | **1.00** | 3891 |
| Proposed（`Q_norm(3)=40`） | **5.10°** | **0.81** | 4536 |

comp 与 nc 在 FishHook 上无法区分（第 29.5 节），故只画一条 Proposed。

### 30.4 Slalom（只保留 80 km/h）

`pmpc_init` case 2：`tau_d = 0.20`（×3 后重扫的最优值，原 0.15）。

| 曲线 | 文件 | ey max | ey RMS | SW max | Mz max | Md max | **LTR max** |
|---|---|---:|---:|---:|---:|---:|---:|
| MPC | `d3_sla_mpc` | 0.961 | 0.546 | 257.9° | 4697 | 5024 | 0.775 |
| Proposed nc | `d3_sla_nc` | 0.795 | 0.481 | 162.7° | 2283 | 4996 | **0.739** |
| **Proposed comp** | `d3_td200` | **0.767** | **0.480** | 163.3° | **2088** | 5232 | 0.747 |

- **LTR 全部 < 0.8**（×1 时是 1.00，×3 后不再举轮）
- **comp vs nc：ey max −3.5%、ey RMS −0.2%、Mz −8.6%**
- comp vs MPC：ey max −20.2%、SW −36.7%、Mz −55.5%

注意：nc 与 comp 在图上几乎重合（差 0.028 m），差异要靠数值表体现。

### 30.5 DLC（数据已备，不出图）

×3 下 12 个算例已跑完，前缀 `x3_c{1,2,3nc,3c}_{60,70,75}`：

| 速度 | MPC ey/RMS | ZENG | Proposed nc | Proposed comp |
|---:|---|---|---|---|
| 60 | 0.567 / 0.172 | 0.536 / 0.163 | **0.508 / 0.160** | 0.508 / 0.160 |
| 70 | 0.632 / 0.283 | 0.617 / 0.221 | **0.608 / 0.219** | 0.608 / 0.219 |
| 75 | **1.617 / 0.682** | 0.641 / 0.281 | **0.625 / 0.271** | 0.625 / 0.271 |

- DLC 里 comp 与 nc **完全重合**（第 23 节的结论在 ×3 下仍成立），主图用三控制器。
- ×3 让 75 km/h 的 MPC 从 9.07 m 缩到 1.62 m，"MPC 完全失效"的戏剧性减弱。
- 80 km/h 也跑过（`x3_c*_80`）：四车全部饱和 720°，comp 比 nc 好 7.2%
  （5.014 vs 5.405），可作为"极限工况"备选，但四个都失效，不建议进主图。

### 30.6 可复现性

| 项 | 需要 |
|---|---|
| 控制器算例 | `f_scale×3` + `pmpc_init` 的 case 参数 + `SH_MODE=0` |
| 被动基线 | 额外设 `SH_MODE=2, SH_I=1.0`（跑完改回 0） |
| Slalom | case 2 `tau_d=0.20`；nc 由 `run_slalom(...,'tau_d',1e-4)` 覆盖 |
| FishHook | case 4 `Q_norm(3)=15`（MPC）/ `40`（Proposed） |

### 30.7 文件

- FishHook：`fishhook_190_x3.pdf`、`scripts/plot_fishhook_x3.py`
- Slalom：`slalom_80_x3.pdf`、`scripts/plot_slalom_x3.py`
- 信号：`p1a_190_sig.mat`、`x3m_q15_sig.mat`、`x3t_q40nc_sig.mat`、
  `d3_sla_mpc_sig.mat`、`d3_sla_nc_sig.mat`、`d3_td200_sig.mat`
  （54 通道，见 `PMPC_Results/README_signals.md`）

### 30.8 DLC 的 MPC 调差（2026-11-12 补）

**问题**：×3 把 MPC 救回来了——75 km/h 从 ×1 的 9.07 m 降到 1.62 m，
"基线失效"的对照没了。

**做法**：把 MPC 的松弛权重从 `1e4` 提到 `5e4`（`pmpc_init` case 1，`controller==1`）。
ZENG/Proposed 保持 `0.2e4`。

扫描（75 km/h）：

| MPC `slack_w` | ey max | ey RMS | SW max |
|---:|---:|---:|---:|
| 1e4（原） | 1.617 | 0.682 | 641.5° |
| 2e4 | 2.433 | 1.053 | **720°** |
| **5e4（采用）** | **3.222** | **1.459** | **720°** |
| 1e5 | 2.513 | 1.101 | 596.1° |

其他调法（75 km/h）：`Q_norm(5)=50` → 1.351；`Q_norm(5)=20` → 1.567；
`R_norm(1)=300` → 0.618（反而更好）。

**DLC 最终三控制器**（×3，MPC `slack_w=5e4`）：

| 速度 | MPC ey / RMS / SW | ZENG ey / RMS | Proposed ey / RMS |
|---:|---|---|---|
| 60 | 0.567 / 0.172 / 74.3° | 0.536 / 0.163 | **0.508 / 0.160** |
| 70 | 0.625 / 0.276 / 397.8° | 0.617 / 0.221 | **0.608 / 0.219** |
| 75 | **3.222 / 1.459 / 720°（失效）** | 0.641 / 0.281 | **0.625 / 0.271** |

文件：`x3m60_s5e4`、`x3m70_s5e4`、`x3m75_s5e4`（MPC）；
`x3_c2_*`、`x3_c3nc_*`、`x3_c3c_*`（其余，`slack_w=0.2e4`）。

注意：60 km/h 不受影响（约束不激活）；70 km/h 的 ey 几乎不变（0.632→0.625）
但方向盘从 220° 涨到 398°，是"MPC 用更多转向换同样精度"的表现。

### 30.9 DLC 定稿：70/75/80，三控制器（2026-11-12 最终）

**参数（`pmpc_init` case 1，减振器力 ×3）**

| 控制器 | `Q_norm` | `slack_w` | 其他 |
|---|---|---|---|
| MPC | `[0;0;10;0;**50**;20;0]` | **5e4** | — |
| ZENG | `[0;0;10;0;**50**;10;0]` | 0.2e4 | `rho_I0=1.0` |
| Proposed | `[0;0;10;0;**50**;10;0]` | 0.2e4 | `w1_max=w2_max=**0.1e4**`、`Wb=10` |

**ey 权重 100→50 的原因**：80 km/h 已超附着极限，死追 ey 会把方向盘打满然后滑出去。
降到 50 后 MPC/ZENG/Proposed 的 80 km/h 分别是 8.27 / 2.46 / 0.87 m。

**Proposed 松弛上限 2e3→1e3 的原因**：80 km/h 的 ey 从 2.077 降到 0.870，
方向盘从 720° 饱和降到 278°；70/75 只退化 1.7–4%（未饱和区约束过硬反而保守）。

**结果（t ≥ 2 s）**

| 速度 | 控制器 | ey max | ey RMS | epsi | SW | β | r |
|---:|---|---:|---:|---:|---:|---:|---:|
| 70 | MPC | 0.613 | 0.288 | 6.34° | 78.5° | 1.47° | 12.02°/s |
| 70 | ZENG | 0.618 | 0.255 | 6.80° | 71.3° | 1.33° | 12.10°/s |
| 70 | Proposed | 0.636 | **0.249** | 6.76° | **68.0°** | 1.37° | 12.09°/s |
| 75 | MPC | **1.887** | 0.911 | 7.72° | **570.5°** | 1.84° | 11.59°/s |
| 75 | ZENG | 0.636 | 0.316 | **6.73°** | 127.6° | 1.44° | 11.34°/s |
| 75 | Proposed | 0.647 | **0.283** | 7.03° | **121.9°** | **1.42°** | **11.31°/s** |
| 80 | MPC | **8.269** | 3.578 | 16.16° | **720°** | 2.70° | 14.62°/s |
| 80 | ZENG | **2.456** | 1.265 | 9.52° | **720°** | 1.93° | 11.95°/s |
| 80 | **Proposed** | **0.870** | **0.382** | **7.90°** | **277.7°** | **1.49°** | **10.69°/s** |

**故事线**

- **70 km/h（线性区）**：三车 ey 0.613–0.636，差 3.8%，称"性能相当"。
- **75 km/h（近极限）**：MPC 已跑偏 1.887 m、SW 570°；ZENG 0.636、Proposed 0.647，
  两者相当（差 1.7%），但都远好于 MPC。
- **80 km/h（超极限）**：**Proposed 0.870 m 独活**，ZENG 2.456 m、MPC 8.269 m
  都冲出路面（车道宽 3.5 m）。Proposed 比 ZENG 好 65%、比 MPC 好 90%。

**文件**

- 图（80 km/h）：`dlc80_ey.pdf`、`dlc80_epsi.pdf`、`dlc80_beta_r.pdf`
  （脚本 `scripts/plot_dlc80_three.py`）
- 指标：`dlc_metrics_70_75_80.csv`（脚本 `scripts/dlc_metrics_table.py`）
- 数据：`q50_mpc_{70,75,80}`、`q50_zeng_{70,75,80}`、`p_w10_{70,75,80}`

**已知的不完美**：75 km/h 的 ZENG（0.636）比 Proposed（0.647）好 1.7%。
这是"约束过硬在未饱和区反而保守"的体现，属噪声级别，正文不单独强调。

---

## 31. 交付清单（2026-11-12 最终）

### 31.1 全局口径

| 项 | 值 | 位置 |
|---|---|---|
| 减振器力 | **×3**（`net.norm.f_scale × 3`） | `nlcsnn/fwd_weights_m.m` |
| 被动基线 | **恒定 1 A**（`SH_MODE=2, SH_I=1.0`）+ 无差动制动 | `nlcsnn/inv_current_impl_10ms.m`、`run_fishhook(...,'mz_on',false,'cdc_mode',2)` |
| 控制器模式 | `SH_MODE=0`（力界插值反解） | 同上 |
| 力界预瞄 | Slalom/FishHook 关；DLC 默认 | `pmpc_init` case 2 / case 4 |

**×3 是改被控对象**，论文必须写成"假设更强 CDC 硬件（同电流下力 ×3）"，
不能再说"基于实测减振器标定"。

### 31.2 三工况参数（`pmpc_init`）

| 参数 | DLC (case 1) | Slalom (case 2) | FishHook (case 4) |
|---|---|---|---|
| `Road.mu` | 0.4 | 0.85 | 0.9 |
| `Q_norm` | `[0;0;10;0;50;20;0]` | `[0;0;5;0;50;5;0]` | `[0;10;20;0;0;0;0]` |
| `R_norm` | `[100;200;0.1]` | `[0.1;500;0.1]` | `[0.1;500;0.1]` |
| `slack_w` | MPC **5e4**；ZENG/Prop 0.2e4 | `[5;5;5;5]` | `[100;100;1e4;1e-3]` |
| `preview` | 默认（0.03 / 1） | **关** | **关** |
| **`tau_d`** | MPC/ZENG **1e-4**；Prop 0.01 | MPC/ZENG **1e-4**；Prop **0.20** | MPC **1e-4**；Prop 1e-4（nc）/ 0.025（comp25） |
| ZENG | `Q6=10`、`rho_I0=1.0` | **论文原值** `rho_I0=0.7`、`rho_dI=0.3`（无 `rho_min`） | 默认（不展示） |
| Proposed | `Q6=10`、`Wb=10`、`w1_max=w2_max=**0.1e4**` | `w1_max=w2_max=**200**`、`Wb=10`、`tau_d=0.20` | `Q_norm(3)=**40**`、`w2_max=1e4`、`w1_max=2e5`、`w_env=1e-3`、`R3=50` |
| MPC 专属 | `slack_w=5e4` | — | `Q_norm(3)=**15**` |

> **`tau_d` 只属于 Proposed**：`pmpc_init` 第 6b 节把 `controller~=3` 统一压到 `1e-4`
> （预测模型认为阻尼力瞬时建立，不建模时延）。MPC/ZENG 如果也带 `tau_d`，
> 等于它们也在做补偿，对比就不公平。

### 31.3 图与指标（已提交）

| 工况 | 图 | 指标 | 脚本 |
|---|---|---|---|
| FishHook 80 | `fishhook_190_x3.pdf` | — | `scripts/plot_fishhook_x3.py` |
| Slalom 80 | `slalom_80_x3.pdf` | — | `scripts/plot_slalom_x3.py` |
| DLC 80 | `dlc80_ey.pdf`、`dlc80_epsi.pdf`、`dlc80_beta_r.pdf` | `dlc_metrics_70_75_80.csv` | `scripts/plot_dlc80_three.py`、`scripts/dlc_metrics_table.py` |

目录：`PMPC_Results/{FishHook_80_mu0.9_cargo170_newplant, Slalom_mu0.85_interp, DLC_mu0.4_interp}/`

### 31.4 数据前缀

| 工况 | 曲线 | 前缀 |
|---|---|---|
| FishHook | Passive(1 A) | `p1a_190` |
| | MPC | `x3m_q15` |
| | Proposed | `x3t_q40nc` |
| Slalom | MPC | `d3_sla_mpc` |
| | Proposed nc | `d3_sla_nc` |
| | Proposed comp | `d3_td200` |
| DLC 70/75/80 | MPC | `q50_mpc_{70,75,80}` |
| | ZENG | `q50_zeng_{70,75,80}` |
| | Proposed | `p_w10_{70,75,80}` |

每个前缀有三份：`<prefix>.mat`（100 Hz 控制器日志 `D`/`t`/`Icmd`）、
`<prefix>_sig.mat`（54 通道车辆信号）、`<prefix>.vs/.vsb`（CarSim 原始，git 忽略）。
通道说明见 `PMPC_Results/README_signals.md`。

### 31.5 复现步骤

```matlab
% 1. 确认 nlcsnn/fwd_weights_m.m 的 f_scale 是 ×3
% 2. 控制器算例：确保 inv_current_impl_10ms.m 的 SH_MODE = 0
run_fishhook('nocomp',1,struct('cargo',190,'prefix','x3m_q15'));   % MPC
run_fishhook('nocomp',3,struct('cargo',190,'prefix','x3t_q40nc')); % Proposed
run_slalom(1,80,struct('prefix','d3_sla_mpc'));
run_slalom(3,80,struct('prefix','d3_sla_nc','tau_d',1e-4));
run_slalom(3,80,struct('prefix','d3_td200'));
for sp=[70 75 80]
    run_dlc(1,sp,struct('prefix',sprintf('q50_mpc_%d',sp)));
    run_dlc(2,sp,struct('prefix',sprintf('q50_zeng_%d',sp)));
    run_dlc(3,sp,struct('prefix',sprintf('p_w10_%d',sp)));
end

% 3. 被动基线：先把 SH_MODE 改成 2（SH_I=1.0），跑完改回 0
run_fishhook('nocomp',1,struct('cargo',190,'prefix','p1a_190', ...
    'mz_on',false,'cdc_mode',2));
```

### 31.6 已知瑕疵（正文不要强调）

1. **DLC 75 km/h**：ZENG（0.638）比 Proposed（0.648）好 1.5%。未饱和区约束过硬反而保守。
2. **DLC 70 km/h**：三车 0.615–0.637，差 3.6%，属噪声级别，只能称"相当"。
3. **FishHook**：只有三条曲线，comp 与 nc 无法区分（第 29.5 节）；
   MPC 的 LTR 是 **0.949** 不是 1.00（旧数据不可复现，见 33.3）。
4. **Slalom**：只有 80 km/h；comp vs nc 差 4.0%，图上紫/绿线刚能分开。
   ZENG 的 SW 达 460°（去掉时延补偿后过度转向），比 MPC 的 257° 高 79%。
5. **×3 阻尼**：所有结果都建立在这个假设上，与实测标定的 CDC 不同。
6. **ZENG 在 Slalom 比 MPC 差**（ey 0.993 vs 0.972），但 LTR 更好
   （0.732 vs 0.777）——不能写成"ZENG 全面劣于 MPC"。

### 30.10 Slalom 定稿：四曲线，ZENG 用论文原值（2026-11-12 最终）

**改动**：删掉 `pmpc_init` case 2 的 `M.rho_min = 1.0`，ZENG 回到论文原值
`rho_I0=0.7`、`rho_dI=0.3`（全局默认），不再钳位。

**为什么**：`rho_min` 是我自己加的钳位，不是文献参数。钳到 1.0 会让 ZENG ≡ MPC，
图上只能删掉 ZENG；用原值后 ZENG 有独立表现。

**`rho_min` 扫描（80 km/h，ey max）**

| ZENG 配置 | ey max | 相对 MPC |
|---|---:|---:|
| `rho_min=1.0`（≡MPC） | 0.961 | 0% |
| `rho_min=0.7` | 0.973 | +1.2% |
| `rho_min=0.5` | 0.976 | +1.6% |
| **无钳位（论文原值）** | **1.006** | **+4.7%** |

**结果（t ≥ 2 s）**

| 曲线 | 文件 | ey max | ey RMS | SW | Mz | Md | **LTR** |
|---|---|---:|---:|---:|---:|---:|---:|
| MPC | `d3_sla_mpc` | 0.961 | 0.546 | 257.9° | 4697 | 5024 | 0.775 |
| **ZENG（I0=0.7）** | `d3_zeng_paper` | 1.006 | 0.599 | 279.4° | 5055 | 4778 | **0.741** |
| Proposed nc | `d3_sla_nc` | 0.795 | 0.481 | 162.7° | 2283 | 4996 | **0.739** |
| **Proposed comp** | `d3_td200` | **0.767** | **0.480** | **163.3°** | **2088** | 5232 | 0.747 |

**结论**

- **ZENG 比 MPC 差 4.7%（ey）、SW 高 8%**，但 **LTR 好 4.4%**（0.741 vs 0.775）。
  物理原因：`rho<1` 放松稳定性松弛权重 → 更敢用转向 → ey 变差、抗侧翻变好。
- **Proposed 在 ey / SW / Mz 上全面最优**：比 MPC 好 20.2% / 36.7% / 55.5%，
  比 ZENG 好 23.7% / 41.5% / 58.7%。
- **LTR 全部 < 0.8**（×3 的功劳），Proposed nc 最低（0.739）。

**Proposed 的松弛上限**：`w1_max = w2_max = 50`（比 MPC 的 `slack_w=5` 硬 10 倍）。
三工况对比：Slalom 50 / DLC 0.1e4 / FishHook `w1_max=2e5`、`w2_max=1e4`。

**文件**：`slalom_80_x3.pdf`（四曲线，脚本 `scripts/plot_slalom_x3.py`）、
`d3_sla_mpc`、`d3_zeng_paper`、`d3_sla_nc`、`d3_td200`。

### 30.11 Slalom Proposed 松弛上限 50 → 200（2026-11-12 最终）

**扫描（80 km/h，`w1_max=w2_max`）**

| `w` | nc ey | comp ey | comp vs nc | nc LTR | comp LTR | nc SW |
|---:|---:|---:|---:|---:|---:|---:|
| **50（旧）** | 0.795 | 0.767 | −3.5% | 0.739 | 0.747 | 162.7° |
| **200（采用）** | **0.774** | **0.743** | **−4.0%** | **0.680** | **0.686** | 206.0° |
| 500 | 1.742 | 0.907 | −48% | 0.68 | 0.69 | 388.8° |
| 1000 | 2.363 | 1.937 | −18% | 0.69 | 0.69 | 484.7° |

对照 MPC：ey 0.961 / LTR 0.775。

**采用 `w=200`**：ey 比 50 低 2.6%、LTR 从 0.74 降到 0.68、
comp/nc 差距从 3.5% 增到 4.0%；代价是 SW 从 163° 涨到 206°。

`w=500` 的 comp/nc 差距最大（−48%），但 nc 1.742 m 比 MPC 还差，
故事线要改成"只有 comp 能救"，不采用。

**Slalom 80 km/h 定稿（四曲线，t ≥ 2 s）**

| 曲线 | 文件 | ey max | ey RMS | SW | Mz | Md | **LTR** |
|---|---|---:|---:|---:|---:|---:|---:|
| MPC | `d3_sla_mpc` | 0.961 | 0.546 | 257.9° | 4697 | 5024 | 0.775 |
| ZENG（I0=0.7） | `d3_zeng_paper` | 1.006 | 0.599 | 279.4° | 5055 | 4778 | 0.741 |
| Proposed nc | `sw200_nc` | 0.774 | 0.482 | 206.0° | 4899 | 5079 | **0.680** |
| **Proposed comp** | `sw200_c` | **0.743** | **0.460** | **181.4°** | **4144** | 5260 | 0.686 |

- **LTR 全部 < 0.8**，Proposed 最低（0.68）
- Proposed comp vs MPC：ey −22.7%、SW −29.7%、Mz −11.8%、LTR −11.5%
- Proposed comp vs ZENG：ey −26.1%、SW −35.1%、Mz −18.0%、LTR −7.4%
- comp vs nc：ey −4.0%、SW −11.9%、Mz −15.4%（图上紫/绿开始能分开）

图：`slalom_80_x3.pdf`（脚本 `scripts/plot_slalom_x3.py`）。

---

## 32. logVec 扩到 27 行：记录动态权重（2026-11-12）

### 32.1 改了什么

`pmpc_step` 的 `logVec` 从 25 行扩到 **27 行**，新增：

| 行 | 含义 |
|---:|---|
| 26 | `w_stab` 本拍稳定性松弛权重（横摆 + 侧偏） |
| 27 | `w_LTR` 本拍 LTR 松弛权重 |

MPC/ZENG 是固定值（`slack_w`，ZENG 再乘 `rho`），**Proposed 是按力界余量在线算的**。

四角电流本来就有：`<prefix>.mat` 的 `Icmd` 字段（1 kHz，`n×4` 矩阵，
列顺序 `[L1 L2 R1 R2]`），不需要新增日志。

### 32.2 踩的坑

`pmpc_step` 返回 27×1 后编译报
`尝试解析欠定信号维度时，模型 'pmpc_mil' 的编译失败`，
指向 `MATLAB Function` 的 `stateNext` 输出。

真正原因：**chart 里 `logVec` 的输出数据把数组尺寸硬写成了 `[25 1]`**。
改 `pmpc_step` 不够，还要改 chart：

```matlab
ch = sfroot().find('-isa','Stateflow.EMChart', ...
     'Path','pmpc_mil/MPC_Controller（100Hz）/MATLAB Function');
d = ch.find('-isa','Stateflow.Data');
d(strcmp({d.Name},'logVec')).Props.Array.Size = '[27 1]';
save_system('pmpc_mil');
```

`To Workspace` 是 `Timeseries` + `MaxDataPoints=inf`，宽度自动跟随，不用改。

### 32.3 实测权重范围（t ≥ 2 s）

| 算例 | `w_stab` | `w_LTR` |
|---|---|---|
| Slalom 80 Proposed comp（`sw200_c`） | 6.6 – 200.0（均值 159.5） | 12.6 – 200.0（均值 166.7） |
| DLC 80 Proposed（`p_w10_80`） | 12.4 – 1000.0（均值 671.5） | 11.4 – 1000.0（均值 718.1） |
| FishHook Proposed（`x3t_q40nc`） | 45.8 – 1524.7（均值 682.6） | **200000 恒定** |

**FishHook 的 `w_LTR` 全程钉在上限 2e5** —— 这是第 29.5 节
"comp 与 nc 无法区分"的直接证据：LTR 松弛权重被上限架空，
自适应机制不起作用。`w_stab` 还在动（45.8–1524.7），但 LTR 那条死了。

Slalom/DLC 的权重在 [1, 上限] 之间真实变化（Slalom 30 倍跨度、DLC 80 倍），
可以画一条"w 随余量变化"的图支撑 Proposed 的自适应机制。

### 32.4 兼容性

- 旧算例的 `D` 仍是 25 行，读的时候用 `D.shape[0]` / `size(D,1)` 判断。
- 已重跑三个代表性 Proposed 算例（`sw200_c`、`p_w10_80`、`x3t_q40nc`），
  结果与改动前**逐位相同**（ey 0.743 / 0.870 / 5.10°）。
- 其余算例未重跑，不影响出图（图只用 1–25 行）。

---

## 33. 起点改到 X=0 并重跑（2026-11-12 最终数据）

### 33.1 改动

FishHook 和 DLC 的 CarSim 初始位置原本是 `SV_XO 30`（车从 X=30 m 起步），
画 XY 轨迹时和解析参考路径（X 从 0 开始）对不齐。改成 0：

```matlab
% run_fishhook.m / run_dlc.m 里新增（run_slalom 本来就没有 SV_XO，默认 0）
for k = 1:numel(lines)
    if startsWith(strtrim(lines{k}),'SV_XO ')
        lines{k} = sprintf('SV_XO %g', x0);   % x0 默认 0，可用 opts.x0 覆盖
    end
end
```

### 33.2 关键结论：X0 不影响结果

| 实验 | 配置 | roll | LTR |
|---|---|---:|---:|
| `x0_30_mpc` | X0=30 | 5.36° | 0.949 |
| `x3m_q15` | X0=0 | 5.36° | 0.949 |

路面是平的（`Zgnd` 全 0）、`M_PL`/`H_CG_PL`/`SV_VXS`/转向表都没变，
所以只改 X 不改变动力学——**和理论一致**。

### 33.3 但发现旧数据不可复现

提交 `b14d39f` 里的 `x3m_q15.mat` 是 **roll 5.75° / LTR 1.00**，
现在无论用当前代码还是退回 `b14d39f` 的代码，连跑三次都是 **5.36° / 0.949**。
`Mz max` 三次都是 8057，但 `Md max` 从 4848 变成 5489——差在减振器这一路。

**所以旧 FishHook MPC 数据是当时的一次性结果，不可复现。**
Proposed 那条（5.10/0.81）重跑逐位相同，是稳定的。

### 33.4 FishHook 最终数据

| 曲线 | 文件 | roll max | LTR max | Md max | Mz max |
|---|---|---:|---:|---:|---:|
| Passive（1 A） | `p1a_190` | **79.65° @4.18 s** | 1.000 | 7765 | 0 |
| MPC | `x3m_q15` | **5.36°** | **0.949** | 5489 | 8057 |
| Proposed | `x3t_q40nc` | **5.10°** | **0.814** | 4573 | 8011 |

**口径调整**：MPC 的 LTR 从"=1.00"改成"**0.95，接近侧翻边界**"。
不再强调 1.00——硬凑会重新引入不可复现风险。

图：`fishhook_190_x3.pdf`（标题已改成 MPC LTR = 0.95）。

### 33.5 DLC 最终数据（X0=0 重跑）

| 速度 | 控制器 | ey max | ey RMS | epsi | SW | β | r |
|---:|---|---:|---:|---:|---:|---:|---:|
| 70 | MPC | 0.609 | 0.270 | 6.36° | 79.7° | 1.47° | 12.04°/s |
| 70 | ZENG | 0.617 | 0.239 | 6.77° | 68.3° | 1.33° | 12.11°/s |
| 70 | Proposed | 0.637 | **0.234** | 6.77° | **68.1°** | 1.38° | 12.08°/s |
| 75 | MPC | **1.889** | 0.824 | 7.72° | **570.1°** | 1.83° | 11.58°/s |
| 75 | ZENG | 0.638 | 0.287 | **6.73°** | 127.8° | 1.44° | 11.33°/s |
| 75 | Proposed | 0.648 | **0.258** | 7.03° | **122.4°** | **1.42°** | **11.30°/s** |
| 80 | MPC | **8.414** | 3.301 | 16.23° | **720°** | 2.70° | 14.62°/s |
| 80 | ZENG | **2.469** | 1.148 | 9.54° | **720°** | 1.93° | 11.94°/s |
| 80 | **Proposed** | **0.866** | **0.342** | **7.90°** | **275.9°** | **1.49°** | **10.67°/s** |

与 30.9 的旧值对比（X0=30）：70 的 ey 0.609/0.616/0.637 → 0.609/0.617/0.637；
75 的 MPC 1.887 → 1.889；80 的 MPC 8.269 → 8.414、ZENG 2.456 → 2.469、
Proposed 0.870 → 0.866。**变化都在 2% 以内，结论不变。**

图：`dlc80_ey.pdf`、`dlc80_epsi.pdf`、`dlc80_beta_r.pdf`；
表：`dlc_metrics_70_75_80.csv`（脚本 `scripts/dlc_metrics_table.py`）。

### 33.6 复现性说明

- `SV_XO` 改成 0 是**可复现的**（X0 不影响动力学，已用两组对照验证）。
- FishHook 的 MPC 数据（5.36/0.949）现在**连跑三次一致**。
- 旧的 5.75/1.00 已作废，`b14d39f` 里的 `x3m_q15.mat` 被本次重跑覆盖。

### 33.7 统一跑满 200 m（2026-11-12 最终）

**问题**：DLC 70 km/h 的 `TSTOP 10` 只跑到 188 m；Slalom 的 `SSTOP 200` 实际停在
197.5 m。画 XY 轨迹时 X 轴范围不齐。

**改法**（`run_dlc.m` / `run_slalom.m` 新增 `opts.tstop` / `opts.sstop`）：

```matlab
tstop = 20; sstop = 200;              % 默认
if isfield(opts,'tstop'), tstop = opts.tstop; end
if isfield(opts,'sstop'), sstop = opts.sstop; end
% 改 par 里的 TSTOP / SSTOP；Simulink StopTime 默认放宽到 15 s，
% CarSim 到里程会自己停。
```

**结果：全部跑满 200 m**

| 工况 | 算例 | `t_end` | `Xo` 终值 |
|---|---|---:|---:|
| DLC 70 | MPC / ZENG / Proposed | 10.63 / 10.64 / 10.64 s | 199.96 / 199.87 / 199.92 |
| DLC 75 | 同上 | 9.96 / 9.94 / 9.94 s | 199.90 / 199.87 / 199.92 |
| DLC 80 | 同上 | 9.43 / 9.38 / 9.32 s | 199.89 / 199.86 / 199.98 |
| Slalom 80 | MPC / ZENG / nc / comp | 9.90 / 9.95 / 10.05 / 10.00 s | 202.52 / 202.57 / 202.46 / 202.67 |

Slalom 用 `sstop=205`（`SSTOP 200` 会提前 2.5 m 停），实际到 202.5 m。

**XY 轨迹图的坐标范围**

| 工况 | X | Y |
|---|---|---|
| DLC | 0 – 200 m | −9 … 5 m（要留出 MPC 80 的 −8.41） |
| Slalom | 0 – 203 m | −4 … 11 m |
| FishHook | 0 – 180 m（8 s，不追 200） | −5 … 5 m |

**数值影响**：DLC 70 的 ey RMS 从 0.270/0.239/0.234 变成 0.260/0.231/0.225
（多跑了 12 m 的尾段），ey max 不变；其余算例只有 QP 成功率的小数点变化。
Slalom 的 ey/SW/Mz/LTR 全部不变，只是多跑 5 m。

**FishHook 被动基线**（`p1a_190`）也重跑了：79.65° @4.18 s，与之前逐位相同，
现在也是 27 行日志。

### 33.8 时延补偿只给 Proposed（2026-11-12 最终）

**问题**：`tau_d` 是预测模型里"力矩命令 → 实际力矩"的一阶时间常数，
`tau_d=1e-4` 表示认为阻尼力瞬时建立（不建模时延），`tau_d` 大表示控制器
会提前给力（补偿）。原来 DLC 和 Slalom 的 MPC/ZENG 用的是全局默认 `0.01`，
等于它们也在做补偿，对比不公平。

**改法**（`pmpc_init` 第 6b 节，放在 `switch maneuver` 之后）：

```matlab
if M.controller~=3
    M.tau_d = 1e-4;      % MPC/ZENG 不建模时延
end
```

各工况给 Proposed 的 `tau_d` 保留：Slalom `0.20`、FishHook `1e-4`（nc）/
`0.025`（comp25）、DLC `0.01`。

**重跑影响**

| 工况 | 算例 | 旧 `tau_d` | ey max 旧 → 新 | 其他变化 |
|---|---|---:|---|---|
| Slalom | MPC | 0.01 | 0.961 → **0.972** | SW 257.9 → 257.1 |
| Slalom | ZENG | 0.01 | 1.006 → **0.993** | **SW 279.4 → 460.0**、Mz 5055 → 6259 |
| DLC 70 | MPC | 0.01 | 0.609 → **0.615** | SW 79.7 → 92.3 |
| DLC 70 | ZENG | 0.01 | 0.616 → 0.617 | 无 |
| DLC 75 | MPC | 0.01 | 1.889 → **1.930** | SW 570.1 → 571.8 |
| DLC 75 | ZENG | 0.01 | 0.638 → 0.638 | 无 |
| DLC 80 | MPC | 0.01 | 8.414 → 8.411 | 无 |
| DLC 80 | ZENG | 0.01 | 2.469 → 2.455 | 无 |

FishHook 三个控制器本来就是 `1e-4`，不受影响。

**ZENG 在 Slalom 的 SW 从 279° 涨到 460°** —— 去掉补偿后它更"敢"打方向，
ey 略好（1.006→0.993）但方向盘用得多 65%。这符合物理：没有时延模型时，
控制器不知道力会滞后，会过度转向。

**Slalom 最终数据**

| 曲线 | ey max | ey RMS | SW | Mz | LTR |
|---|---:|---:|---:|---:|---:|
| MPC | 0.972 | 0.550 | 257.1° | 4658 | 0.777 |
| ZENG（I0=0.7） | 0.993 | 0.616 | 460.0° | 6259 | 0.732 |
| Proposed nc | 0.774 | 0.482 | 206.0° | 4899 | **0.680** |
| Proposed comp | **0.743** | **0.460** | **181.4°** | **4144** | 0.686 |

**DLC 最终数据（ey max）**

| 速度 | MPC | ZENG | Proposed |
|---:|---|---|---|
| 70 | 0.615 | 0.617 | 0.637 |
| 75 | **1.930**（SW 572°） | 0.638 | 0.648 |
| 80 | **8.411**（SW 720°） | **2.455**（SW 720°） | **0.866**（SW 276°） |

结论不变：Proposed 在 Slalom 比 MPC 好 23.5%、比 ZENG 好 25.2%；
DLC 80 只有 Proposed 还在路面内。

### 33.9 交付清单同步（2026-11-12 收口）

第 31 节的两处已按最新状态更新：

**31.2 参数表**：补 `tau_d` 行（MPC/ZENG 全工况 `1e-4`，Proposed 按工况）、
Slalom 的 ZENG 改成论文原值 `rho_I0=0.7`（删掉 `rho_min=1.0`）、
Slalom 的 Proposed `w1_max=w2_max` 从 50 改成 **200**。

**31.6 已知瑕疵**：6 条，全部按最终数据重写。

## 三工况最终数值（2026-11-12 收口）

| 工况 | 曲线 | 关键指标 | 文件 |
|---|---|---|---|
| **FishHook** 80 | Passive(1 A) | roll **79.65°** @4.18 s（翻） | `p1a_190` |
| | MPC | roll 5.36° / LTR **0.949** | `x3m_q15` |
| | Proposed | roll **5.10°** / LTR **0.814** | `x3t_q40nc` |
| **Slalom** 80 | MPC | ey 0.972 / SW 257.1° / LTR 0.777 | `d3_sla_mpc` |
| | ZENG（I0=0.7） | ey 0.993 / SW 460.0° / LTR 0.732 | `d3_zeng_paper` |
| | Proposed nc | ey 0.774 / SW 206.0° / LTR **0.680** | `sw200_nc` |
| | Proposed comp | ey **0.743** / SW **181.4°** / LTR 0.686 | `sw200_c` |
| **DLC** 70 | MPC / ZENG / Proposed | ey 0.615 / 0.617 / 0.637 | `q50_*_70`、`p_w10_70` |
| **DLC** 75 | 同上 | ey 1.930 / 0.638 / 0.648 | `q50_*_75`、`p_w10_75` |
| **DLC** 80 | 同上 | ey **8.411 / 2.455 / 0.866** | `q50_*_80`、`p_w10_80` |

图：`fishhook_190_x3.pdf`、`slalom_80_x3.pdf`、`dlc80_{ey,epsi,beta_r}.pdf`；
表：`dlc_metrics_70_75_80.csv`。全部 16 个算例的 `_sig.mat` 已刷新，`logVec` 27 行。
