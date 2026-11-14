# Proposed_noComp（ctrl-nocomp）结果记录

控制器：`pmpc_core`，7 状态 `[vy, r, phi, p, ey, epsi, Md]` / 3 控制量 `[delta, Mz, Md_cmd]`，
CDC 电流由控制器给（`nlcsnn/inv_current_impl_10ms.m` 的 `SH_MODE = 0`）。
**不补偿时滞**：预测模型里 `tau_d = 1e-4`（阻尼力瞬时建立），`tau_d_mode = 1`。

---

## 1. 时滞口径（重要，容易踩错）

`pmpc_core/pmpc_init.m` 的 **Slalom case** 在 `M.controller == 3` 时设 `M.tau_d = 0.20`
（那是 Proposed_Comp 的补偿口径，见该处注释里的 tau_d 扫描记录）。
所以：

```matlab
run_slalom(3, 80)              % 实际用的是 tau_d = 0.20 —— 这是 comp 口径，不是本分支
run_slalom(3, 80, struct('tau_d', 1e-4))   % ← 本分支必须这样显式覆盖
```

已实测：`[P,~,~,~] = pmpc_init(2,3)` 之后 `P.M.tau_d = 0.2`、`P.M.tau_d_mode = 1`。
本文件里每个算例的实际生效参数都另存为 `<prefix>_params.mat`（字段 `tau_d` /
`tau_d_mode` / `controller` / 权重 / 时间），可从磁盘复核，不依赖内存变量。

DLC case 未覆盖 `tau_d`，`pmpc_init(1,3)` 拿到的是全局默认 `0.01`，同样**不是** `1e-4`，
因此 DLC 算例也显式传 `tau_d=1e-4`。

---

## 1b. 预测时域（三方对齐用）

本分支**没有**改过时域，全部走 `pmpc_init` 自带的默认/工况分支：

| 工况 | 入口 | Ts [s] | Np | Nc | 预测窗口 | 来源 |
|---|---|---|---|---|---|---|
| DLC | `run_dlc(3,speed,opts)` | 0.1 | 10 | 5 | 1.0 s | `pmpc_init.m:10-12` 默认，未传 `horizon_override` |
| Slalom | `run_slalom(3,speed,opts)` | 0.1 | 10 | 5 | 1.0 s | 同上 |
| FishHook | `run_fishhook('nocomp',3,opts)` | **0.05** | **12** | **6** | 0.6 s | `run_fishhook` 自己传 `h`（Ts/Np/Nc），与 COORDINATION 一致 |

执行周期另算：`M.Ts_exec = 0.01`（100 Hz），与预测步长 `Ts` 不要混。

**对齐结论**：Baseline 那支此前 DLC/Slalom 用 0.05/12/6，与本分支（及 Proposed）
的 0.1/10/5 不一致，已由 Baseline 那支改成 0.1/10/5；FishHook 三边本来就都是
0.05/12/6，本分支不用动。上述时域均落在每个算例的 `<prefix>_params.mat` 的
`horizon = [Ts Np Nc]` 字段里（附 `Ts/Np/Nc` 拆开副本），可逐算例复核。

---

## 2. 结果表

`Md_avg` / `Mz_avg` 为 `mean(abs())`，**`Md` 取 `Md_actual`**（`logVec` 第 6 行，不是第 7 行的 `Md_cmd`）；
`QP%` = `D(24,:)>0.5` 的比例；`epsi_max` 已从 rad 换算成 deg；`T` 为仿真终止时刻
（CarSim 到 `SSTOP=200 m` 会自己停，所以 DLC 的 T 小于 `stopTime=15`）。

| 算例 | T [s] | ey_max [m] | ey_rms [m] | epsi_max [deg] | LTR_max | SW_max [deg] | Mz_avg [N·m] | Mz_max [N·m] | Md_avg [N·m] | Md_max [N·m] | Icmd 范围 [A] | 生效 tau_d [s] | Ts/Np/Nc | QP% |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `nc_verify` — DLC 70, 3 s（**链路自检，不用于性能对比**） | 3.00 | 0.0016 | 0.0002 | 0.033 | 0.018 | 5.8 | 2.7 | 79.8 | 23.1 | 163.8 | 0.000–1.600 | 1e-4 | 0.1/10/5 | 100.0 |
| `nc_sla_verify` — Slalom 80, 8 s（链路自检） | 8.00 | 0.7735 | 0.4651 | 3.728 | 0.682 | 206.0 | 2109.0 | 5234.9 | 1502.3 | 5078.5 | 0.000–1.600 | 1e-4 | 0.1/10/5 | 100.0 |
| `nc_dlc_70` — DLC 70 km/h, mu 0.4 | 10.64 | 0.6369 | 0.2028 | 6.769 | 0.325 | 68.1 | 665.8 | 2174.4 | 355.5 | 2409.7 | 0.000–1.600 | 1e-4 | 0.1/10/5 | 100.0 |
| `nc_dlc_75` — DLC 75 km/h, mu 0.4 | 9.94 | 0.6483 | 0.2305 | 7.027 | 0.330 | 122.4 | 856.2 | 2123.8 | 365.9 | 2316.5 | 0.000–1.600 | 1e-4 | 0.1/10/5 | 100.0 |
| `nc_dlc_80` — DLC 80 km/h, mu 0.4 | 9.32 | 0.8656 | 0.3031 | 7.902 | 0.330 | 275.9 | 846.8 | 2044.5 | 369.7 | 2378.2 | 0.000–1.600 | 1e-4 | 0.1/10/5 | 100.0 |
| `nc_sla_80` — Slalom 80 km/h, mu 0.85 | 8.00 | 0.7735 | 0.4651 | 3.728 | 0.682 | 206.0 | 2109.0 | 5234.9 | 1502.3 | 5078.5 | 0.000–1.600 | 1e-4 | 0.1/10/5 | 100.0 |
| `nc_fh_80` — FishHook 80 km/h, mu 0.9, cargo 170 kg | 7.99 | 14.8720 | 6.0360 | 52.003 | 0.790 | 0.0 | 4427.1 | 7762.4 | 650.4 | 4115.9 | 0.000–1.600 | 1e-4 | 0.05/12/6 | 100.0 |

### 备注

- **`nc_verify` / `nc_sla_verify` 是链路自检**（3 s 镜头 + 与 `nc_sla_80` 完全相同的那次
  Slalom 重跑），保留只为可追溯，**不进三方对比的性能列**。
- 五个矩阵算例的 `tau_d` 都显式覆盖成 **1e-4**（`run_*` 的 `opts.tau_d`），
  否则 Slalom 会拿到 comp 口径的 0.20、DLC 会拿到全局默认 0.01；
  每个算例的 `_params.mat` 里 `tau_d / tau_d_mode / horizon` 可逐条复核。
- `SW_max = 0` 在 FishHook 是**正常的**：该工况 `afs_on = false`，驾驶员转角由
  CarSim 开环机器人给，控制器只出 `Mz` 与四角阻尼电流，`SW` 列不参与控制。
- `nc_fh_80` 的 `ey_max = 14.87 m` 与 `epsi_max = 52°` 是鱼钩工况本身的甩尾幅度
  （参考路径是稳态圆周），`LTR_max = 0.790` 刚好压在 0.8 约束内 —— 说明 LTR 约束
  在起作用，而不是失控。
- DLC 三个速度的 `LTR_max` 都是 0.325–0.330，`SW_max` 从 68° 涨到 276°（车速高
  需要的 AFS 附加角更大），`ey_max` 从 0.637 涨到 0.866 m（mu=0.4 已接近附着极限）。
- Slalom 参数全部取 `pmpc_init` case 2 默认值，未调任何权重：
  `Q_norm=[0 0 5 0 50 5 0]`、`R_norm=[0.1 500 0.1]`、`slack_w=[5 5 5 5]`、
  `w1_max=w2_max=200`、`Wb=10`、`preview_L=preview_alpha=0`。
  `SW_max=206°` 与该处注释一致（w1_max 从 50 提到 200 时代价是 SW 163°→206°）。
- `Icmd` 为四角 1 kHz 电流（8001×4 / 3001×4），全部落在 0–1.6 A；
  若恒为 1.0 或恒为 0 则说明 `SH_MODE` 被误改成 2（固定电流基线），本表已排除：
  四角标准差 0.63–0.73，`min=0.000 / max=1.600`。

---

## 3. 文件

```
PMPC_Results/
  NC_RESULTS.md                                          ← 本文件
  DLC_mu0.4_interp/nc_verify.mat / _params.mat            ← 链路自检（3 s）
  DLC_mu0.4_interp/nc_dlc_70.mat / _params.mat           ← 矩阵：DLC 70
  DLC_mu0.4_interp/nc_dlc_75.mat / _params.mat           ← 矩阵：DLC 75
  DLC_mu0.4_interp/nc_dlc_80.mat / _params.mat           ← 矩阵：DLC 80
  Slalom_mu0.85_interp/nc_sla_verify.mat / _params.mat    ← 链路自检（与 nc_sla_80 同一次结果）
  Slalom_mu0.85_interp/nc_sla_80.mat / _params.mat        ← 矩阵：Slalom 80
  FishHook_80_mu0.9_cargo170_newplant/nc_fh_80.mat / _params.mat   ← 矩阵：FishHook 80
```

每个 `<prefix>.mat` 含 `D`（27×N）、`t`、`Icmd`（四角 1 kHz）；
每个 `<prefix>_params.mat` 含
`maneuver / maneuver_name_str / speed / horizon=[Ts Np Nc] / Ts / Np / Nc /
tau_d / tau_d_mode / controller / Q_norm / R_norm / slack_w / SH_MODE /
timestamp / opt`（`opt` 里是 w1_max / w2_max / Wb / rho_* / preview_* / cdc_mode / margin_mode）。
`SH_MODE` 是 `nlcsnn/inv_current_impl_10ms.m` 里的局部常量，base 工作区看不到，
由 `save_run_params.m` 静态解析落盘（本分支必须为 0）。

`D` 为 27×N 诊断日志，行序见 `pmpc_core/pmpc_step.m` 头注释：
`[ey, epsi, phi, p, LTR, Md_actual, Md_cmd, Md_min, Md_max, Fd×4, Fdes×4,
Mz_cmd, SW, Vx, r, deltaMd, deltaMd_max, QP_OK, candidate_used, w_stab, w_LTR]`
（`ey`、`phi`、`p`、`LTR` 为 SI；`epsi` 为 rad，表中已换算）。
