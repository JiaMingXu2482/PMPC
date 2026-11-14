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

## 2. 结果表

`Md_avg` / `Mz_avg` 为 `mean(abs())`；`QP%` = `D(24,:)>0.5` 的比例；
`epsi_max` 已从 rad 换算成 deg；`T` 为仿真终止时刻。

| 算例 | T [s] | ey_max [m] | ey_rms [m] | epsi_max [deg] | LTR_max | SW_max [deg] | Mz_avg [N·m] | Mz_max [N·m] | Md_avg [N·m] | Md_max [N·m] | Icmd 范围 [A] | 生效 tau_d [s] | QP% |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `nc_verify` — DLC 70 km/h, mu 0.4, 3 s | 3.00 | 0.0016 | 0.0002 | 0.033 | 0.018 | 5.8 | 2.7 | 79.8 | 23.1 | 163.8 | 0.000–1.600 | 1e-4 | 100.0 |
| `nc_sla_verify` — Slalom 80 km/h, mu 0.85, 8 s | 8.00 | 0.7735 | 0.4651 | 3.728 | 0.682 | 206.0 | 2109.0 | 5234.9 | 1502.3 | 5078.5 | 0.000–1.600 | 1e-4 | 100.0 |

### 备注

- `nc_verify` 是 **3 s 冒烟窗口**：70 km/h 下 DLC 变道在 X≈61 m（t≈3.1 s）才开始，
  窗口内 `ey` 只有 1.6 mm、`LTR` 0.018，属于直行前段。只用于确认链路通（QP 100%、
  Md 非零、Icmd 非常量），**不是**可用于性能对比的 DLC 算例。
- `nc_sla_verify` 是第一个可用于三方对比的算例（Slalom 80，8 s，mu=0.85）。
  `ey_max = 0.7735` 落在 `pmpc_init` 注释记录的 no-comp 量级（1e-4：0.795 全程值），
  与 comp 的 0.20 口径（0.767）差 0.9%，方向与口径一致（不补偿 → 误差略大）。
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
  NC_RESULTS.md                                  ← 本文件
  DLC_mu0.4_interp/nc_verify.mat                 ← D,t,Icmd
  DLC_mu0.4_interp/nc_verify_params.mat          ← 生效参数（tau_d=1e-4）
  Slalom_mu0.85_interp/nc_sla_verify.mat         ← D,t,Icmd
  Slalom_mu0.85_interp/nc_sla_verify_params.mat  ← 生效参数（tau_d=1e-4）
```

`D` 为 27×N 诊断日志，行序见 `pmpc_core/pmpc_step.m` 头注释：
`[ey, epsi, phi, p, LTR, Md_actual, Md_cmd, Md_min, Md_max, Fd×4, Fdes×4,
Mz_cmd, SW, Vx, r, deltaMd, deltaMd_max, QP_OK, candidate_used, w_stab, w_LTR]`
（`ey`、`phi`、`p`、`LTR` 为 SI；`epsi` 为 rad，表中已换算）。
