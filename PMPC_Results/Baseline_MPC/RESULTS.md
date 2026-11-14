# Baseline MPC · 结果

下界基线（`baseline_mpc/`，模型 `pmpc_base.slx`）。与协调控制器的差别只在控制器本身：
被控对象、侧倾自由度、Zeng 鞍结稳定域、道路包络、`|LTR| <= 0.8` 全部同一套。
基线**不控减振器**，四角恒 `M.cdc_fixed_I = 1.0 A`；没有 Md 状态/控制量，所以没有 `tau_d` 时滞模型。
`logVec` 25 行，6–9（Md）与 14–17（Fdes）恒 NaN。

所有数据都是 `run_baseline` 落盘：`D` 25×N、`t`、`Icmd`（timeseries，四角电流），
与 `run_dlc.m` 的落盘格式一致。

## 算例

| 算例 | T [s] | ey_max [m] | ey_rms [m] | epsi_max [°] | LTR_max | SW_max [°] | Mz_avg [N·m] | Mz_max [N·m] | Md_max | QP % | t_end [s] | Icmd 恒定 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `bl_verify` DLC 70 | 3.00 | 0.0015 | 0.0003 | 0.02 | 0.011 | 3.35 | 2.5 | 38.3 | — | 100.0 | 3.00 | 是，1.000 A |
| `bl_sla_verify` Slalom 80 | 8.00 | 0.9152 | 0.3709 | 3.96 | 0.8004 | 150.60 | 800.9 | 3675.5 | — | 100.0 | 8.00 | 是，1.000 A |
| `bl_dlc70_old` DLC 70 † | 10.78 | 7.6487 | 2.9699 | 18.10 | 0.3689 | 720.00 | 578.2 | 2033.4 | — | 100.0 | 10.78 | 未落盘 |

`Md_max` 对基线恒为 `—`（没有 Md 这条链路，`logVec(6)=NaN`）。

† `bl_dlc70_old` 是**旧模型**的产物（2026-11-14 19:14，手工搭的 `pmpc_base.slx`，
在 `build_pmpc_base` 修好之前）。它 ey 7.65 m、方向盘打到 720° 机械极限，
说明那次运行已经丢车，**不可用于三方对比**，只作为“改了什么”的对照保留。
删掉它没有损失——需要 DLC 全时长数据时重跑 `run_baseline('dlc',70,struct('stopTime',15,'prefix','bl_dlc_70'))`。

## 备注

- `bl_verify` 只有前 3 s，车还在直道上、机动尚未开始，三个控制器在这一段的数字必然一样，
  **没有对比价值**，它的用途只是“模型能跑、约束在建、Icmd 恒定”的自检。
- `bl_sla_verify` 是第一个可用于三方对比的算例。LTR_max = 0.8004 略高于 0.8：
  约束作用在**预测** LTR 上，`logVec(5)` 是 CarSim 实测 `Fz` 算出来的，
  超出量 5e-4 属于正常口径差；要点是这个算例里 LTR 约束**真的顶到了边界**
  （DLC 3 s 那行 LTR 只有 0.011，约束完全没被激活）。
- 参数一律用 `pmpc_base_init` 的 case 默认值，没有为这三个算例调过任何权重：
  DLC 70 → `Q=[0 0 10 0 50 20]`、`R=[100 200]`、`slack_w=0.2e4`；
  Slalom 80 → `Q=[0 0 5 0 50 5]`、`R=[0.1 500]`、`slack_w=[5 5 5 5]`；
  两者共用 `Ts=0.05`（`run_baseline` 的 horizon）、`Np=12`、`Nc=6`、`Ts_exec=0.01`。
- 复现：
  ```
  run_baseline('dlc',70,   struct('stopTime',3,'prefix','bl_verify'))
  run_baseline('slalom',80,struct('stopTime',8,'prefix','bl_sla_verify'))
  ```
