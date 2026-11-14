# Baseline MPC · 结果

下界基线（`baseline_mpc/`，模型 `pmpc_base.slx`）。与协调控制器的差别只在控制器本身：
被控对象、侧倾自由度、Zeng 鞍结稳定域、道路包络、`|LTR| <= 0.8` 全部同一套。
基线**不控减振器**，四角恒 `M.cdc_fixed_I = 1.0 A`；没有 Md 状态/控制量，所以没有 `tau_d` 时滞模型。
`logVec` 25 行，6–9（Md）与 14–17（Fdes）恒 NaN。

**预测时域已对齐 Proposed**（`pmpc_core/pmpc_init.m` 的默认值）：
DLC / Slalom 用 `Ts=0.10 / Np=10 / Nc=5`（1.0 s 预测窗），FishHook 用 `Ts=0.05 / Np=12 / Nc=6`
（两边本来就一致）。改之前基线是 `0.05/12/6`（0.6 s）——比 Proposed 短 40%，
跑出来的差异分不清是"算法不同"还是"看得更近"。见 `baseline_mpc/run_baseline.m` 里的 switch。

所有数据都是 `run_baseline` 落盘：`D` 25×N、`t`、`Icmd`（四角电流，
`SaveFormat = Structure With Time`——必须和 `run_dlc.m` / `pmpc_mil` 的 `Icmd_log`
同格式，否则存成 MATLAB timeseries，scipy 读出来是一个键名 `None` 的垃圾对象）。
`Icmd.signals.values` 是 N×4，四角各一列。

每个算例旁边还有一个 `<prefix>_params.mat`（字段 `meta`），记录这一次跑**实际生效**的
maneuver / speed / horizon / Q_norm / R_norm / slack_w / cdc_fixed_I / 时间戳——
不依赖任何人的记忆，也不依赖当时的工作区。

## 统一测试矩阵

`epsi_max` 已换算成 deg；`ey_max` / `ey_rms` 是 `max(abs())` 与 RMS；
`Mz_avg` = `mean(abs(Mz_cmd))`；`QP%` 只在**真正跑过 QP 的帧**上统计（提前返回的帧不算失败）。

| 算例 | T [s] | ey_max [m] | ey_rms [m] | epsi_max [°] | LTR_max | SW_max [°] | Mz_avg [N·m] | Mz_max [N·m] | QP% | Icmd 恒定 | horizon (Ts/Np/Nc) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `bl_dlc_70` | 15.00 | 0.5908 | 0.1709 | 4.796 | 0.3265 | 79.44 | 1073.8 | 2774.0 | 100.0 | 1.000 A | 0.10/10/5 |
| `bl_dlc_75` | 15.00 | 0.6386 | 0.1895 | 4.783 | 0.3255 | 99.42 | 841.4 | 2539.2 | 100.0 | 1.000 A | 0.10/10/5 |
| `bl_dlc_80` | 15.00 | 0.8425 | 0.2135 | 5.627 | 0.3258 | 222.35 | 802.6 | 2945.6 | 100.0 | 1.000 A | 0.10/10/5 |
| `bl_sla_80` | 8.00 | 0.8484 | 0.2847 | 3.208 | 0.7740 | 168.37 | 439.0 | 3420.3 | 100.0 | 1.000 A | 0.10/10/5 |
| `bl_fh_80` † | 7.99 | — | — | 25.576 | 1.0000 | 0.00 | 3690.7 | 9076.7 | 100.0 | 1.000 A | 0.05/12/6 |

† FishHook 的 `ey` 没有意义：`Q_norm(5)=0`，控制器本来就不跟路径，表中不列。
`LTR_max = 1.0000` 是**翻车**（外侧轮离地），该工况的作用就是暴露这个。
`SW_max = 0` 也是对的：`M.afs_on=false`、`steer_ext=true`，方向盘由 CarSim 开环机器人给，
基线只输出 AFS 增量（0）；滚转只能靠差动制动，所以 `Mz_avg` 高达 3690 N·m。
`T=7.99` 而不是 8.00：7.0 s 左右车已停住（`Vx -> 0`），之后 `meas.Vx <= 0`，
`pmpc_base_step` 提前返回、不再尝试 QP，所以 800 帧里有 125 帧没有 QP 结果——
这 125 帧不是 QP 失败（真正的 QP 帧 675/675 全成功）。

## ⚠️ 已知口径缺口：基线没有纵向速度环，会掉速

`pmpc_base.slx` 的 CarSim 11 路输入里，`IMP_THROTTLE_ENGINE` 与 `IMP_PCON_BK`
接的是两个 `Constant 0`。协调模型那边这两路来自 `pmpc_core/lon_arbitrate.m`
（速度 PID → 油门/制动），所以** Proposed 保速、基线不保速**。实测：

| 算例 | 起始 Vx | 结束 Vx | 说明 |
|---|---|---|---|
| `bl_dlc_70` | 70.0 | 0.6 | |
| `bl_dlc_75` | 75.0 | 4.9 | |
| `bl_dlc_80` | 80.0 | 2.4 | |
| `bl_sla_80` | 80.0 | 34.4 | |
| `bl_fh_80` | 80.0 | 0.0 | 7.0 s 已停住 |
| `nc_sla_verify`（nocmp，同工况） | 80.0 | 69.7 | 对照：协调侧保速 |

后果：DLC 的变道段（X ≈ 80–132 m）实际是在 58–62 km/h 下跑完的，不是 70/75/80；
Slalom 后半段只剩 40 多 km/h。**这一批数字只能证明链路通、约束在建，
不能直接当成"算法差异"和 Proposed 比。** 修法是给基线补上同一个速度环，
要改 `.slx`（把两路 `Constant 0` 换成 `lon_arbitrate` 接线），本轮未做。

## 历史数据（不要用作对比）

| 文件 | 状态 |
|---|---|
| `bl_dlc70_old.mat` | 旧手工模型（`build_pmpc_base` 修好之前）的 DLC 70 运行，方向盘打到 720° 极限、已丢车。仅作"改了什么"的对照 |
| `bl_verify.mat` | DLC 70 × 3 s 冒烟，horizon 0.05/12/6。只用于验证 Icmd 恒定与 NaN 槽位 |
| `bl_sla_verify.mat` | **旧时域（0.05/12/6），已失效**，被 `bl_sla_80`（0.10/10/5）取代；留档不复用 |

## 参数与复现

参数全部取自 `pmpc_base_init` 的 case 默认值，没有为任何算例调过权重：

| 工况 | Q_norm | R_norm | slack_w | mu |
|---|---|---|---|---|
| DLC | `[0 0 10 0 50 20]` | `[100 200]` | `0.2e4 ×4` | 0.4 |
| Slalom | `[0 0 5 0 50 5]` | `[0.1 500]` | `[5 5 5 5]` | 0.85 |
| FishHook | `[0 10 20 0 0 0]` | `[100 500]` | `[100 100 1e4 1e-3]` | 0.9 |

```matlab
run_baseline('dlc',      70, struct('stopTime',15,'prefix','bl_dlc_70'));
run_baseline('dlc',      75, struct('stopTime',15,'prefix','bl_dlc_75'));
run_baseline('dlc',      80, struct('stopTime',15,'prefix','bl_dlc_80'));
run_baseline('slalom',   80, struct('stopTime',8, 'prefix','bl_sla_80'));
run_baseline('fishhook', 80, struct('stopTime',8, 'prefix','bl_fh_80','cargo',170));
```
