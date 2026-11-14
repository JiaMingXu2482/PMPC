# Baseline MPC · 结果

下界基线（`baseline_mpc/`，模型 `pmpc_base.slx`）。与协调控制器的差别只在控制器本身：
被控对象、侧倾自由度、Zeng 鞍结稳定域、道路包络、`|LTR| <= 0.8` 全部同一套。
基线**不控减振器**，四角恒 `M.cdc_fixed_I = 1.0 A`；没有 Md 状态/控制量，所以没有 `tau_d` 时滞模型。
`logVec` 25 行，6–9（Md）与 14–17（Fdes）恒 NaN。

**预测时域已对齐 Proposed**（`pmpc_core/pmpc_init.m` 的默认值）：
DLC / Slalom 用 `Ts=0.10 / Np=10 / Nc=5`（1.0 s 预测窗），FishHook 用 `Ts=0.05 / Np=12 / Nc=6`。

所有数据都是 `run_baseline` 落盘：`D` 25×N、`t`、`Icmd`（四角电流，
`SaveFormat = Structure With Time`，与 `run_dlc.m` / `pmpc_mil` 的 `Icmd_log` 同格式；
`Icmd.signals.values` 是 N×4）。每个算例另有 `<prefix>_params.mat`（字段 `meta`），
记录这一次跑**实际生效**的 maneuver / speed / horizon / Q_norm / R_norm / slack_w /
cdc_fixed_I / 时间戳。

## 纵向接口（与 nc / cp 共用同一份实现）

`pmpc_base.slx` 的 CarSim 后两路输入不再是常数，而是和 `pmpc_mil` 的
`MPC_Controller（100Hz）` 内部**逐块一致**的速度环：

```
v_target[km/h] ── /3.6 ──┐
                         ├─ Sum(+,-) ─→ PID(PI, P=0.2, I=0.005, 输出限 ±1,
        meas.Vx ─────────┘               anti-windup = back-calculation, Kt=Kb=1)
                                          │
   pmpc_base_step 的 Tb(4) ───────────────┤
                                          ↓
                              lon_arbitrate(u_lon, Tb)
                                          │
                        throttle ─→ CARSIM_IN(10) = IMP_THROTTLE_ENGINE
                        p_brake  ─→ CARSIM_IN(11) = IMP_PCON_BK
                        u_applied ─→ Unit Delay ─→ PID 的 Tracking 口
```

- `lon_arbitrate.m` **没有复制到 `baseline_mpc/`**，`run_baseline.m` 与
  `pmpc_base_setup.m` 用 `addpath(../pmpc_core)` 引用同一份文件，三边跑的是同一段代码。
- PID 参数与 Proposed 两支完全相同（`P=0.2 / I=0.005 / N=100 / 限幅 ±1`），没有为基线调过。
- 采样时间全部显式 `0.01`。`pmpc_mil` 靠外层 100 Hz 子系统继承，这里模型根层是 CarSim 的
  1 ms，不写死会按 1 kHz 跑：`lon_arbitrate` 内部的 `DT=0.01` 和 PID 的 `I` 都按 10 ms 定的。
- `lon_arbitrate` 的规则（`TB_ON=150` / `TB_OFF=50` / `HOLD_T=0.15` / `RATE_UP=1.5`）是
  **稳定性优先**的设计：DYC 一旦激活就不额外制动、油门按斜率慢慢放开、油门权限不低于 0.5。
  也就是说差动制动期间速度本来就会被牺牲，这一条对三边同样生效。

## 统一测试矩阵（v2，含速度环）

`epsi_max` 已换算成 deg；`ey_max` / `ey_rms` 是 `max(abs())` 与 RMS；
`Mz_avg` = `mean(abs(Mz_cmd))`；`QP%` 分母只算 `D(24) >= 0` 的帧
（`D(24) == -1` 是"没尝试"，比如 `meas.Vx <= 0` 提前返回）。

| 算例 | T [s] | ey_max [m] | ey_rms [m] | epsi_max [°] | LTR_max | SW_max [°] | Mz_avg [N·m] | Mz_max [N·m] | QP% | Icmd 恒定 | horizon |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `bl_dlc_70_v2` | 10.64 | 0.6178 | 0.2136 | 6.462 | 0.3427 | 71.27 | 728.9 | 2201.6 | 100.0 | 1.000 A | 0.10/10/5 |
| `bl_dlc_75_v2` | 9.94 | 0.7604 | 0.3107 | 6.874 | 0.3425 | 268.78 | 889.6 | 2193.1 | 100.0 | 1.000 A | 0.10/10/5 |
| `bl_dlc_80_v2` | 9.38 | 1.9966 | 0.8103 | 8.973 | 0.3378 | 720.00 | 774.7 | 2057.6 | 100.0 | 1.000 A | 0.10/10/5 |
| `bl_sla_80_v2` | 8.00 | 0.8936 | 0.5300 | 5.024 | 1.0000 | 433.93 | 2126.5 | 5362.6 | 100.0 | 1.000 A | 0.10/10/5 |
| `bl_fh_80_v2` † | 7.99 | — | — | 60.200 | 1.0000 | 0.00 | 797.1 | 5335.3 | 100.0 | 1.000 A | 0.05/12/6 |

† FishHook 的 `ey` 没有意义（`Q_norm(5)=0`，控制器本来就不跟路径）。
`SW_max = 0` 是对的：`M.afs_on=false`、`steer_ext=true`，方向盘由 CarSim 开环机器人给。

要点：
- **DLC 80 已丢车**：`SW_max` 顶到 720°（机械极限）、`ey_max` 2.0 m。
  mu=0.4 + 80 km/h 超出这套基线的能力，不是数值问题。
- **Slalom 80 的 `LTR_max` 到 1.0000**：满载侧向载荷转移到一侧极限，即内侧轮离地。
  同工况 `nc` 是 0.682。**上一版 0.774 是假象**——那时车已经掉到 34 km/h。
- **FishHook 仍然 `LTR_max = 1.0000`**：速度环没有把它救回来（如实记录，没有调参去凑）。
- `T` 不是 `stopTime`：DLC 三次都在 9.4–10.6 s 被 CarSim 自己终止
  （`Termination at simulation time = …`，数据集跑完了），不再像上一版那样跑到 15 s 才停。

## 车速度对比（新旧两版）

| 算例 | v1 无速度环 | v2 有速度环 | 目标 |
|---|---|---|---|
| DLC 70 | 70.0 → **0.6** | **66.8 .. 70.0** | 70 |
| DLC 75 | 75.0 → **4.9** | **71.7 .. 75.0** | 75 |
| DLC 80 | 80.0 → **2.4** | **74.3 .. 80.0** | 80 |
| Slalom 80 | 80.0 → **34.4** | **70.6 .. 80.0** | 80 |
| FishHook 80 | 80.0 → **0.0**（7.0 s 停住） | **68.1 .. 80.0** | 80 |
| `nc_sla_verify`（对照） | — | 69.7 .. 80.0 | 80 |

## v1 矩阵（无速度环，已弃用）

保留作为对照，说明速度环到底改变了什么。**不要用这一批做三方对比**：
车在掉速，DLC 的变道段实际是在 58–62 km/h 跑的。

| 算例 | T [s] | ey_max | ey_rms | epsi_max [°] | LTR_max | SW_max [°] | Mz_avg | Mz_max | QP% |
|---|---|---|---|---|---|---|---|---|---|
| `bl_dlc_70` | 15.00 | 0.5908 | 0.1709 | 4.796 | 0.3265 | 79.44 | 1073.8 | 2774.0 | 100.0 |
| `bl_dlc_75` | 15.00 | 0.6386 | 0.1895 | 4.783 | 0.3255 | 99.42 | 841.4 | 2539.2 | 100.0 |
| `bl_dlc_80` | 15.00 | 0.8425 | 0.2135 | 5.627 | 0.3258 | 222.35 | 802.6 | 2945.6 | 100.0 |
| `bl_sla_80` | 8.00 | 0.8484 | 0.2847 | 3.208 | 0.7740 | 168.37 | 439.0 | 3420.3 | 100.0 |
| `bl_fh_80` | 7.99 | 10.1799 | 7.5119 | 25.576 | 1.0000 | 0.00 | 3690.7 | 9076.7 | 100.0 ‡ |

‡ 旧口径（`D(24) > 0.5`）会算成 84.4%——那 125 帧是 `Vx <= 0` 提前返回，压根没跑 QP。

## 其它历史文件

| 文件 | 状态 |
|---|---|
| `bl_v2_smoke` | 速度环接好后的 3 s 冒烟（DLC 70，Vx 68.3..70.0），只用于确认接线通 |
| `bl_dlc70_old.mat` | 旧手工模型（`build_pmpc_base` 修好之前）的运行，方向盘打到 720° 极限、已丢车。仅作对照 |
| `bl_verify.mat` | DLC 70 × 3 s 冒烟，horizon 0.05/12/6 且无速度环 |
| `bl_sla_verify.mat` | 旧时域（0.05/12/6）、无速度环，已被 `bl_sla_80` / `bl_sla_80_v2` 取代 |

## 参数与复现

参数全部取自 `pmpc_base_init` 的 case 默认值，没有为任何算例调过权重：

| 工况 | Q_norm | R_norm | slack_w | mu |
|---|---|---|---|---|
| DLC | `[0 0 10 0 50 20]` | `[100 200]` | `0.2e4 ×4` | 0.4 |
| Slalom | `[0 0 5 0 50 5]` | `[0.1 500]` | `[5 5 5 5]` | 0.85 |
| FishHook | `[0 10 20 0 0 0]` | `[100 500]` | `[100 100 1e4 1e-3]` | 0.9 |

```matlab
run_baseline('dlc',      70, struct('stopTime',15,'prefix','bl_dlc_70_v2'));
run_baseline('dlc',      75, struct('stopTime',15,'prefix','bl_dlc_75_v2'));
run_baseline('dlc',      80, struct('stopTime',15,'prefix','bl_dlc_80_v2'));
run_baseline('slalom',   80, struct('stopTime',8, 'prefix','bl_sla_80_v2'));
run_baseline('fishhook', 80, struct('stopTime',8, 'prefix','bl_fh_80_v2','cargo',170));
```
