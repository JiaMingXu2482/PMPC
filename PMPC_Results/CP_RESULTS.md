# Proposed_Comp（ctrl-comp）结果记录

控制器：`pmpc_core`，7 状态 `[vy, r, phi, p, ey, epsi, Md]` / 3 控制量 `[delta, Mz, Md_cmd]`，
CDC 电流由控制器给（`nlcsnn/inv_current_impl_10ms.m` 的 `SH_MODE = 0`）。

## 主对照口径：`tau_d_mode = 1`（固定 tau）

2026-11-14 决定：**comp 的主对照改用固定 tau，不再用时变表。**

| 工况 | `tau_d` [s] | 来源 | 其它补偿项 |
|---|---|---|---|
| DLC 70 / 75 / 80 | **0.01** | `pmpc_init` 全局默认，case 未覆盖 | — |
| Slalom 80 | **0.20** | `pmpc_init` case 2，`controller==3` 分支 | `preview_L=preview_alpha=0` |
| FishHook 80 | **0.025** | `run_fishhook('comp25',…)` | `preview_L=0.025`、`preview_alpha=1` |

每个算例都显式确认实际生效值（存进 `_params.mat`，跑完从磁盘读回核对，见 §5）。
`tau_d_mode = 2` 的代码与结果**保留不删**，作为"按实测滞后建模时补偿收益不显著"
的说明性证据，见 §2。

复现本文所有数字：

```
matlab -batch "addpath('tools'); cp_case_metrics; taud_backid"
```

（`cp_case_metrics` 默认就是下面这张固定 tau 的主表。）

---

## 1. 主表（固定 tau，统一测试矩阵）

参数全取 `pmpc_init` case 默认值，未调任何权重。
`Md_avg`/`Mz_avg` 为 `mean(abs())`；`epsi_max` 由 rad 换算；`生效 tau_d` 取 `.mat`
里的 `tau_dtrace`（`run_*.m` 用 `tau_d_effective.m` 按本算例实测口径复算）。

| 算例 | T [s] | ey_max [m] | ey_rms [m] | epsi_max [deg] | LTR_max | SW_max [deg] | Mz_avg [N·m] | Mz_max [N·m] | Md_avg [N·m] | Md_max [N·m] | Icmd 范围 [A] | Vx 范围 [km/h] | 生效 tau_d [s] | QP% |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `cp_dlc_70_fix` — DLC 70, µ0.4 | 10.64 | 0.6373 | 0.2028 | 6.77 | 0.325 | 68.1 | 666 | 2172 | 359 | 2431 | 0.00–1.60 | 66.7–70.0 | 0.0100 | 100.0 |
| `cp_dlc_75_fix` — DLC 75, µ0.4 | 9.94 | 0.6484 | 0.2305 | 7.03 | 0.330 | 122.4 | 856 | 2124 | 369 | 2320 | 0.00–1.60 | 71.6–75.0 | 0.0100 | 100.0 |
| `cp_dlc_80_fix` — DLC 80, µ0.4 | 9.32 | 0.8658 | 0.3031 | 7.90 | 0.330 | 275.9 | 847 | 2045 | 370 | 2385 | 0.00–1.60 | 76.4–80.0 | 0.0100 | 100.0 |
| `cp_sla_80_fix` — Slalom 80, µ0.85 | 8.00 | 0.7433 | 0.4433 | 3.44 | 0.688 | 181.4 | 1939 | 5190 | 1622 | 5260 | 0.00–1.60 | 71.0–80.0 | 0.2000 | 100.0 |
| `cp_fh_80_fix` — FishHook 80, µ0.9, cargo 170 | 7.99 | 14.7167 | 5.9887 | 52.61 | 0.804 | 0.0 | 4506 | 7628 | 611 | 4292 | 0.00–1.60 | 58.9–80.0 | 0.0250 | 100.0 |

```matlab
run_dlc(3,70,struct('stopTime',15,'prefix','cp_dlc_70_fix','tau_d_mode',1))   % 75 / 80 同理
run_slalom(3,80,struct('stopTime',8,'prefix','cp_sla_80_fix','tau_d_mode',1))
run_fishhook('comp25',3,struct('stopTime',8,'prefix','cp_fh_80_fix','cargo',170))
```

### 1.1 口径说明（读表前必看）

- **QP%**：`D(24)=1` 收敛、`0` 未收敛、`-1` 表示**本拍没跑 QP**（`pmpc_step` 在
  无效测量/无参考路径时提前返回，留 `-1` 当哨兵）。分母只数 `D(24)>=0` 的帧，
  `QP% = 100*sum(D(24)>0.5)/sum(D(24)>=0)`。本次五个算例 `-1` 帧数全为 0
  （1065/995/933/801/800 帧），所以数值与旧口径一致。
- **Vx 范围**必须一起看：基线那支没有纵向速度环，历史上有 DLC 掉到 0.6 km/h 的算例。
  本文五个算例的 Vx 下界与目标差 −3.3 / −3.4 / −3.6 / −9.0 / −21.1 km/h，
  都是机动过程中的正常掉速，不是车速失控。
- **DLC 的 T 只有 9.3–10.6 s，不是 15 s**：`run_dlc` 把 CarSim `SSTOP` 设成 200 m，
  到里程就停。三个车速比的是**同一段 200 m**，不是同一时长。
- **FishHook 的 SW_max = 0**：`afs_on=false` + `steer_ext=true`（CarSim 转向机器人给方向盘），
  `D(19)` 只记 AFS 增量，所以恒 0，不要和 DLC/Slalom 的 SW 比。
- **FishHook `LTR_max=0.804` 略超 0.8**：LTR 是软约束（带松弛），该工况鱼钩后侧倾接近
  极限，0.004 的越界量属松弛允许范围；Slalom 的 `LTR_max=0.688 < 0.75`，满足验收。

### 1.2 与 B 支（noComp，`tau_d=1e-4`）同算例对照

同一套矩阵、同一 stopTime，只差时滞口径（B 支数字取自
`Proposed_noComp/PMPC_Results/NC_RESULTS.md`）：

| 算例 | nc `ey_max`（1e-4） | cp `ey_max`（mode2，17–20 ms） | cp `ey_max`（固定 tau） | 固定 tau 相对 nc |
|---|---|---|---|---|
| DLC 70 | 0.6369 | 0.6363 | 0.6373 | +0.06% |
| DLC 75 | 0.6483 | 0.6484 | 0.6484 | +0.02% |
| DLC 80 | 0.8656 | 0.8661 | 0.8658 | +0.02% |
| **Slalom 80** | **0.7735** | 0.7724 | **0.7433** | **−3.9%** |
| **FishHook 80** | **14.8720** | 14.8400 | **14.7167** | **−1.0%** |

- **DLC 三个车速上，三种时滞口径的结果差 ≤0.06%，等于没有区别**：DLC 的减振器几乎不激励
  （`Md` 峰值 2.4 kN·m，`vmag` 中位数不到 10 mm/s），且 0.01 s 与 1e-4 在 0.1 s 采样下
  本身只差 1/10 拍。**DLC 不能用来论证时滞补偿的收益**，只能证明链路一致、无副作用。
- **收益只在大侧向工况出现**：Slalom −3.9%、FishHook −1.0%。这是主对照成立的依据。

---

## 2. 时变口径（`tau_d_mode = 2`）的负结果

时变表按实测压缩速度查 0.017 / 0.019 / 0.022 s。矩阵跑下来：

- 四个算例的 `tau_dtrace` **中位数全是 0.017 s**，最大值仅 0.020 s
  （DLC 0.0170–0.0180、Slalom 0.0170–0.0200、FishHook 0.0170–0.0196）。
  时变表**几乎从不越过第一个断点**（50 mm/s），实质退化成"常数 17 ms"。
- 17 ms 与 noComp 的 1e-4 在闭环里不可区分：Slalom `ey_max` 0.7724 vs 0.7735（差 0.14%），
  比固定 0.20 s 的 0.7433 差 3.8%。
- 也就是说：**按实测物理滞后（17 ms）建模，补偿收益基本为零**；
  带来收益的是 0.20 s 这个"等效整定值"，它把预瞄、力界预测、离散化误差一起吸收进去
  （§3 的反辨识给出的实测表观滞后只有 10–20 ms，比 0.20 s 小一个量级）。

同一算例只差口径的三个跑法（Slalom 80、8 s，同一 stopTime）：

| 口径 | 生效 tau_d | ey_max [m] | SW_max [deg] | 说明 |
|---|---|---|---|---|
| noComp | 1e-4 | 0.7735 | 206.0 | 不建模时滞 |
| comp 时变（mode=2） | 0.0170–0.0200（中位 0.017） | 0.7724 | 209.9 | **负结果**：退化成常数 17 ms |
| comp 固定（mode=1） | 0.20 | **0.7433** | 181.4 | **主对照** |

mode=2 的五个算例（`cp_dlc_70/75/80`、`cp_sla_80`、`cp_fh_80`）**保留在库里不删**：

| 算例 | ey_max | LTR_max | Vx 范围 | 生效 tau_d |
|---|---|---|---|---|
| `cp_dlc_70` | 0.6363 | 0.325 | 66.7–70.0 | 0.01700–0.01744 |
| `cp_dlc_75` | 0.6484 | 0.330 | 71.6–75.0 | 0.01700–0.01744 |
| `cp_dlc_80` | 0.8661 | 0.330 | 76.4–80.0 | 0.01700–0.01799 |
| `cp_sla_80` | 0.7724 | 0.680 | 69.6–80.0 | 0.01700–0.02003 |
| `cp_fh_80` | 14.8400 | 0.799 | 58.8–80.0 | 0.01700–0.01964 |

`pmpc_core/tau_d_effective.m` 保持现状（mode=1 / mode=2 两个分支都在），本轮不删代码。

---

## 3. tau_d 口径记录与反辨识（支撑 §2 的结论）

> 本节用 `cp_sla_mode1` / `cp_sla_mode2` 两个 8 s 算例（§2），与 §1 的矩阵无依赖关系。

### 3.1 A 方案复盘：那次"验证"是无效的

上一轮想"从磁盘侧证 tau_d"，做法是写一个分析脚本，把
`vmag = 1000*mean(abs(vd))` 的代换表（0.017 / 0.019 / 0.022）**硬编码在脚本里**，
再对各算例的 `CmpRD_*` 套一遍，打印 `tau_d` 范围。

错在哪里：那张表是 **mode 2 的口径**。脚本对 mode 1 的算例（`.mat` 里
`tau_d=0.20`、`tau_d_mode=1`）也照样套同一张表，于是给 mode 1 的 run
打印出 `tau_d = 0.017……0.017`，看起来"验证通过"。

这不是验证，是**用结论生成证据**：脚本里预置了想看到的数值，输出当然和它一致。
真正的口径信息（这次跑的是 0.20 还是 1e-4）根本没进过数据。

### 3.2 C 方案定性：这是口径记录，不是验证

现行做法（本轮提交的代码）：`run_dlc / run_slalom / run_fishhook` 在存盘时，
用**这次运行自己的口径**（`opts` 覆盖之后的 `P.M.tau_d` 与 `P.M.tau_d_mode`）
加上**这次运行实测的**四角压缩速度 `CmpRD_*`（取自 CarSim `.vs`），
调用控制器自己的 `pmpc_core/tau_d_effective.m` 逐拍复算，存进 `.mat` 的
`tau_dtrace`（同时存 `tau_d_mode` / `tau_d_const` / `vd_ctrl`）。

> **这是口径记录（traceability），不是验证。它回答"这次用的是哪个值"，
> 不回答"这个值对不对"。**

它解决的问题是"事后能查"，而不是"证明标定正确"。上表里 mode 1 记 0.200、
mode 2 记 0.0170–0.0200，就是它给出的信息。

顺带记一条否掉的路：**不给 `.slx` 加端口回显 tau_d**。
`prepare_step.m` 里
`zeta = [meas.Vy; meas.r; meas.phi; meas.p; ey; epsi; Bd'*Fd; state.u_prev]`，
第 7 维每拍由 CarSim 实测 `Md = Bd'*Fd` 重置，**从来不在闭环里积分**；
`tau_d` 只进预测矩阵 `A(7,7) = -1/tau_d`，作用是"预测时提前多少给力"，
不留任何状态痕迹。为回显一个常数去改三份 worktree 的 `.slx`，收益为零、分叉风险为正。
另外 `pmpc_step` 的 `logVec` 是 Stateflow 数据对象，尺寸锁死 `[27 1]`，
加不了第 28 列（试过，编译直接报错）。

### 3.3 真正该做的：用 logVec 反辨识执行器时间常数

`D` 里本来就有反辨识需要的两个量（1-based 行号）：

| 行 | 含义 | 来源 |
|---|---|---|
| `D(6,:)` | `Md_actual` 实测 | `c.zeta(7) = Bd'*Fd` |
| `D(7,:)` | `Md_cmd` 本拍下发 | `stateNext.u_prev(3)`（`pmpc_step.m:160` 覆盖后的值，不是 `:68` 的上一拍初值） |

用到的两个文件：`cp_sla_mode1.mat`（= `cp_sla_80_fix`，固定 0.20）与
`cp_sla_mode2.mat`（= `cp_sla_80`，时变 17–20 ms），同一算例同一 stopTime。

一阶滞后 `dMd_actual/dt = (Md_cmd - Md_actual)/tau`，中心差分可得

```
tau(k) = (Md_cmd(k) - Md_actual(k)) / (dMd_actual/dt)(k)
```

实现：`tools/taud_backid.m`。只保留 `0 < tau < 1 s` 的样本；分母在命令平台期
接近 0，所以只统计 `|Md_cmd - Md_actual|` 超过门限的样本，并**打印门限扫描表**
——门限一动分位数就动，只报一个数等于藏证据。

**Slalom 8 s（801 拍）结果，tau [ms]，q10/25/50/75/90：**

| 门限（× 峰值 \|Md\|） | mode1 n | mode1 分位数 | mode2 n | mode2 分位数 |
|---|---|---|---|---|
| 0（不过滤） | 546 | 0.8 / 1.3 / 1.7 / 3.0 / 11.6 | 549 | 0.9 / 1.3 / 1.8 / 2.9 / 7.9 |
| 0.2% | 311 | 1.4 / 1.7 / 2.0 / 3.6 / 20.6 | 326 | 1.4 / 1.6 / 2.0 / 3.2 / 9.5 |
| 0.5% | 106 | 1.9 / 2.2 / 3.8 / 20.1 / 117.4 | 120 | 1.9 / 2.2 / 2.9 / 5.1 / 20.7 |
| **0.8%** | **50** | 2.5 / 3.5 / 12.2 / 70.0 / 164.1 | **54** | 2.6 / 3.1 / 5.1 / 10.3 / 36.9 |
| **1%（默认）** | **41** | 2.9 / 5.2 / 33.7 / 99.3 / 208.0 | **35** | 3.2 / 4.6 / 7.5 / 21.4 / 57.8 |
| 2% | 24 | 20.3 / 36.9 / 64.6 / 143.9 / 231.8 | 11 | 6.6 / 7.4 / 9.4 / 23.4 / 76.1 |

与主 agent 的独立试算对照（mode1 49/801、中位 13.3 ms；mode2 54/801、中位 7.8 ms）：
**计数在 0.8% 门限处几乎重合（50/54 vs 49/54），中位在 1% 门限处重合（7.5 vs 7.8 ms）**；
逐点分位数的差异来自门限选择，不是算错。结论对门限不敏感：

1. **100 Hz 采样下约 10 ms 是噪声地板。** 不过滤时中位只有 1.7 / 1.8 ms，
   那是"命令与实测几乎同步"的量化假象，不是物理滞后；任何与 10 ms 同量级的
   点估计都不能当物理量读。
2. **表观滞后 ≤ 20 ms 量级，分辨不出更细。** mode2 在 0.8–1% 门限下中位 5.1–7.5 ms、
   q75 10–21 ms，被采样分辨率压住；**因此 mode2 的 17–22 ms 标定量级是对的**，
   想区分 17 / 19 / 22 三个点得把采样提到 500 Hz–1 kHz。
3. **mode1 的 0.20 s 不是执行器滞后。** 同一套被控对象，实测表观滞后中位只有
   12–34 ms（0.8–1% 门限），比 0.20 s 小一个量级；它的重尾（q90 164–208 ms）
   也说明这个数不是某个稳定时间常数。0.20 是**把预瞄（`preview_L`）、
   力界预测、离散化误差一起吸收的等效补偿量**——这也解释了为什么它比
   物理标定值 17–22 ms 大一个量级。
4. mode1 的表观滞后系统性大于 mode2（中位 12–34 vs 5–8 ms）。物理上也自洽：
   控制器假设执行器慢，就会提前给力，残余误差 `cmd-act` 被人为撑大，
   反辨识读到的滞后随之变大。

**这两个 DLC 3 s 算例不能做反辨识**：`Md` 峰值仅 ~164 N·m，1% 门限落到 ~1.6 N·m，
分子全是噪声（该门限下中位 196 / 108 ms，纯属除以小量的伪值）。反辨识只用
Slalom 这类减振器被真正激励的算例。

---

## 4. 文件

```
PMPC_Results/
  CP_RESULTS.md
  DLC_mu0.4_interp/cp_dlc_70_fix.mat + _params.mat      ← §1 主表
  DLC_mu0.4_interp/cp_dlc_75_fix.mat + _params.mat
  DLC_mu0.4_interp/cp_dlc_80_fix.mat + _params.mat
  Slalom_mu0.85_interp/cp_sla_80_fix.mat + _params.mat
  FishHook_80_mu0.9_cargo170_newplant/cp_fh_80_fix.mat + _params.mat
  DLC_mu0.4_interp/cp_dlc_70/75/80.mat + _params.mat    ← §2 mode=2（负结果，保留）
  Slalom_mu0.85_interp/cp_sla_80.mat + _params.mat
  FishHook_80_mu0.9_cargo170_newplant/cp_fh_80.mat + _params.mat
  Slalom_mu0.85_interp/cp_sla_mode2.mat                 ← §2/§3 对照（= cp_sla_80）
  Slalom_mu0.85_interp/cp_sla_mode1.mat                 ← 与 cp_sla_80_fix 同口径同结果（历史重复）
  DLC_mu0.4_interp/cp_dlc_mode1.mat / cp_dlc_mode2.mat  ← 3 s 自检，不用于对比
tools/
  cp_case_metrics.m    ← §1/§2 的表（逐算例打印，与本文数字一致）
  taud_backid.m        ← §3 的反辨识（公式、适用条件、噪声地板写在头注释）
  cp_write_params.m    ← 写 <prefix>_params.mat（run_*.m 调用）
  vs_read.m            ← 读 CarSim .vs/.vsb（给 run_*.m 复算 tau_dtrace 用）
pmpc_core/
  tau_d_effective.m    ← mode1/mode2 的唯一口径来源（build_prediction 与 run_*.m 共用）
```

`.mat` 内：`D`（27×N，行序见 `pmpc_core/pmpc_step.m` 头注释）、`t`、`Icmd`（四角 1 kHz 电流）、
`tau_dtrace` / `tau_d_mode` / `tau_d_const` / `vd_ctrl`。

`_params.mat` 内（`tools/cp_write_params.m` 写）：`maneuver`、`speed`、`horizon=[Ts Np Nc]`、
`tau_d`、`tau_d_mode`、`tau_d_const`、`tau_d_default`、`controller`、`Q_norm`、`R_norm`、
`slack_w`、`SH_MODE`、`timestamp`，以及各工况补充的 `mu` / `stopTime` / `variant` / `cargo`。

---

## 5. 验收核对（从磁盘读回，`*_fix` 五个算例）

| 算例 | `tau_d_mode` | `.mat` 的 `tau_d_const` | `_params.mat` 的 `tau_d` | `tau_dtrace` | 唯一值数 |
|---|---|---|---|---|---|
| cp_dlc_70_fix | 1 | 0.01 | 0.01 | 0.01000–0.01000 | 1 |
| cp_dlc_75_fix | 1 | 0.01 | 0.01 | 0.01000–0.01000 | 1 |
| cp_dlc_80_fix | 1 | 0.01 | 0.01 | 0.01000–0.01000 | 1 |
| cp_sla_80_fix | 1 | 0.20 | 0.20 | 0.20000–0.20000 | 1 |
| cp_fh_80_fix | 1 | 0.025 | 0.025 | 0.02500–0.02500 | 1 |

FishHook 的 `tau_d_default` 记的是 0.0001（`pmpc_init` case 4 的默认），
实际生效值来自 `run_fishhook('comp25',…)` 的 0.025 —— 两者不一致是预期的，
`tau_d` / `tau_d_const` 才是生效值。
