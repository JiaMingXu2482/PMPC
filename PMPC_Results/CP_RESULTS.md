# Proposed_Comp（ctrl-comp）结果记录

控制器：`pmpc_core`，7 状态 `[vy, r, phi, p, ey, epsi, Md]` / 3 控制量 `[delta, Mz, Md_cmd]`，
CDC 电流由控制器给（`nlcsnn/inv_current_impl_10ms.m` 的 `SH_MODE = 0`）。
**补偿时滞**：`tau_d_mode = 2`（时变，`M.tau_d` 被忽略），查表在 `pmpc_core/tau_d_effective.m`。
每个算例都显式传 `tau_d_mode = 2`：`pmpc_init` 的 case 默认是 `mode = 1`，
不显式传就会落回常数口径（DLC 0.01 / Slalom 0.20 / FishHook 0.025）。

复现本文所有数字（矩阵表 / 口径对照 / 反辨识）：

```
matlab -batch "addpath('tools'); cp_case_metrics; taud_backid"
```

单个算例的口径快照在同目录的 `<prefix>_params.mat` 里（字段见 §4）。

---

## 1. 统一测试矩阵（2026-11-14）

参数一律取 `pmpc_init` 的 case 默认值，未调任何权重；每个算例显式 `tau_d_mode=2`。

| 算例 | T [s] | ey_max [m] | ey_rms [m] | epsi_max [deg] | LTR_max | SW_max [deg] | Mz_avg [N·m] | Mz_max [N·m] | Md_avg [N·m] | Md_max [N·m] | Icmd 范围 [A] | Vx 范围 [km/h] | 生效 tau_d [s] | QP% |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `cp_dlc_70` — DLC 70, µ0.4 | 10.63 | 0.6363 | 0.2026 | 6.81 | 0.325 | 69.1 | 674 | 2166 | 359 | 2409 | 0.00–1.60 | 66.7–70.0 | 0.01700–0.01744 | 100.0 |
| `cp_dlc_75` — DLC 75, µ0.4 | 9.94 | 0.6484 | 0.2305 | 7.03 | 0.330 | 122.4 | 856 | 2124 | 369 | 2324 | 0.00–1.60 | 71.6–75.0 | 0.01700–0.01744 | 100.0 |
| `cp_dlc_80` — DLC 80, µ0.4 | 9.32 | 0.8661 | 0.3033 | 7.90 | 0.330 | 276.0 | 847 | 2045 | 370 | 2390 | 0.00–1.60 | 76.4–80.0 | 0.01700–0.01799 | 100.0 |
| `cp_sla_80` — Slalom 80, µ0.85 | 8.00 | 0.7724 | 0.4618 | 3.70 | 0.680 | 209.9 | 2133 | 5183 | 1513 | 5141 | 0.00–1.60 | 69.6–80.0 | 0.01700–0.02003 | 100.0 |
| `cp_fh_80` — FishHook 80, µ0.9, cargo 170 | 7.99 | 14.8400 | 6.0272 | 52.48 | 0.799 | 0.0 | 4509 | 7786 | 614 | 4369 | 0.00–1.60 | 58.8–80.0 | 0.01700–0.01964 | 100.0 |

命令：

```matlab
run_dlc(3,70,struct('stopTime',15,'prefix','cp_dlc_70','tau_d_mode',2))   % 75 / 80 同理
run_slalom(3,80,struct('stopTime',8,'prefix','cp_sla_80','tau_d_mode',2))
run_fishhook('comp25',3,struct('stopTime',8,'prefix','cp_fh_80','cargo',170,'tau_d_mode',2))
```

### 口径说明（读表前必看）

- **QP%**：`D(24) = 1` 收敛、`0` 未收敛、`-1` 表示**本拍没跑 QP**
  （`pmpc_step` 在无效测量 / 无参考路径时提前返回，留 `-1` 当哨兵）。
  分母只数 `D(24) >= 0` 的帧，**不把 `-1` 算失败**：
  `QP% = 100*sum(D(24)>0.5)/sum(D(24)>=0)`。
  本次五个算例被跳过的帧数全是 0（70/75/80 km/h DLC 各 1064/995/933 帧，
  Slalom 801 帧，FishHook 800 帧），所以 QP% 与旧口径数值一致。
- **Vx 范围**必须一起看：基线那支没有纵向速度环，历史上有 DLC 掉到 0.6 km/h 的算例。
  本文五个算例的 Vx 下界与目标车速的差距分别是 −3.3 / −3.4 / −3.6 / −10.4 / −21.2 km/h，
  都是机动过程中的正常掉速（不是车速失控）；三方对比时先对齐这一列，再谈 `ey`。
- **DLC 的 T 只有 9–11 s，不是 15 s**：`run_dlc` 把 CarSim 的 `SSTOP` 设成 200 m，
  到里程就停（70 km/h 走完 200 m 约 10.3 s）。也就是说三个车速比的是**同一段 200 m**，
  不是同一时长。
- **FishHook 的 SW_max = 0**：该工况 `afs_on=false` 且 `steer_ext=true`（CarSim 转向机器人
  给方向盘），`D(19)` 只记 AFS 增量，所以恒为 0。不要拿它和 DLC/Slalom 的 SW 比。
- FishHook 的 `ey_max=14.84 m`、`epsi_max=52.5°` 是工况本身（鱼钩后大侧倾、大偏航），
  量级天然和其它工况不同，**跨工况不要比 ey**；`LTR_max=0.799` 贴着约束 0.8，说明
  LTR 约束在该工况起作用。

---

## 2. 时滞口径对照（不进对比矩阵）

同一算例只差 `tau_d_mode`，用来看"时变口径 vs 常数口径"的差别，以及留档两条 3 s 自检。

| 算例 | T [s] | ey_max [m] | ey_rms [m] | epsi_max [deg] | LTR_max | SW_max [deg] | Mz_avg [N·m] | Mz_max [N·m] | Md_avg [N·m] | Md_max [N·m] | Icmd 范围 [A] | Vx 范围 [km/h] | 生效 tau_d [s] | QP% |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `cp_sla_mode1` — Slalom 80, 常数口径 | 8.00 | 0.7433 | 0.4433 | 3.44 | 0.688 | 181.4 | 1939 | 5190 | 1622 | 5260 | 0.00–1.60 | 71.0–80.0 | 0.2000 | 100.0 |
| `cp_sla_mode2` — Slalom 80, 时变口径 | 8.00 | 0.7724 | 0.4618 | 3.70 | 0.680 | 209.9 | 2133 | 5183 | 1513 | 5141 | 0.00–1.60 | 69.6–80.0 | 0.01700–0.02003 | 100.0 |
| `cp_dlc_mode1` — DLC 70, **3 s 自检，不用于对比** | 3.00 | 0.0016 | 0.0002 | 0.03 | 0.018 | 5.8 | 3 | 80 | 25 | 164 | 0.00–1.60 | 68.4–70.0 | 0.0100 | 100.0 |
| `cp_dlc_mode2` — DLC 70, **3 s 自检，不用于对比** | 3.00 | 0.0016 | 0.0002 | 0.03 | 0.018 | 5.8 | 3 | 80 | 26 | 163 | 0.00–1.60 | 68.4–70.0 | 0.0170 | 100.0 |

- `cp_sla_mode1/2` 是同一算例的唯一差别（0.20 常数 vs 17–22 ms 时变）：
  `ey_max` 0.7433 → 0.7724（时变口径差 3.9%）、`SW_max` 181° → 210°。
  与 COORDINATION.md 的预期（"切到 mode=2 后 Slalom 的 ey 会从 0.767 回到 0.78 量级"）方向一致。
- `cp_dlc_mode1/2` 是 **3 s 自检**：70 km/h 下变道在 X≈61 m（t≈3.1 s）才开始，
  窗内 `ey` 只有 1.6 mm，只证明链路通，**不用于对比**。
- 更早的 `cp_verify` / `cp_verify8` / `cp_verify8_td1` / `cp_verify_td1`
  同样是 **3 s / 8 s 自检，不用于对比**，前缀混乱且被 `cp_dlc_mode*` 取代，**已删除**
  （见 commit `bdd8cac`）。

---

## 3. tau_d 口径与反辨识

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
  DLC_mu0.4_interp/cp_dlc_70.mat  + cp_dlc_70_params.mat
  DLC_mu0.4_interp/cp_dlc_75.mat  + cp_dlc_75_params.mat
  DLC_mu0.4_interp/cp_dlc_80.mat  + cp_dlc_80_params.mat
  Slalom_mu0.85_interp/cp_sla_80.mat + cp_sla_80_params.mat
  FishHook_80_mu0.9_cargo170_newplant/cp_fh_80.mat + cp_fh_80_params.mat
  DLC_mu0.4_interp/cp_dlc_mode1.mat / cp_dlc_mode2.mat        ← §2 3 s 自检
  Slalom_mu0.85_interp/cp_sla_mode1.mat / cp_sla_mode2.mat    ← §2 口径对照
tools/
  cp_case_metrics.m    ← §1/§2 的表（逐算例打印，与本文数字一致）
  taud_backid.m        ← §3 的反辨识（公式、适用条件、噪声地板写在头注释）
  cp_write_params.m    ← 写 <prefix>_params.mat（run_*.m 调用）
  vs_read.m            ← 读 CarSim .vs/.vsb（给 run_*.m 复算 tau_dtrace 用）
pmpc_core/
  tau_d_effective.m    ← mode1/mode2 的唯一口径来源（build_prediction 与 run_*.m 共用）
```

`.mat` 内：`D`（27×N 诊断日志，行序见 `pmpc_core/pmpc_step.m` 头注释）、`t`、`Icmd`
（四角 1 kHz 电流）、`tau_dtrace` / `tau_d_mode` / `tau_d_const` / `vd_ctrl`。

`_params.mat` 内（由 `tools/cp_write_params.m` 写）：`maneuver`、`speed`、
`horizon = [Ts Np Nc]`、`tau_d`、`tau_d_mode`、`tau_d_const`、`tau_d_default`、
`controller`、`Q_norm`、`R_norm`、`slack_w`、`SH_MODE`、`timestamp`，
以及各工况补充的 `mu` / `stopTime` / `variant` / `cargo`。
