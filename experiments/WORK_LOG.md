# 实验工作记录（2026-11-14 ~ 2026-11-15）

只记做过什么、结论是什么、哪些已作废。不复述推导。

---

## 1. 协作基础设施

| 项 | 内容 |
|---|---|
| `Controller/MAIN/COORDINATION.md` | 三方对比口径（工作区/分支/控制���定义/参数表/命名/提交规则/禁区） |
| `Controller/MAIN/guard_check.sh` | 只读守门：分支、`SH_MODE`、`simfile` 指向、越界改 `pmpc_core`；退出码 0/1 |
| subagent 扩展 | 装到 `~/.pi/agent/extensions/subagent/`，agent 定义在 `~/.pi/agent/agents/`（`ctrl-baseline` / `ctrl-nocomp` / `ctrl-comp`） |
| 关键教训 | subagent **继承父会话 cwd**，派活必须显式传 `cwd`；pi-web 按 git 仓库归组，worktree 共用 `.git` 时选目录会被映射回 MAIN |

## 2. 三方矩阵与对比表

- 三个 worktree（Baseline / noComp / Comp）各自跑 DLC 70/75/80、Slalom 80、FishHook 80。
- 产出：`MAIN/PMPC_Results/THREE_WAY_2026-11-14.md`（已被后续修正替代，见 §6）。
- **发现并修正**：baseline 的 `pmpc_base.slx` 没有纵向速度环（`IMP_THROTTLE_ENGINE`/`IMP_PCON_BK` 接 `Constant 0`），
  DLC 70 车速从 70 掉到 0.6 km/h。移植 `lon_arbitrate` + 同一组 PID（P=0.2/I=0.005）后恢复。

## 3. 减振器台架辨识（MR_NLCSNN，**已作废**）

`MR_NLCSNN/delay_test/continuous_id_20261115/{,revision_02,revision_03,vehicle_log_audit}/`

- v1：全段拟合，混入 IC=1.6 A 冷启动 → 参数失真（`vp200_dn` τ=35 ms）。
- v2：预热 1 s + 事件窗拟合，τ=3.2–25.8 ms；发现**四个方向在事件后 4.00 ms 都有反向峰**
  （幅度占台阶 9.7% / 83.1% / 24.0% / 26.9%），一阶结构无法产生。
- v3：改整数毫秒 L、解析参考自检；**三处结论作废**（写进 `revision_03/ERRATA.md`）：
  ① 10 ms 分箱取窗错（激励在 1.010 s），更正后压缩·降首格误差 346 N；
  ② `tau=0` 的 OFFBY1 是 `exp_ref` 浮点比较 bug，不是"物理分辨率"；
  ③ "τ/L 必然不可分"过度一般化。
- **整轮作废原因**：用的是 MR_NLCSNN 的 **v19 A**（21041 参数），不是论文的 V22 E0。

## 4. 整车日志 Md 误差分解

`continuous_id_20261115/vehicle_log_audit/`

- 数据：`MAIN/PMPC_Results/DLC_mu0.4_interp/v6_c3{c,nc}_{60,70}` 的 `.mat` + `_sig.mat`。
- 结论：**分配环节几乎无误差**（`Md_cmd − Md_alloc` RMS 1.5–1.8 N·m，占 0.2–0.3%），
  误差几乎全在 `Md_alloc → Md_act`（26–39 N·m）。
- τ=0.01 的一阶预测 **比"直接认为已到达"更差**（42.9/58.2 vs 27.6/39.0）。
- 缺 `I_actual`；四角 `CmpRD` 只在 100 Hz。

## 5. 非均匀预测网格

`MAIN/experiments/nonuniform_grid_20261115/`

网格：`dt=[0.005×10, 0.05×3, 0.1×4, 0.2×2]`，19 节点、总长 1 s、`Nc=5`、执行 10 ms。
控制块不变（`[0,0.1,0.2,0.3,0.4] s`），节点→控制块映射 `blk=[1×11,2×2,3,4,5×4]`。

改动：`build_prediction`（每节点 `E0^kmul` 精确 ZOH + 按 `blk` 施加 `Baug`）、
`prepare_step`（`dtg/blk/wnode`、站点推进用 `dtg`）、`pmpc_step`（`Q=kron(diag(wnode),Q)`）、
`pmpc_preview`（按累计时间插值预瞄点）、`pmpc_mil_nu.slx`（副本，`PreLoadFcn` 改路径）。

**两次实现错误已修**（`RESULTS_corrected.md`）：
1. 约束仍用 `min(j,K)` 而预测用 `c.blk` → 状态/约束/候选不在同一输入计划；
2. 预瞄首节点丢掉 `meas.p`（`L=0.03` 取成 `t=0.1` 的值）。

修正后 uniform 档**逐位复现生产基线**（ey 0.6373 / SW 68.14 / LTR 0.325）。

| 速度 | ey_max | ey_rms | SW_max |
|---|---|---|---|
| 70 | −1.6% | −1.0% | −9.7% |
| 75 | −4.5% | −2.9% | **−41.7%** |
| 80 | +1.1% | −7.1% | −1.7% |

Md 跟踪误差（+7.4% / +1.3% / −3.8%）**无一致改善**；TASK-006 的"减半"作废。

## 6. 三控制器 / 四方对比

- 三控制器（uniform，DLC 70/75/80）：80 km/h 时 MPC 丢车（ey 8.44 m、SW 720°）、
  ZENG 2.46 m、**Proposed 0.87 m**。
- 四方（Baseline / noComp / Comp×2 网格，DLC75 + Slalom80）：
  `RESULTS_fairness.md`。
  - DLC75：noComp 与 Comp 同网格几乎相同（0.6483/0.6484、0.6196/0.6193）→
    **DLC 上补偿无效果**；
  - Slalom80：补偿只在均匀网格下有效（0.7735→0.7432），非均匀下消失；
  - **网格的收益不依赖补偿**。

## 7. baseline 固定阻尼修复（未结案）

`Baseline/baseline_mpc/`：`prepare` 加 `MdFrozen=Bd'*meas.Fd` 与 `MdLTR=kL*MdFrozen`，
`predict` 扰动扩到 4 列（`-1/Ix`，与 Proposed `B(4,3)` 同号），`step` 的 LTR 右端扣 `MdLTR` 一次。
开关 `M.md_frozen_on`。

- 检查（`check_md_fix.m`）：Md=0 与旧实现差 1.4e-16；Md=±1000 与 1 ms RK4 差 2.7e-11；LTR 仿射项恰好扣一次。
- **问题**：修复后 Slalom 上 baseline `LTR 0.828 → 1.000`（28 个样本内侧轮离地）、`SW 138→192`。
  `md_frozen_on=false` 能复现 v3（0.834/0.828），说明代码路径干净，差异全来自 Md 项。
  **"加阻尼反而更差"未解释** → DLC70/80 的后续 10 次未跑。

## 8. 减振器模型版本核查

- **Controller 四个项目全部是 V22 E0**（`fwd_mf_impl.m` / `fwd_weights_m.m` 归一化换行后完全相同）；
  仅 Baseline 的 `inv_current_impl_10ms.m` 不同（`SH_MODE=2 / SH_I=1.0`，基线固定电流，故意如此）。
- **MR_NLCSNN 的是 v19 A**（21041 参数、`coder.load('fwd_weights.mat')`）。
- 提交 `b14d39f` 把 V22 E0 的力放大 **3 倍**（`net.norm.f_scale*3`），
  提交信息明确要求论文写成"hypothetical stronger CDC hardware"。
- 因此 §3 的整轮辨识作废，§5/§6 的实验不受影响（它们跑的就是 V22 E0）。

## 9. 用 V22 E0 重建的辨识台架

`MAIN/experiments/actuator_id_v22/`（纯 MATLAB，无 Simulink、无 MR_NLCSNN）

口径 A（模型现接法，10 ms 保持）：

| 条件 | τ | L | 训练 RMSE | 台阶幅值 |
|---|---|---|---|---|
| 压缩·升 | 12.60 ms | 3 ms | 110.5 N | 2490.8 |
| 压缩·降 | 10.01 | 0 | 71.8 | 2490.8 |
| 复原·升 | 2.51 | 0 | 25.1 | 912.5 |
| 复原·降 | 3.98 | 0 | 30.9 | 912.5 |

- 无纯延迟；压缩方向一阶拟合差（3–5%），复原好（约 3%）；
- **DLC 的 `tau_d=0.01` 与实测 10.0–12.6 ms 吻合；Slalom 的 `0.20 s` 是实测的 16–80 倍**。

## 10. 当前未决

1. baseline 修复后 Slalom `LTR→1.0` 的机制；决定用修复版还是回退 `md_frozen_on=false` 作主对照。
2. DLC 70/80 的四格对比（10 次）未跑。
3. 非均匀网格收益的归因（网格 vs 积分权重）未分离。
4. 论文里 Slalom `tau_d=0.20` 如何表述（调参等效值 vs 物理时滞）。
