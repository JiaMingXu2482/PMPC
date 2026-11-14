# 三方对比实验 · 协调说明

本文是三个控制器 agent 的**唯一口径来源**。任何参数、命名、提交规则冲突，以本文为准。
改本文需要主 agent 确认，不要各自改。

---

## 1. 角色与工作区

| 角色 | 会话 | 工作区（绝对路径） | 分支 | 结果前缀 |
|---|---|---|---|---|
| **Baseline** | ControllerA | `D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\Baseline` | `ctrl-baseline` | `bl_` |
| **Proposed_noComp** | ControllerB | `D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\Proposed_noComp` | `ctrl-nocomp` | `nc_` |
| **Proposed_Comp** | ControllerC | `D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\Proposed_Comp` | `ctrl-comp` | `cp_` |
| **主 agent** | MAIN | `D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\MAIN` | `jturn-current` | — |

**路径陷阱**：`D:\...\Code\Jturn` 是指向 `Controller\MAIN` 的 junction。
从它出发算 `..` 会落到 `Code\`，不是 `Controller\`。所有跨目录操作**一律写绝对路径**。

**cwd 陷阱**：pi-web 会话的 cwd 在创建时固定，pi 没有 `/cd` 命令。
如果你的会话 cwd 不是上表里的路径，**每个 bash 调用都要先 `cd` 到绝对路径**，
否则 `edit` / `write` 会落到别人的工作区。

---

## 2. 三个控制器的定义

### Baseline —— 独立实现，不走 `pmpc_core`

**论文的基线就是这一套，没有第二套。** `pmpc_core` 里曾经有过一个
`M.baseline` 开关（把 Md/LTR 数值上关掉），2026-11-14 已从代码里删除：
同一个仓库放两套"基线"只会让人搞不清论文写的是哪套。

目录 `baseline_mpc/`，模型 `pmpc_base.slx`（`build_pmpc_base` 生成）。

| 项 | 值 |
|---|---|
| 状态 | `[vy, r, phi, p, ey, epsi]`（**6 维**，含侧倾角与侧倾角速度） |
| 控制量 | `[delta, Mz]`（2 维） |
| 减振器 | 固定电流 `M.cdc_fixed_I = 1.0 A`，四角相同，不是决策量 |
| LTR 约束 | **有**，`|LTR| <= 0.8`（`pmpc_base_prepare.m` 的 `Hr`/`Gr`/`Or`） |
| Md 状态 | **无** |
| 时滞建模 | **无**（没有 Md 状态，这条链路根本不存在，不是把 `tau_d` 关掉） |
| 松弛变量 | 4（横摆 / 侧偏 / LTR / 道路），权重固定不自适应 |
| `Q_norm` | 6 维 `[0;0;0;0;50;5]` |
| `R_norm` | 2 维 `[100;200]` |
| 实现文件 | `nlcsnn/inv_current_impl_10ms.m` 里 `SH_MODE = 2`、`SH_I = 1.0` |
| 各工况权重 | 见 `pmpc_base_init.m` 的 case 分支（DLC 侧倾权重 10、FishHook 20），
不等于上面的全局默认值 |

与另外两个控制器的关系：**只少了 Md**。侧倾、LTR、Zeng 稳定域、道路包络
与 `pmpc_core` 完全一致，被控对象是同一个模型。

入口：`run_baseline('dlc'|'slalom'|'fishhook', speed, opts)`

### Proposed_noComp —— `pmpc_core`，不补偿时滞

| 项 | 值 |
|---|---|
| 状态 | `[vy, r, phi, p, ey, epsi, Md]`（7 维） |
| 控制量 | `[delta, Mz, Md]`（3 维） |
| 减振器 | 控制器给电流，`SH_MODE = 0` |
| LTR 约束 | 有 |
| 时滞建模 | **无**：`tau_d = 1e-4`（认为阻尼力瞬时建立） |
| `tau_d_mode` | `1`（常数） |

> **关于第 7 维**：实现上 `A` 仍是 7×7，第 7 维就是 Md。`tau_d = 1e-4` 时这一维
> 秒级退化成一拍跟踪（`-1/tau_d` 主导），**等价于 6 状态 + 代数执行器**。
> 换句话说：这个状态存在只是为了跟 comp 共用一份代码，物理上没有引入额外的
> 自由度。论文里写“noComp 不建模时滞”是对的，别写成“6 状态”——代码里是 7。

入口：`run_dlc(3,speed)` / `run_slalom(3,speed)` / `run_fishhook('nocomp',3,opts)`

### Proposed_Comp —— `pmpc_core`，补偿时滞

| 项 | 值 |
|---|---|
| 状态 / 控制量 | 同 noComp |
| 减振器 | 控制器给电流，`SH_MODE = 0` |
| LTR 约束 | 有 |
| 时滞建模 | **有** |
| `tau_d_mode` | **`2`（时变）** ← 2026-11-14 决定 |

`tau_d_mode = 2` 时 `M.tau_d` 被忽略，改由 `build_prediction.m` 按四角压缩速度幅值
`vmag = 1000*mean(abs(c.vd))` 查表：

| 压缩速度 `vmag` [mm/s] | `tau_d` [s] |
|---|---|
| ≤ 50 | 0.017 |
| 50 → 100 | 0.017 → 0.019（线性） |
| 100 → 200 | 0.019 → 0.022（线性） |
| ≥ 200 | 0.022（饱和） |

标定来源：减振器力阶跃响应的 `t10-90`（50/100/200/400 mm/s → 17/19/22/22 ms）。

> **注意**：这些值（17–22 ms）比原先扫出来的常数最优值小一个量级
> （Slalom `tau_d=0.20`、FishHook `0.025`）。常数最优值是把其它滞后
> 一起吸收进去的**等效**补偿量，时变值只代表执行器本身。切到 mode=2
> 后 Slalom 的 `ey` 预计会从 0.767 回到 0.78 量级——要如实记录，不要
> 拿旧 comp 数字对比。

入口：`run_dlc(3,speed,struct('tau_d_mode',2))` /
`run_slalom(3,speed,struct('tau_d_mode',2))` /
`run_fishhook('comp25',3,struct('tau_d_mode',2))`

---

## 3. 参数表（来自 `pmpc_core/pmpc_init.m`，不要各自改）

### 公共

```
Ts=0.1  Np=10  Nc=5  Ts_exec=0.01
tau_mz=0.2   tau_md=0.1
```

`pmpc_init.m` 末尾有一条硬规则：**`M.controller ~= 3` 时强制 `tau_d = 1e-4`**。
即 MPC / ZENG 永远不补偿时滞，只有 Proposed 有补偿能力。不要绕过这条。

### DLC（mu=0.4）

| 参数 | MPC(1) | ZENG(2) | Proposed(3) |
|---|---|---|---|
| `Q_norm` | `[0;0;10;0;50;20;0]` | `[0;0;10;0;50;10;0]` | `[0;0;10;0;50;10;0]` |
| `R_norm` | `[100;200;0.1]` | 同左 | 同左 |
| `slack_w` | `5e4` | `0.2e4` | `0.2e4` |
| `rho_I0` | — | `1.0` | — |
| `Wb` | — | — | `10` |
| `w1_max` / `w2_max` | — | — | `0.1e4` |
| `tau_d` | `1e-4` | `1e-4` | **`0.01`**（全局默认，DLC case 未覆盖） |

### Slalom（mu=0.85）

| 参数 | 值 |
|---|---|
| `Q_norm` | `[0;0;5;0;50;5;0]` |
| `R_norm` | `[0.1;500;0.1]` |
| `slack_w` | `[5;5;5;5]` |
| `preview_L` / `preview_alpha` | `0` / `0`（力界预瞄在 Slalom 净有害，三种控制器统一关） |
| `rho_I0` / `rho_dI` | `0.7` / `0.3`（ZENG 论文原值） |
| Proposed `w1_max` / `w2_max` | `200` |
| Proposed `Wb` | `10` |
| Proposed `tau_d` | 常数口径 `0.20`（仅供参考）/ noComp `1e-4` |
| Proposed `tau_d_mode` | **`2`**（comp，时变）/ `1`（noComp） |

### FishHook（mu=0.9，cargo 170 kg）

| 参数 | 值 |
|---|---|
| `afs_on` | `false`（控制器不控前轮转角） |
| `steer_ext` | `true`（CarSim 开环转动机器人给 SW） |
| `Q_norm` | `[0;10;20;0;0;0;0]`；MPC 的 `Q_norm(3)=15`，Proposed 的 `Q_norm(3)=40` |
| `R_norm` | `[0.1;500;0.1]`；Proposed 的 `R_norm(3)=50` |
| `slack_w` | `[100;100;1e4;1e-3]` |
| Proposed `w1_max` / `w2_max` | `2e5` / `1e4` |
| Proposed `w_env` | `1e-3` |
| `tau_d` / `preview_L` / `preview_alpha` | noComp：`1e-4` / `0` / `0`；comp25：`0.025` / `0.025` / `1` |
| `tau_d_mode` | comp：**`2`**（时变，`tau_d` 被忽略）；noComp：`1` |

### U-turn / J-turn（mu=0.85）

| 参数 | 值 |
|---|---|
| `Q_norm` | `[0;0;5;0;50;5;0]` |
| `R_norm` | `[0.1;500;0.1]` |
| `slack_w` | `[5;5;5;5]` |
| Proposed `w1_max` / `w2_max` | `1e4` |

---

## 4. 结果命名与目录

```
PMPC_Results/<场景目录>/<前缀><工况>_<车速>.mat
```

| 场景 | 目录 | 例 |
|---|---|---|
| DLC | `PMPC_Results/DLC_mu0.4_interp/` | `bl_dlc_80.mat` / `nc_dlc_80.mat` / `cp_dlc_80.mat` |
| Slalom | `PMPC_Results/Slalom_mu0.85_interp/` | `bl_sla_80.mat` / `nc_sla_80.mat` / `cp_sla_80.mat` |
| FishHook | `PMPC_Results/FishHook_80_mu0.9_cargo170_newplant/` | `bl_fh.mat` / `nc_fh.mat` / `cp_fh.mat` |
| Baseline 独立目录 | `PMPC_Results/Baseline_MPC/` | `bl_dlc_80.mat`（`run_baseline` 默认） |

图：同时存 `.fig` + `.pdf`（论文用矢量图），需要位图再加 `.png`（`'Resolution',300`）。

**不要动已有结果文件**，不要为了"整理"重命名或移动。

---

## 5. 提交规则

1. **只提交到自己分支**：`ctrl-baseline` / `ctrl-nocomp` / `ctrl-comp`
2. 提交前 `git status` 必须干净（除了 `*.vs` / `*.vsb` / `slprj/` 这些 gitignore 的）
3. **不要 `git merge` 别人的分支**，不要 `git rebase`，不要 `git push`
4. 每次提交聚焦一件事，message 说清"改了什么 + 为什么"
| `simfile_dlc.sim` / `simfile_slalom.sim` / `simfile.sim` **不要提交**——它们指向各自 worktree 的 CarSim 数据集，提交会互相覆盖。三个都已在 `.gitignore` 里（`simfile.sim` + `simfile_*.sim`）。

---

## 6. 禁区

| 路径 | 谁能改 |
|---|---|
| `baseline_mpc/` | 只有 Baseline |
| `pmpc_core/` | 只有 Proposed_noComp / Proposed_Comp，且只改与自己口径相关的部分 |
| `nlcsnn/` | **谁都不许改**（三个控制器共用减振器模型） |
| `init_valve2.m` | **谁都不许改** |
| 别人的 worktree | 谁都不许碰 |

### `nlcsnn/inv_current_impl_10ms.m` 的 `SH_MODE`

| 分支 | 必须的值 |
|---|---|
| `ctrl-baseline` | `SH_MODE = 2`，`SH_I = 1.0`（固定电流，基线本意） |
| `ctrl-nocomp` | `SH_MODE = 0` |
| `ctrl-comp` | `SH_MODE = 0` |
| `jturn-current` | `SH_MODE = 0` |

`SH_MODE = 2` 会让四角电流恒为 `SH_I`，CDC 整个失效，**而且不报错**，只表现为"效果变差"。
历史上这个坑被踩过一次：三个 agent 的工作区改动把协调控制器的减振器也固定成 0.8 A。

### `simfile*.sim` 指向

| 工作区 | dlc | slalom | fishhook |
|---|---|---|---|
| Baseline | `Run_bl_dlc` | `Run_bl_slalom` | `Run_bl_fishhook` |
| Proposed_noComp | `Run_nc_dlc` | `Run_nc_slalom` | `Run_nc_fishhook` |
| Proposed_Comp | `Run_cp_dlc` | `Run_cp_slalom` | `Run_cp_fishhook` |

**不要改回 `Run_4d669775-...`**，那是 MAIN 的原始数据集。三个 agent 并发时改同一份
`Run_all.par`（`SV_VXS` / `TSTOP` / `SSTOP` / `M_PL`）会互相踩。

---

## 7. 运行约定

- 非交互执行一律 `matlab -batch "..."`，**禁止** `-nodisplay -r`
- 新进程先跑 `worker_boot('<worktree 绝对路径>')`（补 `init_valve2` 的 `Ki`/`Kp`/`D_init`）
- **禁止 `sleep` 等待**。长任务用：启动 → 秒回 → 反复短轮询

  **本机实测（2026-11-14）：`nohup ... &` 在这个环境里不工作**——
  进程会被连同进程组一起回收，日志文件都不创建。必须写成 `.bat` 再用
  `cmd //c "start /b /min <绝对路径>.bat"` 拉起。

  ```bat
  @echo off
  cd /d "D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\Proposed_Comp"
  matlab -batch "worker_boot(pwd); run_slalom(3,80); disp('DONE');" > C:\Users\user\AppData\Local\Temp\cp_sla80.log 2>&1
  echo EXIT=%ERRORLEVEL% >> C:\Users\user\AppData\Local\Temp\cp_sla80.log
  ```

  ```bash
  cmd //c "start /b /min C:\\Users\\user\\AppData\\Local\\Temp\\myjob.bat"
  ```

  然后每次 `tail -20` 那个 log 轮询，不要在一次调用里 sleep 循环。
- `-batch` 要 checkout 许可证；用户开着 MATLAB 桌面时可能抢不到，失败先看许可证而不是代码
- 改 `.slx` 只能 `load_system` / `set_param` / `save_system` 脚本化，不要文本编辑
- `.slx.autosave` 是崩溃残留，不要提交

---

## 7b. 三路并行（2026-11-14 实测可行）

三个 agent 可以同时跑 Simulink+CarSim 联仿，已验证：三个 `matlab -batch`
进程同时启动，各自 `run_baseline` / `run_dlc`，全部 `EXIT=0`、`t_end 3.00`、
`QP 100.0%`。

能并行的原因（都已核实）：

| 项 | 状态 |
|---|---|
| CarSim 许可 | `MSCLIC_SSQ.dat` 是 `permanent uncounted` + `HOSTID=ANY`，不限席位 |
| MATLAB 许可 | 三个 `-batch` 同时跑没问题 |
| CarSim 数据集 | `Run_bl_*` / `Run_nc_*` / `Run_cp_*` 三个独立目录，`Run_all.par` md5 各不相同 |
| 求解器 | `Solver_SF/CarSim S-Function`（Solver in Process），每个 MATLAB 进程一份 |
| 输出 | `Results\Run_xxx\LastRun.*` 各写各的 |
| `slprj/` | 每个 worktree 一份 |

**前提：cwd 必须对。** 三个 worktree 共用 `Controller\MAIN\.git`，
pi-web 按仓库归组，在 UI 里选 `Baseline` 会被映射回 `MAIN` 项目。
新建会话时要先在该目录用 CLI 起一个会话，pi-web 才能列出正确 cwd 的条目。

残余影响：

- 三个 `simfile*.sim` 的 `GUI_REFRESH_V` 是同一个值，CarSim GUI 的动画/曲线会
  在三个仿真间跳。**只是显示，不影响存盘数据。** 并行时别盯 GUI 即可。
- 本机 12 逻辑核，三个仿真 + MATLAB 桌面 + CarSim GUI 同时跑，单个仿真会变慢。
- 三个 worktree 都在 OneDrive 目录下，编译 `slprj/` 时会有同步开销。

---

## 8. 汇报格式

每次交活输出：

```
STATUS: done | blocked
BRANCH: <分支>
COMMITS: <hash> <subject>
RESULTS: <文件列表>
METRICS: <表格：工况 × 车速 × (ey_max, LTR_max, SW, 收敛率, 是否跑完)>
ISSUES: <没有就写 none>
```

指标写进各分支的 `RESULTS.md`（`baseline_mpc/RESULTS.md`、`PMPC_Results/NC_RESULTS.md`、`PMPC_Results/CP_RESULTS.md`）。

---

## 9. 待决事项 / 已定事项

| # | 事项 | 结论 |
|---|---|---|
| 1 | Baseline 用哪套实现 | ✅ 定：`baseline_mpc/`（4 状态 / 2 控制量）。`pmpc_core` 里的 `M.baseline` 已删除 |
| 2 | `ctrl-comp` 用常数 `tau_d` 还是时变 | ✅ 定：`tau_d_mode = 2` 时变 |
| 3 | `ControllerA` / `ControllerC` 会话 cwd 指向 `Code\Jturn`（= MAIN） | ⏳ 待重建会话 |
| 4 | `baseline_mpc/` 只在 `ctrl-baseline` 有，`MAIN` 的是未跟踪副本 | ⏳ 待定：是否合回 `jturn-current` |
