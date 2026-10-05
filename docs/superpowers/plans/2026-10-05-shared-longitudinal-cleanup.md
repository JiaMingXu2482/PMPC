# Shared Longitudinal Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 移除四控制器中已关闭的专属纵向策略与 8×4 实验分支，只保留共用速度 PID、COM 调速和制动分配。

**Architecture:** `setup_pmpc` 只生成现用 6×3/7×3 状态与调参；`pmpc_step` 统一使用 `func_CombinedSpeedReference` 的速度目标和服务制动请求。两个 PMPC 包装函数继续提供固定 12×1 零诊断输出，以保持 Simulink 端口形状。

**Tech Stack:** MATLAB R2024b、Simulink、CarSim 2019、MATLAB Unit Test、Git。

**Spec:** `docs/superpowers/specs/2026-10-05-shared-longitudinal-cleanup-design.md`

## Global Constraints

- 不修改 `Simulink_Model_10_05` 备份、当前 CarSim 54 路接口、四角电流接口或现用横向/侧倾/时延权重。
- 保留 `func_CombinedSpeedReference`、四模型现有速度 PID 和 `func_QPA_DB`；不重新构建 PID Library。
- `pmpc_block` 与 `pmpc_nodelay_block` 第三输出 `long_diag` 始终为 `zeros(12,1)`。
- 不覆盖用户的 `simfile.sim` 本地改动；只提交明确属于本任务的文件。
- 历史结果在备份与 Git 旧标签中保留；删除当前项目休眠策略时同步更新仍可执行的脚本和测试。

## Review Focus

1. `PMPC_LONGCOORD=3` 或 `PMPC_LONGLIM=1` 遗留在 MATLAB base workspace 时，不应重新激活任何专属限速；Task 1 测试。
2. COM 车辆处于 `X=220 m`、实测 90 km/h 时，四控制器应得到相同 80 km/h 目标且共用服务制动仍可请求；Task 1 测试。
3. 非 COM 工况不能因为清理误获得额外服务制动；Task 1 测试速度目标与共用请求。
4. PMPC 和 noDelay 的 `long_diag` 在首拍及连续拍均为 12×1 零向量，同时 `sys` 仍为 54×1、电流为 4×1；Task 1 测试。
5. PMPC 的 7 状态时延与 noDelay 的 6 状态无时延区分、ZENG 的 `rho` 调权在清理后仍有效；Task 3 回归测试。

---

### Task 1: 清理现用控制路径并保持外部接口

**Files:**
- Create: `tests/test_shared_longitudinal_cleanup.m`
- Modify: `config/Atuning_pmpc.m`, `config/Atuning_pmpc_nodelay.m`, `config/Atuning_zeng.m`
- Modify: `setup_pmpc.m`, `controller/pmpc_step.m`, `controller/pmpc_block.m`, `controller/pmpc_nodelay_block.m`
- Modify: `run_combined_carsim.m`
- Extend: `tests/test_combined_speed_schedule.m`

**Interfaces:**
- Consumes: `func_CombinedSpeedReference(course,x,v,vTargetKmh,m) -> [targetKmh,FxDem]`；`func_QPA_DB(...,Fx_dem,...)`。
- Produces: 四个 `setup_pmpc` 变体只含现用 6×3/7×3 状态；两个 PMPC 包装函数仍返回 `[sys,i_cmd,long_diag]`，尺寸分别为 54×1、4×1、12×1。

- [ ] **Step 1: 写失败测试。** 在新测试中分别断言四个初始化结果没有 `S0.LongCoord`、`Constraints.LongCoordMode`/`ZengLong_on`，配置没有专属 `Long_*`/`Zg_*`；在 base workspace 设置旧开关后也不改变结果。扩展 COM 测试覆盖四变体在 `X=220 m` 的共同目标与正服务制动，并覆盖非 COM 无共同制动。两个包装函数在首拍和后续拍输出固定 12×1 零诊断、54×1 `sys` 和 4×1 电流。
- [ ] **Step 2: 验证红灯。** 运行 `& 'D:\Program Files\MATLAB\R2024b\bin\matlab.exe' -batch "startup_pmpc; r=runtests({'tests/test_shared_longitudinal_cleanup.m','tests/test_combined_speed_schedule.m'}); assert(all([r.Passed]))"`；预期新断言失败于现存专属字段或非零诊断，而不是语法/路径错误。
- [ ] **Step 3: 最小实现。** 从调参及 `setup_pmpc` 删除专属纵向字段和 `LongCoord` 状态；从 `pmpc_step` 删除 PMPC 预瞄/安全裕度、ZENG 纵向限速和 8×4 `Vx/Fx` 决策路径，但保留 ZENG `rho`、7×3/6×3 QP、共有 COM `sharedFxDem` 及 `func_QPA_DB(...,Fx_dem,...)`。保留预测输出 `Y` 中仍被 LTR 诊断使用的部分。包装函数令 `long_diag=zeros(12,1)`；`run_combined_carsim` 不再设置或恢复 `PMPC_LONGCOORD`。
- [ ] **Step 4: 验证绿灯。** 运行 Step 2 命令；预期目标测试全通过。另验证 `runtests({'tests/test_db_throttle_gate_models.m','tests/test_jturn_road_radius.m'})`，确认模型油门通路与道路几何没有被清理影响。
- [ ] **Step 5: 提交。** 只提交上述控制代码与测试，提交信息 `refactor: remove private longitudinal controller paths`。

### Task 2: 删除休眠辅助代码并修复仍可运行的脚本

**Files:**
- Delete: `controller/func_PMPCSpeedCoordinator.m`, `controller/func_PMPCSafetyWitness.m`, `controller/func_DynamicalModel8x4.m`, `controller/func_CostWeighting8x4.m`
- Delete: `scripts/update_pmpc_longcoord_model.m`, `scripts/compare_pmpc_longcoord.m`, `scripts/diagnose_current_pmpc_db.m`, `scripts/chk_ver.m`
- Modify: `scripts/run_r69_v20_fairness.m`, `scripts/run_dlc_fairness.m`
- Delete: `tests/test_pmpc_speed_coordinator.m`, `tests/test_pmpc_safety_witness.m`, `tests/test_pmpc_longcoord_integration.m`, `tests/test_pmpc_longcoord_preflight.m`, `tests/test_pmpc_vx_fx_8x4.m`, `tests/test_pmpc_pid_long_brake.m`
- Revise: `tests/test_pmpc_nodelay.m`, `tests/test_controller_tuning_files.m`, relevant active README links; keep dated experiment logs as history.

**Interfaces:**
- Consumes: Task 1 的无专属纵向字段结构与固定诊断输出。
- Produces: 当前入口脚本和现用测试不再引用已删专属函数、字段或 `PMPC_LONGCOORD`；历史日志仍能指向备份/旧 Git 标签。

- [ ] **Step 1: 写失败检查。** 在 `tests/test_shared_longitudinal_cleanup.m` 加入对活动入口文件的引用扫描，断言不再含 `func_PMPCSpeedCoordinator`、`func_PMPCSafetyWitness`、`PMPC_LONGCOORD`、`ZengLong_on`；在 `tests/test_controller_tuning_files.m` 将旧 8×4/纵向字段断言改成当前 6×3/7×3 实值断言。先运行，预期旧入口或过时断言导致失败。
- [ ] **Step 2: 删除与调整。** 删除仅为休眠策略和 8×4 实验服务的函数、脚本与测试；公平性脚本去掉已经不存在的 ZENG 专属开关赋值/断言；保留 noDelay 对减振器时延的测试，去掉仅验证旧纵向协调的部分。更新活动 README 链接，不把旧论文日志改写成新结果。
- [ ] **Step 3: 验证。** 运行 `& 'D:\Program Files\MATLAB\R2024b\bin\matlab.exe' -batch "startup_pmpc; r=runtests({'tests/test_shared_longitudinal_cleanup.m','tests/test_controller_tuning_files.m','tests/test_pmpc_nodelay.m'}); assert(all([r.Passed]))"`；预期全通过。再用 `rg` 扫描 `setup_pmpc.m`、`controller/`、`config/`、当前运行入口，不存在上述专属引用；历史文档和备份不在扫描范围。
- [ ] **Step 4: 提交。** 只提交清理与对应测试/文档，提交信息 `chore: remove obsolete longitudinal experiment helpers`。

### Task 3: 模型更新与非 CarSim 回归

用户后续明确限定：本轮只修改代码和 Simulink 模型，由用户自行手动运行 CarSim–Simulink；PIL 前置检查留到参数确定后。实际连线核对发现 PID 第三输入来自共用 `DB_Request_MFx_sys18`，并非已删专属纵向请求，因此保留该连线；删除仅连接第三诊断输出的旧 Scope/日志，保持第三输出端口并改为 12×1 零值。

**Files:**
- Modify only if required: `pmpc_mil.slx`, `pmpc_nodelay_mil.slx`（仅当 PID 第三输入仍连接已删专属信号时，接固定零；不得改变 CarSim 54 路或 `long_diag` 输出端口）
- Test: `tests/test_shared_longitudinal_cleanup.m`, `tests/test_combined_speed_schedule.m`, `tests/test_db_throttle_gate_models.m` 及现有 ZENG/时延测试
- Update: `README.md` 或 `docs/README_运行配置.md` 的当前版说明

**Interfaces:**
- Consumes: Task 1–2 的 54×1 / 4×1 / 12×1 包装接口与共享纵向路径。
- Produces: 四个模型可更新图，当前控制器源代码不再需要专属纵向状态；保留后续 S32K PIL 工程使用的相同 CarSim/电流端口。

- [ ] **Step 1: 写并运行模型红灯检查。** 扩展 `tests/test_shared_longitudinal_cleanup.m`：两个 PMPC 模型的 `long_diag` 仍是第三输出且宽度为 12，PID 第三输入的直接来源必须是固定零常数而非旧纵向请求 Selector。运行新测试，预期当前连线检查失败。
- [ ] **Step 2: 最小修改模型。** 在 MATLAB 加载两个 PMPC `.slx`，只把休眠策略的 PID 第三输入改接固定零并保存；`PMPC_MF` 输出端口、54 路 CarSim 及电流连线不变。复跑 Step 1 测试，预期通过。
- [ ] **Step 3: 更新图。** 运行 `& 'D:\Program Files\MATLAB\R2024b\bin\matlab.exe' -batch "startup_pmpc; names={'mpc_mil','zeng_mil','pmpc_mil','pmpc_nodelay_mil'}; for k=1:numel(names), load_system(names{k}); set_param(names{k},'SimulationCommand','update'); end"`；预期无 Coder、尺寸或缺字段错误。
- [ ] **Step 4: 回归。** 运行受影响的 MATLAB 测试，再运行完整 `tests/`；逐项记录现存过时断言或外部 CarSim 依赖，不通过删除无关测试掩盖失败。检验 PMPC 7 状态/15 ms 聚合时延、noDelay 6 状态/0 s、ZENG `rho` 诊断及 COM 三组速度目标。
- [ ] **Step 5: 工况抽查。** 在用户可用的 CarSim Run 上复测 COM、J-turn、Slalom 与备份的同配置结果；若环境不可用，明确列为未验证，不声称性能等价或 PIL 可部署。
- [ ] **Step 6: 提交。** 提交必要模型与当前版说明，提交信息 `test: validate shared longitudinal baseline for PIL`。未经用户另行要求不推送。
