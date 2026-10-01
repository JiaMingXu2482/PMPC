# PMPC Longitudinal Coordination Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep the 80 km/h J-turn and DLC inputs unchanged while PMPC anticipates road infeasibility and requests the smallest reachable, smooth longitudinal deceleration.

**Architecture:** A PMPC-only 100 Hz speed coordinator uses spatial path preview, available lateral/braking capacity and predicted four-corner road margin. It supplies one fixed-size speed schedule to path sampling and the existing lateral QP, then sends total brake demand to the existing wheel allocator; the QP decision size and CarSim 50-in/54-out interface do not change.

**Tech Stack:** MATLAB R2024b, Simulink, CarSim 2019, MATLAB Coder-compatible fixed-size controller functions, `matlab.unittest`.

**Spec:** `docs/superpowers/specs/2026-10-01-pmpc-longitudinal-coordination-design.md`

## Global Constraints

- Preserve `Ts_exec=0.01 s`, `Ts=0.05 s`, `Np=20`, `Nc=6`, `Nx=7`, `Nu=3` and the three separate model variants.
- Only PMPC enables the new coordinator: `PMPC_LONGCOORD=0` exactly restores its prior branch, `1` is the proposed road-feasibility design (PMPC default), and `2` is curvature-only ablation. MPC and ZENG remain unchanged, including ZENG longitudinal control.
- Keep J-turn R69 / μ=0.85 and DLC / μ=0.5 at identical 80 km/h initial speed. Do not tune by hard-coding a target speed or editing CarSim procedures.
- Keep the 1 kHz NLCSNN plant, wheel-brake physical limits and external CarSim signal ordering unchanged. All controller state and new arrays must have fixed code-generation dimensions.
- Preserve every existing user modification in the dirty worktree. Stage and commit only files owned by each implementation task.

## Review Focus

- Road end or non-monotone station data: end extrapolation stays finite; invalid station ordering returns `valid=false` and triggers fallback; Tasks 1 and 2 tests.
- Brief DLC curvature peak with positive road margin: coordinator must not brake solely on peak curvature; Task 2 test.
- Reduced friction or near-zero brake authority: requested braking stays attainable and flags infeasibility; Task 2 test.
- NaN state/path input or failed wheel allocation: explicit fallback/fault, no fictitious achieved deceleration; Tasks 2 and 4 tests.
- PMPC switch off, MPC and ZENG: unchanged outputs, dimensions and diagnostics; Task 4 tests.

---

### Task 1: Shared projection and speed-scheduled path samples

**Files:** Modify `controller/func_RefTraj_LocalPlanning.m`; create `controller/func_PathProjection.m`; create `tests/test_pmpc_speed_path.m`.

**Interfaces:** `Proj = func_PathProjection(VehiclePara, WayPoints_Index, WayPoints_Collect, VehStateMeasured)` returns fixed-field `WPIndex`, `PrjP`, `s0`, `valid`. Add optional `Proj` and `Vx_pred(Np,1)` final inputs to `func_RefTraj_LocalPlanning`; omitted inputs retain the current constant-speed path. The PMPC call can compute `Proj` once before speed planning and reuse it for reference generation.

- [ ] **Step 1: Write failing tests** `testLegacyPathSameWithConstantSpeed`, `testPlannedSpeedChangesArcSamples`, `testEndOfRoadIsFinite`, `testNonMonotoneStationIsInvalid`. Pin constant-speed `RefU/Kap_dyn/Kap_node/PrjP` to the pre-change function at one DLC and one J-turn sample; verify lower `Vx_pred` samples no farther along station.
- [ ] **Step 2: Run** `matlab -batch "startup_pmpc; assertSuccess(runtests('tests/test_pmpc_speed_path.m'))"`; expect the new projection interface to be missing.
- [ ] **Step 3: Extract** existing nearest-point, `func_error`, `local_arcpos` and station interpolation logic without changing their arithmetic. Use `s_i=s_{i-1}+Vx_pred(i)*Tk(i)` only when the optional speed schedule exists; use `Vx_pred(i)` in node references. Omitted schedule follows the original scalar `Vel` path.
- [ ] **Step 4: Re-run** the focused test and the existing reference/path tests; expect all pass and finite fixed-size outputs.
- [ ] **Step 5: Commit** only Task 1 files with `feat: support speed-scheduled path preview`.

### Task 2: PMPC road-feasibility speed coordinator

**Files:** Create `controller/func_PMPCSpeedCoordinator.m`; modify `setup_pmpc.m`; create `tests/test_pmpc_speed_coordinator.m`.

**Interfaces:** `[plan,next] = func_PMPCSpeedCoordinator(MPCParameters,VehiclePara,Constraints,WayPoints_Collect,Proj,VehStateMeasured,ParaHAT,margin_prev,prev)`; `plan` has fixed fields `Vx_pred(Np,1)` in m/s, `Vset_pid` in km/h, `Fx_dem` in N, `diag(8,1)`, `valid`; `next` retains the previous speed/force request and trigger state. `margin_prev` is the previous QP's minimum predicted four-corner road margin in metres.

- [ ] **Step 1: Write failing tests** `testStraightKeepsTarget`, `testSustainedR69BrakesBeforeBend`, `testShortDlcBendDoesNotBrakeWhenReachable`, `testLowerMuNeverRaisesSpeedCap`, `testLowBrakeAuthorityFlagsUnreachable`, `testForceSlewAndRecovery`, `testInvalidInputFallsBack`.
- [ ] **Step 2: Run** `matlab -batch "startup_pmpc; assertSuccess(runtests('tests/test_pmpc_speed_coordinator.m'))"`; expect missing function/fields.
- [ ] **Step 3: Implement** fixed-count spatial preview over the braking-reachability distance, lateral-capacity and four-corner corridor check, backward brake-distance pass, and bounded force/target slew. Choose the highest feasible speed at each station; require predicted road risk or sustained curvature/insufficient lateral authority before braking. Compute brake authority from wheel friction/torque limits with reserve for yaw moment. Invalid inputs return measured-speed `Vx_pred`, original `VxTarget`, `Fx_dem=0` and fault code; infeasible road/actuator demand receives a distinct code. Define `PMPC_LONGCOORD` modes 0/1/2, capability reserve, acceleration/force slew and release hysteresis in `setup_pmpc.m`, not as J-turn speed constants.
- [ ] **Step 4: Re-run** the focused test; expect all outputs finite and fixed-size, force/speed bounds respected, no braking for reachable DLC spike.
- [ ] **Step 5: Commit** only Task 2 files with `feat: plan reachable PMPC longitudinal speed`.

### Task 3: Match lateral prediction to planned speed

**Files:** Modify `controller/func_DynamicalModel.m`, `controller/func_SystemFurture.m` only if its curvature input assumes frozen speed; create `tests/test_pmpc_speed_prediction.m`.

**Interfaces:** Add optional final `Vx_pred(Np,1)` argument to `func_DynamicalModel`; omitted argument preserves the existing constant-speed matrices. Scheduled mode uses each `Vx_pred(i)` in the corresponding 0.05 s node for both frozen-tire-tangent and `LTV_on` branches. The scheduled path from Task 1 supplies curvature at the same stations.

- [ ] **Step 1: Write failing tests** `testConstantScheduleMatchesLegacyMatrices`, `testDecreasingSpeedChangesFutureNodesNotFirstState`, `testBothLtvBranchesHaveFixedFiniteDimensions`, `testCurvatureAndVelocityUseSameStations`.
- [ ] **Step 2: Run** `matlab -batch "startup_pmpc; assertSuccess(runtests('tests/test_pmpc_speed_prediction.m'))"`; expect missing schedule support.
- [ ] **Step 3: Implement** per-node `local_ABD`/discretization with the speed floor already used by the current model; do not add a longitudinal state or change matrix dimensions. Leave baseline callers on the original branch.
- [ ] **Step 4: Re-run** focused tests plus `tests/test_dynamical_model_dimensions.m` and `tests/test_qp_dimensions.m`; expect all pass.
- [ ] **Step 5: Commit** only Task 3 files with `feat: schedule PMPC lateral prediction by speed`.

### Task 4: Wire the PMPC-only control path and diagnostics

**Files:** Modify `controller/pmpc_step.m`, `controller/func_QPA_DB.m` (add optional allocation status output), `setup_pmpc.m`, `pmpc_mil.slx` (actual-braking PID gate and internal diagnostics); create `tests/test_pmpc_longcoord_integration.m`.

**Interfaces:** Initialize fixed-field `St.LongCoord` in `setup_pmpc.m`. Before path/QP construction, run Task 1 projection and Task 2 coordinator for variant 3 with the switch on; send Task 2 `Vx_pred` to Tasks 1 and 3. After QP solution, compute current predicted minimum road margin for the next tick. Pass `plan.Fx_dem` and `plan.Vset_pid` through the existing brake/PID outputs, retain yaw-moment priority and wheel limits, and log request, achieved total braking, speed mismatch, margin, reason and failure flags. `func_QPA_DB` gains a fifth optional `exitflag_DB` output without changing its first four outputs.

- [ ] **Step 1: Write failing tests** `testOffIsBaseline`, `testMpcZengUnchanged`, `testPmpcSchedulesBeforeQp`, `testAllocatorFailureIsVisible`, `testNoThrottleBrakeFight`, `testOutput54AndStateDimensions`.
- [ ] **Step 2: Run** `matlab -batch "startup_pmpc; assertSuccess(runtests('tests/test_pmpc_longcoord_integration.m'))"`; expect new state/diagnostics absent.
- [ ] **Step 3: Wire** the branch exactly as interfaces above; preserve existing ZENG speed branch and all 54 CarSim outputs. Gate the longitudinal PID's throttle/integrator with actual brake activity, including symmetric braking with `M_Fx=0`; do not rely solely on the old `|M_Fx|>50` DB-active signal. If allocator fails, flag it and replan from measured speed next tick; never credit requested deceleration as achieved. Route diagnostics only internally, without renumbering CarSim outputs.
- [ ] **Step 4: Re-run** focused tests, `tests/test_controller_variants.m`, `tests/test_func_QPA_DB.m`, Simulink compile of `pmpc_mil`, and MATLAB Coder fixed-size checks; expect all pass.
- [ ] **Step 5: Commit** only Task 4 files with `feat: coordinate PMPC braking with road feasibility`.

### Task 5: Isolated CarSim comparison and regression report

**Files:** Create `scripts/compare_pmpc_longcoord.m`; add only focused invocation guidance to `docs/README_运行配置.md`.

**Interfaces:** `report = compare_pmpc_longcoord()` runs PMPC switch off/on, full ZENG, and curvature-only PMPC ablation in isolated CarSim result clones; it does not overwrite the user's current `LastRun` or alter `simfile.sim`. Store test artifacts outside tracked source files or in a clearly named results directory ignored by Git.

- [ ] **Step 1: Write a harness preflight test** for exact dataset, initial 80 km/h, road μ/R, model choice and unique output paths; reject stale `Send to Simulink` files.
- [ ] **Step 2: Run** the preflight test; expect it to fail before the harness exists.
- [ ] **Step 3: Implement and run** isolated DLC80 μ=0.5 and JT80 R69 μ=0.85 comparisons. Report `e_y` peak/RMS, corner-out percentage, minimum/trajectory speed, brake onset/peak/slew, LTR, QP failures, predicted-vs-real speed, and per-tick timing; do not call smaller `e_y` alone a win if speed loss or DLC regression is material. Preserve every raw run with its parameter snapshot.
- [ ] **Step 4: Run** all targeted MATLAB tests and a final `git diff --check`; inspect failures and artifacts. If PMPC does not beat its own baseline without excess slowing, report the measured failure and root cause instead of silently retuning only J-turn.
- [ ] **Step 5: Commit** only harness/docs/tests with `test: compare PMPC longitudinal coordination on DLC and J-turn`.
