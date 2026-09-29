# NLCSNN Deployment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deploy the uploaded NLCSNN CDC model as the common damper envelope and plant for MPC, ZENG, and PMPC while preserving the unified QP dimensions and existing controller output layout.

**Architecture:** Append four `CmpD_*` signals to the existing CarSim input vector, map CarSim compression coordinates into NLCSNN rebound coordinates in one adapter, and use a two-stage adapter API: prepare kinematics and force bounds before the QP, then invert desired force and advance four independent NLCSNN states after allocation. The controller keeps `Nx=7`, but the old 15 ms plant lag is disabled; the seventh state becomes a near-instant fixed-dimension prediction placeholder while actual damper dynamics come from NLCSNN.

**Tech Stack:** MATLAB R2024b, MATLAB Coder, Simulink, CarSim 2019.0, MATLAB function-based tests, Git.

**Spec:** `docs/superpowers/specs/2026-09-29-nlcsnn-deployment-design.md`

## Global Constraints

- NLCSNN current range is exactly `0..1.6 A`; initial current is `0 A`.
- Temperature is fixed at `42.5 degC` unless a later task adds a measured signal.
- CarSim channels 1..50 keep their current positions; `CmpD_L1/L2/R1/R2` are appended as 51..54.
- CarSim compression-positive signals map to NLCSNN rebound-positive signals with `x=281.0645-CmpD`, `v=-CmpRD`, `a=-d(CmpRD)/dt`.
- CarSim force polarity is the negative of NLCSNN tension-positive force polarity.
- Four corners maintain independent `8x1` hidden states.
- No fixed `I_nom` is introduced; force-space midpoint is used where the QP needs a center.
- MPC, ZENG, and PMPC use the same NLCSNN adapter and plant.
- QP dimensions, `Nx=7`, certificate size, and controller `54x1` output remain fixed.
- Existing unrelated dirty-worktree changes are preserved; commits stage only files owned by the task being committed.

## Review Focus

- A 50-element or misordered CarSim input must fail validation instead of silently treating another signal as `CmpD`; covered in Task 2 input-layout tests.
- Compression/rebound and force polarity must be transformed exactly once; covered in Task 1 sign tests and Task 4 end-to-end tests.
- Non-finite or impulsive acceleration estimates must not produce non-finite force/current; covered in Task 1 finite-output tests.
- Near-zero force authority and multiple inverse crossings must keep current bounded and continuous; covered in Task 1 inverse tests.
- Switching among MPC, ZENG, and PMPC must not reset or reshape NLCSNN state; covered in Task 4 mode tests.

---

### Task 1: Public Predictor and Four-Corner NLCSNN Adapter

**Files:**
- Create: `nlcsnn/nlcsnn_predict_force.m`
- Create: `controller/func_NLCSNNContext.m`
- Create: `controller/func_NLCSNNApply.m`
- Modify: `nlcsnn/nlcsnn_force_to_current.m`
- Modify: `nlcsnn/nlcsnn_damper_step.m`
- Create: `tests/test_nlcsnn_adapter.m`

**Interfaces:**
- Consumes: `net = nlcsnn_damper_init()`, four-corner CarSim `CmpD/CmpRD`, fixed NLCSNN configuration, and fixed-size state.
- Produces: `F = nlcsnn_predict_force(net,x,v,a,curr,temp,h)`, `[ctx,state] = func_NLCSNNContext(net,cmpD,cmpRD,state,cfg)`, and `[F_actual,i_cmd,state] = func_NLCSNNApply(net,ctx,F_des,state,cfg)`.
- `cfg` fields: `dt=0.01`, `temp=42.5`, `i_max=1.6`, `x_ref=281.0645`, `tau_accel=0.02`.
- `state` fields: `h=zeros(8,4)`, `i_prev=zeros(4,1)`, `v_prev=zeros(4,1)`, `a_filt=zeros(4,1)`, `initialized=false`.
- `ctx` fields are fixed `4x1` arrays: `x`, `v`, `a`, `F_lo`, `F_hi`, `F_center` in internal order `[L1;R1;L2;R2]` and CarSim force polarity for the force arrays.

- [ ] **Step 1: Write failing predictor and adapter tests**

Add tests named `testPredictorMatchesStepStartForce`, `testCompressionMapping`, `testCornerStatesAreIndependent`, `testCurrentIsBounded`, `testNearZeroAuthorityIsFinite`, and `testNonFiniteAccelerationFallsBackSafely`. Assert predictor equality below `1e-9 N`, exact sign mapping, unchanged inactive state columns, `0<=i<=1.6`, and finite outputs.

- [ ] **Step 2: Run tests and verify RED**

Run:

```matlab
r = runtests('tests/test_nlcsnn_adapter.m'); assertSuccess(r)
```

Expected: FAIL because the public predictor and adapter functions do not exist.

- [ ] **Step 3: Implement the public force predictor**

Implement `nlcsnn_predict_force(net,x,v,a,curr,temp,h)` with the same normalization, 63-feature NFL, GELU layers, and force scaling already used by the uploaded port. Replace duplicated force-only paths in `nlcsnn_force_to_current` and `nlcsnn_damper_step` with this function without advancing `h`.

- [ ] **Step 4: Implement context preparation**

Implement `func_NLCSNNContext` using:

```text
x = x_ref - CmpD
v = -CmpRD
a_raw = -(CmpRD-v_prev)/dt
alpha = 1-exp(-dt/tau_accel)
a_filt = (1-alpha)*a_filt + alpha*a_raw
```

On the first call set `a_raw=0`. Replace non-finite `CmpD/CmpRD/a_raw` with the last finite state or zero. Compute endpoint forces at `0 A` and `1.6 A`, convert them to CarSim polarity, then store ordered minimum, maximum, and midpoint.

- [ ] **Step 5: Implement force application**

Implement `func_NLCSNNApply` per corner: convert `F_des` to NLCSNN polarity, clamp `i_prev`, call the inverse, compute `dcurr`, call `nlcsnn_damper_step` once, convert force back to CarSim polarity, and update only that corner's `h` and current.

- [ ] **Step 6: Run adapter and golden-vector tests**

Run:

```matlab
r = runtests('tests/test_nlcsnn_adapter.m'); assertSuccess(r)
cd nlcsnn; test_nlcsnn_port
```

Expected: all adapter tests PASS; golden test reports force error `<1e-6 N`, state error `<1e-9`, and `PASS`.

- [ ] **Step 7: Commit Task 1 files**

```bash
git add nlcsnn/nlcsnn_predict_force.m nlcsnn/nlcsnn_force_to_current.m nlcsnn/nlcsnn_damper_step.m controller/func_NLCSNNContext.m controller/func_NLCSNNApply.m tests/test_nlcsnn_adapter.m
git commit -m "feat: add NLCSNN four-corner adapter"
```

### Task 2: Initialize NLCSNN and Expand the Controller Input Contract

**Files:**
- Modify: `startup_pmpc.m`
- Modify: `setup_pmpc.m`
- Modify: `controller/func_StateEstimation.m`
- Modify: `controller/pmpc_block.m`
- Modify: `tests/test_qp_dimensions.m`
- Modify: `tests/test_zeng_diagnostic_channels.m`
- Create: `tests/test_nlcsnn_configuration.m`

**Interfaces:**
- Consumes: Task 1 adapter API and uploaded `nlcsnn/nlcsnn_weights.mat`.
- Produces: `PMPC_P.Pm.NLCSNN` immutable configuration and `PMPC_P.S0.InitialParams.prevstate.nlcsnn` fixed-size state; `pmpc_block` accepts exactly `54x1`.
- `func_StateEstimation` adds raw-mm fields `D_l1/D_l2/D_r1/D_r2` from input 51..54 while preserving current `V_*` fields in `m/s`.

- [ ] **Step 1: Write failing configuration and layout tests**

Assert that startup places `nlcsnn_damper_step` on path, setup loads a network with `Wx1` size `256x63`, defaults are `enabled=true`, `i_max=1.6`, `temp=42.5`, and state sizes are `8x4` and `4x1`. Assert a known `54x1` input maps channels 51..54 to the new `D_*` fields without changing channels 1..50.

- [ ] **Step 2: Run tests and verify RED**

Run:

```matlab
r = runtests({'tests/test_nlcsnn_configuration.m','tests/test_qp_dimensions.m','tests/test_zeng_diagnostic_channels.m'}); assertSuccess(r)
```

Expected: FAIL on missing path/configuration/state and the old 50-element fixtures.

- [ ] **Step 3: Add project path and immutable configuration**

Add `nlcsnn/` in `startup_pmpc`. In `setup_pmpc`, load the network once and create `P.Pm.NLCSNN` with fields `enabled`, `net`, `i_max`, `temp`, `x_ref`, `dt`, and `tau_accel`. Default `PMPC_NLCSNN` to enabled; retain a rollback override without changing dimensions.

- [ ] **Step 4: Add fixed-size initial state**

Create `InitialParams.prevstate.nlcsnn` with the exact Task 1 state fields. Do not reuse `s_act`; leave unrelated legacy state fields unchanged until Task 3 proves they are no longer read on the active path.

- [ ] **Step 5: Expand input parsing and block contract**

Update comments and checks from `50x1` to `54x1`, append `CmpD` parsing at indices 51..54, and update every test fixture to allocate `zeros(54,1)`. Do not renumber existing inputs.

- [ ] **Step 6: Run configuration/layout tests**

Run the Task 2 test command. Expected: PASS with unchanged `Ne=8`, `Nr=3`, certificate `36x1`, and output `54x1`.

- [ ] **Step 7: Commit Task 2 files**

```bash
git add startup_pmpc.m setup_pmpc.m controller/func_StateEstimation.m controller/pmpc_block.m tests/test_qp_dimensions.m tests/test_zeng_diagnostic_channels.m tests/test_nlcsnn_configuration.m
git commit -m "feat: initialize NLCSNN controller state"
```

### Task 3: Replace Legacy CDC Bounds and Plant in the Unified Controller

**Files:**
- Modify: `controller/pmpc_step.m`
- Modify: `controller/func_CostWeightingRegulation_QuadSlacks.m`
- Modify: `controller/func_DynamicalModel.m`
- Modify: `setup_pmpc.m`
- Create: `tests/test_nlcsnn_controller_path.m`

**Interfaces:**
- Consumes: Task 1 `ctx/state/apply` API and Task 2 `Pm.NLCSNN`/persistent state.
- Produces: a `DamperLimits` struct passed into cost/constraint construction with `Fdu`, `Fdl`, `Fd_center`, and `external=true`; actual CarSim damper forces from `func_NLCSNNApply`.

- [ ] **Step 1: Write failing controller-path tests**

Add tests `testCostWeightingUsesNLCSNNBounds`, `testLegacyActuatorIsBypassed`, `testAllThreeModesUseNLCSNN`, `testForceOutputStaysInsideReachableBounds`, and `testQpDimensionsStayFixed`. Instrument through returned state/current and deterministic inputs, not source-text-only assertions.

- [ ] **Step 2: Run tests and verify RED**

Run:

```matlab
r = runtests('tests/test_nlcsnn_controller_path.m'); assertSuccess(r)
```

Expected: FAIL because `pmpc_step` still builds the legacy envelope and calls the legacy actuator.

- [ ] **Step 3: Prepare NLCSNN context before cost construction**

After state estimation and before `func_CostWeightingRegulation_QuadSlacks`, call `func_NLCSNNContext` once. Build `DamperLimits` from `ctx.F_hi`, `ctx.F_lo`, and `ctx.F_center`. Preserve corner order expected by `func_QPA_CDC`.

- [ ] **Step 4: Replace active legacy force bounds and center**

Extend `func_CostWeightingRegulation_QuadSlacks(...,DamperLimits)`. When `external=true`, use supplied endpoint forces and midpoint forces for `Fdu/Fdl/Mdnom`, recompute `Mdmin/Mdmax`, and use the full current-step reachable moment span for `dMdmax`. Keep the old branch only for rollback mode; active MPC/ZENG/PMPC must not call `func_MRDamper` or use `I_nom`.

- [ ] **Step 5: Apply the common NLCSNN plant after allocation**

Replace the active `func_DamperActuator` block with `func_NLCSNNApply`, write the four actual forces into existing output positions 5..8, and store updated NLCSNN state. Perform the same application independent of `ControllerMode` and `ZengRho_on`.

- [ ] **Step 6: Disable the old 15 ms predictor without changing QP size**

Keep `Nx=7`. When NLCSNN is enabled, set the seventh-state delay constant to `1e-4 s` under exact ZOH and remove the legacy `tau_MR` rate restriction from the CDC increment bound; this makes the fixed-size predictive state numerically near-instant while the real four-corner dynamics remain in NLCSNN. Rollback mode retains the old value.

- [ ] **Step 7: Run controller-path and existing regression-tool tests**

Run:

```matlab
r = runtests({'tests/test_nlcsnn_controller_path.m','tests/test_qp_dimensions.m','tests/test_zeng_diagnostic_channels.m','tests/test_regression_tools.m'}); assertSuccess(r)
```

Expected: PASS; three modes share the same adapter; all actual forces and currents are finite and bounded.

- [ ] **Step 8: Commit Task 3 files**

```bash
git add controller/pmpc_step.m controller/func_CostWeightingRegulation_QuadSlacks.m controller/func_DynamicalModel.m setup_pmpc.m tests/test_nlcsnn_controller_path.m
git commit -m "feat: use NLCSNN bounds and damper plant"
```

### Task 4: CarSim/Simulink 54-Channel Integration and Code Generation

**Files:**
- Modify: `simfile.sim`
- Modify: `pmpc_mil.slx`
- Modify: `scripts/cg_block.m`
- Modify externally through CarSim: current `I/O Channels: Export` dataset
- Create: `tests/test_carsim_interface.m`

**Interfaces:**
- Consumes: Task 2 `54x1` controller input and current CarSim export dataset.
- Produces: CarSim and Simulink models agreeing on 54 exported channels in the exact order specified by the design.

- [ ] **Step 1: Write failing interface test**

Assert `simfile.sim` contains `PORTS_EXP 1,54`; inspect the active `Run_all.par` export list and assert entries 1..50 are unchanged and entries 51..54 are `CmpD_L1/L2/R1/R2`.

- [ ] **Step 2: Run interface test and verify RED**

Run:

```matlab
r = runtests('tests/test_carsim_interface.m'); assertSuccess(r)
```

Expected: FAIL because the current interface reports 50 exports.

- [ ] **Step 3: Append channels in the CarSim Export dataset**

Use CarSim's dataset editor to append the four channels, then use `Send to Simulink` so the generated interface records `PORTS_EXP 1,54`. Do not insert channels before the existing 50.

- [ ] **Step 4: Update the Simulink model contract**

Update `pmpc_mil.slx` so the MATLAB Function block receives a 54-element vector and `PMPC_P` remains a non-tunable parameter. Run model update (`Ctrl+D`) and resolve only dimension/type diagnostics caused by this task.

- [ ] **Step 5: Update and run code generation**

Make `scripts/cg_block.m` provide a `54x1` input example. Run:

```matlab
startup_pmpc; setup_pmpc; cg_block
```

Expected: `>>> pmpc_block 编译通过 <<<` with fixed `54x1` output.

- [ ] **Step 6: Run interface test and model update**

Run the Task 4 test and `set_param('pmpc_mil','SimulationCommand','update')`. Expected: PASS and no unresolved port-size errors.

- [ ] **Step 7: Commit Task 4 workspace files**

```bash
git add simfile.sim pmpc_mil.slx scripts/cg_block.m tests/test_carsim_interface.m
git commit -m "feat: expand CarSim interface for NLCSNN"
```

### Task 5: Three-Controller Smoke Runs and Operating Documentation

**Files:**
- Modify: `docs/README_运行配置.md`
- Modify: `README.md`
- Create: `tests/test_nlcsnn_smoke.m`

**Interfaces:**
- Consumes: completed 54-channel CarSim/Simulink integration.
- Produces: repeatable startup instructions and evidence that MPC, ZENG, and PMPC all run the same NLCSNN plant.

- [ ] **Step 1: Write the smoke test**

For each controller mode, initialize a fresh `PMPC_P`, run deterministic controller steps with nonzero `CmpD/CmpRD`, and assert finite output forces, bounded currents, independent hidden states, fixed state shape, and no change to QP dimensions.

- [ ] **Step 2: Run smoke test and verify RED or integration gaps**

Run:

```matlab
r = runtests('tests/test_nlcsnn_smoke.m'); assertSuccess(r)
```

Expected before final wiring: FAIL on any remaining mode/state integration gap.

- [ ] **Step 3: Complete minimal integration fixes**

Fix only defects reproduced by the smoke test. Do not retune controller weights, road geometry, speed logic, or maneuver settings.

- [ ] **Step 4: Run short CarSim/Simulink smoke simulations**

Run a short simulation for MPC, ZENG, and PMPC from a fresh MATLAB state. Confirm all runs start, update the four currents/states, produce finite `IMP_FD_*` outputs, and do not invoke the legacy actuator path.

- [ ] **Step 5: Document the run sequence**

Document the required CarSim dataset, 54-channel order, default NLCSNN parameters, one-command MATLAB initialization, and the need to resend from CarSim after changing a run dataset.

- [ ] **Step 6: Run the full relevant MATLAB suite**

Run:

```matlab
r = runtests('tests'); assertSuccess(r)
cd nlcsnn; test_nlcsnn_port
```

Expected: all tests PASS and the golden-vector script reports `PASS`.

- [ ] **Step 7: Commit Task 5 files**

```bash
git add README.md docs/README_运行配置.md tests/test_nlcsnn_smoke.m
git commit -m "docs: document NLCSNN simulation workflow"
```

### Task 6: Final Verification

**Files:**
- Verify only; modify files only for a reproduced defect and repeat the owning task's test cycle.

**Interfaces:**
- Consumes: all prior task deliverables.
- Produces: deployment evidence and a concise handoff.

- [ ] **Step 1: Verify Git scope**

Run `git status --short` and confirm unrelated pre-existing changes remain uncommitted and untouched.

- [ ] **Step 2: Run complete automated verification**

Run all MATLAB tests, the NLCSNN golden test, code generation, and the Simulink model update using a non-OneDrive MATLAB preference directory.

- [ ] **Step 3: Verify all three controller modes in co-simulation**

Record whether MPC, ZENG, and PMPC each complete the short smoke interval with the same NLCSNN configuration and no non-finite current/force/state.

- [ ] **Step 4: Review the branch diff against the spec**

Confirm no old `I_nom`, `func_MRDamper`, or `func_DamperActuator` call remains on the active three-controller path; confirm the input and output sizes are both fixed at 54.

- [ ] **Step 5: Commit any verification-only documentation update**

If verification required no code change, do not create an empty commit. Otherwise stage only the verified fix and its regression test.
