# NLCSNN v14 Replacement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the supplied v14 damper forward model in the shared 1 kHz CarSim plant and use a validated offline inverse-current LUT outside all four controllers.

**Architecture:** Export the checkpoint and PyTorch golden sequences once, implement a codegen-compatible streaming MATLAB v14 forward, and rebuild the four model variants around its expanded per-corner state. Move force-bound lookup and force-to-current allocation outside the MPC blocks; the S32K-facing controllers receive bounds and request force without evaluating v14.

**Tech Stack:** MATLAB R2024b, Simulink/Stateflow, Python 3 + CPU PyTorch for offline export, MATLAB unit tests, Git.

**Spec:** `docs/superpowers/specs/2026-10-07-nlcsnn-v14-replacement-design.md`

## Global Constraints

- Checkpoint MD5: `8a3bd0d508429ab2c00083321561561a`; source is `D:/OneDrive - mail.scut.edu.cn/周报/10.8/nlcsnn/nlcsnn_v14_py_model.zip`.
- All four controllers share the same 1 kHz v14 plant and offline actuator table; keep their 100 Hz controller schedule and current range `[0, 1.6] A`.
- Preserve `x_rig=x_ref-cmpD`, `v_rig=-cmpRD`, `F_CarSim=-F_v14`; use the checkpoint's own normalization.
- Remove the active `SemiActive_Passivity_1kHz` projection. Preserve v14's nonzero zero-speed force; guard only invalid numbers and input/current range.
- Do not alter MPC/QP dimensions, controller tuning, or `tau_d`; preserve pre-existing dirty files and do not overwrite CarSim `LastRun` during automatic verification.
- Stage only task-owned hunks at each commit. In particular, do not commit pre-existing edits in dirty source files or the binary `pmpc_mil.slx`; leave an inseparable model edit uncommitted and report it.

## Review Focus

1. **Velocity reversal and zero-speed bias:** the new plant must preserve a finite nonzero v14 force at zero velocity; Task 2's golden test and Task 5's force-path test pin this.
2. **State isolation:** four corners with different histories must not share GRU/current-lag state; Task 2's independent-corner sequence test pins this.
3. **Current step:** input `dI/dt` and the finite current-lag convolution must match PyTorch after a 0→1.6 A step; Task 2's step test pins this.
4. **Out-of-table or low-authority inverse request:** all controllers must still emit finite `[0,1.6] A` commands and consistent reachable bounds; Task 3's edge tests pin this.
5. **Dirty `.slx` and channel meanings:** model update must preserve unrelated user wiring and log separate desired force/current/actual force; Task 5's wiring and logging tests pin this.

---

### Task 1: Reproducible checkpoint export and reference sequences

**Files:** Create `nlcsnn/v14_py/__init__.py`, `model_v3.py`, `model_v7.py`, `model_v8.py`, `model_v10.py`, `model_v11.py`, `model_v12.py`, `model_v14.py` from the supplied ZIP; create `nlcsnn/v14_py/export_v14_matlab.py`; create `nlcsnn/v14_weights.mat`, `nlcsnn/v14_golden_vectors.mat`; test with `tests/test_nlcsnn_v14_artifacts.m`.

**Interfaces:** Exported MAT weights contain `state_dict` arrays, `model_cfg`, `norm_cfg`, current-lag coefficient and exact fixed streaming-state length. Golden MAT contains normalized inputs, force, both GRU layer states, and filtered effective current for several independent sequences. Incremental PyTorch `gru(...,h0)` supplies both layer states.

- [ ] **Step 1: Write failing artifact test.** Assert checkpoint hash, required MAT fields, tensor shapes from `model_cfg`, finite golden force, and a zero-speed sequence with nonzero bias force.
- [ ] **Step 2: Run `runtests('tests/test_nlcsnn_v14_artifacts.m')`.** Expected: FAIL because v14 export artifacts do not exist.
- [ ] **Step 3: Export from the supplied checkpoint.** Use `torch.load(..., weights_only=True)` in an isolated CPU PyTorch environment; call the ZIP's v14 model with `eval()`, export its exact normalization/weights and fixed golden sequences. If no compatible PyTorch runtime is available, stop and obtain one before claiming parity; do not guess weights from the prose summary.
- [ ] **Step 4: Rerun the artifact test.** Expected: PASS; record the exact PyTorch version and source/checkpoint hashes in `nlcsnn/README.md`.
- [ ] **Step 5: Commit only Task 1 files.**

### Task 2: Codegen-compatible streaming MATLAB v14 forward

**Files:** Create `nlcsnn/nlcsnn_v14_step.m`, `nlcsnn/nlcsnn_v14_nfl.m`, `nlcsnn/nlcsnn_v14_gru.m`, `nlcsnn/nlcsnn_v14_heads.m`; modify `nlcsnn/nlcsnn_damper_init.m`, `nlcsnn/nlcsnn_damper_step.m`; create `tests/test_nlcsnn_v14_forward.m`.

**Interfaces:** `[F,h_next] = nlcsnn_damper_step(net,x,v,a,I,dI,temp,dt,h)` stays the public physical-units interface; `h` is a fixed column containing two 64-unit GRU states and finite current-lag history. The plant, not this state, owns the previous command needed for `dI`. Its length comes from exported metadata, not a hardcoded 8. Internal `nlcsnn_v14_step(net,u,h)` takes a normalized `6×1` input and returns normalized force plus updated state.

- [ ] **Step 1: Write failing forward tests.** Assert parity with all Task 1 golden sequences for force, both GRU states, and effective current; include four different histories, velocity reversal, nonzero zero-speed force, and a 0→1.6 A step.
- [ ] **Step 2: Run `runtests('tests/test_nlcsnn_v14_forward.m')`.** Expected: FAIL on missing v14 functions.
- [ ] **Step 3: Implement the streaming model.** Follow `model_v14.py` feature and head order, PyTorch GRU gate equations, same-step GRU output, finite current-lag kernel, and checkpoint normalization; use single precision and no per-step file I/O.
- [ ] **Step 4: Rerun forward tests.** Expected: PASS within a documented float32-reference tolerance, including state trajectories; no NaN or dimension change. Task 5 checks Simulink fixed-size compilation.
- [ ] **Step 5: Commit only Task 2 files.**

### Task 3: Offline reachable-bound and inverse-current lookup

**Files:** Create `nlcsnn/v14_py/build_v14_lut.py`, `nlcsnn/v14_actuator_lut.mat`, `nlcsnn/nlcsnn_v14_force_bounds_lut.m`, `nlcsnn/nlcsnn_v14_inverse_lut.m`, `tests/test_nlcsnn_v14_lut.m`, `scripts/validate_nlcsnn_v14_lut.m`.

**Interfaces:** `bounds = nlcsnn_v14_force_bounds_lut(lut,x,v,a)` returns four-corner CarSim-polarity lower/upper forces; `[i_cmd,F_est] = nlcsnn_v14_inverse_lut(lut,x,v,a,F_des,i_prev)` returns four bounded currents and table-estimated forces. `lut` carries axes, values, version/hash, and explicit out-of-domain limits; all controllers use the same file. Training/held-out histories come from distinct files under `D:/OneDrive - mail.scut.edu.cn/Python/MR-NLCSNN/Raw_Card_Categorized_All`.

- [ ] **Step 1: Write failing LUT tests.** Assert axis monotonicity, bounded finite current, ordered force bounds, endpoint saturation, low-authority hold, and output continuity at adjacent cells.
- [ ] **Step 2: Run `runtests('tests/test_nlcsnn_v14_lut.m')`.** Expected: FAIL because the LUT and functions are missing.
- [ ] **Step 3: Generate the fixed table.** Use v14 rollouts from representative rig histories; start with `(x,v,a,F_des)` inverse coordinates and fixed simulation temperature, select grid and compact storage by held-out error versus Flash size. Store force-bound and inverse surfaces from the same checkpoint; no online neural evaluations in the lookup functions.
- [ ] **Step 4: Validate on held-out histories.** Rerun unit tests and `validate_nlcsnn_v14_lut`; report median/p95/max force error (N and fraction of local authority), saturation, and out-of-domain frequency. If history dependence makes the table unsuitable, stop before wiring it into models and request a design decision.
- [ ] **Step 5: Commit only Task 3 files.**

### Task 4: Controller/allocator boundary without online v14

**Files:** Modify `setup_pmpc.m`, `controller/pmpc_step.m`, `controller/pmpc_block.m`, `controller/pmpc_nodelay_block.m`, `controller/mpc_block.m`, `controller/zeng_block.m`, `scripts/build_controller_variants.m`; create `controller/func_DamperForceRequest.m` if needed; update `tests/test_nlcsnn_controller_path.m` and `tests/test_controller_entrypoints.m`.

**Interfaces:** Each controller consumes four lower/upper force bounds from the external LUT, not `h_plant` or `NLCSNN.net`; its damper output is `Fd_des(4×1)` in controller corner order. The external allocation block owns `i_prev` and converts `Fd_des` to `i_cmd` using Task 3. Keep 54-channel `sys`; document that `sys(5:8)` represents requested force rather than achieved/plant force.

- [ ] **Step 1: Write failing controller tests.** Assert all four variants share bounds, output finite force requests, have no v14 weights in `Pm`, and never call `nlcsnn_damper_step` or `nlcsnn_predict_force` from the controller path.
- [ ] **Step 2: Run the two controller test files.** Expected: FAIL on old hidden-state input and in-controller inverse calls.
- [ ] **Step 3: Replace the boundary.** Preserve QP formulation and tuning; expose requested forces and consume the external bounds. Do not fold table interpolation into the PMPC optimizer.
- [ ] **Step 4: Rerun controller tests.** Expected: PASS for MPC, ZENG, PMPC, and PMPC-noDelay with unchanged QP dimensions.
- [ ] **Step 5: Commit only Task 4 files.**

### Task 5: Shared four-model Simulink plant, wiring, and logging

**Files:** Modify `controller/func_NLCSNNPlant4.m`, `scripts/build_nlcsnn_multirate.m`, `scripts/run_build_nlcsnn.m`, `mpc_mil.slx`, `zeng_mil.slx`, `pmpc_mil.slx`, `pmpc_nodelay_mil.slx`, `scripts/lib/func_EnableControllerLogging.m`; update `tests/test_nlcsnn_multirate_model.m` and `tests/test_controller_logging.m`; update `nlcsnn/README.md`.

**Interfaces:** At 1 kHz the plant takes external `i_cmd` and owns four independent Task 2 states plus a four-element previous-current delay for `dI`. At 100 Hz each controller sees Task 3 force bounds and emits `Fd_des`; a shared external LUT block produces `i_cmd`. CarSim receives raw v14 force directly. Log `Fd_des`, `i_cmd`, and actual `F_plant` with distinct names.

- [ ] **Step 1: Write failing wiring tests.** Assert expanded state dimensions, direct v14-to-CarSim force path (no passivity projection), same LUT block in all variants, unchanged unrelated PMPC lines, and separate force/current/plant logs.
- [ ] **Step 2: Run wiring and logging tests.** Expected: FAIL against existing 8×4 state and passivity wiring.
- [ ] **Step 3: Update functions and models in place.** Preserve unrelated user changes in `pmpc_mil.slx`; update builder scripts rather than creating an inconsistent one-off `.slx` edit.
- [ ] **Step 4: Rerun model tests and compile all four variants.** Expected: PASS with no output-size errors; if CarSim's `Solver_SF` library is unavailable in batch MATLAB, report compile unverified rather than claiming success.
- [ ] **Step 5: Commit separable Task 5 source/test hunks only.** Leave the pre-existing dirty binary `pmpc_mil.slx` uncommitted rather than including the user's changes without approval. Do not run real CarSim datasets or overwrite `LastRun`; hand off the manual co-simulation steps and LUT error report.
