# Controller Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce `controller/` to active controller algorithms while preserving the verified `pmpc_mil` interfaces and MPC/ZENG/PMPC comparison behavior.

**Architecture:** Remove data paths proven dead by static usage analysis, localize setup-only helpers inside `setup_pmpc.m`, and move project/CarSim/result utilities to `scripts/lib/` without renaming public functions. Keep focused runtime algorithms separate so `pmpc_step.m` remains an orchestrator rather than becoming a monolith.

**Tech Stack:** MATLAB R2024b, Simulink, CarSim MATLAB solver library, `matlab.unittest`, Git.

**Spec:** `docs/superpowers/specs/2026-09-27-controller-cleanup-design.md`

## Global Constraints

- `pmpc_mil.slx` remains the sole Simulink model.
- Preserve MPC, ZENG, and PMPC comparison modes and all currently connected maneuver/diagnostic branches.
- Preserve `pmpc_block(u, PMPC_P) -> sys`, the 54-element controller output, and all active CarSim dataset interfaces.
- Do not modify `simulation_results/current/erd_0927_base/`.
- Do not add third-party dependencies.

## Review Focus

- A clean MATLAB session must find controller and utility functions after `startup_pmpc`.
- `setup_pmpc` must still build `P.Pm` and `P.S0` after setup helpers become local functions.
- Removing `C_table`, `Cf0`, and `Cr0` must not leave a read in runtime code or a generated-code parameter dependency.
- A base-workspace override must still take precedence after `wsget` becomes local.
- Verification must not commit the MAT-file rewrite produced by `func_WayPoints` during setup.

---

### Task 1: Remove confirmed dead controller code and parameters

**Files:**
- Modify: `tests/test_project_layout.m`
- Modify: `setup_pmpc.m`
- Modify: `controller/pmpc_step.m`
- Delete: `controller/func_bezierInterp.m`
- Delete: `controller/func_FindBezierControlPointsND.m`
- Delete: `controller/func_RLSFilter_Calpha_f.m`
- Delete: `controller/func_RLSFilter_Calpha_r.m`
- Delete: `controller/func_tire_init_Calpha.m`
- Delete: `controller/func_Fz_Calpha.m`

**Interfaces:**
- Consumes: Existing `setup_pmpc() -> P` and `pmpc_step(u, Pm, St) -> [sys, St]`.
- Produces: The same interfaces with unused `P.C_table`, `P.Cf0`, `P.Cr0`, `P.Pm.C_table`, `P.Pm.Cf0`, and `P.Pm.Cr0` fields removed.

- [ ] **Step 1: Add `testDeadControllerFilesAreAbsent` and `testDeadParameterFieldsAreAbsent` to `tests/test_project_layout.m`**

Assert the six dead function files do not exist. Build `P = setup_pmpc()` and assert `C_table`, `Cf0`, and `Cr0` are absent from both `P` and `P.Pm`; also assert `controller/pmpc_step.m` no longer reads those names.

- [ ] **Step 2: Run the new tests and verify they fail**

Run: MATLAB `runtests('tests/test_project_layout.m')`.
Expected: failure because the files and fields still exist.

- [ ] **Step 3: Remove the unused setup calls, fields, and runtime unpacking statements, then delete the six dead files**

Do not change active tire lookup fields `TireF` or `TireR`.

- [ ] **Step 4: Run the layout and timing tests**

Run: MATLAB `runtests({'tests/test_project_layout.m','tests/test_timing_configuration.m'})`.
Expected: all tests pass.

- [ ] **Step 5: Restore `data/WayPoints_Type1.mat` if setup rewrote it and commit**

Commit message: `refactor: remove dead controller code`.

### Task 2: Localize setup-only helpers

**Files:**
- Modify: `tests/test_project_layout.m`
- Modify: `setup_pmpc.m`
- Delete: `controller/func_InitialParams.m`
- Delete: `controller/wsget.m`

**Interfaces:**
- Consumes: Existing initialization structure produced by `func_InitialParams` and override semantics from `wsget(name, default)`.
- Produces: Local functions `localInitialParams() -> struct` and `localWsget(name, default) -> value` in `setup_pmpc.m`, with identical field values and workspace/CarSim lookup behavior.

- [ ] **Step 1: Extend layout tests for localized helpers**

Assert the two public helper files are absent, `P.S0.InitialParams` contains `InitialGapflag`, `prevstate`, and fixed diagnostic buffers, and a base-workspace override such as `PMPC_TS` is honored.

- [ ] **Step 2: Run the targeted tests and verify they fail**

Run: MATLAB `runtests('tests/test_project_layout.m')`.
Expected: failure because the helper files still exist.

- [ ] **Step 3: Move helper bodies into `setup_pmpc.m` and update calls**

Use exact local signatures `localInitialParams()` and `localWsget(name, dflt)`. Replace every `wsget` call in `setup_pmpc.m` with `localWsget` and delete the two standalone files.

- [ ] **Step 4: Run layout, timing, and setup override tests**

Run: MATLAB `runtests({'tests/test_project_layout.m','tests/test_timing_configuration.m'})`.
Expected: all tests pass and `PMPC_TS` override is isolated/cleaned by the test.

- [ ] **Step 5: Restore generated data changes and commit**

Commit message: `refactor: localize controller setup helpers`.

### Task 3: Separate project utilities from controller algorithms

**Files:**
- Create directory: `scripts/lib/`
- Move: the eleven utility files listed in the design spec from `controller/` to `scripts/lib/`
- Modify: `startup_pmpc.m`
- Modify: `tests/test_project_layout.m`
- Modify: `README.md`

**Interfaces:**
- Consumes: Existing public utility function names and signatures.
- Produces: The same callable names from `scripts/lib/`, added to the MATLAB path by `startup_pmpc`.

- [ ] **Step 1: Add `testUtilitiesLiveOutsideController` to the layout test**

After `startup_pmpc`, assert `which('func_ProjectRoot')`, `which('func_RunMode')`, and `which('func_Metrics')` resolve under `scripts/lib/`; assert all eleven old controller paths are absent.

- [ ] **Step 2: Run the new test and verify it fails**

Run: MATLAB `runtests('tests/test_project_layout.m')`.
Expected: failure because utilities still resolve from `controller/`.

- [ ] **Step 3: Move the utility files and add `scripts/lib/` in `startup_pmpc`**

Do not rename functions or alter their behavior. Update README directory descriptions only.

- [ ] **Step 4: Run targeted tests and a static reference scan**

Run: MATLAB layout/timing tests and `rg` for the deleted function names and fields.
Expected: tests pass; no active references remain except negative assertions in tests/design history.

- [ ] **Step 5: Commit the directory boundary change**

Commit message: `refactor: separate controller and project utilities`.

### Task 4: Full verification and push

**Files:**
- Modify only if verification exposes a stale test whose assumptions no longer match the preserved public interface.

**Interfaces:**
- Consumes: Tasks 1-3 outputs.
- Produces: A clean, pushed `main` branch with verified layout and model loading.

- [ ] **Step 1: Run MATLAB unit tests**

Run: MATLAB `runtests('tests')`.
Expected: zero failures. If a pre-existing stale test fails, confirm the production interface first and update only the stale fixture/assertion.

- [ ] **Step 2: Run setup and model-load smoke verification**

Run `startup_pmpc`, `setup_pmpc`, and `load_system('pmpc_mil')`; verify `Ts=0.05`, `Ts_exec=0.01`, and model name `pmpc_mil`.

- [ ] **Step 3: Restore generated data and verify repository state**

Restore only `data/WayPoints_Type1.mat` if its content changed from the setup side effect. Run `git diff --check`, confirm one `.slx`, confirm removed files are absent, and confirm `git status --short` is clean after the final commit.

- [ ] **Step 4: Push `main`**

Run: `git push origin main`.
Expected: remote `main` advances to the controller-cleanup commits.
