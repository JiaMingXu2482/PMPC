# Slalom Ablation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the existing slalom datasets a valid three-controller payload-robustness and PMPC delay-ablation comparison.

**Architecture:** Route Slalom datasets to the exact CarSim X-Y reference path and actual road friction, while retaining a nominal zero-cargo controller model. Give PMPC-noDelay an independently editable tuning bundle initialized identically to PMPC. Verify CarSim run inputs before simulation and report actual outcomes.

**Tech Stack:** MATLAB R2024b, Simulink, CarSim 2019, matlab.unittest.

**Spec:** `docs/superpowers/specs/2026-10-02-slalom-ablation.md`

## Global Constraints

- Physical roof cargo 250 kg; controller model cargo 0 kg.
- Actual road friction 0.85 despite `Slalom_mu1_*` names.
- Initial speed 70 km/h; stop near Station 160 m; retain path past 160 m for preview.
- Initially compare PMPC, PMPC-noDelay, and ZENG; do not run MPC.
- Preserve pre-existing dirty files and do not commit or push without request.

## Review Focus

- A copied path must match the CarSim path at several nontrivial stations, not merely start/end.
- Path preview near Station 160 m must not clamp prematurely.
- The `mu1` label must not result in controller friction 1.0 when actual road friction is 0.85.
- Variant 4 tuning must be independent while matching variant 3 initially, including longitudinal and priority settings.
- Each CarSim result must correspond to the intended dataset and actual 70 km/h initial state; never compare stale outputs.

---

### Task 1: Slalom path and maneuver routing

**Files:**
- Create: `data/Slalom_CarSim_XYS.txt` from user-supplied path
- Modify: `scripts/lib/func_ManeuverFromName.m`, `controller/func_WayPoints.m`, `scripts/lib/func_Metrics.m`, `setup_pmpc.m`
- Test: `tests/test_slalom_course.m`

**Interfaces:**
- Consumes: user X-Y-Station table with 373 rows
- Produces: `func_ManeuverFromName('Slalom_mu1_PMPC')` with type 2, mu 0.85, stop station 160; `func_WayPoints(2,[],false)` with CarSim-matching path

- [ ] Write tests for Slalom routing, numerical path matches at start/middle/end, station preview beyond 160 m, and metric path selection.
- [ ] Run tests and confirm failure on the current DLC fallback / generated sine path.
- [ ] Copy the supplied path into the repository; implement Slalom routing and shared path consumption with existing 9-column waypoint convention.
- [ ] Run targeted tests to green, then run nearby maneuver/path regression tests.

### Task 2: Independent PMPC-noDelay tuning

**Files:**
- Create: `config/Atuning_pmpc_nodelay.m`
- Modify: `setup_pmpc.m`
- Test: `tests/test_pmpc_nodelay.m`

**Interfaces:**
- Consumes: `Atuning_pmpc()` default values
- Produces: `Atuning_pmpc_nodelay()` independent tuning struct selected only for variant 4

- [ ] Write tests showing variant 4 selects its own tuning, initially equals PMPC numerically, and variant 3 is unaffected by temporary noDelay tuning changes.
- [ ] Run tests and confirm failure because the separate tuning function is missing.
- [ ] Add the independent bundle and route all variant-4 control/priority/longitudinal settings to it.
- [ ] Run targeted tests and related controller configuration regressions.

### Task 3: CarSim input validation and three-run comparison

**Files:**
- Modify only in-scope Slalom CarSim datasets if necessary; preserve unrelated runs.
- Test: actual expanded CarSim inputs and three MIL runs.

**Interfaces:**
- Consumes: three existing Slalom datasets and `run_current_carsim` entry point
- Produces: verified 70 km/h, mu 0.85, cargo 250 kg, stop Station 160 m and e_y/LTR results for PMPC, PMPC-noDelay, ZENG

- [ ] Inspect all three live Run datasets and expanded parameters; identify stale 60 km/h or 200 m values.
- [ ] Update only mismatched Slalom Run settings and re-Send to Simulink as needed.
- [ ] Run targeted tests and full repository test suite, reporting any pre-existing failures by name.
- [ ] Run PMPC, PMPC-noDelay, ZENG at identical Slalom conditions; compare e_y, LTR, speed, and completion station.
