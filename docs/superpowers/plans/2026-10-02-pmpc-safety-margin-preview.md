# PMPC Safety-Margin Preview Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace PMPC's 30 m bend-length trigger with a 10 ms, no-extra-QP safety witness and model-based transient-roll braking preview.

**Architecture:** Keep the existing PMPC QP and its five constrained LTR nodes. After each solve, extract the no-slack safety margin of the optimized and projected-warm-start control sequences; feed this witness into the next 10 ms speed-plan tick. Before the QP, a bounded-cost spatial roll and tire-reachability preview supplies braking lead distance; a current-boundary guard and same-tick brake override handle immediate risk without pretending that a 100 ms first prediction node is a 10 ms check.

**Tech Stack:** MATLAB R2024b, MATLAB Coder-compatible MATLAB Function blocks, Simulink, CarSim 2019, matlab.unittest.

**Spec:** `docs/superpowers/specs/2026-10-02-pmpc-controllable-safety-margin-design.md`

## Global Constraints

- Main controller remains at `Ts_exec=0.01 s`, with `Np=10`, `Nc=5`, `Ncons_r=5`, seven PMPC states, three control channels and the existing 54-channel CarSim interface.
- No auxiliary QP, no speed decision variable in the main QP, and no 100 ms monitor cache.
- The normalized witness is a feasible-sequence lower bound on maximum controllability; a negative witness is **not** proof of infeasibility.
- Use the existing 0 kg controller model against the 250 kg roof-cargo, μ=0.85, 70 km/h physical Slalom case; do not retune nominal cargo for that case.
- Preserve existing uncommitted user changes. Stage/commit only files owned by each task. Keep MPC, ZENG and current PMPC-noDelay behavior unchanged; the latter is not a clean delay-only ablation of the revised PMPC until deliberately synchronized later.
- Do not silently enlarge LTR's reliable five-node constraint window or claim hard real-time performance without measuring worst-case elapsed time.

## Review Focus

- A failed QP, nonfinite solution, or invalid projected candidate must yield unknown risk, never a positive safety witness; Task 1 tests this.
- A candidate with positive road margin but negative LTR margin (or the reverse) must report the worse first-layer margin; Task 1 tests row indexing and sign.
- At 20 m/s, a hazard with less than delay-plus-braking distance remaining must trigger before the road boundary is reached; Task 3 tests this.
- Low brake authority or friction-boundary preview must not produce an impossible deceleration or evaluate future high-μ road with current low μ; Tasks 2–3 test this.
- Variant 4, MPC, ZENG, `LongCoordMode=0` and the legacy curvature ablation must retain their existing outputs; Task 3 tests this.

---

### Task 1: Extract the existing QP's feasible-sequence safety witness

**Files:**
- Create: `controller/func_PMPCSafetyWitness.m`
- Create: `tests/test_pmpc_safety_witness.m`

**Interfaces:**
- Consumes: `func_PMPCSafetyWitness(MPCParameters, A_cons, b_cons, x_opt, exitflag, dUc, gc, pc)`; the `A_cons` row layout is `6*Nc` input, `4*Ncons_sh` layer-2, then `2*Ncons_r + 4*Ncons_env` layer-1 rows.
- Produces: fixed scalar struct fields `m_plan`, `m_candidate`, `m_witness`, `plan_valid`, `candidate_valid`, `unknown`. With a successful main solve, `m_witness` is `max` over valid witnesses, or `NaN` if neither is valid; if `exitflag~=1`, `unknown=true` and `m_witness=NaN` even when a projected fallback candidate exists. Set the first-layer slack coefficient to zero when evaluating each sequence; do not change the actual QP solution. `pc(14)` checks candidate input-row violation, and `pc(1)` is an independent cross-check of candidate margin.

- [ ] **Step 1: Write failing tests** in `tests/test_pmpc_safety_witness.m`: `testWorstOfRoadAndLTRRows`, `testCandidateIsFeasibleWitness`, `testNegativeDoesNotClaimGlobalInfeasibility`, and `testFailedOrNonfiniteSolveIsUnknown`. Synthetic fixed-size matrices place known residuals in the row blocks; assert exact margin signs and field types.
- [ ] **Step 2: Verify red** with `& 'D:\Program Files\MATLAB\R2024b\bin\matlab.exe' -batch "startup_pmpc; r=runtests('tests/test_pmpc_safety_witness.m'); assert(all([r.Passed]))"`; expect failure because the function is missing.
- [ ] **Step 3: Implement** the exact signature above, calculating row residuals for the optimized control and the already projected candidate; mark validity only when hard input rows and numeric values are valid. Use only fixed-size indexing and Coder-supported operations.
- [ ] **Step 4: Verify green** with the same command; expect all four tests to pass.
- [ ] **Step 5: Commit only** `controller/func_PMPCSafetyWitness.m` and `tests/test_pmpc_safety_witness.m`.

### Task 2: Build a bounded-cost transient safety preview

**Files:**
- Create: `controller/func_PMPCPreviewSafetyEnvelope.m`
- Create: `tests/test_pmpc_preview_safety_envelope.m`

**Interfaces:**
- Consumes: `func_PMPCPreviewSafetyEnvelope(MPCParameters, VehiclePara, Constraints, W, Projection, ParaHAT, DamperContext, vTarget, previewEndStation)`; `W(:,7)` is station, `W(:,5)` signed curvature, and `DamperContext.F_lo/F_hi` are in `[L1;R1;L2;R2]` order.
- Produces: `[vSafe, hazardStation, peakLTR, valid]` as fixed scalar outputs. Evaluate the path to the existing braking-distance preview bound, clipped at `previewEndStation`; use at most 80 spatial nodes and at most 1000 bounded 10 ms time substeps (skip roll-preview at `vTarget<5 m/s`). Use the roll state equation consistent with `func_DynamicalModel` and LTR expression consistent with `func_Envelope`; calculate aggregate `Md` bounds from `0.5*[-ldf;ldf;-ldr;ldr]` times the NLCSNN force box. Apply the 15 ms moment lag with a stable exact-ZOH update and compare nominal/current versus opposing-roll admissible force sequences, retaining the lower of their *peak* predicted LTRs. Also integrate a signed lateral-displacement deficit from `a_required=v^2*kappa` versus `a_available=clip(a_required, -mu_eff*g, mu_eff*g)` and compare it with the usable road half-width; this is an optimistic far-field tire-reachability screen, not proof of road feasibility. Evaluate target speed plus at most six bisection speeds; do not insert a QP or a 30 m persistence condition.

- [ ] **Step 1: Write failing tests**: `testStraightHasNoCap`, `testSustainedR69NeedsEarlierCap`, `testShortAlternatingCurvesUseTransientResponse`, `testLowerMuCannotRaiseCapForSameSignBend`, and `testPreviewStopsBeforeKnownMuChange`. Use deterministic synthetic road arrays and vehicle/force fixtures; assert finite outputs, monotonicity for the same-sign sustained bend, and no cap from a bend beyond the friction switch.
- [ ] **Step 2: Verify red** with `& 'D:\Program Files\MATLAB\R2024b\bin\matlab.exe' -batch "startup_pmpc; r=runtests('tests/test_pmpc_preview_safety_envelope.m'); assert(all([r.Passed]))"`; expect missing function.
- [ ] **Step 3: Implement** the signature above. Preserve the sign conventions from the two referenced model functions; clip all moment commands to sampled semi-active bounds and report `valid=false` for nonfinite force or path data.
- [ ] **Step 4: Verify green** with the same command; expect all five tests to pass.
- [ ] **Step 5: Commit only** the new preview function and its test.

### Task 3: Integrate the witness and preview without changing other controllers

**Files:**
- Modify: `config/Atuning_pmpc.m`, `setup_pmpc.m`, `controller/func_PMPCSpeedCoordinator.m`, `controller/pmpc_step.m`
- Modify: `tests/test_pmpc_speed_coordinator.m`, `tests/test_pmpc_longcoord_integration.m`, `tests/test_controller_tuning_files.m`

**Interfaces:**
- Consumes: Task 1's fixed witness struct and Task 2's four scalar preview outputs.
- Produces: PMPC-only `LongCoordMode=3` with fixed scalar `St.LongCoord` fields `safety_margin_prev`, `m_plan`, `m_candidate`, `m_witness`, `hazard_station`, `brake_distance`, `ltr_peak_preview`, `witness_unknown`. Preserve modes 0/1/2 and the existing 8×1 Simulink diagnostic port. In `pmpc_step`, the pre-QP planner uses the preceding 10 ms witness and current physical preview; the post-QP witness is stored for the next tick. An imminent current-boundary failure can tighten the current brake/PID request after QP without crediting its unexecuted deceleration to this tick's lateral predictor.

- [ ] **Step 1: Write failing tests**: 20 m/s hazard triggers at or before `v*(Long_delay+Ts_exec)+(v^2-vSafe^2)/(2*a_available)+max(v*Ts_exec,ds/2)`; short safe bend does not brake solely due to peak curvature; QP failure is not safe; low authority bounds force; force request is fed back into the following tick's `Vx_pred`; mode 0 and variants 1/2/4 retain baseline outputs. The current-boundary guard evaluates four corners after one `Ts_exec` using `e_y_dot≈Vy+Vx*e_psi` and can tighten `Fx_dem`/`Vset_pid` before `func_QPA_DB` in the same tick; it also updates `St.LongCoord.Fx_request` so achieved-ratio feedback uses the actual request. Keep the existing mode-1/2 regression tests for backwards compatibility.
- [ ] **Step 2: Verify red** with `& 'D:\Program Files\MATLAB\R2024b\bin\matlab.exe' -batch "startup_pmpc; r=runtests({'tests/test_pmpc_speed_coordinator.m','tests/test_pmpc_longcoord_integration.m','tests/test_controller_tuning_files.m'}); assert(all([r.Passed]))"`; expect the new assertions to fail and record any pre-existing unrelated failures separately.
- [ ] **Step 3: Implement** PMPC variant-3 default mode 3, preserving variant-4 tuning. Add the witness scalar fields to `P.S0.LongCoord`; keep the existing road margin in meters separate from normalized safety margin. Feed `DamperContext` to mode 3, retain old function-call compatibility for modes 0–2, replace the normal 30 m filter only in mode 3, and preserve `previewEndStation` at the combined-road friction switch. Guard all invalid inputs and solver failures.
- [ ] **Step 4: Verify green** with the Task 3 command and Task 1–2 commands; expect all targeted tests to pass. Compile/update `pmpc_mil` and `pmpc_nodelay_mil` with MATLAB `set_param(model,'SimulationCommand','update')`; require no dimension or Coder errors and unchanged 54/8 output dimensions.
- [ ] **Step 5: Run closed-loop evidence** using the same CarSim datasets for mode 0 versus 3 PMPC on Slalom70 μ0.85/roof250 (controller0), COMB90 DLC→J-turn, and standalone J-turn, recording `e_y` peak/RMS, speed trace, LTR peak/time >0.8, road crossing, DB/total brake, QP failures and worst per-tick cost. Check actual dataset/speed/mu before accepting each ERD; if CarSim is unavailable, record that limitation rather than fabricate numbers.
- [ ] **Step 6: Commit only** the Task 3 code/tests after successful unit/compile checks; do not commit unrelated dirty files or claim closed-loop improvement without the Step 5 data.
