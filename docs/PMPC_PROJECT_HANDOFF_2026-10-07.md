# PMPC project continuation log — 2026-10-07

## Start here after changing accounts

Open this file first, then run `git status --short` before editing anything.  The
working tree intentionally contains ongoing, **uncommitted** user work; do not use
`git reset`, `git checkout`, or overwrite the Simulink models.

Suggested first message to the next account:

> Read `docs/PMPC_PROJECT_HANDOFF_2026-10-07.md` completely and continue this
> PMPC/CarSim–Simulink project from its current state. Preserve the listed dirty
> files. The immediate design decision is whether to integrate the supplied
> compact GRU inverse allocator instead of the failed fixed inverse LUT; do not
> alter CarSim LastRun until I ask for a manual co-simulation.

Repository and branch:

```text
D:\OneDrive - mail.scut.edu.cn\周报\4.6\4.7All\DLC_0.85_100\PMPC\SUV_DLC_PMPC\Simulink_Model
branch: codex/pmpc-longcoord
```

## Current project objective and controller scope

The project compares vehicle coordinated controllers in CarSim–Simulink:

| Model | Purpose |
|---|---|
| `mpc_mil` | Conventional MPC baseline |
| `zeng_mil` | ZENG priority/weighting comparison |
| `pmpc_mil` | Proposed priority MPC (PMPC) |
| `pmpc_nodelay_mil` | PMPC ablation without damper delay |

The paper/control narrative currently rests on two contributions:

1. A coordinated-control architecture including a **semi-active** suspension,
   whose nonlinear and time-varying damper delay is obtained through a
   NLCSNN/digital-twin route.  The later lookup-delay formulation remains a
   paper/code task; the current active controller has not yet been converted to
   a delay lookup.
2. A PMPC objective hierarchy with theoretically selected penalty weights.  It
   prioritizes road/roll safety (L1), lateral stability (L2), then tracking and
   actuator use (L3).  This is a hierarchy of **control objectives**, not a
   claim that physical actuators must switch on/off sequentially.

Key implementation conventions:

- Vehicle control runs at 100 Hz (`Ts=0.1 s`, typically `Np=10`, `Nc=5`);
  the damper plant is intended to run at 1 kHz.
- All controller variants should use the same longitudinal layer.  The prior
  PMPC-specific longitudinal rule was removed/zeroed to keep comparisons fair.
  Do not add maneuver-specific curve-speed rules.
- The PMPC nonlinear damper forward model is deliberately outside the MPC/QP
  and outside the future S32K controller.  Its inverse/allocation is also meant
  to be outside the QP boundary.
- The COM route now uses a DLC followed by a longer straight transition and an
  R=80 J-turn.  DLC begins at 90 km/h and the intended J-turn target speed is
  80 km/h.  CarSim road radius and controller reference radius must agree.
- The user manually runs final CarSim–Simulink experiments.  Do not overwrite
  CarSim `LastRun` during automated tests.

The current PMPC tuning source is `config/Atuning_pmpc.m`.  Its pre-existing
defaults include `Ts=0.1`, `Np=10`, `Nc=5`, and a three-input QP convention
(AFS, differential-brake moment, damper action).  It is currently dirty, so
read the file rather than relying on an older snapshot.  The priority penalty
settings recently discussed are `prio_vartheta`, `prio_wmin`, and
`prio_wmax`; their exact current values must be read from the dirty file.

## Important uncommitted work — preserve it

At handoff, `git status --short` reports the following.  These files were not
included in the commits below and must be treated as the user’s current working
state:

```text
 M config/Atuning_mpc.m
 M config/Atuning_pmpc.m
 M controller/pmpc_block.m
 M controller/pmpc_nodelay_block.m
 M pmpc_mil.slx
 M run_current_carsim.m
 M simfile.sim
 M tests/test_shared_longitudinal_cleanup.m
?? controller/func_PriorityDiagnostics.m
?? docs/superpowers/plans/2026-10-07-nlcsnn-v14-replacement.md
?? scripts/lib/func_EnableControllerLogging.m
?? scripts/lib/func_SaveControllerLogs.m
?? tests/test_controller_logging.m
```

This ongoing work is primarily controller logging and current PMPC tuning/model
edits.  The requested logging pattern is to save `logsout` after a successful
manual simulation in the CarSim result directory:

```matlab
evalc('simOut = sim(model);');
logsout = simOut.logsout;
save(fullfile(resdir, [model '_logs.mat']), 'logsout', '-v7.3');
```

Expected useful logged signals include plant/controller outputs (`sys`,
`i_cmd`), requested and actual damper force/current, and PMPC priority
diagnostics such as the level slacks/weights and DB gate.  Before relying on
these signals, inspect `func_EnableControllerLogging.m`,
`func_SaveControllerLogs.m`, and the dirty `run_current_carsim.m` to confirm
their actual logging names and model wiring.

## Recent committed history

The latest committed NLCSNN work is intentionally incremental and reversible:

```text
7d863b0 experiment: evaluate fixed v14 damper inverse LUT
5a50774 fix: keep legacy damper default until v14 model wiring
b3edeb9 feat: port NLCSNN v14 streaming forward model
6b134e8 feat: export NLCSNN v14 checkpoint and golden traces
3466145 docs: design NLCSNN v14 plant and inverse LUT replacement
adcc104 docs: use supplied Eischens bibliography key
8973329 docs: record manuscript alignment with current controller
7e6e870 docs: record COM co-simulation cache fix and validation
df4246a refactor: keep shared longitudinal control across variants
5e7dc50 docs: design shared longitudinal controller cleanup
82e47e6 chore: consolidate results and snapshot 7x3 control baseline
c714ccb feat: add PMPC Vx/Fx QP baseline and experiment log
```

`c714ccb` is a historical Vx/Fx-QP experiment, not the current preferred paper
direction.  The user subsequently decided to avoid adding Vx/Fx into the PMPC
QP so that all controllers remain comparable and the contribution stays focused.

The source-controlled v14 design specification is
`docs/superpowers/specs/2026-10-07-nlcsnn-v14-replacement-design.md`.

## NLCSNN v14 replacement: completed work

### Source and physical mapping

Supplied checkpoint and source package:

```text
D:\OneDrive - mail.scut.edu.cn\周报\10.8\nlcsnn\v14_global_best_12.55pct_E1_session162406.pth
D:\OneDrive - mail.scut.edu.cn\周报\10.8\nlcsnn\nlcsnn_v14_py_model.zip
checkpoint MD5: 8a3bd0d508429ab2c00083321561561a
```

The v14 model has normalized six inputs `[x, v, a, I, dI/dt, T]`, a static
nonlinear feature map (35 features), two 64-unit GRU layers, learned current
lag, and `c/s/b` force heads.  Unlike the former passive projection, its bias
head can yield nonzero force near zero velocity.

CarSim/rig mapping is fixed as follows:

```text
x_rig     = x_ref - cmpD
v_rig     = -cmpRD
F_CarSim  = -F_v14
temperature used for simulation = 42.5 degC
current command range = [0, 1.6] A
```

The user explicitly chose **raw v14 force**.  Therefore, after final wiring,
remove `SemiActive_Passivity_1kHz` from the active force path; retain only
finite-value/domain guards, not a rule that forces zero-speed output to zero.

### Exact MATLAB forward port

The following is committed:

```text
nlcsnn/v14_weights.mat
nlcsnn/v14_golden_vectors.mat
nlcsnn/nlcsnn_v14_nfl.m
nlcsnn/nlcsnn_v14_gru.m
nlcsnn/nlcsnn_v14_heads.m
nlcsnn/nlcsnn_v14_step.m
nlcsnn/nlcsnn_damper_init.m
nlcsnn/nlcsnn_damper_step.m
nlcsnn/v14_py/...
tests/test_nlcsnn_v14_artifacts.m
tests/test_nlcsnn_v14_forward.m
```

It matches the supplied PyTorch streaming order, including the actual `conv1d`
current-lag operator.  Do not replace it with a visually plausible exponential
moving average without revalidating: the source implementation’s effective
time direction differs from the nearby explanatory comment.

One v14 corner state contains 227 values: 128 GRU states plus 99 current-lag
history values (lag kernel length 100).  Four independent corners are required.

**Safety gate:** `nlcsnn_damper_init()` stays legacy by default and loads
`nlcsnn_weights.mat`.  To instantiate v14 explicitly use:

```matlab
P = nlcsnn_damper_init('nlcsnn/v14_weights.mat');
```

This is deliberate: the existing Simulink plant still owns an 8-by-4 legacy
damper-state contract, and handing it a 227-state v14 structure before rewiring
would break the model.  No four-model Simulink rewiring and no CarSim validation
has been performed for v14 yet.

The isolated v14 tests passed (9/9):

```matlab
r = runtests({'tests/test_nlcsnn_v14_artifacts.m', ...
              'tests/test_nlcsnn_v14_forward.m', ...
              'tests/test_nlcsnn_v14_lut.m'});
assertSuccess(r)
```

An older, unrelated baseline test remains known-failing before this work:
`tests/test_nlcsnn_smoke.m`,
`testMpcZengAndPmpcAdvanceCommonPlant` (line 43: actual 7, expected 8).
The user explicitly accepted continuing while recording this baseline failure.
Do not claim that the entire test suite is green.

## Fixed inverse LUT experiment — completed and rejected

The originally planned controller-side static LUT was built and evaluated but
must **not** be integrated as the final inverse.  Files:

```text
nlcsnn/nlcsnn_v14_curve_lut.m
nlcsnn/nlcsnn_v14_force_bounds_lut.m
nlcsnn/nlcsnn_v14_inverse_lut.m
nlcsnn/v14_actuator_lut.mat
nlcsnn/v14_lut_validation.mat
nlcsnn/v14_lut_validation_report.md
nlcsnn/v14_py/build_v14_lut.py
scripts/validate_nlcsnn_v14_lut.m
tests/test_nlcsnn_v14_lut.m
```

It is a small 7x9x5x9 `(x,v,a,I)` table (11,340 bytes), constructed from 32
processed rig histories and tested on 9 checkpoint-held-out files, 242 points.
Data are traceable through:

```text
processed: D:\OneDrive - mail.scut.edu.cn\Matlab\CDCTest\Parsed_Card_force_wide_zerofix
raw archive: D:\OneDrive - mail.scut.edu.cn\Python\MR-NLCSNN\Raw_Card_Categorized_All
```

Do not feed raw acquisition files directly as physical model channels; use the
processed data pipeline.

Validation results prove that a history-free table is not accurate enough for
v14’s GRU/current-lag dynamics:

| Metric | Result |
|---|---:|
| Absolute force error, median / p95 / maximum | 3.0 / 88.7 / 575.0 N |
| p95 error as local available-force-span fraction | 0.931 |
| Samples with authority >=100 N | 38.8% |
| p95 error in authority >=100 N region | 108.6 N / 0.591 span |
| Endpoint saturation | 60.7% |
| Out-of-domain samples | 0.8% |
| p95 full-history vs held-command mismatch | 111.5 N |

Conclusion: the fixed LUT is retained only as a documented negative experiment.
Tasks that would place it at the controller/Simulink boundary were intentionally
stopped; do not silently use it because it is fast.

## Supplied compact-GRU inverse package — analysis and recommended next decision

The user subsequently supplied:

```text
D:\OneDrive - mail.scut.edu.cn\周报\10.8\nlcsnn\inverse_model_package.zip
D:\OneDrive - mail.scut.edu.cn\周报\10.8\nlcsnn\inverse_validation_log.md
```

Package contents include `fwd_student_g1x16.pth`, training/validation scripts,
README, and validation logs.  The PTH is a compact **1x16 GRU forward student**
with `c/s/b` heads (8,277 parameters).  It is used inside a bisection allocator,
not as an arbitrary black-box direct inverse:

1. Static teacher mappings are intended as offline 1D/2D tables.
2. The 1x16 recurrent student preserves limited history online.
3. For each desired force, 12 bisection steps solve `I_e in [0,1]`.
4. Convert with `I=(1-I_e)*1.6` A.
5. Keep one 16-float state per wheel.

This is materially more suitable than the failed history-free LUT: it preserves
history at small embedded cost, while the full v14 teacher remains outside the
controller.  Package claims approximately 1.3 ms for four wheels at 100 Hz and
about 40 KB flash, but these are estimates, **not S32K board measurements**.

Reported validation against the v14 teacher in >=100 N authority regions:

```text
current absolute error: MAE 13 mA, p95 51 mA
```

However, force-chain validation exposes limitations, especially when authority
is low or the commanded-current trajectory departs from the teacher history:

| Held-out case | Force error p50 / p95 | Current error p50 / p95 |
|---|---:|---:|
| SH 50 mm, 0.41 Hz, 0.4 A | 16 / 106 N | 398 / 871 mA |
| SH 50 mm, 0.41 Hz, 1.2 A | 96 / 306 N | 1192 / 1195 mA |
| SR RMS 1.67 mm, 4.97 Hz, 1.6 A | 8 / 36 N | 1589 / 1592 mA |

The available force span is only about 55–163 N in low-authority regions; hence
large current mismatch can coexist with a smaller force consequence.  A 1x36
student did not remove a systematic roughly 43 N bias on one file.  Simple 5/15
ms current filtering did not help and damaged high-frequency behavior.  Student
current jitter p95 was 36 mA versus teacher 10 mA raw.

The package is not fully reproducible by itself: scripts import
`build_distill_data.py`, but that file is absent.  Preserve this fact in any
paper/code claim.

**Recommended next action, pending the user’s decision:** integrate the compact
GRU allocator only after adding an actuator-level low-authority policy: when the
force-current slope/span is too small or a request is outside reachable bounds,
choose the feasible current closest to the previous current (or hold it) rather
than applying a global low-pass filter.  Then verify it against the v14 teacher
on held-out histories before touching any `.slx` plant.  This policy belongs to
actuator allocation, not to maneuver-specific PMPC logic.

## Safe implementation sequence from here

1. Confirm with the user whether to use the compact-GRU allocator plus the
   low-authority/previous-current selection policy.
2. Import/extract the supplied package into a tracked `nlcsnn/` subdirectory;
   export frozen weights/tables and make a MATLAB single-precision step function.
   Keep it outside `pmpc_block`/the QP.
3. Add parity and held-out trajectory tests against the v14 teacher.  Record
   force error, current jitter, saturation, and out-of-domain/low-authority
   counts.  Do not call an estimated runtime an S32K timing result.
4. Only if it passes its quality gate, create a shared external allocator for
   all four models.  It maps requested four damper forces to `[0,1.6]` A
   commands at 100 Hz; the 1 kHz forward plant holds commands between updates.
5. Rewire **all four** `.slx` models together for the 227-state/corner v14
   plant, remove `SemiActive_Passivity_1kHz` from the active force path, and
   preserve old model behavior as a Git-recoverable baseline.  Update fixed-size
   declarations and Simulink delays consistently.
6. Extend logging for force request, allocator current, raw v14 force, current
   saturation, and PMPC diagnostics.  Let the user manually run CarSim and save
   `*_logs.mat` for each case.
7. Revisit paper figures only after the new plant/allocation has passed manual
   CarSim–Simulink validation.  PIL work is deferred until parameters settle.

## Manuscript context

The main manuscript copies are outside this repository, e.g.
`D:\Chrome_Download\main_TVT_Revised (1).tex` and later numbered revisions.
Recent edits condensed the priority optimization/theory/lower-level sections,
combined appendices, and rearranged the integrated 7-state vehicle/damper model.
The cited NLCSNN source is Eischens et al., *Reliability Engineering & System
Safety*, 2025, DOI `10.1016/j.ress.2025.110946`.  Keep the damper reachable-force
bounds notation as `F_{dij}^{l}` and `F_{dij}^{u}`.  The paper may state the
NLCSNN/digital-twin methodology by reference; do not claim that the static LUT
was accepted or that the v14 model was already deployed on S32K.

## Commands useful to resume

Inspect work safely:

```powershell
git status --short
git log --oneline -15
```

Run the focused v14 MATLAB tests from repository root:

```powershell
& 'D:\Program Files\MATLAB\R2024b\bin\matlab.exe' -batch "r=runtests({'tests/test_nlcsnn_v14_artifacts.m','tests/test_nlcsnn_v14_forward.m','tests/test_nlcsnn_v14_lut.m'}); assertSuccess(r);"
```

Do not use the existing global smoke test as proof of success until its legacy
7-versus-8 expected-state mismatch has been independently resolved.

