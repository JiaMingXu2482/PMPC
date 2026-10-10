# PMPC follow-up: repair infeasible damper moment boxes first

> Superseded default: after the user's ZENG/chatter comparison and further
> closed-loop trials, terminal constraints are OFF by default. This document
> records the earlier three-step selection, not the current recommendation.
> See `pmpc_continuous_risk_and_steering_2026-10-10.md` for the final disposition.

## Confirmed cause, reconstructed from saved closed-loop logs

The historical bound shrink scaled each endpoint relative to zero separately.
For a narrow one-sided interval this reverses the endpoints. For example,
the physical interval [98,100] Nm became [107.8,90] Nm. The subsequent
rate-limit box relaxation does not guarantee correction of an already
reversed interval. This is a **hard infeasibility**, not a safety-slack or
priority-weight tuning issue. KWIK sometimes labels it as numerical failure.

`scratchpad/terminal_improve_20261009/audit_cdc_box.m` reconstructs both the
raw shrink and the rate-relaxed QP box from controller_sys channels 20:22,
and compares it with the saved priority_diag solver status.

| Run directory suffix | Reversed QP boxes | Failed samples, including initialization | Reversed box AND failure |
|---|---:|---:|---:|
| refine_20261009_184137_firstTc0 | 243 | 244 | 243 |
| terminal_20261009_191635 | 189 | 190 | 189 |
| terminal_20261009_193401 | 158 | 161 | 158 |
| terminal_20261009_194056 | 301 | 302 | 301 |

The first and second terminal-development experiments therefore did NOT
isolate an otherwise feasible QP. Earlier descriptions of these failures as
primarily numerical conditioning were incomplete.

## Repair

`controller/func_ShrinkDamperMomentBox.m` retains the old result when it is
ordered. If it is reversed, it contracts the physical interval around its
midpoint, retaining 90% of its width. Positive/negative singletons remain
the same physical singleton, not an impossible demand towards zero.

The repair is selected only for ControllerVariant==3 (PMPC) in
`func_BuildQPConstraints`. MPC/ZENG behavior and their tuning are untouched;
their shared legacy shrink branch has not been silently changed. If this
repair is later adopted for those comparators, rerun them for a fair comparison.

No road radius, speed reference, tracking weights or intervention penalty
was changed in this repair. The old weighted single-QP path is the default.

## Experimental paths NOT selected by default

Road-preview/rate-aware backup and nonzero steady-DB center selection are
retained in `func_TerminalRecoverabilityPreview.m`, selected only with
`PMPC_TERMINAL_PREVIEW=1`. The default original terminal construction is in
`func_TerminalRecoverability.m` (three backup steps).

The sequential solver is opt-in via `PMPC_LEX_ON=1`. It is NOT a validated
replacement for the original QP. The interior-point experiment in
`terminal_20261009_194717` failed to complete the road maneuvers and is rejected;
it must not be interpreted as improved steering merely because held controls
gave a smaller step RMS. This optional path also depends on the newer MPC
Toolbox interior-point API, inspected in the installed MATLAB R2024b files;
R2018a/S32 deployment has not been validated.

The raw experiment directories, logs, parameter snapshots and source copies
are retained. Nothing has been deleted or committed to Git in this task.

## Final comparison

Same R75 COM road, same tuning and longitudinal reference:

| Metric | Previous terminal, invalid-box bug | Repaired terminal | Repaired, terminal off |
|---|---:|---:|---:|
| DLC peak abs ey (m) | 0.976934 | 1.057700 | 0.834709 |
| DLC RMS ey (m) | 0.348378 | 0.357896 | 0.297228 |
| J-turn peak abs ey (m) | 0.500793 | 0.464478 | 0.531219 |
| J-turn RMS ey (m) | 0.192708 | 0.165404 | 0.393520 |
| J-turn RMS Vx (km/h) | 78.064 | 78.022 | 78.135 |
| Steering command step RMS, t>=12.5 s (deg, steering wheel) | 2.9218 | 1.0560 | 1.2804 |
| Failed QPs, excluding initial NaN | 189/1999 | 0/1999 | 0/1999 |

The repaired terminal run still has a DLC performance regression; this is
NOT an overall win or a road-safety guarantee. Its road-out metric is 4.2%,
versus 1.4% with terminal rows off. The terminal core is valid in 86.45% / 84.50%
of logged samples, not 100%; invalid construction keeps the original stage
constraints and does not assert terminal safety.

Its positive DB request first exceeds 50 Nm for five consecutive samples
7.1 m before turn entry; negative DB first does so 20.7 m before entry.
These are requested yaw moments, not guaranteed wheel torque or a proven
longitudinal braking benefit. The unchanged seven-state predictor freezes Vx.

Results (relative to project root):

- `scratchpad/com_r75_db_20261008/terminal_20261010_085317/Results/Run_trial`
- `scratchpad/com_r75_db_20261008/cdcbox_weighted_no_terminal_20261010_085646/Results/Run_trial`

Both include parameter snapshots, `pmpc_mil_logs.mat`, reports and CarSim files.
The terminal run includes `terminal_source.zip`; the terminal-off run uses the
same sources with terminal rows disabled. Auditing each run with the repaired
box formula finds zero inverted intervals and zero noninitial solve failures.
Four box regression tests and ten terminal/experimental-solver unit tests passed.

## Final bounded alternatives (after the box repair)

| Configuration | DLC peak / RMS ey (m) | J-turn peak / RMS ey (m) | Late steering step RMS (deg) | Failed QPs |
|---|---:|---:|---:|---:|
| Original terminal, 3 backup steps (selected) | 1.057700 / 0.357896 | 0.464478 / 0.165404 | 1.0560 | 0/1999 |
| Road-preview backup, 3 steps | 1.292009 / 0.484552 | 0.562073 / 0.224411 | 1.4468 | 0/1999 |
| Original terminal, 5 backup steps | 1.082571 / 0.360978 | 0.410583 / 0.147670 | 1.2767 | 0/1999 |

The road-preview path remains rejected: repairing feasibility did not make
its closed-loop performance competitive. Its run is
`scratchpad/com_r75_db_20261008/terminal_20261010_085837`.

The five-step run is retained as an alternative, not the default:
`scratchpad/com_r75_db_20261008/terminal_20261010_090246`.
It improves J-turn RMS by 10.7% relative to three steps, but increases late
steering step RMS by 20.9% and slightly worsens DLC tracking. Its road-out
metric is 4.1%; it is also not a road-safety guarantee. Both alternatives
include saved parameters, control logs, reports and source snapshots.

## Selected configuration and manual rerun

Defaults remain the original terminal construction, three backup steps,
and the weighted single QP. Set these explicitly in the MATLAB workspace
to avoid stale experimental overrides:

```matlab
PMPC_TERMINAL_ON = 1;
PMPC_TERMINAL_STEPS = 3;
PMPC_TERMINAL_PREVIEW = 0;
PMPC_LEX_ON = 0;
run_current_carsim('pmpc_mil');
```

These commands use the user's currently sent CarSim road. The comparisons
above used isolated copies of the R75 COM dataset; this task did not Send
a different dataset or overwrite the user's open MATLAB session.
