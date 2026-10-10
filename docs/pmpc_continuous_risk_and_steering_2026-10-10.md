# Continuous risk preview and steering-command smoothing

## Scope

Follow-up to the user's reproduced R75 COM regression. The damper-box order
repair is retained. No ZENG tuning, CarSim dataset, longitudinal controller,
road margin, or required DB sign was changed. No SLX port changes are needed.

## Implemented alternatives

`PMPC_TERMINAL_MODE=2` selects `func_FutureRiskTail`: propagate the predicted
terminal state for three more 0.1 s steps using the last planned input held
constant and previewed road curvature. The affine physical LTR/road rows share
s1, and yaw/rear-slip rows share s2. There are 30 additional inequalities and
no additional decisions. There is no equilibrium-target penalty, no ellipsoid
validity switching, and no actuator-priority rule. This is a frozen-model
finite risk preview, **not** a proof of invariant recoverability or recursive
feasibility. Frozen speed and tire coefficients remain limitations.

For mode 2, diagnostic status is 2; fields previously named core validity now
indicate active constraint groups. Contraction and equilibrium residual are
NaN. Old certificate flags are cleared to avoid implying invariant safety.
Mode 1 retains the previous experimental core for reproducibility only.

`PMPC_STEER_SMOOTH` controls a nonnegative L3 quadratic penalty:

    weight * sum_i ((Delta_delta_i - Delta_delta_(i-1))/Delta_delta_max)^2

The first preceding increment is the one executed in the previous control
tick, not an unexecuted warm-start increment. The optimized command itself is
sent to the vehicle; there is no post-QP filtering. Its omitted constant term
is included in `func_PriorityCert` when calculating F and the safety weights.
No hard safety/rate bound is removed and no new QP variable is added.

## Validation

Three smoothing algebra/PSD/disabled-path tests, four damper-box tests, and
nine terminal/tail tests passed (16 total). Direct rollout and condensed
tail constraints match. The mode-2 constraints stay active at violation;
they do not switch off because an equilibrium center becomes inadmissible.

Unsmoothed mode-2 COM (`terminal_20261010_103551`) completed 1999 successful
QP solves with no fallback. DLC peak/RMS: 0.957874/0.346771 m; J-turn:
0.594849/0.390068 m. Early/late steering step RMS: 1.7379/1.0124 deg.
This is **rejected as a replacement**: J-turn regressed versus the old core.

The two smoothing comparisons use weight 5000 and otherwise identical R75
COM parameters, with and without the physical-risk tail:

| Configuration | DLC peak / RMS (m) | J-turn peak / RMS (m) | Early / late SW increment RMS (deg) | Road-out |
|---|---:|---:|---:|---:|
| Repaired baseline, no terminal, no smoothing | .834709 / .297228 | .531219 / .393520 | 2.0978 / 1.2804 | 1.4% |
| No terminal, smoothing 5000 | .807114 / .285566 | .818787 / .612785 | 1.7873 / 1.1510 | 18.95% |
| Physical tail, smoothing 5000 | .903451 / .337648 | .760742 / .549855 | 1.4362 / .8712 | 12.4% |

Smoothing trials: `smooth_20261010_143054_tail0` and
`smooth_20261010_143054_tail1`, under `scratchpad/com_r75_db_20261008`.
Both completed with zero fallback and no noninitial upper-QP failure.
**Both are rejected.** Smoother commands do not justify degraded J-turn
tracking or increased road-out. The tail+smoothing run has 49 command steps
above 3 degrees versus 138 for the old core, but it still has 7.2-degree
maximum steps and is not a solution to the overall problem.

A final separate test re-evaluates the previously implemented sequential
solver after the damper-bound fix; the earlier sequential-solver rejection
was based on runs before that fix. It disables both tail and smoothing,
isolating the solver path rather than combining further tuning changes.

The fixed-box sequential trial (`fixedbox_lex_20261010_143650`) also failed:
only 129 of 1999 noninitial QPs succeeded (508 first-stage failures, 1362
second-stage failures). DLC peak error reached 220.85 m and J-turn was not
completed. **It is rejected and remains disabled.** Reduced late steering
variation in this failed run only reflects held commands, not improvement.

## Default restored

Terminal constraints, steering smoothing and the sequential solver are all
disabled by default. The damper-box repair and original weighted reduced QP
remain enabled. Atuning values and longitudinal logic were not changed.
The added experimental functions are retained behind explicit opt-in switches
for reproducibility, not recommended for use or PIL deployment.

To override any experimental flags left in a MATLAB workspace:

```matlab
PMPC_TERMINAL_ON = 0;
PMPC_STEER_SMOOTH = 0;
PMPC_LEX_ON = 0;
run_current_carsim('pmpc_mil');
```

This restores the better DLC baseline but does NOT resolve J-turn tracking or
prove that DB is properly coordinated. No successful overall improvement is
claimed. Correcting and validating prediction/control-time consistency and
the predicted safety response to DB remains necessary before another terminal
or priority formulation can be justified; this is a next investigation, not
a cause established by the above experiments.

## Final default verification

`restored_baseline_20261010_144046/Results/Run_trial` completed to 19.999 s
with the final sources and all three experimental switches off. It exactly
reproduced the earlier repaired nonterminal baseline:

- DLC peak/RMS ey: 0.834709 / 0.297228 m; Vx RMS 88.619 km/h.
- J-turn peak/RMS ey: 0.531219 / 0.393520 m; Vx RMS 78.135 km/h.
- Early/late steering increment RMS: 2.0978 / 1.2804 degrees; maximum
  increment remains 7.2 degrees. Chatter has NOT been resolved.
- No noninitial upper-QP failure and no fallback; the first diagnostic sample
  is uninitialized, not a solve failure. Road-out metric remains 1.4%.

The folder includes `parameters.mat`, `pmpc_mil_logs.mat`, `report.mat`,
CarSim files and `controller_source.zip`. Sixteen regression tests passed
again before this final run. No PIL testing or real-time execution guarantee
was attempted. The GUI MATLAB session and original CarSim output were not
modified by these isolated batch runs.
