# v14 damper twin replacement and validation (2026-10-08)

Source: `D:\OneDrive - mail.scut.edu.cn\周报\10.8\nlcsnn\v14_twin_package.zip`.
The package checkpoint MD5 is `8a3bd0d508429ab2c00083321561561a`, matching
the existing MATLAB export `nlcsnn/v14_weights.mat` (138,512 parameters).

## Accuracy

- Package `verify_v14.py`: streaming versus batch forward maximum force
  difference 0.000714 N; inverse-current parity 0.0001 mA. Its printed
  self-consistency force errors (p50 17.8 N, p95 40.2 N) are *not* asserted.
- Independent eight-stem rig holdout evaluation (`tests/validate_v14_twin_rig.py`):
  pooled RMSE 104.8 N over 101,128 samples. One SH 30 mm / 1.66 Hz / 0 A
  sequence had RMSE 738.8 N; this is a material model limitation.
- Independent reachable-midpoint inverse evaluation
  (`tests/validate_v14_twin_package.py`, 48 same-history dynamic states):
  packaged inverse p50/p95 absolute force error 18.8/68.5 N and 29.2%
  current saturation. A full-v14 nine-current scan followed by two secant
  refinements reduced p50/p95 to 0.007/0.44 N. A dense full-v14 inverse was
  near exact; 35.4% of nine-point force curves were nonmonotone.

The MATLAB replacement uses the full-v14 scan/refinement inverse, not the
package's masked-current diagonal-bisection inverse. Its trial evaluations
freeze the pre-step GRU/current-history state and use the same full forward
model as the plant. In the four-corner MATLAB test, predicted force at a
reachable midpoint differs by less than 2 N. `tests/test_nlcsnn_v14_replacement.m`
and `tests/test_nlcsnn_v14_models.m` pass. The existing v14 artifact,
forward-golden, and LUT tests pass (9/9). Standalone MEX code generation
succeeds for both `nlcsnn_v14_inverse` and `func_NLCSNNPlant4`.

At the initial replacement stage, all four MIL models had a 227-by-4
hidden-state interface and routed raw v14 force to CarSim. Controller
tuning was not changed. Desktop MATLAB four-corner inverse timing was 6.55 ms
median and 12.89 ms maximum over ten calls; this is **not** NI real-time
evidence. Full CarSim/Simulink update was initially blocked in batch MATLAB
because the `Solver_SF` library referenced by CarSim is unavailable in that
session. NI model/RT OS, execution mapping, and target timing remain to be
confirmed before claiming NI deployment or a valid 1 kHz HIL deadline.

## COM co-simulation after interface repair

The saved PMPC COM run (`Run_500df6d7-9f0d-438a-8f7d-fbd7433aecf4`)
completed to 19.999 s on 2026-10-08. The initial MATLAB Function compile
errors were caused by (1) the missing `Fgrid` field in the preallocated
controller context, and (2) a single/double assignment conflict in the
227-state plant fallback. Both were fixed; a model-update check passed.

The completed simulation is **not** a valid performance result: DLC
`max|e_y|=6.72 m`, J-turn `max|e_y|=151.62 m`. CarSim reported wheel-load
table extrapolation beginning near 1.7 s. With the adapter's force/velocity
sign convention, `Fd .* CmpRD < -5000 N mm/s` occurred at 21% of corner
samples before 1 s and 36% before 2 s, reaching about -3.37 kW before 2 s.
This is consistent with the raw v14 model producing non-dissipative forces
as the closed loop leaves its training domain. No passivity projection or
controller retuning was silently reintroduced. The result remains in
`simulation_results/combined/v14_raw_failure_20261008` for diagnosis;
CarSim's current `LastRun` has since been replaced by the repaired run below.

## Approved dissipativity repair and completed COM verification

The user explicitly approved a switchable dissipative protection layer while
retaining raw v14 force logs. Zero-speed force alone is not evidence of an
invalid hysteretic damper. The stronger diagnostic was a prescribed periodic
motion with **constant current**, removing inverse switching as a cause:
at 8.5 Hz and 1 mm amplitude, raw-v14 mean force-times-velocity over the last
two cycles was -2.622, -2.779, and -0.815 W at 0, 0.8, and 1.6 A using
analytical acceleration. With the existing 40 ms acceleration filter it was
-0.328, -0.495, and -0.163 W. These finite-window tests identify net energy
injection in the tested operating region; they are not a general model proof.

Changes:

- `nlcsnn_deployed_force` applies a smooth nonnegative-power envelope with
  the previous soft-state damping floor, 0.8 N/(mm/s). Its velocity and power
  smoothing constants are 5 mm/s and 50 N mm/s. It guarantees
  `F_applied*v >= 0.8*v^2` in NLCSNN sign convention, up to roundoff.
- Plant force, inverse trial forces, and attainable bounds share this map.
  The raw recurrent state and original model weights are unchanged.
- Inverse selection prefers the previous current for force-equivalent
  solutions within 1 N, restoring the earlier weak-authority hold behavior.
- `setup_pmpc` enables the protection by default for all four controllers.
  Base-workspace `PMPC_NLCSNN_GUARD=0` followed by initialization disables
  it for raw-model diagnostics; `PMPC_NLCSNN_GUARD=1` enables it.
- All four `.slx` files expose the added raw-force output. Runtime logging
  saves `damper_force_raw` and `damper_force_applied` at 1 kHz alongside
  `damper_i_cmd`, `controller_sys`, and `priority_diag` in `pmpc_mil_logs.mat`.

The four focused replacement tests passed. The PMPC COM run reached the
requested stop time (last recorded time 19.999 s). Controller tuning was
not changed: SHA256 of `config/Atuning_pmpc.m` remained
`50490B3BD0BF41D19D7301CC41413291B424E57E2A8253990B3C8CDBC387E2A2`.

| Metric | Raw failed run | Protected run |
|---|---:|---:|
| Four-corner force RMS (N) | 1458.6 | 72.5 |
| Four-corner stroke-velocity RMS (mm/s) | 2226.1 | 23.1 |
| Maximum absolute wheel jounce (mm) | 137.5 | 41.0 |
| Maximum absolute LTR | 1.000 | 0.630 |
| Samples with `Fd*CmpRD < -5000 N mm/s` | 50.54% | 0.00% |
| Current-command step RMS (A) | 0.714 | 0.332 |
| DLC maximum / RMS lateral error (m) | 6.723 / 3.838 | 0.819420 / 0.291348 |
| J-turn maximum / RMS lateral error (m) | 151.618 / 80.154 | 0.350455 / 0.178665 |

Protected-run Vx RMS: DLC 88.631 km/h; J-turn 78.743 km/h.
The protection changes force by more than 1 N in **93.11%** of the logged
corner samples. Thus it is a substantial deployment modification, not a
rare safety intervention. These results validate stable MIL operation of
the **protected** twin, not the physical accuracy of raw v14, elimination
of all current-command variation, or NI real-time deadlines. Model
retraining/validation over the vehicle's excitation bandwidth remains
necessary before treating raw v14 as an unprotected vehicle plant.

The repaired run, controller logs, comparison figure (`.png` / `.fig`), and
metrics (`.mat`) are archived in
`simulation_results/combined/v14_repair_20261008`. The comparison can be
regenerated with `scripts/analyze_v14_com_repair.m`. CarSim's current
`LastRun` also contains this repaired run. CarSim still reports some table
extrapolation warnings; the run is not warning-free, but the severe
wheel-load extrapolation sequence of the failed run is absent.
