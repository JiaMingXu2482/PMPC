# PMPC terminal backup-recoverability constraints

> Historical record. The logged failures below were subsequently traced to
> reversed narrow one-sided damper moment boxes, not primarily QP conditioning.
> See `pmpc_terminal_followup_2026-10-10.md` for the repair and current results.

This implements terminal recoverability constraints, not a DB trigger, a road
width reduction, an extra Vx state, or another tuning sweep. Existing tracking,
DB penalties and the objective-level priority weight rule are unchanged.

## Construction

At each control update, freeze the final prediction-node linear model, curvature,
speed, force offsets, and current physical input bounds. Let zeta contain the
seven vehicle states and the three already-existing previous inputs. Compute
an equilibrium with ey=0, zero steady DB and the midpoint admissible CDC command.
Do not impose epsi=0: steady sideslip generally requires a heading offset.

Scale state/input coordinates by physical characteristic magnitudes. A bounded
120-step Riccati iteration supplies a fixed affine backup feedback
`Delta u = -U K S^-1 (zeta-zeta_eq)`. Fixed input channels are explicitly held.
Accept its matrix P only when both P and P-Acl'*P*Acl are positive definite.
An incomplete Riccati convergence does not invalidate the check: the checked
Lyapunov inequality, not an assumption of DARE convergence, is the certificate.

For each layer, express its physical restrictions as `h_j e <= b_j`:

- Layer 1: road corners, LTR, input amplitudes and input increment limits.
- Layer 2: yaw rate, rear slip, input amplitudes and input increment limits.

The ellipsoid radius is

`alpha_l = min_j b_j^2 / (h_j P^-1 h_j')`, requiring nonnegative headroom.

Support functions for a fixed input are evaluated on its invariant zero-error
subspace. The corresponding exact input bounds remain in the backup rollout.
For e'P e<=alpha_l, every included physical restriction holds; the verified
Lyapunov inequality keeps the frozen-model backup trajectory in that ellipsoid.

With P=L'*L, the polytope `|L e|_inf <= sqrt(alpha_l/10)` lies inside the ellipsoid.
It is NOT claimed that this inscribed polytope is itself invariant. The backup
can leave the polytope while remaining in its admissible invariant ellipsoid.

Require three backup steps beyond the nominal prediction endpoint, including
state, amplitude and rate checks, followed by membership in this polytope.
All backup inputs are affine functions of zeta_N, not new QP decision variables.
Condensation yields `A_terminal,l DeltaU - s_l <= b_terminal,l`, normalized with
the physical bounds (core rows by their normal length in physically normalized
state coordinates, never by a vanishing core radius). The SAME s1/s2 cover
the stage and terminal constraints. Zero terminal slack certifies existence
of this particular admissible backup under the stated frozen-model assumptions.

For Np=10, Nc=5: 74 additional L1 rows and 68 additional L2 rows. The allocated
26-variable layout and 18-variable reduced PMPC QP do not grow. The Riccati and
set computations add runtime work; no embedded timing claim is made here.

## Safety and proof boundaries

- This is an inner approximation under one constructed backup policy, not the
  maximal recoverable set. Outside it does NOT prove that all recovery is impossible.
- Nonzero slack relaxes membership and removes the zero-slack safety certificate.
- Model mismatch, varying curvature/speed/friction, future damper bounds and
  nonlinear tire behavior are not robustly bounded in this implementation.
- The existing weighted QP is retained. Adding these rows does not establish
  global exact lexicographic optimality, recursive feasibility, or real-vehicle safety.
- A singular equilibrium, failed Lyapunov check or empty admissible core disables
  only the affected terminal layer, leaves the original stage constraints active,
  records invalid status, and clears the reported prior safety certificate flags.
  It must not be interpreted as a successful terminal certificate.
- The backup assumes its command is applied over Ts, with increments limited
  to the existing 10 ms command-increment bound. This is deliberately conservative.

Background: [Rawlings, Mayne and Diehl, Model Predictive Control](https://sites.engineering.ucsb.edu/~jbraw/mpc/MPC-book-2nd-edition-4th-printing.pdf),
terminal regions and invariant-set constructions. The implementation's additional
approximations and limitations are explicitly listed above, not attributed to the book.

## Integration and switch

- `controller/func_TerminalRecoverability.m`: equilibrium, backup, invariant cores,
  affine terminal rows; optional proof matrices for tests.
- `controller/func_TerminalRowCounts.m`: fixed compile-time row counts.
- `func_BuildQPConstraints`: inserts terminal L2 rows with stability rows, L1 rows
  with road rows, sharing existing slack columns.
- `func_PriorityCert`: candidate margins and weight calculations include these rows.
- `setup_pmpc`: PMPC-only `terminal_on`, three backup steps, expanded diagnostic state.
- `pmpc_mil`: same output ports; priority_diag expands from 12 to 20 channels.
  MPC/ZENG/no-delay diagnostics remain unchanged.

```matlab
PMPC_TERMINAL_ON = 1; % terminal experiment
run_current_carsim('pmpc_mil');
% PMPC_TERMINAL_ON = 0 disables the added rows, preserving current tuning.
```

Additional priority_diag channels:

| Channel | Meaning |
|---|---|
| 13 | Terminal mode enabled |
| 14 / 15 | Valid L1 / L2 core construction |
| 16 / 17 | Minimum unrelaxed L1 / L2 terminal margin of the optimized plan |
| 18 | Lyapunov-metric backup contraction factor |
| 19 | Equilibrium residual |
| 20 | Construction status: 1 accepted at least one layer; -1 singular equilibrium; -2 residual failure; -3 Lyapunov failure; -4 both cores inadmissible |

Margins are NaN for invalid construction or a failed QP. Valid construction
alone is NOT terminal membership: inspect the margins/slacks as well.

## Verification

Tests cover ellipsoid contraction and physical constraints, pre-boundary risk,
nonzero DB sensitivity in the condensed rows, invalid-center handling, and the
unchanged/expanded diagnostic contracts. A small-core regression checks that
shrinking the core does not blow up the constraint row coefficients.
All five tests passed in a fresh MATLAB batch after the final edits.

## COM R75 result (2026-10-09)

**Implemented and simulated, but not an overall performance improvement.**
The terminal-on model completed 20 s of CarSim co-simulation. The terminal-off
model also passed a Simulink update/compile check. No controller tuning was
changed for this comparison (R1=100, V2=8000, full-Np stage constraints).

| Metric | Same-tuning baseline, terminal off | Terminal on |
|---|---:|---:|
| DLC peak absolute ey (m) | 0.823387 | 0.976934 |
| DLC RMS ey (m) | 0.295642 | 0.348378 |
| J-turn peak absolute ey (m) | 0.529637 | 0.500793 |
| J-turn RMS ey (m) | 0.393334 | 0.192708 |
| J-turn RMS Vx (km/h) | 78.169 | 78.064 |
| Maximum absolute LTR | 0.6382 | 0.6583 |
| Steering-command step RMS, t >= 12.5 s (deg) | 1.5516 | 2.9218 |

J-turn RMS ey improved by about 51%, but DLC RMS ey worsened by about 18%
and steering became rougher. The early DB request is a **negative yaw moment**:
its first sustained onset moved from 10.5 m after turn entry to 20.7 m before
entry. Positive DB request onset moved from 28.0 m after entry to 13.0 m after
entry; this is NOT evidence of positive DB before the turn. Onset here means
five consecutive logged samples beyond +/-50 Nm in a +/-50 m entry window,
not measured wheel brake torque or proof of a deceleration mechanism.

Valid core construction occurred in 77.8% / 76.1% of samples for L1 / L2.
There were 443 samples with neither core admissible. Invalid layers explicitly
fall back to the original stage constraints. Valid construction alone does not
mean the optimized trajectory satisfies the unrelaxed terminal constraints.
The solve log contained 187 numerical failures (-12), two infeasible solves
(-11), 1810 successful solves and one initial NaN; solver failures remain.

The feature is currently enabled by default for PMPC, as an **experimental**
implementation of the requested terminal scheme, not a validated replacement
for the previous baseline. Set `PMPC_TERMINAL_ON=0` before running to disable it.
No private longitudinal speed controller was introduced. The frozen-speed
prediction cannot credit DB with future speed-reduction benefits.

All paths below are relative to the project root:

- Terminal result: `scratchpad/com_r75_db_20261008/terminal_20261009_191635/Results/Run_trial`
- Same-tuning baseline: `scratchpad/com_r75_db_20261008/refine_20261009_184137_firstTc0/Results/Run_trial`
- The terminal run includes CarSim `.vs/.vsb`, `pmpc_mil_logs.mat`, parameter
  snapshots, `report.mat`, and `terminal_source.zip` (sources at simulation time).
- Earlier normalization-development result is retained under
  `terminal_20261009_190907`; it is NOT the result reported in the table.

后续若继续优化，应优先解决终端集合过于保守/失效和数值条件问题，而不是
直接增大优先级权重或把更早的负向 DB 解释为正确的入弯辅助制动。
