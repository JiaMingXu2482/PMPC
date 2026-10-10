# Terminal recovery revision 2 — development record

> Experimental history, NOT the default controller configuration. Sequential
> and preview paths below were disabled by default after adverse results.
> See `pmpc_terminal_followup_2026-10-10.md` for the confirmed damper-box bug,
> the selected configuration and validation after its repair.

## Scope

The previous terminal implementation improved J-turn RMS error but degraded
DLC and steering smoothness. This revision keeps the seven vehicle states,
existing control decisions, controller tuning and shared longitudinal controller.
It changes the backup prediction and offers a tolerance-qualified hierarchical
solver. MPC and ZENG retain their existing solver paths.

## Road-preview backup

The previous backup froze the terminal road curvature forever. A DLC curvature
lobe is transient, so that assumption can demand recovery to the wrong bend.
The new tail uses the reference path's heading increments and node curvatures
for ten additional 0.1 s steps. These are fixed-feedback backup steps, not new
QP control variables. Road-corner curvature corrections use the corresponding
tail nodes. The affine backup follows each node's equilibrium and converges
towards the final core once the endpoint disturbance is held constant.

The final linear model, speed, force offsets and physical actuator bounds are
still frozen; this is not a nonlinear or robust invariant-set guarantee. The
finite tail is condensed exactly, including its changing affine disturbance.
The final invariant-core certificate is conditional on the endpoint curvature
and other frozen quantities remaining constant thereafter.

There are 200 L1 and 180 L2 terminal rows for ten backup steps. The reduced QP
still has 18 variables. More rows and up to three solves add computation; no
S32K144 runtime claim is made.

The second development iteration additionally scales the backup input cost by
the permitted increments, rather than by full actuator amplitudes. Steady DB
is no longer forced to zero: an affine family of equilibria is searched for
the largest common admissible Lyapunov radius. This is a one-dimensional
piecewise-affine support problem, evaluated at interval endpoints and support
line intersections. It does not prescribe the optimized DB sign or turn it on
according to an external threshold.

## Hierarchical solver

1. Minimize L1 slack, with a small bounded numerical regularizer.
2. Lock the achieved L1 slack (within a configured numerical tolerance), then
   minimize L2 slack.
3. Lock both, then minimize tracking and actuator effort.

Input/gamma variables and finite worst-case slack bounds are normalized to
boxes with coordinate magnitudes at most one. For n variables the regularizer
is at most `reg*n/2`; its possible effect on either physical slack is bounded
by the selected scaling and is below `lex_tol/8`. Each lock adds `lex_tol`.
Solver feasibility tolerances add numerical error; this is a
**tolerance-qualified hierarchy**, not an exact-arithmetic strict-priority proof.
All stages must succeed and the final constraints/locks are checked.

Locked slacks are substituted out of the second and third stages. For
monotone softened inequalities this preserves the admissible control set of
an upper-bound lock, while avoiding almost coincident active constraints.
The three solves have 18, 17 and 16 variables. Reported slacks include the
small locking tolerance, not necessarily the exact minimum violation of the
final control plan.

`lex_tol=1e-5` is in the existing dimensionless physical slack coordinates.
Large weighted penalties w1/w2 are not used in this solver; the old weighted-QP
certificate/weight log fields are NaN, not reinterpreted as a new certificate.
DB retains its L3 intervention cost, with no prescribed actuator activation order.

## Switches

```matlab
PMPC_TERMINAL_ON = 1;
PMPC_TERMINAL_PREVIEW = 1;
PMPC_TERMINAL_STEPS = 10;
PMPC_LEX_ON = 1;
run_current_carsim('pmpc_mil');
```

To recover the pre-terminal weighted QP with the existing tuning, set BOTH
`PMPC_TERMINAL_ON=0` and `PMPC_LEX_ON=0`. Merely disabling the terminal rows
does not disable the independently selectable hierarchical solver.

## Verification and results

Unit checks include conflicting objectives, very large competing tracking cost,
physical slack scaling, and an independent direct rollout check of the curved
backup condensation. Closed-loop results will be recorded after validation.

Rejected first development run:
`scratchpad/com_r75_db_20261008/terminal_20261009_193401/Results/Run_trial`.
Preview plus the initial sequential solver produced DLC RMS ey=0.431963 m and
J-turn RMS ey=0.395843 m. Earlier DB did not mean improved tracking. This run
is retained as a rejected experiment, not the delivered performance claim.
