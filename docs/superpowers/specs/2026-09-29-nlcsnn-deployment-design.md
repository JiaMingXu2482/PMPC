# NLCSNN Deployment Design

## Goal

Replace the legacy MR damper plant and force envelope with the uploaded NLCSNN CDC model for MPC, ZENG, and PMPC. All three controllers must use the same damper model and the same CarSim interface so their comparison remains fair.

The deployed chain is:

```text
controller desired force
  -> NLCSNN force-to-current inverse
  -> current command
  -> NLCSNN forward dynamics
  -> actual damper force
  -> CarSim IMP_FD input
```

The legacy fixed-current concept is not carried over. NLCSNN has no checkpoint parameter named `I_nom`; its valid current range is `0..1.6 A`.

## Confirmed Configuration

- Maximum current: `1.6 A`.
- Initial current: `0 A` (fail-safe hard state).
- Temperature: constant `42.5 degC`.
- Controllers using NLCSNN: MPC, ZENG, and PMPC.
- CarSim exports four additional `CmpD_*` channels.
- Legacy `func_DamperActuator` delay is bypassed when NLCSNN is active.
- Each corner owns an independent 8-state NLCSNN hidden state.
- No fixed nominal current is introduced.

## CarSim Interface

The current 50 export channels remain unchanged. Four damper-compression channels are appended so every existing index keeps its current meaning:

| Index | Channel |
|---:|---|
| 51 | `CmpD_L1` |
| 52 | `CmpD_L2` |
| 53 | `CmpD_R1` |
| 54 | `CmpD_R2` |

The MATLAB Function input therefore changes from `50x1` to `54x1`. The controller output remains `54x1`.

The CarSim Export dataset and generated `simfile.sim` must both report `PORTS_EXP 1,54`. Re-sending a CarSim dataset must preserve the appended channel order.

## Coordinate and Sign Mapping

CarSim defines `CmpD` and `CmpRD` as compression and compression rate, so positive values mean damper compression. NLCSNN uses positive displacement rate for rebound and positive force for tension. The mapping is therefore:

```matlab
x_nl = 281.0645 - CmpD_mm;
v_nl = -CmpRD_mm_s;
a_nl = -d(CmpRD_mm_s)/dt;

F_des_nl = -F_des_carsim;
F_carsim  = -F_nl;
```

`CmpD` is already damper displacement, so no suspension motion ratio is applied. Acceleration is computed once per controller execution from the compression-rate difference and filtered with a fixed first-order filter before conversion to the NLCSNN sign convention.

The mapping must live in one damper-adapter function. Controller algorithms must not contain scattered sign changes.

## NLCSNN State

The persistent controller state gains fixed-size fields:

- `h`: `8x4`, one hidden-state column per corner;
- `i_prev`: `4x1`, initialized to zero;
- `v_prev`: `4x1`, previous CarSim compression rate in `mm/s`;
- `a_filt`: `4x1`, filtered CarSim compression acceleration in `mm/s^2`.

The fixed corner order is `[L1; R1; L2; R2]` inside the NLCSNN adapter, matching the existing CDC allocation order. Boundary conversion at the CarSim input/output interface preserves the external order `[L1; L2; R1; R2]`.

For each controller sample and each corner:

1. Build `x_nl`, `v_nl`, and `a_nl` from `CmpD` and `CmpRD`.
2. Convert desired CarSim force to NLCSNN force polarity.
3. Call `nlcsnn_force_to_current` with the step-start hidden state and previous current.
4. Compute `dcurr = (i_cmd - i_prev)/dt`.
5. Call `nlcsnn_damper_step` exactly once.
6. Convert predicted force back to CarSim polarity.
7. Store the new hidden state and current.

Candidate-current evaluations in the inverse must never advance hidden state.

## Controller-Side Force Bounds

The old analytic `Gu/Gl` envelope and `func_MRDamper` nominal force are removed from the active CDC path.

At each step, the NLCSNN adapter evaluates the current-state force at `0 A` and `1.6 A`. Their minimum and maximum form each corner's reachable force interval. These intervals feed the existing CDC allocation and yaw-moment bound calculations.

Where the QP currently requires a centered CDC reference, use the midpoint of the instantaneous reachable force interval:

```matlab
F_center = 0.5 * (F_min + F_max);
```

This is a force-space normalization reference, not a claimed physical nominal current. It avoids inventing an `I_nom` that the trained model does not define.

The same force bounds and plant are used in MPC, ZENG, and PMPC. Only their controller logic and weights differ.

## Legacy and Runtime Behavior

NLCSNN is enabled by default through a project parameter. A temporary rollback switch may retain the legacy path during verification, but it must not alter QP dimensions or MATLAB Function output dimensions.

When NLCSNN is enabled:

- the old `func_DamperActuator` call is bypassed;
- no fixed 15 ms legacy force delay is applied;
- NLCSNN hidden-state dynamics provide the damper's dynamic response;
- `func_MRDamper` and the old `I_nom` are not used by MPC, ZENG, or PMPC.

The controller baseline/passive-only diagnostic mode is outside the three-controller comparison. If retained, its behavior must be explicitly labeled and must not silently reintroduce the old MR table into active controller modes.

## Model Initialization

`startup_pmpc` adds `nlcsnn/` to the MATLAB path. `setup_pmpc` loads `nlcsnn_weights.mat` once and stores the fixed network structure in the immutable parameter bundle passed to `pmpc_block`.

No per-step file I/O is allowed. All NLCSNN arrays and states have fixed dimensions for MATLAB Coder and the Simulink MATLAB Function block.

## Diagnostics and Failure Handling

The implementation exposes, at minimum, current command and achieved-force diagnostics to MATLAB-side tests. Existing 54 controller outputs are not expanded during the first deployment; adding plot channels is a separate change.

Runtime safeguards:

- clamp current and previous current to `0..1.6 A`;
- reject or saturate non-finite mapped inputs;
- keep each hidden state independent;
- ensure all CarSim force outputs are finite;
- use the inverse model's nearest-current branch when multiple force crossings exist;
- retain previous current when force authority is below the inverse model threshold.

## Testing and Acceptance

Implementation follows test-driven development. Required automated checks are:

1. Existing `test_nlcsnn_port.m` passes with the uploaded weights and golden vectors.
2. A sign-mapping test verifies positive CarSim compression becomes negative NLCSNN velocity and converts force back correctly.
3. A state-isolation test proves exciting one corner does not change the other three hidden states.
4. An inverse round-trip test covers the golden states and verifies achieved-force error remains below the established tolerance.
5. A bounds test proves every allocated force and actual force lies inside the current-state NLCSNN reachable interval, allowing numerical tolerance.
6. MPC, ZENG, and PMPC tests all traverse the same NLCSNN adapter.
7. The existing fixed QP dimensions and `54x1` output are unchanged.
8. `pmpc_block` code generation succeeds with a `54x1` input and constant parameter bundle.
9. Simulink model update succeeds after CarSim exports 54 channels.
10. A short co-simulation for each controller produces finite currents, hidden states, and CarSim forces without invoking `func_DamperActuator`.

Deployment is accepted only after both unit-level MATLAB tests and a CarSim/Simulink smoke run pass. Performance comparison runs are performed afterward and are not used to conceal integration failures.

## Files Expected to Change

- `startup_pmpc.m`
- `setup_pmpc.m`
- `controller/pmpc_block.m`
- `controller/pmpc_step.m`
- `controller/func_StateEstimation.m`
- `controller/func_CostWeightingRegulation_QuadSlacks.m` or its CDC-boundary interface
- one focused NLCSNN-to-CarSim adapter under `controller/` or `nlcsnn/`
- MATLAB tests under `tests/`
- CarSim Export dataset and generated `simfile.sim`
- run documentation describing the 54-channel requirement

Unrelated controller behavior, QP layout, road configuration, and maneuver settings remain unchanged.
