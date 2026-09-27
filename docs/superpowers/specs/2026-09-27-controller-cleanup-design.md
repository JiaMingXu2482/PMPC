# Controller Cleanup Design

## Goal

Reduce clutter in `controller/` without changing the verified `pmpc_mil` control behavior or removing the MPC, ZENG, and PMPC comparison modes used by the current regression workflow.

## Scope

- Delete controller functions that have no callers or whose outputs are never consumed.
- Merge setup-only helpers into `setup_pmpc.m` as local functions when a separate public file provides no useful boundary.
- Move project, CarSim, ERD, and result-analysis utilities out of `controller/` into `scripts/lib/`.
- Preserve focused runtime algorithms as separate files instead of growing `pmpc_step.m` into a monolith.
- Update startup paths, tests, scripts, and documentation to match the new layout.

## Preserved Behavior

- `pmpc_mil.slx` remains the sole Simulink model.
- MPC, ZENG, and PMPC comparison modes remain supported.
- DLC, J-turn, fishhook, ablation, and diagnostic branches remain available because they are still connected to setup or runtime code. They are not considered dead code without a separate request to remove them.
- Existing controller inputs, 54-element output interface, parameter structures, and CarSim dataset selection remain unchanged.
- The current regression baseline in `simulation_results/current/erd_0927_base/` remains unchanged.

## Confirmed Dead Code

- `func_bezierInterp.m` and `func_FindBezierControlPointsND.m`: no callers in project source.
- `func_RLSFilter_Calpha_f.m` and `func_RLSFilter_Calpha_r.m`: setup initializes private persistent state, but no runtime code ever calls either filter or consumes that state.
- `func_tire_init_Calpha.m` and `func_Fz_Calpha.m`: generated `C_table`, `Cf0`, and `Cr0` are copied into `Pm` and unpacked by `pmpc_step`, but never used afterward.

The corresponding setup calls, `P`/`Pm` fields, and unpacking statements will be removed together with these files.

## Consolidation

- Move `func_InitialParams` into `setup_pmpc.m` as a local `localInitialParams` function.
- Move `wsget` into `setup_pmpc.m` as a local function because it is used only there.
- Keep runtime controller algorithms such as QP construction, state estimation, prediction, actuation allocation, and priority certification in separate files.

## Directory Boundary

The following files are project/runtime utilities rather than controller algorithms and will move to `scripts/lib/` without interface changes:

- `func_CarSimLib.m`
- `func_CarSimResDir.m`
- `func_CarSimRunning.m`
- `func_ErdDir.m`
- `func_ManeuverFromName.m`
- `func_Metrics.m`
- `func_ProjectRoot.m`
- `func_ReadERD.m`
- `func_RunMode.m`
- `func_SimModel.m`
- `func_WaitERD.m`

`startup_pmpc.m` will add `scripts/lib/` explicitly. Existing function names remain stable, so callers do not need behavioral changes.

## Verification

- Static reference scan shows no active references to deleted functions or deleted parameter fields.
- Project layout tests verify the new utility directory and absence of dead controller files.
- Unit tests for QP helpers continue to pass.
- `setup_pmpc` completes with the expected timing and controller parameters.
- `pmpc_mil` loads successfully after startup.
- The working tree remains clean after verification; generated MAT-file timestamp changes are restored before commit.

## Git Strategy

Implement the cleanup in one focused commit after tests pass, then push `main` to `origin`.
