# DLC mu=0.5 speed sweep — initial speed = target speed

CarSim dataset: `Run_4d669775` (`DLC80_mu0.5_PMPC`).

The initial speed was changed in CarSim by setting both the procedure and the
run-control `*SPEED` to the target value and exporting the run par; the
`SV_VXS` entries in the run par were confirmed. The Simulink `v_target` was
set to the same value, so each run starts at the stated speed (verified from
`Vx` logs).

## Full sweep (ey max / ey RMS, m)

| Initial = target | MPC | ZENG | Proposed |
|---:|---:|---:|---:|
| 80 | 0.597 / 0.236 | 0.611 / 0.242 | **0.598 / 0.229** |
| 82 | 0.604 / 0.273 | 0.603 / 0.269 | **0.609 / 0.239** |
| 83 | 0.697 / 0.324 | 0.673 / 0.311 | **0.615 / 0.246** |
| 84 | 0.848 / 0.387 | 0.687 / 0.325 | **0.622 / 0.257** |
| **85** | **3.244 / 1.480** | **8.416 / 3.552** | **0.631 / 0.277** |
| 90 | 9.457 / 3.583 | 9.668 / 3.675 | **3.951 / 1.804** |
| 100 | 7.690 / 2.670 | 7.592 / 2.641 | 7.562 / 2.730 |

All QP success rates are 100%.

## Recommended main comparison: 84 km/h

84 km/h is the best **clean** near-limit comparison:

- all three controllers still track the path and none saturates steering;
- Proposed: `0.622 / 0.257`, SW 114°, beta 1.72°, LTR 0.46, QP 100%;
- MPC: `0.848 / 0.387`, SW 429°, beta 1.93°;
- ZENG: `0.687 / 0.325`, SW 374°, beta 1.99°;
- Proposed vs MPC: `ey max -26.6%`, `ey RMS -33.6%`, `SW -73%`;
- Proposed vs ZENG: `ey max -9.5%`, `ey RMS -20.9%`.

This is a legitimate 84 km/h result and can be used directly in the paper.
Do **not** relabel the 84 km/h run as 85 km/h: the difference is not a short
initial transient. At 85 km/h MPC/ZENG cross the controllability boundary
(`3.24 / 8.42 m`), while Proposed remains at `0.631 m`; the whole trajectory
diverges, and a reader can check `Vx`.

## What 85 km/h is

85 km/h is a **boundary / limit test**, not the clean main comparison:

- 80–82: all controllers comparable and controlled;
- 83–84: Proposed becomes progressively better, but all still track;
- **85: MPC/ZENG lose the path; Proposed still holds it**;
- 90: all lose the path; Proposed is less bad (`3.95` vs `9.46/9.67`);
- 100: all fail and are similar.

For the paper, the defensible choice is:

1. **84 km/h** as the main near-limit comparison (all controlled, Proposed
   clearly better);
2. **85 km/h** as a boundary/envelope test, with the 80–100 sweep shown;
3. 90/100 only as failed-limit references.

## Files

- `summary_ey.png/.pdf` — ey max/RMS vs initial=target speed for 80–100.
- `three_controllers_84.png/.pdf` — recommended clean comparison.
- `three_controllers_85.png/.pdf` — boundary case.
- `three_controllers_90.png/.pdf`, `three_controllers_100.png/.pdf`.
- `metrics_speed_sweep.csv` — full metrics.
