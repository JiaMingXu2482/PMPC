# DLC mu=0.4 speed sweep — initial speed = target speed

CarSim DLC road friction was changed from 0.5 to 0.4 in the split-mu friction
map (`RdMu_c76...par`, table `#DiagramTwo0`) and the inlined friction table in
`Results\Run_4d669775\Run_all.par`. The actual starting `Vx` equals the target
speed in every run.

## Results (ey max / ey RMS, m)

| Initial = target | MPC | ZENG | Proposed | Note |
|---:|---:|---:|---:|---|
| 60 | 0.600 / 0.180 | 0.594 / 0.174 | **0.558 / 0.169** | easy; 10 s run covers 160 m, not full path |
| **70** | **0.750 / 0.345** | **0.814 / 0.367** | **0.631 / 0.252** | **all controlled; Proposed best** |
| 75 | 2.463 / 1.095 | 1.163 / 0.548 | **0.743 / 0.293** | MPC saturates; Proposed best |
| 80 | 6.031 / 2.585 | **0.906 / 0.398** | 1.010 / 0.349 | MPC fails; ZENG best max, Proposed best RMS/SW/beta |

## Recommended comparison: 70 km/h

70 km/h is the highest speed where **all three controllers remain controlled**
and it is a clean, defensible comparison:

| Controller | ey max | ey RMS | SW max | Mz max | Md max | beta max | yaw max | LTR max | QP |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| MPC | 0.750 | 0.345 | 429.7° | 2768 | 1553 | 2.27° | 15.2°/s | 0.41 | 100% |
| ZENG | 0.814 | 0.367 | 525.0° | 2906 | 1626 | 2.47° | 16.2°/s | 0.44 | 100% |
| **Proposed** | **0.631** | **0.252** | **87.0°** | 2876 | **1389** | **1.73°** | **13.8°/s** | **0.39** | **100%** |

Proposed vs MPC: `ey max -15.9%`, `ey RMS -27.0%`, `SW -79.8%`,
`beta -23.8%`, `Md -10.6%`, `LTR -4.9%`; only `Mz max` is +3.9% higher.

Proposed vs ZENG: `ey max -22.5%`, `ey RMS -31.3%`, `SW -83.4%`,
`beta -30.0%`, `Md -14.6%`, `yaw -14.8%`, `LTR -11.4%`.

## Why not 75/80?

- 75: MPC already saturates (`ey 2.46`, SW 720°); Proposed is better but the
  baseline has lost the path, so it is a boundary case, not a clean tracking
  comparison.
- 80: MPC fails badly (`6.03`), ZENG `0.906` is slightly better than Proposed
  `1.010` on max; Proposed is better on RMS/SW/beta. Not a clean Proposed win.
- 60: all three are close and easy; not enough discrimination.

So the paper can use **70 km/h at mu=0.4** as the main low-mu comparison,
with **75 km/h** as a boundary case and 80 as a failed-limit reference.

## Files

- `summary_ey.png/.pdf` — ey max/RMS for 60/70/75/80.
- `three_controllers_70.png/.pdf` — recommended clean comparison.
- `three_controllers_75.png/.pdf`, `three_controllers_80.png/.pdf`.
- `metrics_speed_sweep.csv` — full metrics.
