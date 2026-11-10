# DLC 80 km/h, mu=0.5 — final results

Condition: CarSim `Run_4d669775` (`DLC80_mu0.5_PMPC`), initial/target 80 km/h,
updated vehicle model (mass/inertia/CG, `Kt=81.5e3`, tyre `cq` scaling,
steering compliance `cs_fy=-3.2e-6` with `sw_prev` rate limit).

## Final controller parameters

- MPC: `Q_epsi=20`, `R2=200`, `slack=[0.5e4;0.5e4;1e4;1e4]`
- ZENG: `Q_epsi=10`, `I0=1.0`, `slack=1e4`, `R2=200`
- Proposed: `Q_epsi=5`, `Wb=10`, `w2_max=w1_max=2e3`, gains=3, `R2=200`

## Final comparison

| Controller | ey max | ey RMS | ey final | SW max | Mz max | Md max | beta max | yaw max | LTR max | QP |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| MPC | 0.597 | 0.236 | 0.036 | 96.5 deg | 2729 | 1423 | 1.58 deg | 13.4 deg/s | 0.46 | 100% |
| ZENG | 0.611 | 0.242 | 0.034 | 112.5 deg | 2807 | 1837 | 1.66 deg | 14.7 deg/s | 0.47 | 100% |
| Proposed | 0.598 | 0.229 | 0.024 | 77.0 deg | 2717 | 1289 | 1.59 deg | 13.3 deg/s | 0.45 | 100% |

## Files

- `three_controllers_new_model_tuned.png/.pdf`
  Final three-controller comparison with the retuned Proposed.
- `three_controllers_new_model_tuned_metrics.csv`
  Metrics for the final comparison.
- `three_controllers_updated_model.png/.pdf`
  New vehicle model, Proposed before the final `Q_epsi/Wb` retune.
- `three_controllers_updated_model_metrics.csv`
  Metrics for the pre-retune comparison.
- `prop_tuned_vs_mpc_zeng.png/.pdf`
  Old-vehicle-model Proposed result, kept as a historical reference for
  `CONTROLLERS.md` section 13.

Old intermediate result folders were removed on 2026-11-10. A full backup
is at `C:\Users\user\AppData\Local\Temp\opencode\PMPC_Results_backup_2026-11-10`.
