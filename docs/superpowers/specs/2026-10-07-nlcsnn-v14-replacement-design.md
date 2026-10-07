# NLCSNN v14 replacement design

Date: 2026-10-07

## Goal and scope

Replace the legacy eight-state NLCSNN damper model in the shared CarSim–Simulink plant with the supplied v14 checkpoint. MPC, ZENG, PMPC, and PMPC-noDelay must use the same damper plant and the same offline inverse lookup table. Keep the vehicle MPC/QP formulations, controller tuning files, 1 kHz plant / 100 Hz controller schedule, CarSim force polarity, and 0–1.6 A current limits unchanged. Do not overwrite existing user edits to tuning files, `simfile.sim`, or model wiring unrelated to this replacement.

The forward model remains outside the embedded controller. The S32K-facing path uses lookup/interpolation, not PyTorch or an online neural inverse. This is a replacement of the damper model and actuator allocation, not a new maneuver-specific control rule.

## Source of truth

- Checkpoint: `D:/OneDrive - mail.scut.edu.cn/周报/10.8/nlcsnn/v14_global_best_12.55pct_E1_session162406.pth`; MD5 `8a3bd0d508429ab2c00083321561561a` (verified).
- Network implementation and its local imports: `nlcsnn_v14_py_model.zip` in the same directory. Export from the checkpoint's `model_cfg`, `norm_cfg`, and `state_dict`; do not infer layer shapes or normalization solely from prose.
- v14 forward: normalized six-input static NFL; two-layer, 64-unit GRU; learned current lag; current-modulated viscous/friction heads and unconstrained bias head. Preserve its input, output, and state-update order exactly.
- Existing rig/vehicle mapping remains `x_rig = x_ref - cmpD`, `v_rig = -cmpRD`, `F_CarSim = -F_v14`. Preserve the current 42.5 °C simulation setting unless source calibration requires otherwise.

## Data flow and interfaces

1. A 1 kHz four-corner plant block runs the exact v14 forward from actual CarSim displacement/velocity, filtered acceleration, commanded current, current derivative, and temperature. Each corner owns two 64-element GRU states and the finite current-lag history required to match PyTorch. Update Simulink delays, fixed-size declarations, build scripts, and all four `.slx` models together; no legacy 8×4 state contract remains in the active path.
2. Remove `SemiActive_Passivity_1kHz` from the active force path, as explicitly chosen by the user. CarSim receives the raw finite v14 force with the existing polarity conversion. Retain only numeric-failure handling and documented current/input-domain limits, without projecting non-passive or zero-speed bias force to zero. Log raw force so the absence of the old projection is auditable.
3. At 100 Hz, the controller receives observable damper kinematics and an offline-table estimate of the four reachable force intervals; it must not evaluate v14 or carry the checkpoint weights. The QP continues to request four damper forces. The controller output for this boundary is desired force, not current.
4. A shared actuator-allocation block outside each controller maps the four desired forces to four current commands using the offline inverse LUT, with interpolation, current saturation to [0, 1.6] A, and explicit out-of-domain handling. The 1 kHz plant holds those commands between 100 Hz updates. Preserve separate logging of force request, current command, and actual plant force; document the changed meaning of any reused `sys(5:8)` diagnostic channels.
5. Separate plant-only network weights from controller parameters (`Pm`). Keep the controller-side lookup tables bounded in size and suitable for later C export; do not claim an S32K execution time until board measurement.

## Offline lookup construction

Build both force-bound and inverse-current tables from the same v14 checkpoint. Begin with observable rig position, velocity, acceleration, and requested force as inverse-table coordinates, with temperature fixed to the simulation setting. Generate samples from representative rig histories rather than only steady-state points; v14's GRU and current lag make a history-free inverse approximate. Choose grid/quantization from held-out force-error and Flash-size measurements, not by adding maneuver-specific cases. Report median, 95th-percentile, maximum, and out-of-domain errors in newtons and relative to available force span. Where current has negligible authority or the request is beyond reachable bounds, hold or saturate current according to a documented actuator-level rule. If history dependence prevents an acceptable fixed table, do not silently switch to a per-tick neural table; report the limitation for a design decision.

## Verification and handoff

- Use an isolated CPU PyTorch environment for one-time checkpoint export and reference generation; no PyTorch runtime belongs in MATLAB/Simulink or on the S32K. If a compatible local runtime cannot be obtained, pause before claiming numerical parity.
- Generate golden sequences covering velocity reversals, current steps, nonzero force at zero velocity, and all four independent corner states. Compare every step's force, GRU state, and current-lag state between PyTorch and MATLAB single-precision implementation; set and report a numerical tolerance justified by the reference run.
- Test the LUT on histories not used to construct it and publish force-error distributions and saturation counts. Treat a poor LUT result as a failed approximation rather than changing the forward model to hide the error.
- Run MATLAB unit tests and compile the four model variants without rewriting unrelated connections. Do not run a CarSim dataset or overwrite `LastRun` as part of the automatic verification; the user will run the final co-simulation manually after the model is ready.
- Keep the legacy weights and model implementation recoverable in Git. The rollout must be reversible without altering existing controller tuning.

## Non-goals

No new MPC state/control dimensions, no change to the fixed prediction delay `tau_d`, no maneuver-specific tuning, no S32K PIL timing claim, and no reconstruction of v14 training methodology.
