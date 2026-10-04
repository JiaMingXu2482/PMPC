# Slalom ablation setup

The user-supplied CarSim slalom path is the reference for three current comparisons: PMPC, PMPC-noDelay, and ZENG. The MPC dataset exists but is not run in this comparison. Preserve the full X-Y path (through station 187.639 m) for controller preview while ending simulation and metric evaluation near station 160 m.

The physical CarSim vehicle carries 250 kg of roof cargo. The controller deliberately uses its nominal 0 kg vehicle model to test robustness to payload mismatch. Actual road friction is 0.85; the `mu1` substring in existing dataset names must not override this value. Initial speed is 70 km/h. The three comparison datasets must have the same speed, road, cargo, and stop condition.

PMPC-noDelay is a fourth peer controller, not a mode that shares PMPC's editable tuning file. `config/Atuning_pmpc_nodelay.m` starts with the same numerical tuning as PMPC; the difference for the initial ablation remains the controller's missing damper delay state. Preserve existing user changes and avoid modifying unrelated CarSim datasets.
