# v14 fixed LUT prototype: held-out check (2026-10-07)

Status: **not connected to any controller or Simulink model**. The existing
8-state plant remains the default until the inverse approximation is resolved.

The prototype table uses 7 displacement × 9 velocity × 5 acceleration × 9
current nodes (11,340 bytes of single-precision force data). It was generated
from 32 processed rig files with the v14 checkpoint, then checked on 242
points from nine distinct checkpoint-held-out files. At each point, the target
force is drawn from that history's v14 force/current curve; the metric compares
the true curve at the LUT-selected current against that target. This isolates
the table's inverse error from the separate full-history/held-command mismatch.

| Metric | Result |
|---|---:|
| Absolute force error, median / p95 / maximum | 3.0 / 88.7 / 575.0 N |
| Error p95 / local current-authority span | 0.931 |
| With local authority ≥100 N: sample fraction | 38.8% |
| With local authority ≥100 N: error p95 | 108.6 N |
| With local authority ≥100 N: error p95 / authority | 0.591 |
| Requested current at 0 or 1.6 A | 60.7% |
| Kinematics outside table axes | 0.8% |
| Full-history force versus held-command curve, p95 | 111.5 N |

Conclusion: this history-free `(x,v,a,I)` fixed table is **not yet suitable**
for four-model integration. In particular, active-region error consumes much
of the force-control authority; simply passing unit tests for finite currents
does not establish allocation accuracy. The v14 source also contains a
current-lag `conv1d` ordering that differs from its EMA comment; the MATLAB
forward reproduces the source operation, but the inverse table cannot infer
that history from instantaneous kinematics alone.

Reproduction: generate artifacts with `nlcsnn/v14_py/build_v14_lut.py`, then
in MATLAB run `addpath('scripts'); validate_nlcsnn_v14_lut()`. The generator
reads processed `Parsed_Card_force_wide_zerofix` files traceable to the raw
archive; it does not treat raw acquisition-card CSVs as physical signals.
