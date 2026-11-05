# Current PMPC controller snapshot

Source: `D:\OneDrive - mail.scut.edu.cn\Thesis\Code\PMPC`.

This directory preserves the files saved on disk at capture time. See
`SNAPSHOT_MANIFEST.json` for the captured file list and SHA-256 hashes.

- `pmpc_mil.slx`: current saved Simulink model.
- `pmpc_core/`: controller initialization, prediction, optimization, allocation, bus definitions, and result scripts present in the source folder.
- `nlcsnn/`: damper forward/inverse/boundary/current dynamics code and MATLAB weight files.
- `simfile.sim`: current local CarSim run pointer.

The model callbacks and CarSim pointer contain local absolute paths. The CarSim installation and its referenced run datasets are external dependencies and are not bundled. Unsaved MATLAB/Simulink editor changes are not included.

Generated `slprj`, `codegen`, `.slxc`, editor recovery files and temporary files are excluded. No control parameters or algorithm logic were changed for this snapshot. No simulation, compilation or control validation was run for this upload.

The repository root retains the earlier project; this snapshot is a separate project directory.
