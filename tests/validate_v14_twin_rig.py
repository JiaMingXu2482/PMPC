"""Evaluate supplied v14 checkpoint on its recorded rig holdout files.

This independently reads the processed force-wide CSVs; it does not use
training logs or the supplied verify_v14.py's synthetic sequence.
"""
from pathlib import Path
import sys

import numpy as np
import pandas as pd
import torch

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "scratchpad" / "v14_twin_eval" / "v14_twin_package"
DATA = Path(r"D:\OneDrive - mail.scut.edu.cn\Matlab\CDCTest\Parsed_Card_force_wide")
sys.path.insert(0, str(PACKAGE))
from run_v14 import V14Twin  # noqa: E402


def main():
    torch.set_num_threads(2)
    ckpt = PACKAGE / "v14_global_best_12.55pct_E1_session162406.pth"
    twin = V14Twin(str(ckpt))
    meta = torch.load(ckpt, map_location="cpu", weights_only=False)
    nc = meta["norm_cfg"]
    files = list(DATA.glob("*/*.csv"))
    total_sq = 0.0
    total_n = 0
    for stem in meta["holdout_stems"]:
        found = [p for p in files if p.stem == stem]
        if len(found) != 1:
            print(f"MISSING/AMBIGUOUS {stem}: {len(found)}")
            continue
        df = pd.read_csv(found[0])
        u = np.column_stack([
            (df["rod_length"].to_numpy() - nc["x_ref"]) / nc["x_scale"],
            df["velocity"].to_numpy() / nc["v_scale"],
            df["accel"].to_numpy() / nc["a_scale"],
            df["current"].to_numpy() / nc["i_scale"],
            df["current_dot"].to_numpy() / nc["di_scale"],
            (df["temp"].to_numpy() - nc["t_ref"]) / nc["t_scale"],
        ]).astype(np.float32)
        actual = df["force"].to_numpy(dtype=np.float32)
        pred = twin.forward_batch(u)
        err = pred[1000:] - actual[1000:]
        rmse = float(np.sqrt(np.mean(err**2)))
        mae = float(np.mean(np.abs(err)))
        p2p = float(np.ptp(actual[1000:]))
        total_sq += float(np.sum(err**2))
        total_n += len(err)
        print(f"{stem}: n={len(err)}, MAE={mae:.1f} N, "
              f"RMSE={rmse:.1f} N, RMSE/p2p={rmse/max(p2p,1):.1%}")
    print(f"Pooled RMSE={np.sqrt(total_sq/total_n):.1f} N "
          f"across {total_n} holdout samples")


if __name__ == "__main__":
    main()
