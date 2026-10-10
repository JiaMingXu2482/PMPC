"""Independent dynamic inverse check for the supplied v14 twin package.

At each tested sample the target is the midpoint of the *actual* one-step
plant endpoint forces from one shared pre-step history. The inverse then
uses the package's documented forward_step/inverse_step sequence. This
tests dynamic force closure, not merely parity with model.inverse_current.
"""
from collections import deque
from pathlib import Path
import sys
import time

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "scratchpad" / "v14_twin_eval" / "v14_twin_package"
sys.path.insert(0, str(PACKAGE))
from run_v14 import V14Twin  # noqa: E402


def main():
    torch.set_num_threads(1)
    ckpt = PACKAGE / "v14_global_best_12.55pct_E1_session162406.pth"
    twin = V14Twin(str(ckpt), n_bisect=20)
    n = 1400
    t = np.arange(n) / 1000.0
    x = 279.1 + 18 * np.sin(2 * np.pi * 0.7 * t) + 4 * np.sin(2 * np.pi * 4.0 * t)
    v = np.gradient(x) * 1000
    a = np.gradient(v) * 1000
    current = np.clip(0.8 + 0.55 * np.sin(2 * np.pi * 0.37 * t), 0, 1.6)
    records = []

    for k in range(n):
        prior_h = None if twin.h is None else twin.h.clone()
        prior_buf = list(twin.ie_buf)

        def plant_at(i):
            twin.h = None if prior_h is None else prior_h.clone()
            twin.ie_buf = deque(prior_buf, maxlen=twin.ie_buf.maxlen)
            return twin.forward_step(x[k], v[k], a[k], i, 0.0, 40.0)

        if k >= 200 and k % 25 == 0:
            f0 = plant_at(0.0)
            f1 = plant_at(1.6)
            f_target = (f0 + f1) / 2
            plant_at(current[k])  # establish the package's same-step context
            i_sol = twin.inverse_step(f_target)
            f_ach = plant_at(i_sol)
            # Diagnostic oracle: invert the actual one-step v14 plant from
            # the same pre-step history. This is not a timing claim for NI.
            tic = time.perf_counter()
            grid = np.linspace(0.0, 1.6, 9)
            forces = np.array([plant_at(i) for i in grid])
            linear_errors = []
            refined_error = np.nan
            for stride in (2, 1):  # five-point and nine-point grids
                gi = grid[::stride]
                gf = forces[::stride]
                brackets = np.flatnonzero((gf[:-1]-f_target) *
                                          (gf[1:]-f_target) <= 0)
                if len(brackets):
                    bj = int(brackets[0])
                    frac = (f_target-gf[bj]) / (gf[bj+1]-gf[bj])
                    i_linear = gi[bj] + frac * (gi[bj+1]-gi[bj])
                    f_linear = plant_at(i_linear)
                    linear_errors.append(abs(f_linear-f_target))
                    if stride == 1:
                        left_i, right_i = gi[bj], gi[bj+1]
                        left_f, right_f = gf[bj], gf[bj+1]
                        for _ in range(2):
                            if (f_linear-f_target) * (left_f-f_target) <= 0:
                                right_i, right_f = i_linear, f_linear
                            else:
                                left_i, left_f = i_linear, f_linear
                            if abs(right_f-left_f) < 1e-9:
                                break
                            i_linear = left_i + (f_target-left_f) * (
                                right_i-left_i) / (right_f-left_f)
                            f_linear = plant_at(i_linear)
                        refined_error = abs(f_linear-f_target)
                else:
                    linear_errors.append(abs(gf[np.argmin(abs(gf-f_target))]-f_target))
            crossings = np.flatnonzero((forces[:-1]-f_target) *
                                       (forces[1:]-f_target) <= 0)
            if len(crossings):
                j = int(crossings[0]); lo, hi = grid[j], grid[j+1]
                increasing = forces[j+1] > forces[j]
                for _ in range(16):
                    mid = (lo + hi) / 2
                    if (plant_at(mid) < f_target) == increasing:
                        lo = mid
                    else:
                        hi = mid
                i_exact = (lo + hi) / 2
            else:
                i_exact = grid[np.argmin(abs(forces-f_target))]
            f_exact = plant_at(i_exact)
            solve_ms = (time.perf_counter() - tic) * 1000
            records.append((k, abs(f1 - f0), abs(f_ach - f_target),
                            i_sol, abs(f_exact-f_target), solve_ms,
                            int(not (np.all(np.diff(forces)>=0) or
                                     np.all(np.diff(forces)<=0))),
                            linear_errors[0], linear_errors[1], refined_error))

        # Advance the actual chronological state once, not twice.
        plant_at(current[k])

    arr = np.asarray(records)
    print(f"samples={len(arr)}")
    print(f"authority p50/p95={np.percentile(arr[:,1],[50,95])} N")
    print(f"dynamic inverse |F-Fdes| p50/p95/max="
          f"{np.percentile(arr[:,2],[50,95,100])} N")
    active = arr[:,1] >= 100
    if active.any():
        print(f"authority>=100 N samples={active.sum()}, "
              f"error p50/p95={np.percentile(arr[active,2],[50,95])} N")
    print(f"current saturation fraction="
          f"{np.mean((arr[:,3]<1e-4)|(arr[:,3]>1.6-1e-4)):.1%}")
    print(f"exact same-history inverse error p50/p95="
          f"{np.percentile(arr[:,4],[50,95])} N")
    print(f"full-model inverse Python-CPU time p50/p95="
          f"{np.percentile(arr[:,5],[50,95])} ms/corner")
    print(f"nonmonotone F(I) grid fraction={np.mean(arr[:,6]):.1%}")
    print(f"five-point interpolated inverse error p50/p95="
          f"{np.percentile(arr[:,7],[50,95])} N")
    print(f"nine-point interpolated inverse error p50/p95="
          f"{np.percentile(arr[:,8],[50,95])} N")
    print(f"nine-point + two secant updates error p50/p95="
          f"{np.percentile(arr[:,9],[50,95])} N")


if __name__ == "__main__":
    main()
