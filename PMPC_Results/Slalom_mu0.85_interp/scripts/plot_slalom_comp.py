"""Slalom 时延补偿对比：no-comp / comp(tau_d=0.15) / 25ms+侧倾预瞄（Proposed，80 & 90 km/h）。"""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parents[1]
plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3,
                     "figure.dpi": 150, "savefig.dpi": 300})

VARIANTS = [("t_nc_%d",  "no-comp (tau_d=1e-4)", "#1f77b4", "-"),
            ("sla_c3_%d","comp (tau_d=0.15)",    "#2ca02c", "--"),
            ("t_c25_%d", "25 ms + roll preview", "#d62728", "-.")]
SPEEDS = [80, 90]

fig, axes = plt.subplots(len(SPEEDS), 5, figsize=(16, 6.4), sharex="row")
for r, sp in enumerate(SPEEDS):
    for stem, lab, col, ls in VARIANTS:
        f = ROOT / (stem % sp + ".mat")
        if not f.exists():
            print("missing", f); continue
        S = loadmat(f); D = S["D"]; t = S["t"].ravel()
        axes[r,0].plot(t, D[0,:], col, ls=ls, lw=1.4, label=lab)
        axes[r,1].plot(t, np.rad2deg(D[1,:]), col, ls=ls, lw=1.4)
        axes[r,2].plot(t, D[18,:], col, ls=ls, lw=1.4)
        axes[r,3].plot(t, D[19,:], col, ls=ls, lw=1.4)
        axes[r,4].plot(t, D[3,:],  col, ls=ls, lw=1.4)
    for c, name in enumerate(["$e_y$ [m]", "$e_\psi$ [deg]", "$M_z$ [N·m]",
                              "SW [deg]", r"$\beta$ [rad]"]):
        axes[r,c].set_xlabel("t [s]")
        if r == 0: axes[r,c].set_title(name)
        axes[r,c].axhline(0, color="k", lw=0.5, alpha=0.4)
    axes[r,0].set_ylabel(f"{sp} km/h")
    axes[r,0].legend(loc="best", fontsize=7)

fig.suptitle("Slalom: damper delay compensation, Proposed controller (mu=0.85, A=1.2)", fontsize=11)
fig.tight_layout(rect=[0,0,1,0.96])
fig.savefig(ROOT / "slalom_comp.pdf", bbox_inches="tight")
print("saved", ROOT / "slalom_comp.pdf")

# 指标表
print("\n%4s %-16s %8s %8s %8s %8s %8s" % ("sp","variant","ey_max","ey_rms","epsi","SW","LTR"))
for sp in SPEEDS:
    for stem, lab, _, _ in VARIANTS:
        f = ROOT / (stem % sp + ".mat")
        if not f.exists(): continue
        S = loadmat(f); D = S["D"]; t = S["t"].ravel()
        m = (t >= 2.0) & np.isfinite(D[0,:])
        print("%4d %-16s %8.3f %8.3f %8.2f %8.1f %8.3f" % (
            sp, lab, np.nanmax(np.abs(D[0,m])), np.sqrt(np.nanmean(D[0,m]**2)),
            np.rad2deg(np.nanmax(np.abs(D[1,m]))),
            np.nanmax(np.abs(D[18,:])), np.nanmax(np.abs(D[3,:]))))
