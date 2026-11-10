"""ZengSaddleDB_SUV.mat 三维可视化（一次性预览用，产物在 PMPC_Results/ZengSaddleDB/）"""
import numpy as np
import scipy.io as sio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["font.size"] = 10

MAT = "pmpc_core/ZengSaddleDB_SUV.mat"
OUT = "PMPC_Results/ZengSaddleDB"

d = sio.loadmat(MAT)
mu = d["mu_grid"].ravel()
V = d["V_grid_kmh"].ravel()
de = d["delta_grid_deg"].ravel()
bL, bR, ok = d["betaL"], d["betaR"], d["ok"].astype(bool)

# 有效节点展平
I, J, K = np.meshgrid(np.arange(len(mu)), np.arange(len(V)),
                      np.arange(len(de)), indexing="ij")
m = ok
Xv, Yv, Zv = V[J[m]], de[K[m]], mu[I[m]]          # (V, delta, mu)
Lv, Rv = bL[m], bR[m]

# ---------------------------------------------------------------- 图1 三维散点
fig = plt.figure(figsize=(11, 7.5))
ax = fig.add_subplot(111, projection="3d")
norm = matplotlib.colors.Normalize(vmin=mu.min(), vmax=mu.max())
cmap = cm.viridis

for i, m_ in enumerate(mu):
    s = m == i
    if not s.any():
        continue
    c = cmap(norm(m_))
    ax.scatter(V[J[s]], de[K[s]], bL[s], marker="v", s=55, color=c,
               edgecolor="k", linewidth=0.4, alpha=0.9,
               label=f"μ={m_:.1f}  βL" if i == 0 else None)
    ax.scatter(V[J[s]], de[K[s]], bR[s], marker="^", s=55, color=c,
               edgecolor="k", linewidth=0.4, alpha=0.9)

ax.set_xlabel("车速 V [km/h]")
ax.set_ylabel("前轮转角 δ [deg]")
ax.set_zlabel("鞍点边界 β [rad]")
ax.set_title("ZengSaddleDB_SUV：稳定域鞍点边界（▼ βL 下界 / ▲ βR 上界，颜色=μ）")
ax.view_init(elev=22, azim=-58)
sm = cm.ScalarMappable(norm=norm, cmap=cmap)
sm.set_array([])
fig.colorbar(sm, ax=ax, shrink=0.6, pad=0.08, label="路面附着 μ")
fig.tight_layout()
fig.savefig(f"{OUT}/fig1_saddle_3d_scatter.png", dpi=160)
plt.close(fig)

# ------------------------------------------------- 图2 每个 μ 的 βL/βR 曲面
fig = plt.figure(figsize=(15, 8.5))
for i, m_ in enumerate(mu):
    ax = fig.add_subplot(2, 3, i + 1, projection="3d")
    s = m == i
    if s.any():
        # 用有效节点构造散点曲面（数据稀疏，不做插值以免误导）
        ax.plot_trisurf(V[J[s]], de[K[s]], bR[s], cmap="Reds", alpha=0.75,
                        linewidth=0.3, edgecolor="gray")
        ax.plot_trisurf(V[J[s]], de[K[s]], bL[s], cmap="Blues", alpha=0.75,
                        linewidth=0.3, edgecolor="gray")
        ax.scatter(V[J[s]], de[K[s]], bR[s], color="darkred", s=14)
        ax.scatter(V[J[s]], de[K[s]], bL[s], color="navy", s=14)
    ax.set_xlabel("V [km/h]", labelpad=2)
    ax.set_ylabel("δ [deg]", labelpad=2)
    ax.set_zlabel("β [rad]", labelpad=2)
    ax.set_title(f"μ = {m_:.1f}   (有效节点 {int(s.sum())}/8)", fontsize=10)
    ax.view_init(elev=20, azim=-60)
    ax.tick_params(labelsize=8)
fig.suptitle("各附着系数下的稳定域边界曲面（红=βR 上界，蓝=βL 下界）", fontsize=13)
fig.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig(f"{OUT}/fig2_surfaces_per_mu.png", dpi=160)
plt.close(fig)

# ------------------------------------------------------- 图3 ok 掩码 + 稳定域宽度
fig, axes = plt.subplots(2, 3, figsize=(14, 7.5))
for i, m_ in enumerate(mu):
    ax = axes.flat[i]
    im = ax.imshow(ok[i].astype(int), origin="lower", aspect="auto",
                   cmap="Greys", vmin=0, vmax=1.6,
                   extent=[de[0] - 2, de[-1] + 2, -0.5, len(V) - 0.5])
    ax.set_yticks(range(len(V)))
    ax.set_yticklabels([f"{v:.0f}" for v in V])
    ax.set_xticks(de)
    ax.set_xlabel("δ [deg]")
    ax.set_ylabel("V [km/h]")
    ax.set_title(f"μ = {m_:.1f}   ok={int(ok[i].sum())}/40")
    for j in range(len(V)):
        for k in range(len(de)):
            if ok[i, j, k]:
                ax.text(de[k], j, f"{bR[i,j,k]:.2f}", ha="center", va="center",
                        fontsize=6.5, color="crimson")
axes.flat[-1].axis("off")
axes.flat[-1].text(0.02, 0.5,
                   "灰=有效节点(ok=1)\n白=无效节点(ok=0)\n\n"
                   "红字=该节点 βR 上界 [rad]\nβL = −βR（近似对称）\n\n"
                   "有效节点仅 64/200 = 32%，\n"
                   "V=100 km/h 全无效，\n"
                   "V≥60 km/h 只剩 |δ|=2° 有效。",
                   fontsize=10, va="center", family="Microsoft YaHei")
fig.suptitle("数据库有效节点分布（ok 掩码）", fontsize=13)
fig.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(f"{OUT}/fig3_ok_mask.png", dpi=160)
plt.close(fig)

# --------------------------------------------- 图4 稳定域宽度随车速（δ=±2° 切片）
fig, ax = plt.subplots(figsize=(8, 5))
for i, m_ in enumerate(mu):
    w = []
    for j in range(len(V)):
        k = np.where(de == 2)[0][0]
        w.append(bR[i, j, k] - bL[i, j, k] if ok[i, j, k] else np.nan)
    ax.plot(V, w, "o-", label=f"μ={m_:.1f}")
ax.set_xlabel("车速 V [km/h]")
ax.set_ylabel("稳定域宽度 βR − βL [rad]")
ax.set_title("δ = +2° 切片：稳定域宽度随车速收缩")
ax.grid(alpha=0.3)
ax.legend()
fig.tight_layout()
fig.savefig(f"{OUT}/fig4_width_vs_speed.png", dpi=160)
plt.close(fig)

print("done")
