#!/usr/bin/env python3
"""export_matlab.py — 从训练 checkpoint 可复现地生成 MATLAB 用的两个 .mat 文件.

生成:
  nlcsnn_weights.mat   网络权重 + 归一化参数 (给 nlcsnn_damper_init.m 用)
  golden_vectors.mat   100 步黄金测试向量 (给 test_nlcsnn_port.m 用)

依赖: torch, numpy, scipy
用法: python export_matlab.py [checkpoint.pth] [输出目录]

全部约定(单位/顺序/公式)都写在注释里, 与 MATLAB 端一一对应。
"""
import hashlib
import os
import sys

import numpy as np
import scipy.io as sio
from scipy.special import erf

# ---------------------------------------------------------------- 输入
CKPT_PATH = sys.argv[1] if len(sys.argv) > 1 else "nlcsnn_legacy_best_all_val.pth"
OUT_DIR = sys.argv[2] if len(sys.argv) > 2 else "."

# ------------------------------------------------- 归一化参数 (checkpoint norm_cfg)
# 物理输入向量顺序 (固定, 与训练一致):
#   u_phys = [x, v, a, I, dI/dt, T]
#   单位   = [mm, mm/s, mm/s^2, A, A/s, degC]
# 归一化公式:
#   u = [(x-x_ref)/x_scale, v/v_scale, a/a_scale,
#        I/i_scale, (dI/dt)/di_scale, (T-t_ref)/t_scale]
# 力反归一化:  F[N] = y * f_scale
NORM = dict(
    x_ref=281.0645, x_scale=62.039501,   # x: 台架作动器绝对位置 [mm], 中心~281mm
    v_scale=1000.0,                      # v: 台架速度 [mm/s], 必须 = dx/dt
    a_scale=50000.0,                     # a: 台架加速度 [mm/s^2], = dv/dt
    i_scale=1.6,                         # I: 电流 [A], 台架 0~1.6A
    di_scale=142.517001,                 # dI/dt: 电流变化率 [A/s]
    t_ref=40.0, t_scale=40.0,            # T: 油温 [degC]
    f_scale=7662.190001,                 # F: 台架力 [N]
    input_clamp=5.0,                     # 归一化输入限幅 ±5
)
H_DIM = 8
DT_TRAIN = 0.001  # 隐状态 RK4 基准步长 [s]


def normalize(u_phys):
    u = np.empty_like(u_phys, dtype=np.float64)
    u[..., 0] = (u_phys[..., 0] - NORM["x_ref"]) / NORM["x_scale"]
    u[..., 1] = u_phys[..., 1] / NORM["v_scale"]
    u[..., 2] = u_phys[..., 2] / NORM["a_scale"]
    u[..., 3] = u_phys[..., 3] / NORM["i_scale"]
    u[..., 4] = u_phys[..., 4] / NORM["di_scale"]
    u[..., 5] = (u_phys[..., 5] - NORM["t_ref"]) / NORM["t_scale"]
    return u


def gelu(x):
    return 0.5 * x * (1.0 + erf(x / np.sqrt(2.0)))


def build_nfl(u, h):
    """63 维 physics NFL (physics_nfl=true), 顺序与 MATLAB 端 build_nfl 一致."""
    uc = np.clip(u, -NORM["input_clamp"], NORM["input_clamp"])
    x, v, a, ci = uc[0], uc[1], uc[2], uc[3]
    v_abs = abs(v)
    v_pos = max(v, 0.0)
    v_neg = -max(-v, 0.0)
    sgn_v = np.tanh(50.0 * v)
    laminar = [v, v_pos, v_neg]                                          # 3
    turbulent = [v_abs * v, v_pos * v, v_neg * v_abs]                     # 3
    current_mod = [v * ci, v * ci * ci, v_abs * v * ci, v_abs * v * ci * ci]  # 4
    inertia = [a, a * v]                                                 # 2
    gas = [x, x * x, x * x * x]                                          # 3
    friction = [sgn_v, ci * sgn_v]                                       # 2
    valve = [np.tanh(5.0 * v_pos), np.tanh(20.0 * v_pos),                # 6
             np.tanh(5.0 * v_neg), np.tanh(20.0 * v_neg),
             ci * np.tanh(10.0 * v_pos), ci * np.tanh(10.0 * v_neg)]
    state = np.concatenate([h, h * v, h * v_abs, h * ci, h * a])          # 40
    return np.concatenate([laminar, turbulent, current_mod, inertia,
                           gas, friction, valve, state])                 # =63


class NLCSNN:
    def __init__(self, w):
        self.w = w

    def state_derivative(self, u, h):
        w = self.w
        z = gelu(build_nfl(u, h) @ w["nn_x_0_W"].T + w["nn_x_0_b"])
        z = gelu(z @ w["nn_x_2_W"].T + w["nn_x_2_b"])
        return z @ w["nn_x_4_W"].T

    def predict_force(self, u, h):
        w = self.w
        nfl = build_nfl(u, h)
        z = gelu(nfl @ w["nn_y_0_W"].T + w["nn_y_0_b"])
        z = gelu(z @ w["nn_y_2_W"].T + w["nn_y_2_b"])
        return float((z @ w["nn_y_4_W"].T + w["nn_y_4_b"])[0])

    def rk4_step(self, u, h, dt=DT_TRAIN):
        k1 = self.state_derivative(u, h)
        k2 = self.state_derivative(u, h + 0.5 * dt * k1)
        k3 = self.state_derivative(u, h + 0.5 * dt * k2)
        k4 = self.state_derivative(u, h + dt * k3)
        return h + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def load_weights(path):
    import torch
    sd = torch.load(path, map_location="cpu", weights_only=False)["state_dict"]
    rename = (("nn_x.0.weight", "nn_x_0_W"), ("nn_x.0.bias", "nn_x_0_b"),
              ("nn_x.2.weight", "nn_x_2_W"), ("nn_x.2.bias", "nn_x_2_b"),
              ("nn_x.4.weight", "nn_x_4_W"),
              ("nn_y.0.weight", "nn_y_0_W"), ("nn_y.0.bias", "nn_y_0_b"),
              ("nn_y.2.weight", "nn_y_2_W"), ("nn_y.2.bias", "nn_y_2_b"),
              ("nn_y.4.weight", "nn_y_4_W"), ("nn_y.4.bias", "nn_y_4_b"))
    w = {}
    for k, v in sd.items():
        for a, b in rename:
            k = k.replace(a, b)
        w[k] = v.detach().cpu().numpy().astype(np.float64)
    return w


def export_weights(w, path):
    """权重: MATLAB 里 Wx1 是 (256,63), 做 y = W*x (不用转置)。"""
    mat = {
        "Wx1": w["nn_x_0_W"], "bx1": w["nn_x_0_b"].reshape(-1, 1),
        "Wx2": w["nn_x_2_W"], "bx2": w["nn_x_2_b"].reshape(-1, 1),
        "Wx3": w["nn_x_4_W"],
        "Wy1": w["nn_y_0_W"], "by1": w["nn_y_0_b"].reshape(-1, 1),
        "Wy2": w["nn_y_2_W"], "by2": w["nn_y_2_b"].reshape(-1, 1),
        "Wy3": w["nn_y_4_W"], "by3": float(w["nn_y_4_b"][0]),
        "norm": np.array(
            [(NORM["x_ref"], NORM["x_scale"], NORM["v_scale"], NORM["a_scale"],
              NORM["i_scale"], NORM["di_scale"], NORM["t_ref"], NORM["t_scale"],
              NORM["f_scale"], NORM["input_clamp"])],
            dtype=[("x_ref", "f8"), ("x_scale", "f8"), ("v_scale", "f8"),
                   ("a_scale", "f8"), ("i_scale", "f8"), ("di_scale", "f8"),
                   ("t_ref", "f8"), ("t_scale", "f8"),
                   ("f_scale", "f8"), ("input_clamp", "f8")]),
    }
    sio.savemat(path, mat)


def export_golden(model, path):
    """100 步黄金向量: 2Hz/±15mm 正弦 + 0.4/1.2A 方波电流, 10ms 宏步.

    关键: v = dx/dt, a = dv/dt 严格按解析导数构造, 与台架"速度是位移的导数"
    的约定一致。两种推理顺序的力都保存; 隐状态演化与顺序无关, 故 h_final
    两份字段值相同。
    """
    dt_macro, N = 0.01, 100
    t = np.arange(N) * dt_macro
    f0, Xamp = 2.0, 15.0
    x = NORM["x_ref"] + Xamp * np.sin(2 * np.pi * f0 * t)
    v = Xamp * 2 * np.pi * f0 * np.cos(2 * np.pi * f0 * t)
    a = -Xamp * (2 * np.pi * f0) ** 2 * np.sin(2 * np.pi * f0 * t)
    curr = np.where((t // 0.25) % 2 == 0, 0.4, 1.2)
    dcurr = np.gradient(curr, dt_macro)
    temp = np.full(N, 42.5)
    U = np.stack([x, v, a, curr, dcurr, temp], axis=1)

    n_sub = int(round(dt_macro / DT_TRAIN))
    dts = dt_macro / n_sub

    def rollout(order):
        h = np.zeros(H_DIM)
        F = np.empty(N)
        for n in range(N):
            u = normalize(U[n])
            if order == "predict_then_update":
                y = model.predict_force(u, h)          # 先算力(步初 h)
            for _ in range(n_sub):
                h = model.rk4_step(u, h, dts)          # 再推进隐状态
            if order == "update_then_predict":
                y = model.predict_force(u, h)
            F[n] = y * NORM["f_scale"]
        return F, h

    F_pre, h_pre = rollout("predict_then_update")
    F_upd, h_upd = rollout("update_then_predict")
    assert np.allclose(h_pre, h_upd, atol=1e-12), "h 不应随推理顺序变化"
    sio.savemat(path, {
        "U_phys": U, "dt_macro": dt_macro,
        "F_predict_then_update": F_pre,
        "F_update_then_predict": F_upd,
        "h_final_predict_then_update": h_pre,
        "h_final_update_then_predict": h_upd,
    })
    return U, F_pre


def md5(path):
    return hashlib.md5(open(path, "rb").read()).hexdigest()


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print(f"[1/4] loading checkpoint: {CKPT_PATH}")
    w = load_weights(CKPT_PATH)
    model = NLCSNN(w)

    pw = os.path.join(OUT_DIR, "nlcsnn_weights.mat")
    print(f"[2/4] writing {pw}")
    export_weights(w, pw)

    pg = os.path.join(OUT_DIR, "golden_vectors.mat")
    print(f"[3/4] writing {pg}")
    U, F_pre = export_golden(model, pg)

    print("[4/4] verifying reload + spot check")
    w2 = sio.loadmat(pw)
    g2 = sio.loadmat(pg)
    assert w2["Wx1"].shape == (256, 63) and w2["Wy3"].shape == (1, 256)
    assert g2["U_phys"].shape == (100, 6)
    assert "h_final_predict_then_update" in g2 and "h_final_update_then_predict" in g2
    # 抽查: 第 0 步前向力应与 golden 一致 (predict_then_update, h=0)
    y0 = model.predict_force(normalize(U[0]), np.zeros(H_DIM)) * NORM["f_scale"]
    assert abs(y0 - F_pre[0]) < 1e-9, f"spot check failed: {y0} vs {F_pre[0]}"
    print("  reload OK, spot check OK")
    print(f"  md5({os.path.basename(pw)}) = {md5(pw)}")
    print(f"  md5({os.path.basename(pg)}) = {md5(pg)}")
    print("DONE")


if __name__ == "__main__":
    main()
