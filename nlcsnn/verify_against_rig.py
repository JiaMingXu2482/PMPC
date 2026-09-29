#!/usr/bin/env python3
"""verify_against_rig.py — 用台架实测 CSV 验证模型方向约定 (x/v/F 正负号).

原理: 模型学的是训练 CSV 里记录的方向。把同一批 CSV 的 (x,v,a,I,T) 送入
模型, 对比预测 F 与实测 F:
  预测 ≈ 实测   -> 方向全对
  预测 ≈ -实测  -> F 极性反了 (s_F = -1)
  相位差半个周期 -> x/v 方向反了 (s_x = -1, 或 v 不是 x 的导数)

用法:
  python verify_against_rig.py <rig.csv> --ckpt nlcsnn_legacy_best_all_val.pth

CSV 列映射在下面的 COL 里改: 列号(0起)或列名(有表头时)。
只需要: 时间, 位移x[mm], 力F[N], 电流I[A]; 速度/加速度/温度列若没有,
脚本会从 x 数值微分 / 用默认值补。
"""
import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from export_matlab import NLCSNN, load_weights, normalize, NORM, DT_TRAIN, H_DIM

# ------------------------------- 按你的 CSV 改这里 -------------------------------
# 填列号(0起); 有表头时也可填列名字符串; 没有的列填 None。
COL = dict(
    time=0,      # 时间 [s]
    x=1,         # 作动器位移 [mm]
    v=None,      # 速度 [mm/s], None 则由 x 微分
    a=None,      # 加速度 [mm/s^2], None 则由 v 微分
    F=2,         # 实测力 [N]
    I=3,         # 电流 [A]
    T=None,      # 油温 [degC], None 则用 40
)
HAS_HEADER = True   # CSV 首行是否为表头
# --------------------------------------------------------------------------------


def col_idx(header, spec):
    if spec is None:
        return None
    if isinstance(spec, int):
        return spec
    return header.index(spec)


def load_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    if HAS_HEADER:
        header, data = rows[0], rows[1:]
    else:
        header, data = None, rows
    M = np.array(data, dtype=np.float64)
    ci = lambda s: col_idx(header, COL[s])  # noqa: E731
    t = M[:, ci("time")]
    x = M[:, ci("x")]
    F_meas = M[:, ci("F")]
    I = M[:, ci("I")]
    v = M[:, ci("v")] if ci("v") is not None else np.gradient(x, t)
    a = M[:, ci("a")] if ci("a") is not None else np.gradient(v, t)
    T = M[:, ci("T")] if ci("T") is not None else np.full_like(t, 40.0)
    return t, x, v, a, I, T, F_meas


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="台架实测 CSV")
    ap.add_argument("--ckpt", default="nlcsnn_legacy_best_all_val.pth")
    ap.add_argument("--out", default="rig_verify_out.csv",
                    help="输出对比 CSV (MATLAB 里画图用)")
    args = ap.parse_args()

    t, x, v, a, I, T, F_meas = load_csv(args.csv)
    dI = np.gradient(I, t)
    U = np.stack([x, v, a, I, dI, T], axis=1)

    model = NLCSNN(load_weights(args.ckpt))
    dt = float(np.median(np.diff(t)))
    n_sub = max(1, int(round(dt / DT_TRAIN)))
    dts = dt / n_sub

    h = np.zeros(H_DIM)
    F_pred = np.empty(len(t))
    for n in range(len(t)):
        u = normalize(U[n])
        F_pred[n] = model.predict_force(u, h) * NORM["f_scale"]
        for _ in range(n_sub):
            h = model.rk4_step(u, h, dts)

    # ---- 数据自检: v 是否真是 x 的导数? ----
    v_from_x = np.gradient(x, t)
    c_vx = np.corrcoef(v, v_from_x)[0, 1]

    # ---- 方向诊断 ----
    rmse = lambda p, m: float(np.sqrt(np.mean((p - m) ** 2)))
    r = float(np.corrcoef(F_pred, F_meas)[0, 1])
    e_pos = rmse(F_pred, F_meas)
    e_neg = rmse(F_pred, -F_meas)
    std_m = float(np.std(F_meas))

    print(f"样本数: {len(t)}, dt≈{dt*1000:.2f}ms, x∈[{x.min():.1f},{x.max():.1f}]mm, "
          f"I∈[{I.min():.2f},{I.max():.2f}]A")
    print(f"[自检] corr(v列, dx/dt) = {c_vx:.4f}  " +
          ("(v 与 x 自洽)" if c_vx > 0.99 else "(! v 不是 x 的导数, 先检查列映射)"))
    print(f"[方向] corr(F预测, F实测) = {r:.4f}")
    print(f"       RMSE(预测 vs 实测)  = {e_pos:.1f} N  (相对 {e_pos/std_m*100:.1f}% std)")
    print(f"       RMSE(预测 vs -实测) = {e_neg:.1f} N")
    if e_pos < e_neg and r > 0.8:
        print(">>> 结论: 方向正确, s_x=+1, s_F=+1, 可直接用。")
    elif e_neg < e_pos and r < -0.8:
        print(">>> 结论: F 极性反了! 车辆侧用 F_carsim = -F_model (s_F=-1)。")
    else:
        print(">>> 结论: 不确定。请检查列映射(COL), 或看输出 CSV 的曲线: "
              "若相位差半个周期则是 x/v 反了 (s_x=-1)。")

    # 输出对比 CSV, 降采样到至多 5000 点, MATLAB 里直接 plot 即可
    idx = np.linspace(0, len(t) - 1, min(len(t), 5000)).astype(int)
    with open(args.out, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["t_s", "x_mm", "v_mms", "I_A", "F_meas_N", "F_pred_N"])
        wr.writerows(np.stack(
            [t[idx], x[idx], v[idx], I[idx], F_meas[idx], F_pred[idx]], axis=1))
    print(f"对比数据已写入 {args.out} (列: t,x,v,I,F_meas,F_pred)")


if __name__ == "__main__":
    main()
