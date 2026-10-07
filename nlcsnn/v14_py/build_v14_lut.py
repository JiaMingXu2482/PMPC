"""Build a compact fixed force/current surface from v14 and rig histories.

The runtime inverse searches one interpolated current row; no neural model
is evaluated in MATLAB or on an embedded controller. Held-out histories are
exported separately so MATLAB can report actual inverse error before wiring.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.io import savemat
from scipy.spatial import cKDTree
import torch

from .export_v14_matlab import digest
from .model_v14 import NLCSNN_v14


WINDOW = 200
CURRENT_A = np.linspace(0.0, 1.6, 9, dtype=np.float32)
CATEGORIES = ("1_Steady_Harmonic", "2_Steady_Random", "5_Step_Current_Triangle")


def normalize_file(path: Path, norm: dict) -> tuple[np.ndarray, np.ndarray]:
    cols = ["rod_length", "velocity", "accel", "current", "current_dot", "temp"]
    df = pd.read_csv(path, usecols=cols).replace([np.inf, -np.inf], np.nan).dropna()
    physical = df[cols].to_numpy(dtype=np.float32)
    u = np.column_stack((
        (physical[:, 0] - norm["x_ref"]) / norm["x_scale"],
        physical[:, 1] / norm["v_scale"],
        physical[:, 2] / norm["a_scale"],
        physical[:, 3] / norm["i_scale"],
        physical[:, 4] / norm["di_scale"],
        (physical[:, 5] - norm["t_ref"]) / norm["t_scale"],
    )).astype(np.float32)
    return u, physical


def selected_training_files(data_root: Path, holdout: set[str]) -> list[Path]:
    chosen = []
    for category, count in zip(CATEGORIES, (16, 8, 8)):
        files = [p for p in sorted((data_root / category).glob("*.csv"))
                 if p.stem not in holdout]
        if not files:
            continue
        indices = np.unique(np.linspace(0, len(files) - 1, min(count, len(files)),
                                        dtype=int))
        chosen.extend(files[index] for index in indices)
    return chosen


def evaluate_histories(model: NLCSNN_v14, histories: np.ndarray,
                       f_scale: float, batch_size: int = 36) -> np.ndarray:
    out = np.empty(len(histories), dtype=np.float32)
    with torch.inference_mode():
        for start in range(0, len(histories), batch_size):
            part = torch.from_numpy(histories[start:start + batch_size])
            out[start:start + len(part)] = (
                -model(part)[:, -1].numpy() * f_scale  # CarSim force polarity
            )
    return out


def candidate_histories(base: np.ndarray, xyz: np.ndarray,
                        norm: dict) -> np.ndarray:
    result = np.repeat(base[None, :, :], len(CURRENT_A), axis=0)
    result[:, -1, 0] = (xyz[0] - norm["x_ref"]) / norm["x_scale"]
    result[:, -1, 1] = xyz[1] / norm["v_scale"]
    result[:, -1, 2] = xyz[2] / norm["a_scale"]
    result[:, -1, 3] = CURRENT_A / norm["i_scale"]
    result[:, -1, 4] = 0.0  # held command at the 100 Hz allocation boundary
    result[:, -1, 5] = (42.5 - norm["t_ref"]) / norm["t_scale"]
    return result


def load_model(checkpoint: Path) -> tuple[NLCSNN_v14, dict, dict]:
    ckpt = torch.load(checkpoint, map_location="cpu", weights_only=True)
    cfg = ckpt["model_cfg"]
    model = NLCSNN_v14(gru_hidden=cfg["gru_hidden"],
                       gru_layers=cfg["gru_layers"],
                       mlp_hidden=cfg["mlp_hidden"],
                       g_hidden=cfg["g_hidden"], sigmas=tuple(cfg["sigmas"]))
    model.load_state_dict(ckpt["state_dict"], strict=True)
    model.eval()
    return model, ckpt["norm_cfg"], ckpt


def build(checkpoint: Path, data_root: Path, out_dir: Path) -> None:
    torch.set_num_threads(4)
    model, norm, ckpt = load_model(checkpoint)
    holdout = set(ckpt["holdout_stems"])
    training = selected_training_files(data_root, holdout)
    if len(training) < 8:
        raise RuntimeError("Insufficient processed training histories for LUT")
    sequences = []
    samples = []
    for path in training:
        u, physical = normalize_file(path, norm)
        u = u[:3000]
        physical = physical[:3000]
        if len(u) <= WINDOW:
            continue
        seq_id = len(sequences)
        sequences.append(u)
        for t in range(WINDOW, len(u), 60):
            samples.append((seq_id, t, physical[t, :3].copy()))
    locations = np.stack([item[2] for item in samples])
    scales = np.array([norm["x_scale"], norm["v_scale"], norm["a_scale"]],
                      dtype=np.float32)
    tree = cKDTree(locations / scales)
    quantiles = ((0, .1, .3, .5, .7, .9, 1),
                 (0, .04, .12, .25, .5, .75, .88, .96, 1),
                 (0, .1, .5, .9, 1))
    axes = [np.unique(np.quantile(locations[:, j], q).astype(np.float32))
            for j, q in enumerate(quantiles)]
    if any(len(axis) < 2 for axis in axes):
        raise RuntimeError("Degenerate LUT axis in processed rig data")
    shape = tuple(len(axis) for axis in axes) + (len(CURRENT_A),)
    force = np.empty(shape, dtype=np.float32)
    pending = []
    indices = []
    for index in np.ndindex(*shape[:3]):
        xyz = np.array([axes[j][index[j]] for j in range(3)], dtype=np.float32)
        _, nearest = tree.query(xyz / scales)
        seq_id, t, _ = samples[int(nearest)]
        base = sequences[seq_id][t-WINDOW+1:t+1]
        pending.extend(candidate_histories(base, xyz, norm))
        indices.append(index)
        if len(pending) >= 36:
            values = evaluate_histories(model, np.stack(pending), norm["f_scale"])
            for j, key in enumerate(indices):
                force[key] = values[j*len(CURRENT_A):(j+1)*len(CURRENT_A)]
            pending.clear(); indices.clear()
    if pending:
        values = evaluate_histories(model, np.stack(pending), norm["f_scale"])
        for j, key in enumerate(indices):
            force[key] = values[j*len(CURRENT_A):(j+1)*len(CURRENT_A)]

    out_dir.mkdir(parents=True, exist_ok=True)
    lut = {
        "x_axis_mm": axes[0], "v_axis_mmps": axes[1],
        "a_axis_mmps2": axes[2], "i_axis_A": CURRENT_A,
        "force_N": force, "authority_tol_N": np.float32(10),
        "checkpoint_md5": digest(checkpoint), "version": "v14-fixed-lut-1",
    }
    savemat(out_dir / "v14_actuator_lut.mat", {"lut": lut},
            do_compression=True, oned_as="column")

    heldout_paths = [p for p in sorted(data_root.glob("*/*.csv"))
                     if p.stem in holdout]
    target_data = []
    candidates = []
    for path in heldout_paths:
        u, physical = normalize_file(path, norm)
        if len(u) <= WINDOW:
            continue
        u = u[:3000]; physical = physical[:3000]
        with torch.inference_mode():
            full_force = (-model(torch.from_numpy(u[None]))[0].numpy()
                          * norm["f_scale"])
        for t in range(WINDOW, len(u), 100):
            xyz = physical[t, :3]
            base = u[t-WINDOW+1:t+1]
            candidates.extend(candidate_histories(base, xyz, norm))
            target_data.append((xyz[0], xyz[1], xyz[2], full_force[t],
                                physical[t, 3]))
    if not target_data:
        raise RuntimeError("No held-out processed histories found")
    true_curves = evaluate_histories(model, np.stack(candidates), norm["f_scale"])
    true_curves = true_curves.reshape(len(target_data), len(CURRENT_A))
    V = np.asarray(target_data, dtype=np.float32)
    reachable_target = np.array([
        np.interp(np.clip(V[j, 4], 0, 1.6), CURRENT_A, true_curves[j])
        for j in range(len(V))], dtype=np.float32)
    savemat(out_dir / "v14_lut_validation.mat", {
        "x_mm": V[:, 0], "v_mmps": V[:, 1], "a_mmps2": V[:, 2],
        "target_force_N": reachable_target,
        "full_model_force_N": V[:, 3],
        "actual_current_A": V[:, 4],
        "true_force_curve_N": true_curves,
        "current_axis_A": CURRENT_A,
        "heldout_file_count": np.int32(len(heldout_paths)),
        "training_file_count": np.int32(len(training)),
        "checkpoint_md5": digest(checkpoint),
    }, do_compression=True, oned_as="column")
    print(f"LUT {shape}, {force.nbytes} force bytes, "
          f"train={len(training)} files, heldout={len(heldout_paths)} files, "
          f"validation={len(target_data)} points")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", type=Path, required=True)
    p.add_argument("--data-root", type=Path, required=True)
    p.add_argument("--out-dir", type=Path, required=True)
    a = p.parse_args()
    build(a.checkpoint, a.data_root, a.out_dir)
