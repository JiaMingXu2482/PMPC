"""Export the supplied v14 checkpoint and independent PyTorch reference traces.

Run offline with CPU PyTorch and SciPy. No PyTorch code is used at simulation time.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import numpy as np
from scipy.io import savemat
import torch

from .model_v12 import static_nfl_v12
from .model_v14 import NLCSNN_v14


def digest(path: Path) -> str:
    h = hashlib.md5()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def make_inputs(length: int = 160) -> np.ndarray:
    """Four normalized histories: zero speed, reversal, current step, mixed."""
    t = np.arange(length, dtype=np.float32)
    u = np.zeros((4, length, 6), dtype=np.float32)
    u[:, :, 5] = np.float32((42.5 - 40.0) / 40.0)
    u[0, :, 0] = 0.1
    u[0, :, 3] = np.linspace(0.1, 0.8, length, dtype=np.float32)
    u[1, :, 0] = 0.4 * np.sin(t / 9.0)
    u[1, :, 1] = 0.18 * np.sin(t / 5.0)
    u[1, :, 2] = 0.12 * np.cos(t / 5.0)
    u[1, :, 3] = 0.5
    u[2, :, 0] = -0.25
    u[2, :, 1] = 0.16
    u[2, 10:, 3] = 1.0
    u[3, :, 0] = -0.35 + t / 90.0
    u[3, :, 1] = -0.13 + t / 250.0
    u[3, :, 2] = -0.07
    u[3, :, 3] = 0.45 + 0.25 * np.sin(t / 6.0)
    # dI/dt normalization follows the checkpoint's 1.6 A / 142.517 A/s.
    u[:, 1:, 4] = np.diff(u[:, :, 3], axis=1) * (1.6 / 0.001 / 142.517001)
    return u


def export(checkpoint: Path, source_zip: Path, out_dir: Path) -> None:
    torch.set_num_threads(1)
    torch.manual_seed(0)
    ckpt = torch.load(checkpoint, map_location="cpu", weights_only=True)
    cfg = ckpt["model_cfg"]
    model = NLCSNN_v14(
        gru_hidden=cfg["gru_hidden"],
        gru_layers=cfg["gru_layers"],
        mlp_hidden=cfg["mlp_hidden"],
        g_hidden=cfg["g_hidden"],
        sigmas=tuple(cfg["sigmas"]),
    )
    model.load_state_dict(ckpt["state_dict"], strict=True)
    model.eval()
    a = torch.sigmoid(model.i_lag.log_a).item()
    tau = -1.0 / np.log(1.0 - a + 1e-12)
    lag_length = int(6.0 * tau) + 1
    # CurrentLag uses conv1d cross-correlation with padding K-1. In
    # chronological streaming order its kernel is therefore reversed.
    lag_kernel = (a * (1.0 - a) ** np.arange(lag_length - 1, -1, -1))
    md5 = digest(checkpoint)
    weights = {
        key.replace(".", "_"): value.detach().cpu().numpy().astype(np.float32)
        for key, value in model.state_dict().items()
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    matlab_cfg = {key: value for key, value in cfg.items() if value is not None}
    savemat(out_dir / "v14_weights.mat", {
        "weights": weights,
        "model_cfg": matlab_cfg,
        "norm_cfg": ckpt["norm_cfg"],
        "current_lag_a": np.float32(a),
        "current_lag_kernel": lag_kernel.astype(np.float32),
        "current_lag_length": np.int32(lag_length),
        "state_length": np.int32(2 * cfg["gru_hidden"] + lag_length - 1),
        "checkpoint_md5": md5,
        "source_zip_md5": digest(source_zip),
        "torch_version": torch.__version__,
    }, do_compression=True, oned_as="column")

    raw = make_inputs()
    u = torch.from_numpy(raw)
    with torch.no_grad():
        force = model(u).numpy()
        nfl = static_nfl_v12(u, sigma_v=model.nfl_sigma_v)
        hidden = torch.zeros(model.gru_layers, u.shape[0], model.gru_hidden)
        states = []
        for t in range(u.shape[1]):
            _, hidden = model.gru(nfl[:, t:t + 1, :], hidden)
            states.append(hidden.permute(1, 0, 2).reshape(u.shape[0], -1).numpy().copy())
        states = np.stack(states, axis=1)
        effective = model.i_lag(1.0 - u[:, :, 3].abs().clamp(0, 1)).numpy()
    savemat(out_dir / "v14_golden_vectors.mat", {
        "u_norm": raw.transpose(2, 1, 0),
        "force_norm": force.T,
        "gru_state": states.transpose(2, 1, 0),
        "i_effective": effective.T,
        "checkpoint_md5": md5,
    }, do_compression=True, oned_as="column")
    print(f"Exported v14: checkpoint={md5}, torch={torch.__version__}, "
          f"lag_length={lag_length}, sequences={raw.shape[0]}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--source-zip", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()
    export(args.checkpoint, args.source_zip, args.out_dir)
