"""
Offline training script for OdometryNet.

Usage:
    python -m background.pipelines.fusion.learned_odometry.train
    python -m background.pipelines.fusion.learned_odometry.train --data path/to/data.npz
    python -m background.pipelines.fusion.learned_odometry.train --data data.npz --epochs 200 --lr 5e-4

Outputs (written to assets/ next to this file):
    odometry_net.pt   — trained model weights
    imu_mean.npy      — per-feature mean   (shape: 6,)
    imu_std.npy       — per-feature std    (shape: 6,)

Minimum recommended dataset: 2000 samples. 5000+ for good generalization.
Each sample is one (window_size=200, 6) IMU window paired with a (2,) UWB delta.
"""

import argparse
import time
from pathlib import Path

import numpy as np

_ASSETS = Path(__file__).parent / "assets"


def _split(X: "np.ndarray", Y: "np.ndarray", val_frac: float = 0.15):
    n = len(X)
    idx = np.random.permutation(n)
    split = int(n * (1 - val_frac))
    tr, va = idx[:split], idx[split:]
    return X[tr], Y[tr], X[va], Y[va]


def train(
    data_path: str = "odometry_training_data.npz",
    epochs: int = 100,
    batch_size: int = 32,
    lr: float = 1e-3,
    val_frac: float = 0.15,
    seed: int = 42,
) -> None:
    import torch
    import torch.nn as nn
    from torch.utils.data import DataLoader, TensorDataset
    from .model import OdometryNet

    np.random.seed(seed)
    torch.manual_seed(seed)

    # ── Load data ─────────────────────────────────────────────────────────────
    p = Path(data_path)
    if not p.exists():
        raise FileNotFoundError(
            f"Training data not found: {p}\n"
            "Run the visualizer with --collect to record a session first."
        )

    data = np.load(p)
    X_raw = data["windows"].astype(np.float32)  # (N, W, 6)
    Y_raw = data["deltas"].astype(np.float32)   # (N, 2)
    N = len(X_raw)
    print(f"[Train] Loaded {N} samples from {p}")

    if N < 200:
        print(f"[Train] WARNING: only {N} samples — model will likely overfit. "
              "Collect more data (target: 2000+).")

    # ── Normalization stats ────────────────────────────────────────────────────
    # Compute over all frames in all windows
    all_frames = X_raw.reshape(-1, X_raw.shape[-1])   # (N*W, 6)
    mean = all_frames.mean(axis=0)
    std  = all_frames.std(axis=0) + 1e-8

    _ASSETS.mkdir(exist_ok=True)
    np.save(_ASSETS / "imu_mean.npy", mean)
    np.save(_ASSETS / "imu_std.npy",  std)
    print(f"[Train] Saved normalization stats → {_ASSETS}")

    # Normalize
    X_norm = (X_raw - mean) / std  # broadcasts over (N, W, 6)

    # ── Train / val split ─────────────────────────────────────────────────────
    X_tr, Y_tr, X_va, Y_va = _split(X_norm, Y_raw, val_frac)
    print(f"[Train] Train: {len(X_tr)}  Val: {len(X_va)}")

    X_tr_t = torch.tensor(X_tr)
    Y_tr_t = torch.tensor(Y_tr)
    X_va_t = torch.tensor(X_va)
    Y_va_t = torch.tensor(Y_va)

    loader = DataLoader(
        TensorDataset(X_tr_t, Y_tr_t),
        batch_size=batch_size,
        shuffle=True,
        drop_last=False,
    )

    # ── Model, optimizer, loss ────────────────────────────────────────────────
    model   = OdometryNet(input_size=6, hidden=256)
    opt     = torch.optim.Adam(model.parameters(), lr=lr)
    # Cosine LR decay — smoothly reduces lr to lr/10 over training
    sched   = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs, eta_min=lr / 10)
    loss_fn = nn.MSELoss()

    best_val  = float('inf')
    best_path = _ASSETS / "odometry_net.pt"
    t0 = time.time()

    print(f"\n{'Epoch':>6}  {'Train RMSE (mm)':>16}  {'Val RMSE (mm)':>14}  {'LR':>8}  {'Time':>6}")
    print("─" * 60)

    for epoch in range(1, epochs + 1):
        # ── training pass ──────────────────────────────────────────────────
        model.train()
        tr_loss = 0.0
        for x_b, y_b in loader:
            pred = model(x_b)
            loss = loss_fn(pred, y_b)
            opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            opt.step()
            tr_loss += loss.item() * len(x_b)
        tr_rmse = (tr_loss / len(X_tr)) ** 0.5 * 1000  # → mm

        # ── validation pass ────────────────────────────────────────────────
        model.eval()
        with torch.no_grad():
            val_pred = model(X_va_t)
            val_rmse = float(loss_fn(val_pred, Y_va_t).item() ** 0.5) * 1000  # mm

        sched.step()
        cur_lr = sched.get_last_lr()[0]

        if val_rmse < best_val:
            best_val = val_rmse
            torch.save(model.state_dict(), best_path)

        if epoch % 10 == 0 or epoch == 1:
            elapsed = time.time() - t0
            print(f"{epoch:>6}  {tr_rmse:>16.1f}  {val_rmse:>14.1f}  {cur_lr:>8.6f}  {elapsed:>5.0f}s")

    print("─" * 60)
    print(f"[Train] Best val RMSE: {best_val:.1f} mm")
    print(f"[Train] Saved model → {best_path}")
    print(
        "\nNext step: run the visualizer normally (no --collect) and "
        "the green Odometry (NN) curve will appear."
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train OdometryNet")
    parser.add_argument("--data",   default="odometry_training_data.npz",
                        help="Path to .npz file from the collector")
    parser.add_argument("--epochs", type=int,   default=100)
    parser.add_argument("--lr",     type=float, default=1e-3)
    parser.add_argument("--batch",  type=int,   default=32)
    parser.add_argument("--seed",   type=int,   default=42)
    args = parser.parse_args()

    train(
        data_path=args.data,
        epochs=args.epochs,
        lr=args.lr,
        batch_size=args.batch,
        seed=args.seed,
    )
