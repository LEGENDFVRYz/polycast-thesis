"""Offline training script for OdometryNet.

Standard training:
    python -m background.pipelines.fusion.learned_odometry.train \\
        data/odometry/training_data.npz --save-dir assets/

Optuna hyperparameter search (pip install optuna first):
    python -m background.pipelines.fusion.learned_odometry.train \\
        data/odometry/training_data.npz --save-dir assets/ --optuna --trials 50
"""

from __future__ import annotations
import os
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")

import argparse
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, random_split

import json
from .model import build_model, nll_loss, build_denoiser


# ---------------------------------------------------------------------------
# Augmentation
# ---------------------------------------------------------------------------

def _rotate_augment(
    window: np.ndarray,
    label:  np.ndarray,
    max_deg: float = 30.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Random 2D rotation — wider range (±30°) than the original ±15°."""
    theta = np.radians(np.random.uniform(-max_deg, max_deg))
    c = np.cos(theta).astype(np.float32)
    s = np.sin(theta).astype(np.float32)
    R = np.array([[c, -s], [s, c]], dtype=np.float32)
    w = window.copy()
    w[:, :2] = window[:, :2] @ R.T   # rotate ax_world, ay_world
    return w, (R @ label).astype(np.float32)


def _noise_augment(window: np.ndarray, noise_std: float = 0.02) -> np.ndarray:
    """Gaussian noise on accel channels (first 3) in normalised space.

    Simulates sensor noise variation across recording sessions.  Applied
    after z-score normalisation so noise_std is in units of σ, not m/s².
    """
    w = window.copy()
    w[:, :3] += np.random.normal(0.0, noise_std, (len(w), 3)).astype(np.float32)
    return w


def _time_scale_augment(
    window: np.ndarray,
    scale_range: tuple[float, float] = (0.85, 1.15),
) -> np.ndarray:
    """Randomly resample the time axis to simulate different drawing speeds.

    scale > 1 → stretched (slower drawing, sees less of the stroke)
    scale < 1 → compressed (faster drawing, sees more of the stroke)
    The label is unchanged — valid because the label is the full known
    physical displacement regardless of how fast the stroke was drawn.
    Uses linear interpolation between adjacent frames.
    """
    T = len(window)
    scale = np.random.uniform(scale_range[0], scale_range[1])
    # Source indices in the original window for each output frame
    src = np.linspace(0, T - 1, int(round(T * scale)), dtype=np.float32)
    src = np.clip(src, 0, T - 1)
    lo  = np.floor(src).astype(np.int32)
    hi  = np.minimum(lo + 1, T - 1)
    frac = (src - lo).astype(np.float32)[:, None]   # (T', 1)
    resampled = window[lo] * (1.0 - frac) + window[hi] * frac  # (T', 7)

    # Crop or pad back to original length T
    T_new = len(resampled)
    if T_new >= T:
        return resampled[:T]
    # Pad end with last frame if shorter (edge case: scale < 1 and T' < T)
    pad = np.tile(resampled[-1:], (T - T_new, 1))
    return np.concatenate([resampled, pad], axis=0).astype(np.float32)


# ---------------------------------------------------------------------------
# Dataset
# ---------------------------------------------------------------------------

class OdometryDataset(Dataset):
    def __init__(
        self,
        npz_path:  str,
        augment:   bool  = True,
        aug_deg:   float = 30.0,
        aug_noise: float = 0.02,
    ):
        data = np.load(npz_path)
        self.windows   = data['windows'].astype(np.float32)   # (N, W, 7)
        self.deltas    = data['deltas'].astype(np.float32)    # (N, 2)
        self.augment   = augment
        self.aug_deg   = aug_deg
        self.aug_noise = aug_noise

        flat       = self.windows.reshape(-1, 7)
        self.mean  = flat.mean(axis=0).astype(np.float32)
        self.std   = (flat.std(axis=0) + 1e-8).astype(np.float32)

    def __len__(self) -> int:
        return len(self.windows)

    def __getitem__(self, idx: int):
        w = (self.windows[idx] - self.mean) / self.std
        d = self.deltas[idx].copy()
        if self.augment:
            w = _time_scale_augment(w)
            w, d = _rotate_augment(w, d, self.aug_deg)
            if self.aug_noise > 0.0:
                w = _noise_augment(w, self.aug_noise)
        return torch.from_numpy(w), torch.from_numpy(d)


# ---------------------------------------------------------------------------
# Training loop
# ---------------------------------------------------------------------------

def train(
    npz_path:     str,
    save_dir:     str   = 'assets',
    epochs:       int   = 150,
    batch_size:   int   = 32,
    lr:           float = 1e-3,
    val_split:    float = 0.15,
    arch:         str   = 'tcn',
    hidden:       int   = 128,
    channels:     int   = 32,
    dropout:      float = 0.4,
    weight_decay: float = 1e-4,
    aug_deg:      float = 30.0,
    aug_noise:    float = 0.02,
    patience:     int   = 25,
    device:       str   = 'auto',
    uncertainty:  bool  = False,
    save_model:   bool  = True,
    verbose:      bool  = True,
    trial=None,
) -> float:
    """Train OdometryNet and return best validation RMSE (cm)."""

    # ── Device ──────────────────────────────────────────────────────────────
    if device == 'cuda' and not torch.cuda.is_available():
        print('[WARN] --device cuda requested but CUDA is not available — falling back to cpu.')
        device = 'cpu'
    elif device == 'auto':
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
    dev = torch.device(device)
    if verbose:
        tag = f'cuda ({torch.cuda.get_device_name(0)})' if device == 'cuda' else 'cpu'
        print(f'Device : {tag}')

    save_dir = save_dir.rstrip('/\\')
    os.makedirs(save_dir, exist_ok=True)

    # ── Data ────────────────────────────────────────────────────────────────
    dataset = OdometryDataset(npz_path, augment=True,
                              aug_deg=aug_deg, aug_noise=aug_noise)
    if save_model:
        np.save(os.path.join(save_dir, 'imu_mean.npy'), dataset.mean)
        np.save(os.path.join(save_dir, 'imu_std.npy'),  dataset.std)
    if verbose:
        size_param = f'hidden={hidden}' if arch == 'lstm' else f'channels={channels}'
        print(f'Dataset: {len(dataset)} samples  |  arch={arch}  {size_param}'
              f'  dropout={dropout}  wd={weight_decay:.0e}'
              f'  aug_deg=±{aug_deg:.0f}°  aug_noise={aug_noise}')

    n_val   = max(1, int(len(dataset) * val_split))
    n_train = len(dataset) - n_val
    train_ds, val_ds = random_split(
        dataset, [n_train, n_val],
        generator=torch.Generator().manual_seed(42),
    )
    train_dl = DataLoader(train_ds, batch_size=batch_size,
                          shuffle=True,  num_workers=0, drop_last=True)
    val_dl   = DataLoader(val_ds,   batch_size=batch_size,
                          shuffle=False, num_workers=0)

    # ── Model ───────────────────────────────────────────────────────────────
    model   = build_model(arch=arch, hidden=hidden, channels=channels,
                          dropout=dropout, uncertainty=uncertainty).to(dev)
    opt     = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=weight_decay)
    sched   = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs, eta_min=lr * 0.01)
    # NLL loss when uncertainty head is enabled, MSE otherwise.
    # NLL trains the model to also predict its own confidence per axis.
    loss_fn = nll_loss if uncertainty else nn.MSELoss()

    best_val    = float('inf')
    no_improve  = 0
    model_path  = os.path.join(save_dir, 'odometry_net.pt')

    # ── Loop ────────────────────────────────────────────────────────────────
    for epoch in range(1, epochs + 1):
        model.train()
        train_loss = 0.0
        train_mse  = 0.0
        for x, y in train_dl:
            x, y = x.to(dev), y.to(dev)
            opt.zero_grad()
            pred = model(x)
            loss = loss_fn(pred, y)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            opt.step()
            train_loss += loss.item() * len(x)
            # Track pure MSE for RMSE reporting regardless of loss type
            train_mse  += nn.functional.mse_loss(pred[:, :2], y).item() * len(x)
        train_loss /= n_train
        train_mse  /= n_train
        sched.step()

        model.eval()
        val_loss = 0.0
        val_mse  = 0.0
        with torch.no_grad():
            for x, y in val_dl:
                x, y = x.to(dev), y.to(dev)
                pred = model(x)
                val_loss += loss_fn(pred, y).item() * len(x)
                val_mse  += nn.functional.mse_loss(pred[:, :2], y).item() * len(x)
        val_loss /= n_val
        val_mse  /= n_val

        # Optuna pruning — prune unpromising trials early
        if trial is not None:
            trial.report(val_loss, epoch)
            if trial.should_prune():
                import optuna
                raise optuna.exceptions.TrialPruned()

        if verbose and (epoch % 10 == 0 or epoch == 1):
            loss_tag = f'nll={val_loss:.4f}  ' if uncertainty else ''
            print(
                f'Epoch {epoch:4d}/{epochs}'
                f'  train={_rmse_cm(train_mse):.1f}cm'
                f'  val={_rmse_cm(val_mse):.1f}cm'
                f'  {loss_tag}'
                f'lr={sched.get_last_lr()[0]:.2e}'
                f'  {"v" if val_mse < best_val else " "}'
            )

        if val_mse < best_val:
            best_val   = val_mse
            no_improve = 0
            if save_model:
                torch.save(model.state_dict(), model_path)
        else:
            no_improve += 1
            if no_improve >= patience:
                if verbose:
                    print(f'\n  Early stop — no val improvement for {patience} epochs.')
                break

    if save_model:
        meta = {'arch': arch, 'hidden': hidden, 'channels': channels,
                'dropout': dropout, 'uncertainty': uncertainty}
        with open(os.path.join(save_dir, 'arch_config.json'), 'w') as f:
            json.dump(meta, f, indent=2)

    if verbose:
        print(f'\nBest val RMSE = {_rmse_cm(best_val):.1f} cm')
        if save_model:
            print(f'Saved → {model_path}')

    return _rmse_cm(best_val)


# ---------------------------------------------------------------------------
# Optuna search
# ---------------------------------------------------------------------------

def optuna_search(
    npz_path:      str,
    save_dir:      str,
    n_trials:      int = 50,
    device:        str = 'auto',
    study_name:    str = 'odometry_search',
    study_db:      str | None = None,
    optuna_archs:  list[str] | None = None,
) -> dict:
    try:
        import optuna
    except ImportError:
        raise SystemExit('[ERROR] Optuna not installed.  Run: pip install optuna')

    # Resolve and confirm device before any trials run
    if device == 'cuda' and not torch.cuda.is_available():
        print('[WARN] --device cuda requested but CUDA unavailable — falling back to cpu.')
        device = 'cpu'
    elif device == 'auto':
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
    tag = f'cuda ({torch.cuda.get_device_name(0)})' if device == 'cuda' else 'cpu'

    # Default DB lives alongside the saved assets
    if study_db is None:
        os.makedirs(save_dir, exist_ok=True)
        study_db = os.path.join(save_dir, 'optuna_study.db')
    storage = f'sqlite:///{os.path.abspath(study_db)}'

    archs = optuna_archs or ['lstm', 'tcn', 'cnn_pool']
    print(f'Device : {tag}')
    print(f'Optuna : {n_trials} trials  |  arch search: {" / ".join(archs)}')
    print(f'Study  : {study_name}  →  {study_db}')

    optuna.logging.set_verbosity(optuna.logging.WARNING)

    pruner = optuna.pruners.MedianPruner(n_startup_trials=10, n_warmup_steps=20)
    study  = optuna.create_study(
        study_name     = study_name,
        direction      = 'minimize',
        pruner         = pruner,
        storage        = storage,
        load_if_exists = True,   # resume from prior runs; skip completed trials
    )

    done = [t for t in study.trials if t.state.is_finished()]
    remaining = max(0, n_trials - len(done))
    if done:
        print(f'Resuming: {len(done)} trial(s) already complete, {remaining} left to run.')
        if study.best_trial in done:
            print(f'  Current best: trial #{study.best_trial.number}'
                  f'  RMSE={study.best_value:.1f} cm'
                  f'  params={study.best_params}')
    print()

    if remaining == 0:
        print('[INFO] All requested trials already complete — nothing to run.')
        print('       Increase --trials to add more, or use --show-trials to inspect history.')
    else:
        def objective(trial):
            arch = trial.suggest_categorical('arch', archs)
            # Architecture-specific size param
            hidden   = trial.suggest_categorical('hidden',   [64, 128]) if arch == 'lstm' else 128
            channels = trial.suggest_categorical('channels', [32, 64])  if arch != 'lstm' else 32
            params = dict(
                arch         = arch,
                hidden       = hidden,
                channels     = channels,
                dropout      = trial.suggest_float('dropout',          0.1,  0.5),
                lr           = trial.suggest_float('lr',               1e-4, 1e-2, log=True),
                weight_decay = trial.suggest_float('weight_decay',     1e-6, 1e-3, log=True),
                aug_deg      = trial.suggest_float('aug_deg',          15.0, 45.0),
                aug_noise    = trial.suggest_float('aug_noise',        0.0,  0.08),
                batch_size   = trial.suggest_categorical('batch_size', [16, 32, 64]),
            )
            return train(
                npz_path   = npz_path,
                save_dir   = save_dir,
                epochs     = 80,        # shorter runs during search
                val_split  = 0.20,      # larger val set for reliable signal
                patience   = 15,
                device     = device,
                save_model = False,     # don't overwrite real checkpoint during search
                verbose    = False,
                trial      = trial,
                **params,
            )

        study.optimize(objective, n_trials=remaining, show_progress_bar=True)

    print('\n' + '=' * 62)
    print(f'  Optuna complete — best val RMSE: {study.best_value:.1f} cm')
    print(f'  Best hyperparameters:')
    for k, v in study.best_params.items():
        print(f'    {k:<15} = {v}')
    print('=' * 62)

    # Retrain with best params and save
    print('\nRetraining with best params...')
    train(
        npz_path   = npz_path,
        save_dir   = save_dir,
        epochs     = 150,
        val_split  = 0.15,
        patience   = 25,
        device     = device,
        save_model = True,
        verbose    = True,
        **study.best_params,
    )
    return study.best_params


# ---------------------------------------------------------------------------
# Denoiser dataset + training
# ---------------------------------------------------------------------------

class DenoisingDataset(Dataset):
    """Pairs of (noisy z-scored window, clean z-scored window).

    Corruption is applied on-the-fly so each epoch sees different noise draws.
    """

    def __init__(self, npz_path: str, noise_std: float = 0.15):
        data        = np.load(npz_path)
        windows     = data['windows'].astype(np.float32)   # (N, W, 7)
        flat        = windows.reshape(-1, 7)
        mean        = flat.mean(axis=0).astype(np.float32)
        std         = (flat.std(axis=0) + 1e-8).astype(np.float32)
        self.clean     = ((windows - mean) / std).astype(np.float32)
        self.noise_std = noise_std

    def __len__(self) -> int:
        return len(self.clean)

    def __getitem__(self, idx: int):
        clean = self.clean[idx]
        noisy = (clean + np.random.normal(0.0, self.noise_std, clean.shape)
                 ).astype(np.float32)
        return torch.from_numpy(noisy), torch.from_numpy(clean)


def train_denoiser(
    npz_path:   str,
    save_dir:   str   = 'assets',
    epochs:     int   = 100,
    batch_size: int   = 64,
    lr:         float = 1e-3,
    noise_std:  float = 0.15,
    channels:   int   = 32,
    val_split:  float = 0.15,
    patience:   int   = 20,
    device:     str   = 'auto',
    save_model: bool  = True,
    verbose:    bool  = True,
    trial=None,
) -> float:
    """Train IMUDenoiser and save denoiser.pt + denoiser_config.json to save_dir.

    Returns best validation reconstruction MSE (normalised units²).
    """
    if device == 'auto':
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
    dev = torch.device(device)

    save_dir = save_dir.rstrip('/\\')
    os.makedirs(save_dir, exist_ok=True)

    dataset = DenoisingDataset(npz_path, noise_std=noise_std)
    n_val   = max(1, int(len(dataset) * val_split))
    n_train = len(dataset) - n_val
    train_ds, val_ds = random_split(
        dataset, [n_train, n_val],
        generator=torch.Generator().manual_seed(42),
    )
    train_dl = DataLoader(train_ds, batch_size=batch_size, shuffle=True,  num_workers=0, drop_last=True)
    val_dl   = DataLoader(val_ds,   batch_size=batch_size, shuffle=False, num_workers=0)

    model   = build_denoiser(channels=channels).to(dev)
    opt     = torch.optim.Adam(model.parameters(), lr=lr)
    sched   = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs, eta_min=lr * 0.01)
    loss_fn = nn.MSELoss()

    best_val   = float('inf')
    no_improve = 0
    out_path   = os.path.join(save_dir, 'denoiser.pt')

    if verbose:
        print(f'[Denoiser] device={device}  samples={len(dataset)}'
              f'  channels={channels}  noise_std={noise_std}')

    for epoch in range(1, epochs + 1):
        model.train()
        for noisy, clean in train_dl:
            noisy, clean = noisy.to(dev), clean.to(dev)
            opt.zero_grad()
            loss_fn(model(noisy), clean).backward()
            opt.step()
        sched.step()

        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for noisy, clean in val_dl:
                noisy, clean = noisy.to(dev), clean.to(dev)
                val_loss += loss_fn(model(noisy), clean).item() * len(noisy)
        val_loss /= n_val

        if trial is not None:
            trial.report(val_loss, epoch)
            if trial.should_prune():
                import optuna
                raise optuna.exceptions.TrialPruned()

        if verbose and (epoch % 10 == 0 or epoch == 1):
            print(f'  Epoch {epoch:4d}/{epochs}  val_mse={val_loss:.5f}'
                  f'  {"▼" if val_loss < best_val else " "}')

        if val_loss < best_val:
            best_val   = val_loss
            no_improve = 0
            if save_model:
                torch.save(model.state_dict(), out_path)
        else:
            no_improve += 1
            if no_improve >= patience:
                if verbose:
                    print(f'  Early stop at epoch {epoch}.')
                break

    if save_model:
        with open(os.path.join(save_dir, 'denoiser_config.json'), 'w') as f:
            json.dump({'channels': channels, 'noise_std': noise_std}, f, indent=2)
        if verbose:
            print(f'[Denoiser] Best val MSE = {best_val:.5f}  →  {out_path}')

    return best_val


def optuna_denoiser_search(
    npz_path:   str,
    save_dir:   str,
    n_trials:   int = 30,
    device:     str = 'auto',
    study_name: str = 'denoiser_search',
    study_db:   str | None = None,
) -> dict:
    """Optuna hyperparameter search for IMUDenoiser.

    Tunes: channels, noise_std, lr, batch_size.
    Saves best denoiser.pt + denoiser_config.json after search completes.
    Uses the same SQLite DB as the odometry search (different study name).
    """
    try:
        import optuna
    except ImportError:
        raise SystemExit('[ERROR] Optuna not installed.  Run: pip install optuna')

    if device == 'auto':
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
    elif device == 'cuda' and not torch.cuda.is_available():
        print('[WARN] --device cuda requested but CUDA unavailable — falling back to cpu.')
        device = 'cpu'
    tag = f'cuda ({torch.cuda.get_device_name(0)})' if device == 'cuda' else 'cpu'

    if study_db is None:
        os.makedirs(save_dir, exist_ok=True)
        study_db = os.path.join(save_dir, 'optuna_study.db')
    storage = f'sqlite:///{os.path.abspath(study_db)}'

    print(f'Device : {tag}')
    print(f'Optuna : {n_trials} trials  |  denoiser search')
    print(f'Study  : {study_name}  →  {study_db}')

    optuna.logging.set_verbosity(optuna.logging.WARNING)

    pruner = optuna.pruners.MedianPruner(n_startup_trials=5, n_warmup_steps=15)
    study  = optuna.create_study(
        study_name     = study_name,
        direction      = 'minimize',
        pruner         = pruner,
        storage        = storage,
        load_if_exists = True,
    )

    done      = [t for t in study.trials if t.state.is_finished()]
    remaining = max(0, n_trials - len(done))
    if done:
        print(f'Resuming: {len(done)} trial(s) already complete, {remaining} left to run.')
        if study.best_trial in done:
            print(f'  Current best: trial #{study.best_trial.number}'
                  f'  MSE={study.best_value:.5f}'
                  f'  params={study.best_params}')
    print()

    if remaining == 0:
        print('[INFO] All requested trials complete. Increase --trials to add more.')
    else:
        def objective(trial):
            params = dict(
                channels  = trial.suggest_categorical('channels',  [16, 32, 64]),
                noise_std = trial.suggest_float('noise_std', 0.05, 0.30),
                lr        = trial.suggest_float('lr',        1e-4, 1e-2, log=True),
                batch_size= trial.suggest_categorical('batch_size', [32, 64, 128]),
            )
            return train_denoiser(
                npz_path   = npz_path,
                save_dir   = save_dir,
                epochs     = 60,
                val_split  = 0.20,
                patience   = 12,
                device     = device,
                save_model = False,
                verbose    = False,
                trial      = trial,
                **params,
            )

        study.optimize(objective, n_trials=remaining, show_progress_bar=True)

    print('\n' + '=' * 62)
    print(f'  Optuna complete — best val MSE: {study.best_value:.5f}')
    print(f'  Best hyperparameters:')
    for k, v in study.best_params.items():
        print(f'    {k:<15} = {v}')
    print('=' * 62)

    print('\nRetraining with best params...')
    train_denoiser(
        npz_path   = npz_path,
        save_dir   = save_dir,
        epochs     = 100,
        val_split  = 0.15,
        patience   = 20,
        device     = device,
        save_model = True,
        verbose    = True,
        **study.best_params,
    )
    return study.best_params


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _rmse_cm(mse_m2: float) -> float:
    return float(np.sqrt(mse_m2) * 100.0)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    p = argparse.ArgumentParser(
        description='Train OdometryNet on collected whiteboard data.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('npz',                          help='Path to training_data.npz')
    p.add_argument('--save-dir',   default='assets')
    p.add_argument('--epochs',     type=int,   default=150)
    p.add_argument('--batch-size', type=int,   default=32)
    p.add_argument('--lr',         type=float, default=1e-3)
    p.add_argument('--val-split',  type=float, default=0.15)
    p.add_argument('--arch',       default='tcn', choices=['lstm', 'tcn', 'cnn_pool'],
                                               help='Model architecture (default: tcn)')
    p.add_argument('--hidden',     type=int,   default=128,  help='LSTM hidden size — only used when --arch lstm')
    p.add_argument('--channels',   type=int,   default=32,   help='TCN/CNNPool channel width (default 32)')
    p.add_argument('--dropout',    type=float, default=0.4,  help='Dropout before FC head (default 0.4)')
    p.add_argument('--weight-decay', type=float, default=1e-4)
    p.add_argument('--aug-deg',    type=float, default=30.0, help='Rotation aug range ±deg (default 30)')
    p.add_argument('--aug-noise',  type=float, default=0.02, help='Accel noise std in norm space (default 0.02)')
    p.add_argument('--patience',   type=int,   default=25,   help='Early stopping patience (default 25)')
    p.add_argument('--device',      default='auto', choices=['auto', 'cuda', 'cpu'])
    p.add_argument('--uncertainty', action='store_true',
                                    help='Enable uncertainty head — model predicts (Dx,Dy,sx,sy), uses NLL loss')
    p.add_argument('--optuna',      action='store_true', help='Run Optuna hyperparameter search')
    p.add_argument('--trials',        type=int,   default=50,   help='Optuna trial count (default 50)')
    p.add_argument('--study-name',    default='odometry_search', help='Optuna study name (default: odometry_search)')
    p.add_argument('--study-db',      default=None, help='SQLite path for Optuna storage (default: <save-dir>/optuna_study.db)')
    p.add_argument('--show-trials',   action='store_true', help='Print all completed Optuna trials and exit')
    p.add_argument('--optuna-archs',  nargs='+', default=None,
                   choices=['lstm', 'tcn', 'cnn_pool'],
                   help='Restrict Optuna arch search to these (default: all three)')
    p.add_argument('--denoiser',      action='store_true', help='Train the IMUDenoiser instead of odometry model')
    p.add_argument('--denoiser-noise-std', type=float, default=0.15,
                                      help='Corruption noise std in normalised space (default 0.15)')
    p.add_argument('--denoiser-channels', type=int,   default=32,
                                      help='Denoiser conv channel width (default 32)')
    args = p.parse_args()

    if args.show_trials:
        try:
            import optuna
        except ImportError:
            raise SystemExit('[ERROR] Optuna not installed.  Run: pip install optuna')
        import os as _os
        db = args.study_db or _os.path.join(args.save_dir, 'optuna_study.db')
        storage = f'sqlite:///{_os.path.abspath(db)}'
        study = optuna.load_study(study_name=args.study_name, storage=storage)
        trials = [t for t in study.trials if t.state.is_finished()]
        print(f'Study "{args.study_name}" — {len(trials)} completed trial(s):\n')
        print(f'  {"#":>4}  {"RMSE(cm)":>10}  {"state":<9}  params')
        print(f'  {"─"*4}  {"─"*10}  {"─"*9}  {"─"*40}')
        for t in sorted(trials, key=lambda x: x.value or float('inf')):
            val = f'{t.value:.3f}' if t.value is not None else 'pruned'
            print(f'  {t.number:>4}  {val:>10}  {t.state.name:<9}  {t.params}')
        if study.best_trial:
            print(f'\nBest: trial #{study.best_trial.number}  RMSE={study.best_value:.1f} cm')
        raise SystemExit(0)

    if args.denoiser and args.optuna:
        optuna_denoiser_search(
            npz_path   = args.npz,
            save_dir   = args.save_dir,
            n_trials   = args.trials,
            device     = args.device,
            study_name = args.study_name if args.study_name != 'odometry_search' else 'denoiser_search',
            study_db   = args.study_db,
        )
    elif args.denoiser:
        train_denoiser(
            npz_path  = args.npz,
            save_dir  = args.save_dir,
            epochs    = args.epochs,
            batch_size= args.batch_size,
            lr        = args.lr,
            noise_std = args.denoiser_noise_std,
            channels  = args.denoiser_channels,
            val_split = args.val_split,
            patience  = args.patience,
            device    = args.device,
        )
    elif args.optuna:
        optuna_search(
            npz_path      = args.npz,
            save_dir      = args.save_dir,
            n_trials      = args.trials,
            device        = args.device,
            study_name    = args.study_name,
            study_db      = args.study_db,
            optuna_archs  = args.optuna_archs,
        )
    else:
        train(
            npz_path     = args.npz,
            save_dir     = args.save_dir,
            epochs       = args.epochs,
            batch_size   = args.batch_size,
            lr           = args.lr,
            val_split    = args.val_split,
            arch         = args.arch,
            hidden       = args.hidden,
            channels     = args.channels,
            dropout      = args.dropout,
            weight_decay = args.weight_decay,
            aug_deg      = args.aug_deg,
            aug_noise    = args.aug_noise,
            patience     = args.patience,
            device       = args.device,
        )
