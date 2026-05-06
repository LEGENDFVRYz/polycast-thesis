"""
RangeDenoiserInferer — loads trained per-anchor CNN weights and runs inference.

One inferer instance per anchor. If assets are absent, predict() returns the
raw input value unchanged (identity fallback) and prints a one-time warning.
"""

from pathlib import Path
import numpy as np

_ASSETS = Path(__file__).parent / "assets"


class RangeDenoiserInferer:
    def __init__(self, anchor_idx: int, window_size: int = 20):
        self._anchor  = anchor_idx
        self._ready   = False
        self._warned  = False
        self._mean    = None
        self._std     = None
        self._model   = None
        self._ws      = window_size

        model_path = _ASSETS / f"range_denoiser_a{anchor_idx}.pt"
        mean_path  = _ASSETS / f"range_mean_a{anchor_idx}.npy"
        std_path   = _ASSETS / f"range_std_a{anchor_idx}.npy"

        if not (model_path.exists() and mean_path.exists() and std_path.exists()):
            return

        try:
            import torch
            from .model import RangeDenoiserNet

            self._mean  = float(np.load(mean_path))
            self._std   = float(np.load(std_path))
            net = RangeDenoiserNet(window_size=window_size)
            net.load_state_dict(torch.load(model_path, map_location="cpu"))
            net.eval()
            self._model = net
            self._ready = True
            print(f"[RangeDenoiser] A{anchor_idx} model loaded.")
        except Exception as exc:
            print(f"[RangeDenoiser] A{anchor_idx} failed to load: {exc}")

    def predict(self, window: np.ndarray) -> float:
        """
        window: (window_size, 1) float32 — raw range samples
        Returns denoised range (metres). Falls back to last sample if no model.
        """
        if not self._ready:
            if not self._warned:
                print(f"[RangeDenoiser] A{self._anchor}: no model in assets/ — "
                      "using Kalman fallback. Run train.py to activate.")
                self._warned = True
            return float(window[-1, 0])   # identity: return latest raw sample

        import torch
        normed = (window - self._mean) / (self._std + 1e-8)
        x = torch.tensor(normed, dtype=torch.float32).unsqueeze(0)  # (1, W, 1)
        with torch.no_grad():
            out = self._model(x).squeeze().item()
        # Denormalize
        return out * (self._std + 1e-8) + self._mean
