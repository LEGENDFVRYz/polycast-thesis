"""
OdometryInferer — loads trained weights and runs inference.

If the model assets are absent (pre-training), predict() returns None and
emits a one-time console warning. Callers must handle None gracefully.
"""

from pathlib import Path

import numpy as np


_ASSETS = Path(__file__).parent / "assets"


class OdometryInferer:
    def __init__(self):
        self._ready = False
        self._warned = False
        self.mean: "np.ndarray | None" = None
        self.std:  "np.ndarray | None" = None
        self._model = None

        model_path = _ASSETS / "odometry_net.pt"
        mean_path  = _ASSETS / "imu_mean.npy"
        std_path   = _ASSETS / "imu_std.npy"

        if not (model_path.exists() and mean_path.exists() and std_path.exists()):
            return  # assets not ready; predict() will return None

        try:
            import torch
            from .model import OdometryNet

            self.mean  = np.load(mean_path)
            self.std   = np.load(std_path)
            net = OdometryNet()
            net.load_state_dict(torch.load(model_path, map_location="cpu"))
            net.eval()
            self._model = net
            self._ready = True
            print("[OdometryInferer] Model loaded from", model_path)
        except Exception as exc:
            print(f"[OdometryInferer] Failed to load model: {exc}")

    def predict(self, window: np.ndarray) -> "np.ndarray | None":
        """
        window: (window_size, 6) float32 ndarray
        Returns (2,) array [Δx, Δy] in meters, or None if model not ready.
        """
        if not self._ready:
            if not self._warned:
                print("[OdometryInferer] No trained model found in assets/ — "
                      "green odometry curve disabled until model is dropped in.")
                self._warned = True
            return None

        import torch

        normed = (window - self.mean) / self.std
        x = torch.tensor(normed, dtype=torch.float32).unsqueeze(0)  # (1, N, 6)
        with torch.no_grad():
            out = self._model(x).squeeze().numpy()  # (2,)
        return out
