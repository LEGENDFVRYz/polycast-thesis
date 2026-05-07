from __future__ import annotations
import numpy as np
import torch
from .model import OdometryNet


class OdometryInferer:
    """Wraps OdometryNet for single-window inference.

    Handles:
      - Loading model weights and per-channel normalization stats.
      - Z-score normalisation using stats computed from the training set.
      - Quaternion component order fix: imu.py emits (qx,qy,qz,qw) but
        the training frame format is [ax,ay,az, qx,qy,qz,qw] — the order
        is already correct as long as frames are assembled at push_packet
        time by concatenating acc_world[:3] + quat[:4] in that order.

    Usage:
        inferer = OdometryInferer.from_assets('assets/')
        delta_xy = inferer.predict(window)   # window: (200, 7)
    """

    def __init__(
        self,
        model_path: str,
        mean_path:  str,
        std_path:   str,
        device:     str = 'cpu',
    ):
        self._device = torch.device(device)

        self._mean = np.load(mean_path).astype(np.float32)   # (7,)
        self._std  = np.load(std_path).astype(np.float32)    # (7,)

        model = OdometryNet()
        model.load_state_dict(
            torch.load(model_path, map_location=self._device, weights_only=True)
        )
        model.eval()
        self._model = model.to(self._device)

    @classmethod
    def from_assets(cls, assets_dir: str, device: str = 'cpu') -> 'OdometryInferer':
        """Convenience constructor — load all three files from a single directory."""
        import os
        return cls(
            model_path=os.path.join(assets_dir, 'odometry_net.pt'),
            mean_path =os.path.join(assets_dir, 'imu_mean.npy'),
            std_path  =os.path.join(assets_dir, 'imu_std.npy'),
            device=device,
        )

    @torch.no_grad()
    def predict(self, window: np.ndarray) -> np.ndarray:
        """Predict displacement for one IMU window.

        window  — (window_size, 7) float32 array from IMUOdometryBuffer.
        Returns — (2,) float32 array [Δx, Δy] in metres, world frame.
        """
        x = (window - self._mean) / (self._std + 1e-8)
        t = torch.from_numpy(x).unsqueeze(0).to(self._device)   # (1, window_size, 7)
        out = self._model(t)                                      # (1, 2)
        return out.squeeze(0).cpu().numpy().astype(np.float32)   # (2,)
