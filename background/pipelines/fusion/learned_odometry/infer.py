from __future__ import annotations
import json
import os
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")  # prevent libiomp5md.dll double-load on Windows

import numpy as np
import torch
from .model import build_model, build_denoiser


class OdometryInferer:
    """Wraps any OdometryNet architecture for single-window inference.

    Reads arch_config.json from the assets directory to instantiate the
    correct model class (lstm / tcn / cnn_pool) automatically.

    Usage:
        inferer = OdometryInferer.from_assets('assets/')
        delta_xy = inferer.predict(window)   # window: (window_size, 7)
    """

    def __init__(
        self,
        model_path:    str,
        mean_path:     str,
        std_path:      str,
        config_path:   str,
        denoiser_path: str | None = None,
        denoiser_config_path: str | None = None,
        device:        str = 'cpu',
    ):
        self._device = torch.device(device)

        self._mean = np.load(mean_path).astype(np.float32)   # (7,)
        self._std  = np.load(std_path).astype(np.float32)    # (7,)

        # Load arch config — falls back to lstm defaults if file is missing
        # (backward compatibility with checkpoints trained before arch support)
        if os.path.exists(config_path):
            with open(config_path) as f:
                cfg = json.load(f)
        else:
            cfg = {'arch': 'lstm', 'hidden': 128, 'channels': 32, 'dropout': 0.0}

        model = build_model(
            arch     = cfg.get('arch',     'lstm'),
            hidden   = cfg.get('hidden',   128),
            channels = cfg.get('channels', 32),
            dropout  = 0.0,   # disabled at inference time
        )
        model.load_state_dict(
            torch.load(model_path, map_location=self._device, weights_only=True)
        )
        model.eval()
        self._model = model.to(self._device)

        # Optional denoiser — loaded only when denoiser.pt is present
        self._denoiser = None
        if denoiser_path and os.path.exists(denoiser_path):
            d_cfg = {}
            if denoiser_config_path and os.path.exists(denoiser_config_path):
                with open(denoiser_config_path) as f:
                    d_cfg = json.load(f)
            denoiser = build_denoiser(channels=d_cfg.get('channels', 32))
            denoiser.load_state_dict(
                torch.load(denoiser_path, map_location=self._device, weights_only=True)
            )
            denoiser.eval()
            self._denoiser = denoiser.to(self._device)

    @classmethod
    def from_assets(cls, assets_dir: str, device: str = 'cpu') -> 'OdometryInferer':
        """Load all assets from a directory.

        Automatically loads denoiser.pt + denoiser_config.json if present.
        """
        return cls(
            model_path           = os.path.join(assets_dir, 'odometry_net.pt'),
            mean_path            = os.path.join(assets_dir, 'imu_mean.npy'),
            std_path             = os.path.join(assets_dir, 'imu_std.npy'),
            config_path          = os.path.join(assets_dir, 'arch_config.json'),
            denoiser_path        = os.path.join(assets_dir, 'denoiser.pt'),
            denoiser_config_path = os.path.join(assets_dir, 'denoiser_config.json'),
            device               = device,
        )

    @torch.no_grad()
    def predict(self, window: np.ndarray) -> np.ndarray:
        """Predict displacement for one IMU window.

        window  — (window_size, 7) float32 array from IMUOdometryBuffer.
        Returns — (2,) float32 array [Δx, Δy] in metres, world frame.
        """
        x = (window - self._mean) / (self._std + 1e-8)
        t = torch.from_numpy(x).unsqueeze(0).to(self._device)   # (1, window_size, 7)
        if self._denoiser is not None:
            t = self._denoiser(t)                                # (1, window_size, 7)
        out = self._model(t)                                     # (1, 2)
        return out.squeeze(0).cpu().numpy().astype(np.float32)   # (2,)
