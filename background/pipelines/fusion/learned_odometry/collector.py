"""
OdometryDataCollector — synchronized (imu_window, delta_uwb) pair logger.

Attach alongside the normal pipeline during a drawing session (--collect mode).
Pairs are only recorded when:
  - stroke_active is True  (pen confirmed drawing, not hovering)
  - a UWB fix arrives to provide a ground-truth displacement label
  - the IMU buffer has a full window ready

Call order per packet (mirrors visualizer._poll_pipeline):
    1. collector.on_imu_frame(imu_frame, stroke_active)  → after each IMU event
    2. collector.on_uwb_fix(uwb_pos)                     → after each UWB position event
    3. collector.save(path)                              → on shutdown (Ctrl+C / window close)

Output: .npz file with keys:
    windows  — (N, window_size, 6)  float32
    deltas   — (N, 2)               float32  [Δx, Δy] in meters
"""

import time
from pathlib import Path

import numpy as np

from .buffer import IMUOdometryBuffer


class OdometryDataCollector:
    def __init__(self, window_size: int = 200, stride: int = 200):
        # For collection we use stride == window_size (non-overlapping windows)
        # so each window maps cleanly to one UWB delta. Overlapping windows
        # would require more careful UWB bookkeeping.
        self._buf = IMUOdometryBuffer(window_size=window_size, stride=stride)
        self._last_uwb: "np.ndarray | None" = None
        self._pending_window: "np.ndarray | None" = None  # waiting for UWB delta
        self._windows: list = []
        self._deltas:  list = []
        self._rejected = 0

    # ── per-IMU-frame call ────────────────────────────────────────────────────
    def on_imu_frame(self, imu_frame: np.ndarray, stroke_active: bool) -> None:
        """
        imu_frame: shape (6,) — [ax, ay, qw, qx, qy, qz]
        stroke_active: from fused['stroke_active']
        """
        if not stroke_active:
            # Don't accumulate idle / air-move frames as training data
            return

        window = self._buf.push_packet([imu_frame])
        if window is not None:
            # Store the window; it will be paired with the next UWB delta
            self._pending_window = window.copy()

    # ── per-UWB-fix call ─────────────────────────────────────────────────────
    def on_uwb_fix(self, uwb_pos: np.ndarray, stroke_active: bool) -> bool:
        """
        uwb_pos: shape (2,) — [x, y] in meters (from pos_clean)
        stroke_active: from the most recent fused event

        Returns True if a sample pair was recorded.
        """
        if self._last_uwb is None:
            self._last_uwb = uwb_pos.copy()
            return False

        delta = uwb_pos - self._last_uwb
        self._last_uwb = uwb_pos.copy()

        if self._pending_window is None or not stroke_active:
            return False

        # Sanity gate: reject implausibly large displacements (UWB outlier)
        delta_norm = float(np.linalg.norm(delta))
        if delta_norm > 0.30:  # > 30 cm between consecutive UWB fixes → skip
            self._rejected += 1
            self._pending_window = None
            return False

        self._windows.append(self._pending_window)
        self._deltas.append(delta.astype(np.float32))
        self._pending_window = None
        return True

    # ── reset on stroke end ───────────────────────────────────────────────────
    def on_stroke_end(self) -> None:
        self._buf.reset()
        self._pending_window = None

    # ── save ──────────────────────────────────────────────────────────────────
    def save(self, path: str = "odometry_training_data.npz") -> int:
        n = len(self._windows)
        if n == 0:
            print("[Collector] No samples recorded — nothing saved.")
            return 0

        out = Path(path).resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        windows = np.stack(self._windows, axis=0).astype(np.float32)  # (N, W, 6)
        deltas  = np.stack(self._deltas,  axis=0).astype(np.float32)  # (N, 2)
        np.savez(out, windows=windows, deltas=deltas)
        print(f"[Collector] Saved {n} samples  (rejected {self._rejected} outliers)")
        print(f"[Collector] → {out}")
        return n

    @property
    def sample_count(self) -> int:
        return len(self._windows)
