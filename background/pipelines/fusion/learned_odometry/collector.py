from __future__ import annotations
import numpy as np
from scipy.signal import savgol_filter
from .buffer import IMUOdometryBuffer


class OdometryDataCollector:
    """Pairs IMU windows with UWB displacement labels for offline training.

    Attach to the pipeline during recording sessions.  Feed every IMU packet
    and every accepted UWB fix; the collector gates samples on is_drawing and
    aligns windows with UWB deltas automatically.

    Label strategies (call one before save()):
      - smooth_uwb_labels()  — Savitzky-Golay smoothed UWB trajectory (recommended
                               for freehand stroke data; low effort, good quality).
      - use_known_points()   — ruler-measured start/end pairs (cleanest labels;
                               use for a calibration dataset).

    After recording, call save() to write a .npz file for train.py.
    """

    def __init__(self, window_size: int = 200):
        # stride == window_size so every window is non-overlapping — each sample
        # covers exactly one UWB-delta interval with no repeated frames.
        self.buf = IMUOdometryBuffer(window_size=window_size, stride=window_size)
        self._last_uwb_pos: np.ndarray | None = None
        self._pending_window: np.ndarray | None = None   # last emitted but un-paired window
        self._dataset: list[tuple[np.ndarray, np.ndarray]] = []
        # Raw UWB trajectory for post-hoc smoothing (position per fix).
        self._uwb_trajectory: list[np.ndarray] = []

    # ------------------------------------------------------------------
    # Real-time feed methods
    # ------------------------------------------------------------------

    def on_imu_packet(
        self,
        imu_frames: list[np.ndarray],
        timestamps: list[int] | None = None,
    ) -> None:
        """Call with each batch of 3 pre-processed IMU frames from imu.py."""
        window = self.buf.push_packet(imu_frames, timestamps)
        if window is not None:
            self._pending_window = window

    def on_uwb_fix(self, uwb_pos: np.ndarray, is_drawing: bool) -> None:
        """Call with each accepted UWB position fix (world-frame metres).

        Pairs the most recently completed IMU window with the UWB delta
        if the pen is drawing.  Accumulates the raw UWB trajectory for
        post-hoc smoothing via smooth_uwb_labels().
        """
        uwb_pos = np.asarray(uwb_pos, dtype=np.float32)
        self._uwb_trajectory.append(uwb_pos.copy())

        if self._last_uwb_pos is None:
            self._last_uwb_pos = uwb_pos
            return

        delta = uwb_pos - self._last_uwb_pos
        self._last_uwb_pos = uwb_pos

        if self._pending_window is not None and is_drawing:
            self._dataset.append((self._pending_window.copy(), delta.copy()))
            self._pending_window = None

    def reset_stroke(self) -> None:
        """Call on pen-up to clear the IMU buffer and pending state."""
        self.buf.reset()
        self._pending_window = None
        self._last_uwb_pos   = None

    # ------------------------------------------------------------------
    # Post-hoc label improvement
    # ------------------------------------------------------------------

    def smooth_uwb_labels(
        self,
        window_length: int = 21,
        polyorder: int = 3,
    ) -> None:
        """Replace raw UWB delta labels with Savitzky-Golay smoothed deltas.

        Preserves stroke shape (low-frequency) while removing UWB jitter
        (high-frequency).  Requires at least window_length UWB fixes.
        Call before save().
        """
        if len(self._uwb_trajectory) < window_length:
            return
        traj = np.stack(self._uwb_trajectory, axis=0)   # (T, 2)
        smooth_x = savgol_filter(traj[:, 0], window_length, polyorder)
        smooth_y = savgol_filter(traj[:, 1], window_length, polyorder)
        smooth   = np.stack([smooth_x, smooth_y], axis=1).astype(np.float32)
        # Recompute deltas from the smoothed trajectory and re-pair with windows.
        # This is a best-effort re-pairing by index; for calibration data prefer
        # use_known_points() which gives exact labels.
        smooth_deltas = np.diff(smooth, axis=0)
        n = min(len(self._dataset), len(smooth_deltas))
        for i in range(n):
            win, _ = self._dataset[i]
            self._dataset[i] = (win, smooth_deltas[i].copy())

    def use_known_points(
        self,
        start: tuple[float, float],
        end:   tuple[float, float],
    ) -> None:
        """Override all labels with a single ruler-measured displacement.

        Use when the entire recording session is one stroke from a fixed
        start to a fixed end (tape-marker calibration).  The same (Δx,Δy)
        is assigned to every sample in the dataset.
        """
        delta = np.array(
            [end[0] - start[0], end[1] - start[1]], dtype=np.float32
        )
        self._dataset = [(w, delta.copy()) for w, _ in self._dataset]

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def save(self, path: str = 'training_data.npz') -> int:
        """Write (windows, deltas) to a .npz file.  Returns number of samples saved."""
        if not self._dataset:
            return 0
        windows = np.array([d[0] for d in self._dataset], dtype=np.float32)
        deltas  = np.array([d[1] for d in self._dataset], dtype=np.float32)
        np.savez(path, windows=windows, deltas=deltas)
        return len(self._dataset)

    def __len__(self) -> int:
        return len(self._dataset)
