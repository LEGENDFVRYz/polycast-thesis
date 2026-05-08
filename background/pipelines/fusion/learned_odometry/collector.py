from __future__ import annotations
import numpy as np

FRAME_DIM = 7


class OdometryDataCollector:
    """Per-stroke data collector for known-point odometry training data.

    Design rationale
    ----------------
    The sliding IMUOdometryBuffer is designed for continuous inference — it
    emits windows every stride frames regardless of stroke boundaries.  For
    data collection with known start/end points (Option 2), that model fails:
    strokes typically end before the buffer fills, so reset_stroke() clears
    it before any window is emitted.

    This collector instead accumulates all frames for the current stroke in a
    plain list.  On reset_stroke() (pen-up) it takes the first window_size
    frames and saves them with a placeholder label of zeros.  After all
    recording sessions, call use_known_points() to replace every label with
    the ruler-measured displacement.

    If a stroke is shorter than window_size frames (pen lifted too early or
    drawn too slowly), it is skipped and the caller is notified via the
    return value of reset_stroke().
    """

    def __init__(self, window_size: int = 512):
        self.window_size = window_size
        self._stroke_buf: list[np.ndarray] = []   # frames for the active stroke
        self._dataset: list[tuple[np.ndarray, np.ndarray]] = []
        self._uwb_trajectory: list[np.ndarray] = []  # kept for diagnostics / Option 1

    # ------------------------------------------------------------------
    # Real-time feed
    # ------------------------------------------------------------------

    def on_imu_packet(
        self,
        imu_frames: list[np.ndarray],
        timestamps: list[int] | None = None,
    ) -> None:
        """Accumulate frames from the active stroke.  Call on every IMU packet."""
        for frame in imu_frames:
            f = np.asarray(frame, dtype=np.float32)
            if f.shape == (FRAME_DIM,):
                self._stroke_buf.append(f)

    def on_uwb_fix(self, uwb_pos: np.ndarray, is_drawing: bool) -> None:
        """Track UWB trajectory (used by smooth_uwb_labels; not needed for Option 2)."""
        self._uwb_trajectory.append(
            np.asarray(uwb_pos, dtype=np.float32).copy()
        )

    # Strokes longer than window_size * OVERSHOOT_LIMIT are discarded.
    # Beyond this ratio the first-window label mismatch is too large to trust:
    # the window only sees a fraction of the journey but the label claims the
    # full displacement.  1.5× = stroke up to ~4.3 s at 180 Hz is acceptable.
    OVERSHOOT_LIMIT: float = 1.5

    def reset_stroke(self) -> tuple[bool, int, str]:
        """Call on pen-up.

        Takes the first window_size frames from the stroke buffer and saves
        them as a training sample with a zero placeholder label.

        Returns (saved: bool, frame_count: int, status: str) where status is
        one of 'SAVED', 'TOO_SHORT', or 'TOO_LONG'.
        """
        n = len(self._stroke_buf)

        if n < self.window_size:
            self._stroke_buf.clear()
            return False, n, 'TOO_SHORT'

        if n > self.window_size * self.OVERSHOOT_LIMIT:
            self._stroke_buf.clear()
            return False, n, 'TOO_LONG'

        window = np.stack(self._stroke_buf[:self.window_size], axis=0)  # (W, 7)
        self._dataset.append((window, np.zeros(2, dtype=np.float32)))
        self._stroke_buf.clear()
        return True, n, 'SAVED'

    @property
    def active_frame_count(self) -> int:
        """Number of frames accumulated in the current (not yet closed) stroke."""
        return len(self._stroke_buf)

    # ------------------------------------------------------------------
    # Label assignment (call after all recording sessions)
    # ------------------------------------------------------------------

    def use_known_points(
        self,
        start: tuple[float, float],
        end:   tuple[float, float],
    ) -> None:
        """Replace all placeholder labels with the ruler-measured displacement."""
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
