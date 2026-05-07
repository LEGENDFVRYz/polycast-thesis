from __future__ import annotations
from collections import deque
import numpy as np

DEFAULT_WINDOW_SIZE = 200
DEFAULT_STRIDE      = 40
FRAME_DIM           = 7


class IMUOdometryBuffer:
    """Sliding-window accumulator for learned inertial odometry.

    Accepts gravity-free world-frame IMU frames from preprocess/imu.py
    in batches of up to 3 (HAL constraint: 3 samples per ESP32 packet).
    Emits a (window_size, 7) numpy array every `stride` new frames once
    the buffer is full.  Resets cleanly on pen-up.
    """

    def __init__(self, window_size: int = DEFAULT_WINDOW_SIZE, stride: int = DEFAULT_STRIDE):
        self.window_size = window_size
        self.stride      = stride
        self._buf:    deque[np.ndarray] = deque(maxlen=window_size)
        self._ts_buf: deque[int]        = deque(maxlen=window_size)
        self._new_frames_since_emit     = 0
        self._fresh_after_reset         = True
        self._window_start_ts: int | None = None  # hw-ts of the oldest frame in last emitted window

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def push_packet(
        self,
        imu_frames: list[np.ndarray],
        timestamps: list[int] | None = None,
    ) -> np.ndarray | None:
        """Push up to 3 frames per ESP32 packet.

        imu_frames  — list of (7,) arrays:
                       [ax_world, ay_world, az_world, qx, qy, qz, qw]
                       sourced from preprocess/imu.py (gravity-free, world-frame).
        timestamps  — parallel list of hw timestamps (µs).  Pass None to skip
                       anchor-ts tracking (e.g. during offline replay).

        Returns a (window_size, 7) float32 array when stride frames have
        accumulated since the previous emit, otherwise None.
        """
        ts_list = timestamps if timestamps is not None else [0] * len(imu_frames)
        for frame, ts in zip(imu_frames, ts_list):
            frame = np.asarray(frame, dtype=np.float32)
            if frame.shape != (FRAME_DIM,):
                continue
            self._buf.append(frame)
            self._ts_buf.append(int(ts))
            self._new_frames_since_emit += 1
        return self._try_emit()

    def reset(self) -> None:
        """Clear state on pen-up / stroke boundary."""
        self._buf.clear()
        self._ts_buf.clear()
        self._new_frames_since_emit = 0
        self._fresh_after_reset     = True
        self._window_start_ts       = None

    @property
    def window_start_ts(self) -> int | None:
        """Hardware timestamp of the oldest frame in the most recently emitted window.

        Used by the ESKF to look up p_anchor via _interpolate_at().
        None until the first window has been emitted after the last reset.
        """
        return self._window_start_ts

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _try_emit(self) -> np.ndarray | None:
        if len(self._buf) < self.window_size:
            return None
        if self._new_frames_since_emit < self.stride:
            return None

        # On first emit after reset we just clear the flag; subsequent emits
        # fall through the same path.  No early return here — that was the bug
        # in the original design where only the first window was ever emitted.
        if self._fresh_after_reset:
            self._fresh_after_reset = False

        # Snapshot the hw-ts of the oldest frame in the full buffer so callers
        # can look up the ESKF state at window-start time.
        self._window_start_ts       = int(self._ts_buf[0]) if self._ts_buf else None
        self._new_frames_since_emit = 0
        return np.stack(self._buf, axis=0)  # (window_size, 7)
