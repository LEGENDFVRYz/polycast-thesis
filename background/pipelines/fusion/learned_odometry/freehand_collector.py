from __future__ import annotations
import numpy as np
from scipy.signal import savgol_filter
from .buffer import IMUOdometryBuffer


class FreehandOdometryCollector:
    """Sliding-window collector for freehand UWB-labeled odometry data (Option 1).

    How it works
    ------------
    Unlike OdometryDataCollector (Option 2 / known points) which emits one
    sample per stroke, this collector runs the sliding IMUOdometryBuffer
    continuously during drawing — firing a new window every `stride` frames
    (~0.22 s at 180 Hz).  UWB position fixes are recorded alongside with
    hardware timestamps.  On process_and_save(), the full UWB trajectory is
    smoothed with Savitzky-Golay, and each window's label is computed by
    interpolating the smoothed trajectory at the window's start and end time.

    Key difference from Option 2
    -----------------------------
    Labels are NOT ruler-measured fixed displacements — they are derived from
    the smoothed UWB trajectory, which means:
      - Every stroke direction / magnitude is representable (continuous coverage)
      - Short handwriting strokes produce real, small displacement labels
      - Label error ≈ ±0.5–1 cm (smoothed UWB) vs ±0 cm (known points)

    Draw style
    ----------
    Write SLOWLY and CONTINUOUSLY with minimal pen lifts.  Every pen-up
    resets the buffer, so strokes must be ≥ window_size frames (≈ 2.84 s at
    180 Hz) to emit any windows.  Cursive / connected writing works well.
    """

    def __init__(self, window_size: int = 512, stride: int = 40):
        self.window_size = window_size
        self._buf        = IMUOdometryBuffer(window_size=window_size, stride=stride)
        self._last_ts: int = 0

        # Raw collected windows before label computation
        self._raw: list[tuple[np.ndarray, int, int]] = []   # (window, start_ts, end_ts)

        # UWB log — kept for post-hoc smoothing
        self._uwb_log: list[tuple[int, float, float]] = []  # (ts_hw, x, y)

    # ------------------------------------------------------------------
    # Real-time feed  (call from PipelineTracker event loop)
    # ------------------------------------------------------------------

    def on_imu_packet(
        self,
        imu_frames: list[np.ndarray],
        timestamps: list[int] | None = None,
    ) -> None:
        """Feed frames while pen is actively drawing.

        Only call when stroke_active=True — air frames are excluded by the
        caller so every window contains only drawing motion.
        """
        ts_list = timestamps if timestamps is not None else [0] * len(imu_frames)
        if ts_list:
            self._last_ts = ts_list[-1]

        window = self._buf.push_packet(imu_frames, ts_list)
        if window is not None:
            start_ts = self._buf.window_start_ts or 0
            self._raw.append((window.copy(), int(start_ts), int(self._last_ts)))

    def on_uwb_fix(self, ts: int, x: float, y: float) -> None:
        """Record a UWB position fix — called regardless of pen state."""
        self._uwb_log.append((int(ts), float(x), float(y)))

    def reset_stroke(self) -> None:
        """Call on pen-up.  Clears the IMU buffer so the next window
        starts cleanly at the next pen-down, never spanning a lift."""
        self._buf.reset()

    # ------------------------------------------------------------------
    # Post-processing  (call after recording session ends)
    # ------------------------------------------------------------------

    def process_and_save(
        self,
        out_path:      str,
        window_length: int = 21,
        polyorder:     int = 3,
    ) -> int:
        """Smooth the UWB trajectory, compute labels, save .npz.

        Returns the number of samples saved.
        Skips windows whose timestamps fall outside UWB coverage.
        """
        if not self._raw:
            print('[WARN] No IMU windows collected — nothing saved.')
            return 0

        if len(self._uwb_log) < window_length:
            print(f'[WARN] Only {len(self._uwb_log)} UWB fixes recorded '
                  f'(need ≥ {window_length} for Savitzky-Golay). Nothing saved.')
            return 0

        # Build smoothed UWB trajectory
        ts_uwb   = np.array([e[0] for e in self._uwb_log], dtype=np.float64)
        x_raw    = np.array([e[1] for e in self._uwb_log], dtype=np.float32)
        y_raw    = np.array([e[2] for e in self._uwb_log], dtype=np.float32)

        x_smooth = savgol_filter(x_raw, window_length, polyorder).astype(np.float32)
        y_smooth = savgol_filter(y_raw, window_length, polyorder).astype(np.float32)

        # Print smoothing quality summary
        jitter_x = float(np.abs(x_raw - x_smooth).mean() * 100)
        jitter_y = float(np.abs(y_raw - y_smooth).mean() * 100)
        print(f'  UWB jitter removed: x={jitter_x:.1f}cm  y={jitter_y:.1f}cm (avg per fix)')

        # Pair each window with smoothed UWB displacement
        windows, deltas = [], []
        skipped = 0

        for window, start_ts, end_ts in self._raw:
            # Skip windows whose timestamps are outside UWB log range
            if start_ts < ts_uwb[0] or end_ts > ts_uwb[-1]:
                skipped += 1
                continue

            p_sx = float(np.interp(start_ts, ts_uwb, x_smooth))
            p_sy = float(np.interp(start_ts, ts_uwb, y_smooth))
            p_ex = float(np.interp(end_ts,   ts_uwb, x_smooth))
            p_ey = float(np.interp(end_ts,   ts_uwb, y_smooth))

            delta = np.array([p_ex - p_sx, p_ey - p_sy], dtype=np.float32)
            windows.append(window)
            deltas.append(delta)

        if skipped:
            print(f'  Skipped {skipped} windows outside UWB coverage.')

        if not windows:
            print('[WARN] No valid windows after UWB interpolation — nothing saved.')
            return 0

        import os
        os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
        np.savez(
            out_path,
            windows=np.array(windows, dtype=np.float32),
            deltas =np.array(deltas,  dtype=np.float32),
        )

        # Label distribution summary
        d = np.array(deltas)
        print(f'  Label Δx: min={d[:,0].min():+.3f}m  max={d[:,0].max():+.3f}m  '
              f'std={d[:,0].std():.3f}m')
        print(f'  Label Δy: min={d[:,1].min():+.3f}m  max={d[:,1].max():+.3f}m  '
              f'std={d[:,1].std():.3f}m')

        return len(windows)

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    @property
    def window_count(self) -> int:
        return len(self._raw)

    @property
    def uwb_fix_count(self) -> int:
        return len(self._uwb_log)
