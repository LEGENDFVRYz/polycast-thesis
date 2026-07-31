"""
(Post-Process Helper) Per-stroke UWB point accumulator.

This is a plain data-collection helper, not a correction step: 

Its only job is remembering "which UWB points arrived while this particular stroke 
was being drawn," because nothing else in the pipeline tracks that (for centroid_align module).

centroid_align module needs exactly that window - a clean UWB point cloud scoped to one 
stroke's duration - to compute a trustworthy centroid, so this buffer exists to supply it.

Usage (mirrored by StrokeReconstructor):
    feed(event)                 call on every fused event while a stroke is open
    drain(start_ts, end_ts)     call once at pen-up; returns this stroke's
                                points and clears the buffer for the next one
    reset()                     call if a stroke is abandoned instead of closed

Preferred point sources, in order:
    1. event['eskf']['last_uwb_tip'] or event['eskf']['z_uwb_tip']  (canonical tip UWB)
    2. event['imu_cleaner']['uwb']                                  (IMU-row cached tip UWB)
    3. event['uwb_x'], event['uwb_y']                               (raw fallback only)
"""


class UWBStrokeBuffer:
    """Accumulates deduplicated UWB tip points for the currently open stroke."""

    def __init__(self, xy_tolerance_m: float = 1e-4):
        self._points: list[tuple[float, float, int]] = []
        self._last_xy: tuple[float, float] | None = None
        self._xy_tolerance_m = float(xy_tolerance_m)

    def _extract_xy(self, event: dict):
        """Return the best available UWB tip point from a fused event."""

        eskf = event.get('eskf') or {}
        for key in ('last_uwb_tip', 'z_uwb_tip'):
            value = eskf.get(key)
            if value is not None and len(value) >= 2:
                return value[0], value[1]

        imu_cleaner = event.get('imu_cleaner') or {}
        value = imu_cleaner.get('uwb')
        if value is not None and len(value) >= 2:
            return value[0], value[1]

        return event.get('uwb_x'), event.get('uwb_y')

    def feed(self, event: dict) -> None:
        """Call with every fused event while a stroke is open."""

        x, y = self._extract_xy(event)
        ts = event.get('ts_hw')
        if x is None or y is None or ts is None:
            return
        try:
            x, y, ts = float(x), float(y), int(ts)
        except (TypeError, ValueError):
            return

        # Deduplicate by XY only. UWB fixes are often copied into many IMU
        # rows with different IMU timestamps; deduping by (x, y, ts) would
        # still let a stale fix dominate the centroid.
        if self._last_xy is not None:
            last_x, last_y = self._last_xy
            if abs(x - last_x) <= self._xy_tolerance_m and abs(y - last_y) <= self._xy_tolerance_m:
                return

        self._last_xy = (x, y)
        self._points.append((x, y, ts))

    def drain(self, start_ts: int, end_ts: int) -> list[tuple[float, float, int]]:
        """Return accumulated UWB points within [start_ts, end_ts], then reset."""

        result = [
            (x, y, ts) for x, y, ts in self._points
            if start_ts <= ts <= end_ts
        ]
        self.reset()
        return result

    def reset(self) -> None:
        self._points.clear()
        self._last_xy = None
