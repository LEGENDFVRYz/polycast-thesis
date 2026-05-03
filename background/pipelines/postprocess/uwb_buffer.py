"""
uwb_buffer.py
=============
Per-stroke UWB point accumulator.

Collects lever-arm-corrected UWB tip points during an open stroke, then
releases them when the stroke closes.  The caller may feed every fused event
(IMU and UWB); this buffer deduplicates repeated UWB fixes so stale UWB points
copied across many IMU ticks do not bias the centroid.

Preferred point sources, in order:
  1. ev['eskf']['last_uwb_tip'] or ev['eskf']['z_uwb_tip']  # canonical tip UWB
  2. ev['imu_cleaner']['uwb']                              # IMU-row cached tip UWB
  3. ev['uwb_x'], ev['uwb_y']                              # raw fallback only
"""


class UWBStrokeBuffer:
    def __init__(self, xy_tol_m: float = 1e-4):
        self._points: list[tuple[float, float, int]] = []
        self._last_xy: tuple[float, float] | None = None
        self._xy_tol_m = float(xy_tol_m)

    def _extract_xy(self, ev: dict):
        """Return the best available UWB tip point from a fused event."""
        eskf = ev.get('eskf') or {}
        for key in ('last_uwb_tip', 'z_uwb_tip'):
            val = eskf.get(key)
            if val is not None and len(val) >= 2:
                return val[0], val[1]

        payload = ev.get('imu_cleaner') or {}
        val = payload.get('uwb')
        if val is not None and len(val) >= 2:
            return val[0], val[1]

        return ev.get('uwb_x'), ev.get('uwb_y')

    def feed(self, ev: dict) -> None:
        """Call with fused events while a stroke is open."""
        ux, uy = self._extract_xy(ev)
        ts = ev.get('ts_hw')
        if ux is None or uy is None or ts is None:
            return
        try:
            ux, uy, ts = float(ux), float(uy), int(ts)
        except (TypeError, ValueError):
            return

        # Deduplicate by XY only.  UWB fixes are often copied into many IMU rows
        # with different IMU timestamps; deduping by (x, y, ts) would still let a
        # stale fix dominate the centroid.
        if self._last_xy is not None:
            lx, ly = self._last_xy
            if abs(ux - lx) <= self._xy_tol_m and abs(uy - ly) <= self._xy_tol_m:
                return

        self._last_xy = (ux, uy)
        self._points.append((ux, uy, ts))

    def drain(self, start_ts: int, end_ts: int) -> list[tuple[float, float, int]]:
        """Return accumulated UWB points within [start_ts, end_ts], then reset."""
        result = [
            (x, y, t) for x, y, t in self._points
            if start_ts <= t <= end_ts
        ]
        self.reset()
        return result

    def reset(self) -> None:
        self._points.clear()
        self._last_xy = None
