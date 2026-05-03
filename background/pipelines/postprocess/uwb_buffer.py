"""
uwb_buffer.py
=============
Per-stroke UWB point accumulator.

Collects (uwb_x, uwb_y, ts_hw) triples from every fused event that arrives
while a stroke is open, then releases them as a list when the stroke closes.
The caller (StrokeReconstructor) feeds ALL fused events here — both IMU and
UWB-originated — before the usual IMU-only early return.

Design rules:
  - Deduplicates on (x, y, ts) so a stale UWB sample repeated across many
    IMU ticks does not skew the centroid estimate.
  - Stores only samples whose ts_hw falls within the open stroke window
    (filtered by the caller passing start_ts at drain time).
  - Zero external dependencies (no NumPy) — keeps the hot real-time path light.
"""


class UWBStrokeBuffer:
    def __init__(self):
        self._points: list[tuple[float, float, int]] = []
        self._last: tuple[float, float, int] | None = None

    def feed(self, ev: dict) -> None:
        """Call with every fused event (IMU or UWB) while a stroke is open."""
        ux = ev.get('uwb_x')
        uy = ev.get('uwb_y')
        ts = ev.get('ts_hw')
        if ux is None or uy is None or ts is None:
            return
        try:
            ux, uy, ts = float(ux), float(uy), int(ts)
        except (TypeError, ValueError):
            return
        # Deduplicate consecutive identical samples.
        sample = (ux, uy, ts)
        if sample == self._last:
            return
        self._last = sample
        self._points.append(sample)

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
        self._last = None
