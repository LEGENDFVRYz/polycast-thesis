"""
tracker.py — Pipeline Tracker Bridge
Applies docs/bg_rules.md (Pipelines / Sensor Fusion).

Bridges the full background/pipelines/ stack to the _prototype_thread.py
interface. The prototype thread owns the serial port and delivers one
readline() result at a time; this module accepts raw bytes and runs them
through the same ESKF pipeline used by visualizer.py, without touching
the serial layer.

Interface (drop-in for fusion_v14.StrokeTracker):
    tracker = StrokeTracker()
    bounds  = tracker.get_bbox()          → {'b_min': 0.0, 'b_max': 1.25}
    result  = tracker.process_packet(b)  → (x_m, y_m, is_drawing) | None
"""

from background.pipelines.config                         import cfg
from background.pipelines.cleaner.unpacker               import SerialStreamer
from background.pipelines.cleaner.normalizer             import StreamNormalizer
from background.pipelines.cleaner.time_alignment         import TimeAlignLayer
from background.pipelines.preprocess.imu                 import IMUPreprocessor
from background.pipelines.preprocess.contact             import ContactStateDetector
from background.pipelines.preprocess.uwb.range           import UWBRangePreprocessor
from background.pipelines.preprocess.uwb.trilateration   import UWBSolver
from background.pipelines.preprocess.uwb.position        import UWBPositionFilter
from background.pipelines.fusion.eskf                    import ESKF
from background.pipelines.fusion.baseline                import FusionEngine


class PipelineTracker:
    """
    Drop-in replacement for fusion_v14.StrokeTracker that runs the full
    production ESKF pipeline instead of the legacy integrator.
    """

    def __init__(self, fusion_mode: str = 'eskf'):
        self._norm       = StreamNormalizer()
        self._aligner    = TimeAlignLayer(buffer_size=500)
        self._imu_prep   = IMUPreprocessor()
        self._contact    = ContactStateDetector()
        self._range_prep = UWBRangePreprocessor(offsets=cfg.uwb.range_offsets_m)
        self._trilat     = UWBSolver()
        self._pos_filt   = UWBPositionFilter()
        self._fusion     = ESKF() if fusion_mode == 'eskf' else FusionEngine(fusion_alpha=0.15)

        # ContactStateDetector.__init__ does not initialise _was_drawing;
        # only reset() does.  Call it now to avoid AttributeError on first event.
        self._contact.reset()

    def get_bbox(self) -> dict:
        return {'b_min': 0.0, 'b_max': cfg.anchors.board_size_x}

    def process_packet(self, line_bytes: bytes) -> tuple | None:
        """
        Feed one raw readline() result through the full pipeline.

        Returns:
            (x_meters, y_meters, is_drawing)  on a valid IMU-derived fused position
            None                              for UWB-only corrections, parse errors,
                                              or preprocessor rejects
        """
        pkt = SerialStreamer.parse_raw_line(line_bytes)
        if pkt is None:
            return None

        evs = self._norm.normalize([pkt])
        if not evs:
            return None

        self._aligner.add_events(evs)
        sorted_evs = self._aligner.get_all_sorted()
        self._aligner.clear()

        result = None

        for ev in sorted_evs:
            if ev['sensor'] == 'IMU':
                p = self._imu_prep.process_one(ev)
                if not p:
                    continue
                s     = self._contact.process_one(p)
                fused = self._fusion.process_event(s)
                if fused:
                    result = (
                        fused['fused_x'],
                        fused['fused_y'],
                        bool(fused.get('stroke_active', False)),
                    )

            elif ev['sensor'] == 'UWB':
                for r in self._range_prep.feed([ev]):
                    rp = self._trilat.process_one(r)
                    if not rp:
                        continue
                    clean = self._pos_filt.process_one(rp)
                    if not clean:
                        continue
                    self._fusion.process_event(clean)

        return result

    def reset(self) -> None:
        """Reset all stateful pipeline stages (call on serial reconnect)."""
        self._norm.reset_stats()
        self._imu_prep.reset()
        self._contact.reset()
        self._trilat.reset()
        self._pos_filt.reset()
        self._fusion.reset()


# Drop-in alias for _prototype_thread.py compatibility
StrokeTracker = PipelineTracker


# ── Smoke test (no serial port required) ─────────────────────────────────────
if __name__ == '__main__':
    tracker = PipelineTracker()
    print(f'[tracker] bbox       : {tracker.get_bbox()}')

    # Synthetic valid IMU line
    imu_line = b'I,1,0.0,0.0,0.0,1.0,0.01,0.0,9.81,0.0,1000000\n'
    result = tracker.process_packet(imu_line)
    print(f'[tracker] IMU result : {result}')

    # Synthetic valid UWB line
    uwb_line = b'U,1,0.85,1.10,0.72,0.90,2000000\n'
    result = tracker.process_packet(uwb_line)
    print(f'[tracker] UWB result : {result}  (expected None)')

    # Corrupt line
    result = tracker.process_packet(b'garbage\n')
    print(f'[tracker] Bad result : {result}  (expected None)')

    print('[tracker] Smoke test passed.')
