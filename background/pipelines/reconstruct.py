"""
tv8_reconstruct.py
==================
Module 8 — Stroke Reconstructor

Turns the flat stream of fused events from FusionEngine into finished stroke
polylines. A stroke is a contiguous sequence of points drawn while the pen
was in the CONTACT_DRAWING state (stroke_active=True).

Input (from FusionEngine.process_event — fused events):
    {
        'source':        'IMU' | 'UWB',
        'ts_hw':         int,
        'fused_x':       float,
        'fused_y':       float,
        'state':         str,         ← IMU stroke_state or 'UWB_CORRECTION'
        'stroke_id':     int,         ← non-zero only while active
        'stroke_active': bool
    }

Output (one stroke dict, emitted on stroke close):
    {
        'stroke_id': int,
        'points':    [(x, y, ts_hw), ...],   ← world-frame board coordinates
        'start_ts':  int,                    ← ts_hw of first point
        'end_ts':    int,                    ← ts_hw of last point
        'closed':    True
    }

Rules:
  - UWB-originated fused events carry stroke_active=False and are ignored here
    (they moved the fused position, but no ink is laid down on a correction).
  - A new stroke_id starts a new stroke — even if the previous one was not
    explicitly closed by an inactive event first.
  - Consecutive identical (x, y) points are deduped (keeps first timestamp).
"""

from background.pipelines.config import cfg


class StrokeReconstructor:
    def __init__(self, dedup_tol: float = 1e-6):
        self._dedup_tol = dedup_tol
        self._current: dict | None = None
        self._closed_count = 0
        self._point_count = 0  # total points across all closed strokes

    # ── Main entry ────────────────────────────────────────────────────────────
    def process_event(self, ev: dict) -> dict | None:
        """
        Consume one fused event. Returns a closed stroke dict the moment a
        stroke finishes, else None.
        """
        # Only IMU-originated fused events carry authoritative stroke state.
        # UWB corrections always set stroke_active=False; if we treated them
        # as a close signal, every UWB update mid-stroke would split the ink.
        if ev.get('source') != 'IMU':
            return None

        active = bool(ev.get('stroke_active', False))
        sid    = int(ev.get('stroke_id', 0))

        # Inactive IMU event — close any open stroke, emit it.
        if not active or sid == 0:
            return self._close_current()

        # Active event — ensure a current stroke exists under the right id.
        if self._current is None:
            self._start_stroke(sid, ev)
            return None

        if self._current['stroke_id'] != sid:
            # Different stroke id while still active → boundary, emit old, start new.
            closed = self._close_current()
            self._start_stroke(sid, ev)
            return closed

        self._append_point(ev)
        return None

    def flush(self) -> dict | None:
        """Close any open stroke (e.g. at end of a session)."""
        return self._close_current()

    # ── Stats ─────────────────────────────────────────────────────────────────
    def stats(self) -> dict:
        return {
            'closed_strokes': self._closed_count,
            'total_points':   self._point_count,
            'open_stroke_id': self._current['stroke_id'] if self._current else None,
            'open_points':    len(self._current['points']) if self._current else 0,
        }

    def reset(self):
        self._current      = None
        self._closed_count = 0
        self._point_count  = 0

    # ── Internals ─────────────────────────────────────────────────────────────
    def _start_stroke(self, sid: int, ev: dict):
        pt = (float(ev['fused_x']), float(ev['fused_y']), int(ev['ts_hw']))
        self._current = {
            'stroke_id': sid,
            'points':    [pt],
            'start_ts':  pt[2],
            'end_ts':    pt[2],
        }

    def _append_point(self, ev: dict):
        x  = float(ev['fused_x'])
        y  = float(ev['fused_y'])
        ts = int(ev['ts_hw'])

        last_x, last_y, _ = self._current['points'][-1]
        if abs(x - last_x) < self._dedup_tol and abs(y - last_y) < self._dedup_tol:
            # Duplicate sample — keep first timestamp, do not extend.
            return

        self._current['points'].append((x, y, ts))
        self._current['end_ts'] = ts

    def _close_current(self) -> dict | None:
        if self._current is None:
            return None
        stroke = self._current
        stroke['closed'] = True
        self._current = None
        self._closed_count += 1
        self._point_count  += len(stroke['points'])
        return stroke


# ==============================================================================
# SELF-TEST
#   Default: synthetic fused-event streams (no hardware / replayer required)
#   --live : wire the full live pipeline (Serial → … → Fusion → Reconstruct)
# ==============================================================================
def _run_live(fusion_mode: str = 'complementary'):
    """Live pipeline: SerialStreamer → Normalizer → TimeAlign →
       (IMU branch + UWB branch) → Fusion → StrokeReconstructor.
       Prints one line per stroke close. Ctrl+C to stop.

       fusion_mode: 'complementary' (default) → baseline FusionEngine
                    'eskf'                     → ESKF
    """
    import time
    import os

    os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

    from background.pipelines.cleaner.unpacker     import SerialStreamer
    from background.pipelines.cleaner.normalizer   import StreamNormalizer
    from background.pipelines.cleaner.time_alignment import TimeAlignLayer
    from background.pipelines.preprocess.imu       import IMUPreprocessor
    from background.pipelines.preprocess.contact   import ContactStateDetector
    from background.pipelines.preprocess.uwb.range         import UWBRangePreprocessor
    from background.pipelines.preprocess.uwb.trilateration import UWBSolver
    from background.pipelines.preprocess.uwb.position      import UWBPositionFilter

    if fusion_mode == 'eskf':
        from background.pipelines.fusion.eskf import ESKF
        fusion = ESKF()
    else:
        from background.pipelines.fusion.baseline import FusionEngine
        fusion = FusionEngine(fusion_alpha=0.15)

    port = getattr(cfg.serial, 'port', 'COM20')
    baud = getattr(cfg.serial, 'baud', 115200)

    streamer   = SerialStreamer(port=port, baud=baud)
    norm       = StreamNormalizer()
    aligner    = TimeAlignLayer(buffer_size=500)

    imu_prep   = IMUPreprocessor()
    contact    = ContactStateDetector()

    uwb_offs   = getattr(cfg.uwb, 'range_offsets_m', (0.0, 0.0, 0.0, 0.0))
    range_prep = UWBRangePreprocessor(offsets=uwb_offs)
    trilat     = UWBSolver()
    pos_filter = UWBPositionFilter()

    rec        = StrokeReconstructor()

    print("=" * 60)
    print(f"  [TEST] MODULE 8 LIVE: Stroke Reconstructor on {port}")
    print(f"  Fusion engine : {fusion_mode}")
    print("  Draw strokes with pen-lifts between. Ctrl+C to stop.")
    print("=" * 60)

    def _report(stroke: dict):
        span_s = (stroke['end_ts'] - stroke['start_ts']) / 1_000_000.0
        print(f"  [stroke closed] id={stroke['stroke_id']:>3}  "
              f"pts={len(stroke['points']):>4}  span={span_s:.3f}s")

    try:
        while True:
            raw = streamer.read_new_packets()
            if raw:
                evs = norm.normalize(raw)
                aligner.add_events(evs)
                sorted_evs = aligner.get_all_sorted()
                aligner.clear()

                for ev in sorted_evs:
                    if ev['sensor'] == 'IMU':
                        p = imu_prep.process_one(ev)
                        if p:
                            s = contact.process_one(p)
                            fused = fusion.process_event(s)
                            if fused:
                                closed = rec.process_event(fused)
                                if closed:
                                    _report(closed)

                    elif ev['sensor'] == 'UWB':
                        for r in range_prep.feed([ev]):
                            raw_pos = trilat.process_one(r)
                            if raw_pos:
                                clean = pos_filter.process_one(raw_pos)
                                if clean:
                                    fused = fusion.process_event(clean)
                                    if fused:
                                        closed = rec.process_event(fused)
                                        if closed:
                                            _report(closed)

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[STOP] Halting — flushing any open stroke.")
        tail = rec.flush()
        if tail:
            _report(tail)
        streamer.close()

        s = rec.stats()
        print("-" * 60)
        print(f"  Total strokes closed : {s['closed_strokes']}")
        print(f"  Total points logged  : {s['total_points']}")
        print("=" * 60)


def _run_synthetic():
    def make_ev(x, y, ts, sid, active, source='IMU'):
        return {
            'source': source, 'ts_hw': ts,
            'fused_x': x, 'fused_y': y,
            'state': 'CONTACT_DRAWING' if active else 'IDLE',
            'stroke_id': sid if active else 0,
            'stroke_active': active,
        }

    def run_case(name, events, expected_strokes):
        rec = StrokeReconstructor()
        got = []
        for ev in events:
            out = rec.process_event(ev)
            if out:
                got.append(out)
        tail = rec.flush()
        if tail:
            got.append(tail)

        ok = len(got) == len(expected_strokes)
        details = []
        if ok:
            for g, exp in zip(got, expected_strokes):
                if g['stroke_id'] != exp['stroke_id'] or len(g['points']) != exp['points']:
                    ok = False
                    details.append(
                        f"  expected id={exp['stroke_id']} pts={exp['points']}, "
                        f"got id={g['stroke_id']} pts={len(g['points'])}"
                    )

        status = 'PASS' if ok else 'FAIL'
        print(f"[{status}] {name}  — closed={len(got)} (expected {len(expected_strokes)})")
        for g in got:
            print(f"        stroke_id={g['stroke_id']} pts={len(g['points'])} "
                  f"span_ts={g['end_ts'] - g['start_ts']}")
        for d in details:
            print(d)
        return ok

    print("=" * 60)
    print("  [TEST] MODULE 8: Stroke Reconstructor — synthetic inputs")
    print("=" * 60)

    all_ok = True

    # Case 1 — single stroke, 4 active IMU events, then pen-up.
    evs = [
        make_ev(0.10, 0.20, 1_000, 1, True),
        make_ev(0.11, 0.20, 1_020, 1, True),
        make_ev(0.12, 0.21, 1_040, 1, True),
        make_ev(0.13, 0.22, 1_060, 1, True),
        make_ev(0.13, 0.22, 1_080, 0, False),   # pen-up → closes
    ]
    all_ok &= run_case("Case 1: single stroke, pen-up closes",
                       evs, [{'stroke_id': 1, 'points': 4}])

    # Case 2 — two strokes separated by pen-up.
    evs = [
        make_ev(0.0, 0.0, 2_000, 1, True),
        make_ev(0.1, 0.0, 2_020, 1, True),
        make_ev(0.1, 0.0, 2_040, 0, False),    # close stroke 1
        make_ev(0.5, 0.5, 2_200, 2, True),
        make_ev(0.6, 0.5, 2_220, 2, True),
        make_ev(0.7, 0.5, 2_240, 2, True),
        make_ev(0.7, 0.5, 2_260, 0, False),    # close stroke 2
    ]
    all_ok &= run_case("Case 2: two strokes, pen-up between",
                       evs, [{'stroke_id': 1, 'points': 2},
                             {'stroke_id': 2, 'points': 3}])

    # Case 3 — UWB correction events interleaved (active=False); must be ignored.
    evs = [
        make_ev(0.0, 0.0, 3_000, 1, True),
        make_ev(0.0, 0.0, 3_010, 0, False, source='UWB'),   # ignored
        make_ev(0.1, 0.0, 3_020, 1, True),
        make_ev(0.0, 0.0, 3_025, 0, False, source='UWB'),   # ignored
        make_ev(0.2, 0.0, 3_040, 1, True),
        make_ev(0.2, 0.0, 3_060, 0, False),                  # close
    ]
    all_ok &= run_case("Case 3: UWB corrections interleaved (ignored)",
                       evs, [{'stroke_id': 1, 'points': 3}])

    # Case 4 — stroke_id flips mid-stream without inactive gap → boundary.
    evs = [
        make_ev(0.0, 0.0, 4_000, 1, True),
        make_ev(0.1, 0.0, 4_020, 1, True),
        make_ev(0.2, 0.5, 4_040, 2, True),    # id change → closes 1, starts 2
        make_ev(0.3, 0.5, 4_060, 2, True),
        make_ev(0.3, 0.5, 4_080, 0, False),   # close 2
    ]
    all_ok &= run_case("Case 4: stroke_id changes mid-stream",
                       evs, [{'stroke_id': 1, 'points': 2},
                             {'stroke_id': 2, 'points': 2}])

    # Case 5 — duplicate consecutive points deduped.
    evs = [
        make_ev(0.00, 0.00, 5_000, 1, True),
        make_ev(0.00, 0.00, 5_010, 1, True),   # dup → dropped
        make_ev(0.00, 0.00, 5_020, 1, True),   # dup → dropped
        make_ev(0.05, 0.00, 5_030, 1, True),
        make_ev(0.05, 0.00, 5_040, 0, False),
    ]
    all_ok &= run_case("Case 5: duplicate samples deduped",
                       evs, [{'stroke_id': 1, 'points': 2}])

    # Case 6 — flush() closes an open stroke at stream end.
    evs = [
        make_ev(0.0, 0.0, 6_000, 7, True),
        make_ev(0.1, 0.1, 6_020, 7, True),
        make_ev(0.2, 0.2, 6_040, 7, True),
        # no closing inactive event — flush() must still emit the stroke
    ]
    all_ok &= run_case("Case 6: flush closes an unterminated stroke",
                       evs, [{'stroke_id': 7, 'points': 3}])

    print("=" * 60)
    print(f"  RESULT: {'ALL TESTS PASSED' if all_ok else 'FAILURES DETECTED'}")
    print("=" * 60)


if __name__ == '__main__':
    import sys

    # --fusion eskf|complementary  (default: complementary)
    fusion_mode = 'complementary'
    for i, arg in enumerate(sys.argv):
        if arg == '--fusion' and i + 1 < len(sys.argv):
            fusion_mode = sys.argv[i + 1]
        elif arg.startswith('--fusion='):
            fusion_mode = arg.split('=', 1)[1]

    if '--live' in sys.argv:
        _run_live(fusion_mode)
    else:
        _run_synthetic()
