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

import math

from background.pipelines.config import cfg


def _pair_norm(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1])


def _pair_add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _pair_sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _pair_scale(a, s):
    return (a[0] * s, a[1] * s)


def _is_finite_pair(v):
    return (
        isinstance(v, (tuple, list)) and len(v) >= 2 and
        math.isfinite(float(v[0])) and math.isfinite(float(v[1]))
    )


def _bbox_diag(points):
    if not points:
        return 0.0
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return math.sqrt((max(xs) - min(xs)) ** 2 + (max(ys) - min(ys)) ** 2)


class StrokeFinalizationIMUCleaner:
    """Reference-style post-stroke IMU cleaner.

    This class intentionally mirrors the two useful batch steps from
    john2zy/IMU-Position-Tracking, but applies them in the MewlyCast coordinate
    system after pen-up:

      1. removeAccErr-style acceleration drift removal:
         detect motion start/end from acceleration magnitude, estimate the final
         acceleration drift, subtract a linearly increasing drift term across the
         moving segment, then subtract constant drift from the end segment.

      2. zupt-style velocity correction:
         integrate acceleration, and whenever a still phase is detected, distribute
         the predicted residual velocity backward over the previous moving segment.

    MewlyCast then anchors the corrected relative IMU trajectory back to the
    fused/UWB-supported start/end placement so we clean shape without moving the
    stroke to a new board location.
    """

    def __init__(self):
        self.cfg = cfg.stroke_cleaner

    def clean(self, stroke: dict) -> dict:
        c = self.cfg
        if not c.enabled:
            stroke['cleaner'] = {'applied': False, 'reason': 'disabled'}
            return stroke

        points = stroke.get('points') or []
        samples = stroke.get('samples') or []
        n = len(points)
        if n < c.min_points or len(samples) != n:
            stroke['cleaner'] = {'applied': False, 'reason': 'too_few_or_missing_samples'}
            stroke.pop('samples', None)
            return stroke

        raw_xy = [(float(x), float(y)) for x, y, _ in points]
        acc = []
        dts = []
        for i, sample in enumerate(samples):
            a = sample.get('acc_board_hp_tip', (0.0, 0.0))
            if not _is_finite_pair(a):
                a = (0.0, 0.0)
            acc.append((float(a[0]), float(a[1])))

            dt = sample.get('dt_s')
            if not isinstance(dt, (float, int)) or not math.isfinite(float(dt)) or float(dt) <= 0.0:
                if i > 0:
                    dt = max(1e-4, (points[i][2] - points[i - 1][2]) / 1_000_000.0)
                else:
                    dt = 1.0 / cfg.imu.sample_rate_hz
            dts.append(float(dt))

        acc_corr, drift_info = self._remove_acc_err(acc, c.acc_motion_threshold)
        velocities, zupt_info = self._zupt(acc_corr, dts, c.zupt_acc_threshold)

        if c.force_zero_velocity_at_end and velocities:
            end_info = self._force_final_zero_velocity(velocities)
            zupt_info.update(end_info)

        rel = self._position_track(acc_corr, velocities, dts)
        if not rel:
            stroke['cleaner'] = {'applied': False, 'reason': 'empty_reintegration'}
            stroke.pop('samples', None)
            return stroke

        # Reference integration produces a relative trajectory. Re-anchor it to the
        # original fused stroke start, then distribute the remaining end-point error
        # across the stroke. This keeps UWB-supported placement while allowing the
        # IMU-cleaned local shape to replace part of the live preview drift.
        start_anchor = raw_xy[0]
        end_anchor = raw_xy[-1]
        candidate = [_pair_add(start_anchor, r) for r in rel]
        end_error = _pair_sub(end_anchor, candidate[-1])
        denom = max(1, n - 1)
        anchored = []
        for i, p in enumerate(candidate):
            t = i / denom
            anchored.append(_pair_add(p, _pair_scale(end_error, c.endpoint_anchor_blend * t)))

        raw_diag = max(_bbox_diag(raw_xy), 1e-6)
        clean_diag = _bbox_diag(anchored)
        if clean_diag > raw_diag * c.max_bbox_ratio:
            stroke['cleaner'] = {
                'applied': False,
                'reason': 'bbox_guard',
                'raw_bbox_diag_m': raw_diag,
                'clean_bbox_diag_m': clean_diag,
            }
            stroke.pop('samples', None)
            return stroke

        blend = min(1.0, max(0.0, float(c.shape_blend)))
        cleaned_points = []
        for i, ((_, _, ts), raw_p, clean_p) in enumerate(zip(points, raw_xy, anchored)):
            x = raw_p[0] * (1.0 - blend) + clean_p[0] * blend
            y = raw_p[1] * (1.0 - blend) + clean_p[1] * blend
            cleaned_points.append((x, y, ts))

        stroke['raw_points'] = points
        stroke['points'] = cleaned_points
        stroke['start_ts'] = cleaned_points[0][2]
        stroke['end_ts'] = cleaned_points[-1][2]
        stroke['cleaner'] = {
            'applied': True,
            'method': 'removeAccErr+ZUPT+positionTrack_endpoint_anchor',
            'shape_blend': blend,
            'raw_bbox_diag_m': raw_diag,
            'clean_bbox_diag_m': clean_diag,
            **drift_info,
            **zupt_info,
        }
        stroke.pop('samples', None)
        return stroke

    def _remove_acc_err(self, acc: list[tuple[float, float]], threshold: float):
        # Same structure as reference removeAccErr(): find first motion sample,
        # find last sample that differs from final acceleration, infer final drift,
        # subtract linearly increasing drift through motion, constant drift at tail.
        n = len(acc)
        a = [tuple(v) for v in acc]
        if n < 3:
            return a, {'acc_drift_removed': False}

        t_start = 0
        for t in range(n):
            if _pair_norm(a[t]) > threshold:
                t_start = t
                break

        t_end = n - 1
        final_a = a[-1]
        for t in range(n - 1, -1, -1):
            if _pair_norm(_pair_sub(a[t], final_a)) > threshold:
                t_end = t
                break

        if t_end <= t_start:
            return a, {
                'acc_drift_removed': False,
                'acc_t_start': t_start,
                'acc_t_end': t_end,
            }

        tail = a[t_end:]
        drift = (
            sum(v[0] for v in tail) / len(tail),
            sum(v[1] for v in tail) / len(tail),
        )
        span = max(1, t_end - t_start)
        drift_rate = _pair_scale(drift, 1.0 / span)

        for i in range(span):
            idx = t_start + i
            a[idx] = _pair_sub(a[idx], _pair_scale(drift_rate, i + 1))
        for idx in range(t_end, n):
            a[idx] = _pair_sub(a[idx], drift)

        return a, {
            'acc_drift_removed': True,
            'acc_t_start': t_start,
            'acc_t_end': t_end,
            'acc_drift_norm': _pair_norm(drift),
        }

    def _zupt(self, acc: list[tuple[float, float]], dts: list[float], threshold: float):
        # Same structure as reference zupt(): integrate velocity, detect stillness,
        # and when entering stillness, distribute predicted residual velocity backward
        # over the previous moving segment.
        velocities: list[tuple[float, float]] = []
        prevt = -1
        still_phase = False
        v = (0.0, 0.0)
        corrections = 0

        for t, at in enumerate(acc):
            dt = dts[t]
            if _pair_norm(at) < threshold:
                if not still_phase:
                    predict_v = _pair_add(v, _pair_scale(at, dt))
                    span = max(1, t - prevt)
                    v_drift_rate = _pair_scale(predict_v, 1.0 / span)
                    for i in range(t - prevt - 1):
                        idx = prevt + 1 + i
                        velocities[idx] = _pair_sub(velocities[idx], _pair_scale(v_drift_rate, i + 1))
                    v = (0.0, 0.0)
                    prevt = t
                    still_phase = True
                    corrections += 1
            else:
                v = _pair_add(v, _pair_scale(at, dt))
                still_phase = False

            velocities.append(v)

        return velocities, {'zupt_segments': corrections}

    def _force_final_zero_velocity(self, velocities: list[tuple[float, float]]):
        if len(velocities) < 2:
            return {'end_velocity_forced_zero': False}
        residual = velocities[-1]
        n = len(velocities)
        denom = max(1, n - 1)
        for i in range(n):
            t = i / denom
            velocities[i] = _pair_sub(velocities[i], _pair_scale(residual, t))
        return {
            'end_velocity_forced_zero': True,
            'end_velocity_residual_norm': _pair_norm(residual),
        }

    def _position_track(self, acc, velocities, dts):
        # Same structure as reference positionTrack(): p += v*dt + 0.5*a*dt².
        p = (0.0, 0.0)
        positions = []
        for at, vt, dt in zip(acc, velocities, dts):
            p = _pair_add(p, _pair_add(_pair_scale(vt, dt), _pair_scale(at, 0.5 * dt * dt)))
            positions.append(p)
        return positions


class StrokeReconstructor:
    def __init__(self, dedup_tol: float = 1e-6):
        self._dedup_tol = dedup_tol
        self._current: dict | None = None
        self._closed_count = 0
        self._point_count = 0  # total points across all closed strokes
        self._imu_cleaner = StrokeFinalizationIMUCleaner()

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
    def _sample_from_event(self, ev: dict) -> dict:
        payload = ev.get('imu_cleaner') or {}
        return {
            'acc_board_hp_tip': payload.get('acc_board_hp_tip', (0.0, 0.0)),
            'dt_s': payload.get('dt_s'),
            'vel': payload.get('vel'),
            'rel_pos': payload.get('rel_pos'),
            'uwb': payload.get('uwb'),
            'contact': payload.get('contact', ev.get('contact_raw', True)),
            'is_static': payload.get('is_static', False),
        }

    def _start_stroke(self, sid: int, ev: dict):
        # Phase-3: skip first point if physical contact is already gone.
        # contact_raw defaults True for backward-compat with pre-Phase-3 events.
        if not ev.get('contact_raw', True):
            return
        x, y = float(ev['fused_x']), float(ev['fused_y'])
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        pt = (x, y, int(ev['ts_hw']))
        self._current = {
            'stroke_id': sid,
            'points':    [pt],
            'samples':   [self._sample_from_event(ev)],
            'start_ts':  pt[2],
            'end_ts':    pt[2],
        }

    def _append_point(self, ev: dict):
        # Phase-3: skip ink when physical contact is gone (tail samples in debounce window).
        if not ev.get('contact_raw', True):
            return
        x  = float(ev['fused_x'])
        y  = float(ev['fused_y'])
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        ts = int(ev['ts_hw'])

        last_x, last_y, _ = self._current['points'][-1]
        if abs(x - last_x) < self._dedup_tol and abs(y - last_y) < self._dedup_tol:
            # Duplicate sample — keep first timestamp, do not extend.
            return

        self._current['points'].append((x, y, ts))
        self._current['samples'].append(self._sample_from_event(ev))
        self._current['end_ts'] = ts

    def _close_current(self) -> dict | None:
        if self._current is None:
            return None
        stroke = self._current
        stroke['closed'] = True
        self._current = None

        stroke = self._imu_cleaner.clean(stroke)
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
