"""
Module 8 - Stroke Reconstructor

Turns the flat stream of fused events from the fusion engine into finished
stroke polylines. A stroke is a contiguous sequence of points drawn while the
pen was in the CONTACT_DRAWING state (stroke_active=True).

Input (from ESKF.process_event - fused events):
    {
        'source':        'IMU' | 'UWB',
        'ts_hw':         int,
        'fused_x':       float,
        'fused_y':       float,
        'state':         str,         IMU stroke_state or 'UWB_CORRECTION'
        'stroke_id':     int,         non-zero only while active
        'stroke_active': bool
    }

Output (one stroke dict, emitted on stroke close):
    {
        'stroke_id': int,
        'points':    [(x, y, ts_hw), ...],   world-frame board coordinates
        'start_ts':  int,                    ts_hw of first point
        'end_ts':    int,                    ts_hw of last point
        'closed':    True
    }

Rules:
    -   UWB-originated fused events carry stroke_active=False and are ignored
        here (they moved the fused position, but no ink is laid down on a
        correction).
    -   A new stroke_id starts a new stroke, even if the previous one was not
        explicitly closed by an inactive event first.
    -   Consecutive identical (x, y) points are deduped (keeps first timestamp).

Run directly for a synthetic self-test, or --live to wire the full pipeline
from a serial port:
    python -m background.pipelines.reconstruct
    python -m background.pipelines.reconstruct --live
"""

import math

from background.pipelines.config import cfg
from background.pipelines.postprocess import StrokePostprocessor, UWBStrokeBuffer


def _pair_norm(pair):
    return math.sqrt(pair[0] * pair[0] + pair[1] * pair[1])


def _pair_add(left, right):
    return (left[0] + right[0], left[1] + right[1])


def _pair_sub(left, right):
    return (left[0] - right[0], left[1] - right[1])


def _pair_scale(pair, factor):
    return (pair[0] * factor, pair[1] * factor)


def _is_finite_pair(pair):
    return (
        isinstance(pair, (tuple, list)) and len(pair) >= 2 and
        math.isfinite(float(pair[0])) and math.isfinite(float(pair[1]))
    )


def _bbox_diag(points):
    if not points:
        return 0.0
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return math.sqrt((max(xs) - min(xs)) ** 2 + (max(ys) - min(ys)) ** 2)


class StrokeFinalizationIMUCleaner:
    """
    Re-integrates a closed stroke's IMU samples to remove drift accumulated
    during the live pass, then anchors the result back to the original
    fused start/end points so cleanup never relocates the stroke.

        1.  removeAccErr-style acceleration drift removal:
            detect motion start/end from acceleration magnitude, estimate the
            final acceleration drift, subtract a linearly increasing drift term
            across the moving segment, then subtract constant drift from the
            end segment.

        2.  zupt-style velocity correction:
            integrate acceleration, and whenever a still phase is detected,
            distribute the predicted residual velocity backward over the
            previous moving segment.

    The corrected relative IMU trajectory is then anchored back to the
    fused/UWB-supported start/end placement, so shape is cleaned without
    moving the stroke to a new board location.
    """

    def __init__(self):
        self.cfg = cfg.stroke_cleaner

    def clean(self, stroke: dict) -> dict:
        settings = self.cfg
        if not settings.enabled:
            stroke['cleaner'] = {'applied': False, 'reason': 'disabled'}
            return stroke

        points = stroke.get('points') or []
        samples = stroke.get('samples') or []
        point_count = len(points)
        
        if point_count < settings.min_points or len(samples) != point_count:
            stroke['cleaner'] = {'applied': False, 'reason': 'too_few_or_missing_samples'}
            stroke.pop('samples', None)
            return stroke

        raw_xy = [(float(x), float(y)) for x, y, _ in points]
        acceleration = []
        delta_times = []
        
        for index, sample in enumerate(samples):
            sample_acc = sample.get('acc_board_hp_tip', (0.0, 0.0))
            if not _is_finite_pair(sample_acc):
                sample_acc = (0.0, 0.0)
            acceleration.append((float(sample_acc[0]), float(sample_acc[1])))

            dt = sample.get('dt_s')
            if not isinstance(dt, (float, int)) or not math.isfinite(float(dt)) or float(dt) <= 0.0:
                if index > 0:
                    dt = max(1e-4, (points[index][2] - points[index - 1][2]) / 1_000_000.0)
                else:
                    dt = 1.0 / cfg.imu.sample_rate_hz
            delta_times.append(float(dt))

        corrected_acceleration, drift_info = self._remove_acc_err(
            acceleration, settings.acc_motion_threshold
        )
        velocities, zupt_info = self._zupt(
            corrected_acceleration, delta_times, settings.zupt_acc_threshold
        )

        if settings.force_zero_velocity_at_end and velocities:
            end_info = self._force_final_zero_velocity(velocities)
            zupt_info.update(end_info)

        relative_positions = self._position_track(corrected_acceleration, velocities, delta_times)
        if not relative_positions:
            stroke['cleaner'] = {'applied': False, 'reason': 'empty_reintegration'}
            stroke.pop('samples', None)
            return stroke

        # Reference integration produces a relative trajectory. Re-anchor it to
        # the original fused stroke start, then distribute the remaining
        # end-point error across the stroke.
        start_anchor = raw_xy[0]
        end_anchor = raw_xy[-1]
        candidate_points = [_pair_add(start_anchor, rel) for rel in relative_positions]
        end_error = _pair_sub(end_anchor, candidate_points[-1])
        denominator = max(1, point_count - 1)
        anchored_points = []
        
        for index, point in enumerate(candidate_points):
            fraction = index / denominator
            anchored_points.append(
                _pair_add(point, _pair_scale(end_error, settings.endpoint_anchor_blend * fraction))
            )

        raw_diagonal = max(_bbox_diag(raw_xy), 1e-6)
        clean_diagonal = _bbox_diag(anchored_points)
        
        if clean_diagonal > raw_diagonal * settings.max_bbox_ratio:
            stroke['cleaner'] = {
                'applied': False,
                'reason': 'bbox_guard',
                'raw_bbox_diag_m': raw_diagonal,
                'clean_bbox_diag_m': clean_diagonal,
            }
            stroke.pop('samples', None)
            return stroke

        blend = min(1.0, max(0.0, float(settings.shape_blend)))
        cleaned_points = []
        for (_, _, ts), raw_point, clean_point in zip(points, raw_xy, anchored_points):
            x = raw_point[0] * (1.0 - blend) + clean_point[0] * blend
            y = raw_point[1] * (1.0 - blend) + clean_point[1] * blend
            cleaned_points.append((x, y, ts))

        stroke['raw_points'] = points
        stroke['points'] = cleaned_points
        stroke['start_ts'] = cleaned_points[0][2]
        stroke['end_ts'] = cleaned_points[-1][2]
        stroke['cleaner'] = {
            'applied': True,
            'method': 'removeAccErr+ZUPT+positionTrack_endpoint_anchor',
            'shape_blend': blend,
            'raw_bbox_diag_m': raw_diagonal,
            'clean_bbox_diag_m': clean_diagonal,
            **drift_info,
            **zupt_info,
        }
        stroke.pop('samples', None)
        return stroke

    def _remove_acc_err(self, acceleration: list[tuple[float, float]], threshold: float):
        # Same structure as reference removeAccErr(): find first motion sample,
        # find last sample that differs from final acceleration, infer final
        # drift, subtract linearly increasing drift through motion, constant
        # drift at tail.
        sample_count = len(acceleration)
        corrected = [tuple(value) for value in acceleration]
        if sample_count < 3:
            return corrected, {'acc_drift_removed': False}

        motion_start = 0
        for index in range(sample_count):
            if _pair_norm(corrected[index]) > threshold:
                motion_start = index
                break

        motion_end = sample_count - 1
        final_acceleration = corrected[-1]
        for index in range(sample_count - 1, -1, -1):
            if _pair_norm(_pair_sub(corrected[index], final_acceleration)) > threshold:
                motion_end = index
                break

        if motion_end <= motion_start:
            return corrected, {
                'acc_drift_removed': False,
                'acc_t_start': motion_start,
                'acc_t_end': motion_end,
            }

        tail = corrected[motion_end:]
        drift = (
            sum(value[0] for value in tail) / len(tail),
            sum(value[1] for value in tail) / len(tail),
        )
        span = max(1, motion_end - motion_start)
        drift_rate = _pair_scale(drift, 1.0 / span)

        for offset in range(span):
            index = motion_start + offset
            corrected[index] = _pair_sub(corrected[index], _pair_scale(drift_rate, offset + 1))
        for index in range(motion_end, sample_count):
            corrected[index] = _pair_sub(corrected[index], drift)

        return corrected, {
            'acc_drift_removed': True,
            'acc_t_start': motion_start,
            'acc_t_end': motion_end,
            'acc_drift_norm': _pair_norm(drift),
        }

    def _zupt(self, acceleration: list[tuple[float, float]], delta_times: list[float], threshold: float):
        # Same structure as reference zupt(): integrate velocity, detect
        # stillness, and when entering stillness, distribute predicted residual
        # velocity backward over the previous moving segment.
        velocities: list[tuple[float, float]] = []
        previous_still_index = -1
        in_still_phase = False
        velocity = (0.0, 0.0)
        correction_count = 0

        for index, sample_acc in enumerate(acceleration):
            dt = delta_times[index]
            if _pair_norm(sample_acc) < threshold:
                if not in_still_phase:
                    predicted_velocity = _pair_add(velocity, _pair_scale(sample_acc, dt))
                    span = max(1, index - previous_still_index)
                    drift_rate = _pair_scale(predicted_velocity, 1.0 / span)
                    for offset in range(index - previous_still_index - 1):
                        target_index = previous_still_index + 1 + offset
                        velocities[target_index] = _pair_sub(
                            velocities[target_index], _pair_scale(drift_rate, offset + 1)
                        )
                    velocity = (0.0, 0.0)
                    previous_still_index = index
                    in_still_phase = True
                    correction_count += 1
            else:
                velocity = _pair_add(velocity, _pair_scale(sample_acc, dt))
                in_still_phase = False

            velocities.append(velocity)

        return velocities, {'zupt_segments': correction_count}

    def _force_final_zero_velocity(self, velocities: list[tuple[float, float]]):
        if len(velocities) < 2:
            return {'end_velocity_forced_zero': False}
        residual = velocities[-1]
        sample_count = len(velocities)
        denominator = max(1, sample_count - 1)
        
        for index in range(sample_count):
            fraction = index / denominator
            velocities[index] = _pair_sub(velocities[index], _pair_scale(residual, fraction))
            
        return {
            'end_velocity_forced_zero': True,
            'end_velocity_residual_norm': _pair_norm(residual),
        }

    def _position_track(self, acceleration, velocities, delta_times):
        # Same structure as reference positionTrack(): p += v*dt + 0.5*a*dt^2.
        position = (0.0, 0.0)
        positions = []
        for sample_acc, velocity, dt in zip(acceleration, velocities, delta_times):
            position = _pair_add(
                position, _pair_add(_pair_scale(velocity, dt), _pair_scale(sample_acc, 0.5 * dt * dt))
            )
            positions.append(position)
        return positions


class StrokeReconstructor:
    """Assembles fused events into closed stroke polylines."""

    def __init__(self, dedup_tolerance: float = 1e-6):
        self._dedup_tolerance = dedup_tolerance
        self._current: dict | None = None
        self._closed_count = 0
        self._point_count = 0  # total points across all closed strokes
        self._imu_cleaner = StrokeFinalizationIMUCleaner()
        self._postprocessor = StrokePostprocessor()
        self._uwb_buffer = UWBStrokeBuffer()

    # -------------------------------------------------------------------------
    # Main entry
    # -------------------------------------------------------------------------

    def process_event(self, event: dict) -> dict | None:
        """
        Consume one fused event. Returns a closed stroke dict the moment a
        stroke finishes, else None.
        """

        source = event.get('source')

        # UWB-originated correction events never create or close ink, but
        # while a stroke is open they are the cleanest samples for the UWB
        # centroid.
        if source != 'IMU':
            if self._current is not None:
                self._uwb_buffer.feed(event)
            return None

        active = bool(event.get('stroke_active', False))
        stroke_id = int(event.get('stroke_id', 0))

        # Inactive IMU event - close any open stroke, emit it. Do not feed this
        # pen-up event to the UWB buffer because it belongs to the air/release
        # phase, not the finished ink interval.
        if not active or stroke_id == 0:
            return self._close_current()

        # Active event - ensure a current stroke exists under the right id.
        if self._current is None:
            self._start_stroke(stroke_id, event)
            if self._current is not None:
                self._uwb_buffer.feed(event)
            return None

        if self._current['stroke_id'] != stroke_id:
            # Different stroke id while still active -> boundary. Close old
            # stroke before feeding this event so the first sample of the new
            # stroke is not lost from the UWB buffer.
            closed = self._close_current()
            self._start_stroke(stroke_id, event)
            if self._current is not None:
                self._uwb_buffer.feed(event)
            return closed

        self._uwb_buffer.feed(event)
        self._append_point(event)
        return None

    def flush(self) -> dict | None:
        """Close any open stroke (e.g. at end of a session)."""

        return self._close_current()

    # -------------------------------------------------------------------------
    # Stats
    # -------------------------------------------------------------------------

    def stats(self) -> dict:
        return {
            'closed_strokes': self._closed_count,
            'total_points':   self._point_count,
            'open_stroke_id': self._current['stroke_id'] if self._current else None,
            'open_points':    len(self._current['points']) if self._current else 0,
        }

    def reset(self):
        self._current = None
        self._closed_count = 0
        self._point_count = 0
        self._uwb_buffer.reset()

    # -------------------------------------------------------------------------
    # Internals
    # -------------------------------------------------------------------------

    def _sample_from_event(self, event: dict) -> dict:
        payload = event.get('imu_cleaner') or {}
        return {
            'acc_board_hp_tip': payload.get('acc_board_hp_tip', (0.0, 0.0)),
            'dt_s': payload.get('dt_s'),
            'vel': payload.get('vel'),
            'rel_pos': payload.get('rel_pos'),
            'uwb': payload.get('uwb'),
            'contact': payload.get('contact', event.get('contact_raw', True)),
            'is_static': payload.get('is_static', False),
        }

    def _start_stroke(self, stroke_id: int, event: dict):
        # Phase-3: skip first point if physical contact is already gone.
        # contact_raw defaults True for backward-compat with pre-Phase-3 events.
        if not event.get('contact_raw', True):
            return
        x, y = float(event['fused_x']), float(event['fused_y'])
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        point = (x, y, int(event['ts_hw']))
        self._current = {
            'stroke_id': stroke_id,
            'points':    [point],
            'samples':   [self._sample_from_event(event)],
            'start_ts':  point[2],
            'end_ts':    point[2],
        }

    def _append_point(self, event: dict):
        # Phase-3: skip ink when physical contact is gone (tail samples in debounce window).
        if not event.get('contact_raw', True):
            return
        x = float(event['fused_x'])
        y = float(event['fused_y'])
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        ts = int(event['ts_hw'])

        last_x, last_y, _ = self._current['points'][-1]
        if abs(x - last_x) < self._dedup_tolerance and abs(y - last_y) < self._dedup_tolerance:
            # Duplicate sample - keep first timestamp, do not extend.
            return

        self._current['points'].append((x, y, ts))
        self._current['samples'].append(self._sample_from_event(event))
        self._current['end_ts'] = ts

    def _close_current(self) -> dict | None:
        if self._current is None:
            return None
        stroke = self._current
        stroke['closed'] = True
        self._current = None

        stroke = self._imu_cleaner.clean(stroke)

        uwb_points = self._uwb_buffer.drain(stroke['start_ts'], stroke['end_ts'])
        stroke = self._postprocessor.process(stroke, uwb_points)

        self._closed_count += 1
        self._point_count += len(stroke['points'])
        return stroke


# -----------------------------------------------------------------------------
# Module Testing
#   Default: synthetic fused-event streams (no hardware / replayer required)
#   --live : wire the full live pipeline (Serial -> ... -> Fusion -> Reconstruct)
# -----------------------------------------------------------------------------

def _run_live():
    """
    Live pipeline: SerialStreamer -> Normalizer -> TimeAlign ->
    (IMU branch + UWB branch) -> ESKF -> StrokeReconstructor.

    Prints one line per stroke close. Ctrl+C to stop.
    """

    import os
    import time

    os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

    from background.pipelines.cleaner.normalizer import StreamNormalizer
    from background.pipelines.cleaner.time_alignment import TimeAlignLayer
    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.fusion.eskf import ESKF
    from background.pipelines.preprocess.contact import ContactStateDetector
    from background.pipelines.preprocess.imu import IMUPreprocessor
    from background.pipelines.preprocess.uwb.position import UWBPositionFilter
    from background.pipelines.preprocess.uwb.range import UWBRangePreprocessor
    from background.pipelines.preprocess.uwb.trilateration import UWBSolver

    fusion = ESKF()

    port = getattr(cfg.serial, 'port', 'COM20')
    baud = getattr(cfg.serial, 'baud', 115200)

    streamer = SerialStreamer(port=port, baud=baud)
    normalizer = StreamNormalizer()
    aligner = TimeAlignLayer(buffer_size=500)

    imu_prep = IMUPreprocessor()
    contact = ContactStateDetector()

    uwb_offsets = getattr(cfg.uwb, 'range_offsets_m', (0.0, 0.0, 0.0, 0.0))
    range_prep = UWBRangePreprocessor(offsets=uwb_offsets)
    trilateration = UWBSolver()
    position_filter = UWBPositionFilter()

    reconstructor = StrokeReconstructor()

    print("=" * 60)
    print(f"  [TEST] MODULE 8 LIVE: Stroke Reconstructor on {port}")
    print("  Fusion engine : eskf")
    print("  Draw strokes with pen-lifts between. Ctrl+C to stop.")
    print("=" * 60)

    def _report(stroke: dict):
        span_s = (stroke['end_ts'] - stroke['start_ts']) / 1_000_000.0
        print(f"  [stroke closed] id={stroke['stroke_id']:>3}  "
              f"pts={len(stroke['points']):>4}  span={span_s:.3f}s")

    try:
        while True:
            raw_packets = streamer.read_new_packets()
            if raw_packets:
                aligner.add_events(normalizer.normalize(raw_packets))
                sorted_events = aligner.get_all_sorted()
                aligner.clear()

                for event in sorted_events:
                    if event['sensor'] == 'IMU':
                        preprocessed = imu_prep.process_one(event)
                        if preprocessed:
                            with_contact = contact.process_one(preprocessed)
                            fused = fusion.process_event(with_contact)
                            if fused:
                                closed = reconstructor.process_event(fused)
                                if closed:
                                    _report(closed)

                    elif event['sensor'] == 'UWB':
                        for ranged in range_prep.feed([event]):
                            solved = trilateration.process_one(ranged)
                            if solved:
                                positioned = position_filter.process_one(solved)
                                if positioned:
                                    fused = fusion.process_event(positioned)
                                    if fused:
                                        closed = reconstructor.process_event(fused)
                                        if closed:
                                            _report(closed)

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[STOP] Halting - flushing any open stroke.")
        tail = reconstructor.flush()
        if tail:
            _report(tail)
        streamer.close()

        stats = reconstructor.stats()
        print("-" * 60)
        print(f"  Total strokes closed : {stats['closed_strokes']}")
        print(f"  Total points logged  : {stats['total_points']}")
        print("=" * 60)


def _run_synthetic():
    def make_event(x, y, ts, stroke_id, active, source='IMU'):
        return {
            'source': source, 'ts_hw': ts,
            'fused_x': x, 'fused_y': y,
            'state': 'CONTACT_DRAWING' if active else 'IDLE',
            'stroke_id': stroke_id if active else 0,
            'stroke_active': active,
        }

    def run_case(name, events, expected_strokes):
        reconstructor = StrokeReconstructor()
        closed_strokes = []
        for event in events:
            closed = reconstructor.process_event(event)
            if closed:
                closed_strokes.append(closed)
        tail = reconstructor.flush()
        if tail:
            closed_strokes.append(tail)

        ok = len(closed_strokes) == len(expected_strokes)
        mismatches = []
        if ok:
            for closed, expected in zip(closed_strokes, expected_strokes):
                if closed['stroke_id'] != expected['stroke_id'] or len(closed['points']) != expected['points']:
                    ok = False
                    mismatches.append(
                        f"  expected id={expected['stroke_id']} pts={expected['points']}, "
                        f"got id={closed['stroke_id']} pts={len(closed['points'])}"
                    )

        status = 'PASS' if ok else 'FAIL'
        print(f"[{status}] {name}  - closed={len(closed_strokes)} (expected {len(expected_strokes)})")
        for closed in closed_strokes:
            print(f"        stroke_id={closed['stroke_id']} pts={len(closed['points'])} "
                  f"span_ts={closed['end_ts'] - closed['start_ts']}")
        for mismatch in mismatches:
            print(mismatch)
        return ok

    print("=" * 60)
    print("  [TEST] MODULE 8: Stroke Reconstructor - synthetic inputs")
    print("=" * 60)

    all_ok = True

    # Case 1 - single stroke, 4 active IMU events, then pen-up.
    events = [
        make_event(0.10, 0.20, 1_000, 1, True),
        make_event(0.11, 0.20, 1_020, 1, True),
        make_event(0.12, 0.21, 1_040, 1, True),
        make_event(0.13, 0.22, 1_060, 1, True),
        make_event(0.13, 0.22, 1_080, 0, False),   # pen-up -> closes
    ]
    all_ok &= run_case("Case 1: single stroke, pen-up closes",
                       events, [{'stroke_id': 1, 'points': 4}])

    # Case 2 - two strokes separated by pen-up.
    events = [
        make_event(0.0, 0.0, 2_000, 1, True),
        make_event(0.1, 0.0, 2_020, 1, True),
        make_event(0.1, 0.0, 2_040, 0, False),    # close stroke 1
        make_event(0.5, 0.5, 2_200, 2, True),
        make_event(0.6, 0.5, 2_220, 2, True),
        make_event(0.7, 0.5, 2_240, 2, True),
        make_event(0.7, 0.5, 2_260, 0, False),    # close stroke 2
    ]
    all_ok &= run_case("Case 2: two strokes, pen-up between",
                       events, [{'stroke_id': 1, 'points': 2},
                                {'stroke_id': 2, 'points': 3}])

    # Case 3 - UWB correction events interleaved (active=False); must be ignored.
    events = [
        make_event(0.0, 0.0, 3_000, 1, True),
        make_event(0.0, 0.0, 3_010, 0, False, source='UWB'),   # ignored
        make_event(0.1, 0.0, 3_020, 1, True),
        make_event(0.0, 0.0, 3_025, 0, False, source='UWB'),   # ignored
        make_event(0.2, 0.0, 3_040, 1, True),
        make_event(0.2, 0.0, 3_060, 0, False),                  # close
    ]
    all_ok &= run_case("Case 3: UWB corrections interleaved (ignored)",
                       events, [{'stroke_id': 1, 'points': 3}])

    # Case 4 - stroke_id flips mid-stream without inactive gap -> boundary.
    events = [
        make_event(0.0, 0.0, 4_000, 1, True),
        make_event(0.1, 0.0, 4_020, 1, True),
        make_event(0.2, 0.5, 4_040, 2, True),    # id change -> closes 1, starts 2
        make_event(0.3, 0.5, 4_060, 2, True),
        make_event(0.3, 0.5, 4_080, 0, False),   # close 2
    ]
    all_ok &= run_case("Case 4: stroke_id changes mid-stream",
                       events, [{'stroke_id': 1, 'points': 2},
                                {'stroke_id': 2, 'points': 2}])

    # Case 5 - duplicate consecutive points deduped.
    events = [
        make_event(0.00, 0.00, 5_000, 1, True),
        make_event(0.00, 0.00, 5_010, 1, True),   # dup -> dropped
        make_event(0.00, 0.00, 5_020, 1, True),   # dup -> dropped
        make_event(0.05, 0.00, 5_030, 1, True),
        make_event(0.05, 0.00, 5_040, 0, False),
    ]
    all_ok &= run_case("Case 5: duplicate samples deduped",
                       events, [{'stroke_id': 1, 'points': 2}])

    # Case 6 - flush() closes an open stroke at stream end.
    events = [
        make_event(0.0, 0.0, 6_000, 7, True),
        make_event(0.1, 0.1, 6_020, 7, True),
        make_event(0.2, 0.2, 6_040, 7, True),
        # no closing inactive event - flush() must still emit the stroke
    ]
    all_ok &= run_case("Case 6: flush closes an unterminated stroke",
                       events, [{'stroke_id': 7, 'points': 3}])

    print("=" * 60)
    print(f"  RESULT: {'ALL TESTS PASSED' if all_ok else 'FAILURES DETECTED'}")
    print("=" * 60)


if __name__ == '__main__':
    import sys

    if '--live' in sys.argv:
        _run_live()
    else:
        _run_synthetic()
