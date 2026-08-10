"""
Drives the full ESKF pipeline directly from a CSV file with no serial I/O and no real-time delays.

Replaces the COM-port + _tx1replay.py loop for automated tuning.

Event routing (mirrors the live pipeline):
    IMU  line → IMUPreprocessor → ContactStateDetector → ESKF.process_event()
    UWB  line → UWBRangePreprocessor → UWBSolver → UWBPositionFilter → ESKF.process_event()

TimeAlignLayer and StreamNormalizer are intentionally skipped — they exist for
out-of-order serial arrival; sorted CSV events are already in timestamp order.

All six pipeline objects are freshly instantiated on every run_dataset() call
so they pick up any cfg patches applied before the call.
"""

import re
from background.pipelines.cleaner.unpacker import parse_packet_line
from background.pipelines.config import cfg
from background.pipelines.preprocess.imu import IMUPreprocessor
from background.pipelines.preprocess.contact import ContactStateDetector
from background.pipelines.preprocess.uwb.range import UWBRangePreprocessor
from background.pipelines.preprocess.uwb.trilateration import UWBSolver
from background.pipelines.preprocess.uwb.position import UWBPositionFilter
from background.pipelines.fusion.eskf import ESKF

_SPLIT_RE = re.compile(r'[\t, ]+')


def parse_dataset(csv_path: str) -> list[dict]:
    """
    Parse a dataset CSV into normalized event dicts sorted by ts_hw.

    Line parsing is delegated to the pipeline's own `parse_packet_line`, which
    is the single source of truth for the wire format and accepts both IMU
    layouts:

      IMU v2:  I,<seq>,<qx>,<qy>,<qz>,<qw>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<force>,<ts>
      IMU v1:  I,<seq>,<qx>,<qy>,<qz>,<qw>,<ax>,<ay>,<az>,<force>,<ts>
      UWB:     U,<seq>,<d0>,<d1>,<d2>,<d3>,<ts>

    This previously had its own copy of the format that only accepted the
    11-field legacy IMU layout. Every dataset in test/_datasets is v2 with
    gyro, so all 3549 IMU lines of abcde_1 were silently dropped and only UWB
    survived: with no contact signal the batch reported strokes=0 for every
    dataset and produced blank plots. Delegating keeps the two in step when
    the format changes again.

    Trailing commas in v2/v3 datasets are stripped before parsing, since
    `parse_packet_line` identifies layouts by field count.
    """

    events = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # Collapse whitespace/tab separators and drop empty fields so a
            # trailing comma cannot change the field count.
            normalized = ','.join(p for p in _SPLIT_RE.split(line) if p)
            packet = parse_packet_line(normalized)
            if packet is not None:
                events.append(packet)
    events.sort(key=lambda e: e['ts_hw'])
    return events


def run_dataset(csv_path: str) -> list[dict]:
    """
    Drive the full ESKF pipeline on a dataset CSV.

    Returns a list of fused output dicts from ESKF._emit() — one entry per
    accepted IMU frame and per accepted UWB frame. Each dict has the full
    schema from ESKF._emit() plus a '_contact' key (bool) derived from the
    contact state for convenient scoring.

    All pipeline objects are freshly instantiated to pick up any cfg patches.
    No file I/O, no serial ports, no timing delays.
    """

    events = parse_dataset(csv_path)

    imu_prep   = IMUPreprocessor()
    contact    = ContactStateDetector()
    range_prep = UWBRangePreprocessor(offsets=cfg.uwb.range_offsets_m)
    solver     = UWBSolver()
    pos_filter = UWBPositionFilter()
    eskf       = ESKF()

    results = []
    for ev in events:
        if ev['sensor'] == 'IMU':
            processed = imu_prep.process_one(ev)
            if processed is None:
                continue
            annotated = contact.process_one(processed)
            fused = eskf.process_event(annotated)
            if fused is not None:
                fused['_contact'] = bool(annotated.get('stroke_active', False))
                fused['_is_static'] = bool(annotated.get('is_static', False))
                results.append(fused)

        elif ev['sensor'] == 'UWB':
            ranged = range_prep.process_one(ev)
            if ranged is None:
                continue
            solved = solver.process_one(ranged)
            if solved is None:
                continue
            positioned = pos_filter.process_one(solved)
            if positioned is None:
                continue
            fused = eskf.process_event(positioned)
            if fused is not None:
                fused['_contact'] = False
                fused['_is_static'] = False
                results.append(fused)

    return results
