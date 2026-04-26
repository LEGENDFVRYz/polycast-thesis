"""
Headless dataset runner — drives the full ESKF pipeline directly from a CSV
file with no serial I/O and no real-time delays.

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

    CSV formats:
      IMU: I,<seq>,<qx>,<qy>,<qz>,<qw>,<ax>,<ay>,<az>,<force>,<ts_hw>  — 11 fields
      UWB: U,<seq>,<d0>,<d1>,<d2>,<d3>,<ts_hw>[,,,...]                  — 7+ fields

    Uses the same split logic as _tx1replay.py (re.split + empty-string filter)
    to handle trailing commas in v2/v3 datasets.
    """
    events = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = [p for p in _SPLIT_RE.split(line) if p]
            if len(parts) < 2:
                continue
            t = parts[0]
            try:
                if t == 'I' and len(parts) == 11:
                    events.append({
                        'sensor':     'IMU',
                        'packet_id':  int(parts[1]),
                        'sample_idx': 0,
                        'quat':  (float(parts[2]), float(parts[3]),
                                  float(parts[4]), float(parts[5])),
                        'acc':   (float(parts[6]), float(parts[7]), float(parts[8])),
                        'force': float(parts[9]),
                        'ts_hw': int(parts[10]),
                    })
                elif t == 'U' and len(parts) >= 7:
                    events.append({
                        'sensor':     'UWB',
                        'packet_id':  int(parts[1]),
                        'sample_idx': 0,
                        'dists': (float(parts[2]), float(parts[3]),
                                  float(parts[4]), float(parts[5])),
                        'ts_hw': int(parts[6]),
                    })
            except (ValueError, IndexError):
                continue
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
