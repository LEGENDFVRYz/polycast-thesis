"""
run_all.py -- run every verification layer on every available dataset and
write a single markdown report to verification/out/report.md.

Usage (from kuru_method/asynchronous_stream/):
    python verification/run_all.py
"""

from __future__ import annotations

import time
import traceback
from pathlib import Path

from _common import OUT_DIR, write_report, LayerResult

import layer0_stream_health   as L0
import layer1_uwb_raw         as L1
import layer2_uwb_position    as L2
import layer3_imu_raw         as L3
import layer4_imu_integration as L4
import layer5_force_contact   as L5
import layer6_ekf_end_to_end  as L6
import layer7_ablations       as L7
import layer8_penup           as L8
import layer9_strokes         as L9
import layer10_legibility     as L10
import layer11_latency        as L11


LAYERS = [
    # Tier A — sensor-level
    ('layer0_stream_health',      L0),
    ('layer1_uwb_raw',            L1),
    ('layer3_imu_raw',            L3),
    # Tier B — fusion-level
    ('layer2_uwb_position',       L2),
    ('layer4_imu_integration',    L4),
    ('layer5_force_contact',      L5),
    ('layer6_ekf_end_to_end',     L6),
    ('layer7_ablations',          L7),
    ('layer8_penup',              L8),
    # Tier C — output-level
    ('layer9_strokes',            L9),
    ('layer10_legibility',        L10),
    ('layer11_latency',           L11),
]


def _safe_run(name: str, mod) -> list[LayerResult]:
    t0 = time.time()
    try:
        out = mod.run_all()
    except Exception as e:
        traceback.print_exc()
        return [LayerResult(
            layer=name, dataset='<exception>', passed=False,
            metrics={}, notes=[f'exception: {e!r}'])]
    dt = time.time() - t0
    print(f'  {name}: {len(out)} results in {dt:.1f}s')
    for r in out:
        print('    ' + r.summary_line())
    return out


def main() -> None:
    all_results: list[LayerResult] = []
    for name, mod in LAYERS:
        print(f'[run_all] {name} ...')
        all_results.extend(_safe_run(name, mod))

    report = OUT_DIR / 'report.md'
    write_report(all_results, report)
    print(f'\nReport written: {report}')

    total  = len(all_results)
    passed = sum(1 for r in all_results if r.passed)
    print(f'Overall: {passed}/{total} cells passed')


if __name__ == '__main__':
    main()
