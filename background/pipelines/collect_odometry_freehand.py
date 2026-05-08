"""
Freehand Odometry Collection Script — Option 1: Smoothed UWB Labels
====================================================================

Records freehand whiteboard strokes and computes displacement labels
from Savitzky-Golay smoothed UWB trajectory after the session ends.

Unlike Option 2 (fixed points), you draw ANYTHING freely — letters,
words, shapes, spirals, lines.  One 5-minute session produces ~400–600
training samples with continuous, diverse displacement coverage.

DRAW STYLE (important):
  ✓ Write SLOWLY and CONTINUOUSLY — like drawing in slow motion
  ✓ Minimise pen lifts — each lift resets the buffer
  ✓ Each continuous stroke must last ≥ 2.8s to produce samples
  ✓ Cover ALL board areas and ALL directions over the session
  ✓ Vary stroke length: short (letters), medium (words), long (lines)
  ✗ Do NOT draw fast — the model needs to learn natural writing pace

Usage:
    python -m background.pipelines.collect_odometry_freehand
    python -m background.pipelines.collect_odometry_freehand --duration 600
    python -m background.pipelines.collect_odometry_freehand --out data/odometry/freehand_2.npz
"""

from __future__ import annotations
import os
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")

import argparse
import time
import sys

import numpy as np


def _assemble_frame(ev: dict) -> np.ndarray | None:
    """Build (7,) frame from a processed IMU/contact event.
    Layout: [ax_world, ay_world, az_world, qx, qy, qz, qw]
    """
    acc = ev.get('acc_tip_world') or ev.get('acc_world')
    q   = ev.get('quat')
    if acc is None or q is None:
        return None
    ax, ay, az      = acc
    qx, qy, qz, qw = q
    return np.array([ax, ay, az, qx, qy, qz, qw], dtype=np.float32)


def collect(duration: int, out_path: str, fusion_mode: str = 'eskf') -> None:

    from background.pipelines.cleaner.unpacker          import SerialStreamer
    from background.pipelines.cleaner.normalizer        import StreamNormalizer
    from background.pipelines.cleaner.time_alignment    import TimeAlignLayer
    from background.pipelines.preprocess.imu            import IMUPreprocessor
    from background.pipelines.preprocess.contact        import ContactStateDetector
    from background.pipelines.preprocess.uwb.range          import UWBRangePreprocessor
    from background.pipelines.preprocess.uwb.trilateration  import UWBSolver
    from background.pipelines.preprocess.uwb.position       import UWBPositionFilter
    from background.pipelines.fusion.learned_odometry.freehand_collector import FreehandOdometryCollector
    from background.pipelines.config                    import cfg

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

    collector = FreehandOdometryCollector(
        window_size = cfg.odometry.window_size,
        stride      = cfg.odometry.stride,
    )

    win_s = cfg.odometry.window_size / 180.0

    print('=' * 62)
    print('  [FREEHAND] Option 1 — Smoothed UWB Labels')
    print(f'  Duration : {duration}s  ({duration // 60}m {duration % 60}s)')
    print(f'  Output   : {out_path}')
    print(f'  Window   : {cfg.odometry.window_size} frames = {win_s:.2f}s  '
          f'(stride={cfg.odometry.stride})')
    print()
    print('  DRAW STYLE:')
    print(f'    - Each stroke must be ≥ {win_s:.1f}s without lifting the pen')
    print('    - Write slowly — letters, words, shapes, lines')
    print('    - Cover all board areas, all directions')
    print('    - Pen lifts reset the buffer — minimise them')
    print()
    print('  Press Ctrl+C to stop early and save collected data.')
    print('=' * 62)

    prev_stroke_active = False
    start_time         = time.time()
    last_print         = start_time

    try:
        while True:
            elapsed = time.time() - start_time
            if elapsed >= duration:
                print(f'\n  [DONE] {duration}s elapsed.')
                break

            if time.time() - last_print >= 15.0:
                remaining = duration - elapsed
                print(
                    f'  t={elapsed:5.0f}s'
                    f'  windows={collector.window_count:>5}'
                    f'  uwb_fixes={collector.uwb_fix_count:>6}'
                    f'  remaining={remaining:.0f}s'
                )
                last_print = time.time()

            raw = streamer.read_new_packets()
            if not raw:
                time.sleep(0.005)
                continue

            evs = norm.normalize(raw)
            aligner.add_events(evs)
            sorted_evs = aligner.get_all_sorted()
            aligner.clear()

            for ev in sorted_evs:

                # ── IMU branch ──────────────────────────────────────────
                if ev['sensor'] == 'IMU':
                    p = imu_prep.process_one(ev)
                    if not p:
                        continue
                    s = contact.process_one(p)
                    fused = fusion.process_event(s)
                    if not fused:
                        continue

                    stroke_active_now = bool(fused.get('stroke_active', False))

                    # Only push frames while actively drawing
                    if stroke_active_now:
                        frame = _assemble_frame(s)
                        if frame is not None:
                            collector.on_imu_packet(
                                imu_frames=[frame],
                                timestamps=[s.get('ts_hw', 0)],
                            )

                    # Pen-up → reset buffer so next window starts clean
                    if prev_stroke_active and not stroke_active_now:
                        collector.reset_stroke()

                    prev_stroke_active = stroke_active_now

                # ── UWB branch ──────────────────────────────────────────
                elif ev['sensor'] == 'UWB':
                    for r in range_prep.feed([ev]):
                        raw_pos = trilat.process_one(r)
                        if not raw_pos:
                            continue
                        clean = pos_filter.process_one(raw_pos)
                        if not clean:
                            continue
                        fused = fusion.process_event(clean)
                        if not fused:
                            continue
                        # Record UWB fix for post-hoc label computation
                        collector.on_uwb_fix(
                            ts=int(fused.get('ts_hw', clean.get('ts_hw', 0))),
                            x =float(fused.get('uwb_x', 0.0)),
                            y =float(fused.get('uwb_y', 0.0)),
                        )

    except KeyboardInterrupt:
        print('\n  [STOP] Ctrl+C — saving collected data...')

    finally:
        streamer.close()

    # ── Post-processing ──────────────────────────────────────────────────
    print()
    print(f'  Raw windows   : {collector.window_count}')
    print(f'  UWB fixes     : {collector.uwb_fix_count}')
    print()
    print('  Applying Savitzky-Golay smoothing...')

    n = collector.process_and_save(out_path)

    if n > 0:
        print()
        print('=' * 62)
        print(f'  Saved {n} samples → {out_path}')
        print()
        print('  Merge with existing Option 2 data and retrain:')
        print('    python -m background.pipelines.merge_odometry_datasets')
        print('    python -m background.pipelines.fusion.learned_odometry.train \\')
        print('        data/odometry/training_data.npz \\')
        print('        --save-dir background/pipelines/fusion/learned_odometry/assets/ \\')
        print('        --optuna --trials 60 --device cuda')
        print('=' * 62)
    else:
        print('  [WARN] No samples saved.')
        print('  Common causes:')
        print('    - Strokes too short (< {:.1f}s) — draw slower'.format(
            cfg.odometry.window_size / 180.0))
        print('    - UWB not providing fixes — check anchor connections')


def main() -> None:
    os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

    p = argparse.ArgumentParser(
        description='Collect freehand odometry data (Option 1 — smoothed UWB labels).',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--duration', type=int, default=300,
                   help='Recording duration in seconds (default: 300 = 5 min)')
    p.add_argument('--out',      default=None,
                   help='Output .npz path (auto-numbered if omitted)')
    p.add_argument('--fusion',   default='eskf', choices=['eskf', 'complementary'])
    args = p.parse_args()

    # Auto-number freehand output files
    if args.out is None:
        base = os.path.join('data', 'odometry')
        os.makedirs(base, exist_ok=True)
        i = 1
        while os.path.exists(os.path.join(base, f'freehand_{i}.npz')):
            i += 1
        args.out = os.path.join(base, f'freehand_{i}.npz')

    collect(args.duration, args.out, args.fusion)


if __name__ == '__main__':
    main()
