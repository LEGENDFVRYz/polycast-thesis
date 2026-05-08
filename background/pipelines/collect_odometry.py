"""
Odometry Data Collection Script  —  Option 2: Known Physical Points
====================================================================

Runs the full live pipeline and feeds processed IMU frames into
OdometryDataCollector.  On Ctrl+C (or after --reps strokes), it
applies the known physical label and saves a .npz ready for training.

BEFORE RUNNING:
  1. Edit LABEL_POINTS below to match your actual ruler measurements (metres).
  2. Place tape markers on the whiteboard at each defined point.

Usage:
  python -m background.pipelines.collect_odometry \\
      --from TL --to BR --reps 25 --out data/odometry/TL_to_BR.npz

  --from   name of start point (must be a key in LABEL_POINTS)
  --to     name of end  point  (must be a key in LABEL_POINTS)
  --reps   number of stroke repetitions to record (default: 25)
  --out    output .npz path (default: data/odometry/<FROM>_to_<TO>.npz)
  --fusion eskf | complementary  (default: eskf)

HOW TO RECORD ONE REPETITION:
  1. Touch the pen tip to the --from tape marker.
  2. Draw to the --to tape marker (any path — straight, wobbly, all fine).
  3. Lift the pen.
  The script counts completed strokes and prints progress automatically.
"""

from __future__ import annotations
import os
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")  # prevent libiomp5md.dll double-load on Windows

import argparse
import time
import sys

import numpy as np

# ============================================================
# ⚙  EDIT THESE — measure with a ruler, units: metres
#
#  Board: 1.25 m (x, horizontal) × 1.20 m (y, vertical)
#  UWB anchors occupy all four corners — DO NOT use corner points.
#  All points below are inset 10 cm from every board edge.
#
#  Physical layout (top = high y):
#
#    TL -------- TC -------- TR        y = 1.10
#    |                        |
#    ML -------- MC -------- MR        y = 0.60
#    |                        |
#    BL -------- BC -------- BR        y = 0.10
#
#    x:  0.10   0.625   1.15
#
#  Recommended stroke pairs (run one --from/--to session per row):
#    Horizontals : TL→TR,  ML→MR,  BL→BR          (covers ±x)
#    Verticals   : TL→BL,  TC→BC,  TR→BR          (covers ±y)
#    Diagonals   : TL→BR,  TR→BL                  (covers both axes, max ~1.45 m)
#    Shorts      : TL→TC,  MC→MR,  MC→BC          (covers small displacements)
#
#  Max stroke in this grid  ≈ 1.45 m  (TL↔BR diagonal)
#  Window duration          ≈ 3.0 s   (512 frames @ ~170 Hz)
#  Min speed for max stroke ≈ 0.48 m/s — normal drawing pace
# ============================================================
LABEL_POINTS: dict[str, tuple[float, float]] = {
    # Top row  (y = 1.10)
    "TL": (0.10,  1.10),   # near top-left anchor, inset 10 cm
    "TC": (0.625, 1.10),   # top-centre
    "TR": (1.15,  1.10),   # near top-right anchor, inset 10 cm
    # Middle row  (y = 0.60)
    "ML": (0.10,  0.60),   # mid-left
    "MC": (0.625, 0.60),   # centre
    "MR": (1.15,  0.60),   # mid-right
    # Bottom row  (y = 0.10)
    "BL": (0.10,  0.10),   # near bot-left anchor, inset 10 cm
    "BC": (0.625, 0.10),   # bottom-centre
    "BR": (1.15,  0.10),   # near bot-right anchor, inset 10 cm
}
# ============================================================


def _assemble_frame(ev: dict) -> np.ndarray | None:
    """Build a (7,) float32 frame from a processed IMU / contact event.

    Frame layout: [ax_world, ay_world, az_world, qx, qy, qz, qw]
    Source fields (imu.py output):
      acc_tip_world — 3D world-frame linear accel, tip-corrected, gravity-free
      quat          — unit quaternion [qx, qy, qz, qw]
    """
    acc = ev.get('acc_tip_world') or ev.get('acc_world')
    q   = ev.get('quat')
    if acc is None or q is None:
        return None
    ax, ay, az    = acc
    qx, qy, qz, qw = q
    return np.array([ax, ay, az, qx, qy, qz, qw], dtype=np.float32)


def collect(
    from_name: str,
    to_name:   str,
    n_reps:    int,
    out_path:  str,
    fusion_mode: str = 'eskf',
) -> None:

    if from_name not in LABEL_POINTS:
        sys.exit(f"[ERROR] '--from {from_name}' not in LABEL_POINTS. "
                 f"Available: {list(LABEL_POINTS)}")
    if to_name not in LABEL_POINTS:
        sys.exit(f"[ERROR] '--to {to_name}' not in LABEL_POINTS. "
                 f"Available: {list(LABEL_POINTS)}")

    start_pt = LABEL_POINTS[from_name]
    end_pt   = LABEL_POINTS[to_name]
    delta_m  = (end_pt[0] - start_pt[0], end_pt[1] - start_pt[1])

    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)

    # ── Pipeline components (same as reconstruct._run_live) ─────────────────
    from background.pipelines.cleaner.unpacker          import SerialStreamer
    from background.pipelines.cleaner.normalizer        import StreamNormalizer
    from background.pipelines.cleaner.time_alignment    import TimeAlignLayer
    from background.pipelines.preprocess.imu            import IMUPreprocessor
    from background.pipelines.preprocess.contact        import ContactStateDetector
    from background.pipelines.preprocess.uwb.range          import UWBRangePreprocessor
    from background.pipelines.preprocess.uwb.trilateration  import UWBSolver
    from background.pipelines.preprocess.uwb.position       import UWBPositionFilter
    from background.pipelines.fusion.learned_odometry   import OdometryDataCollector
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

    collector  = OdometryDataCollector(window_size=cfg.odometry.window_size)

    # ── State tracking ───────────────────────────────────────────────────────
    prev_stroke_active = False
    completed_strokes  = 0   # saved samples (gates the exit condition)
    attempt_count      = 0   # total pen-up events (includes too-short strokes)

    # ── Header ───────────────────────────────────────────────────────────────
    print("=" * 62)
    print(f"  [COLLECT] Odometry data — {from_name} → {to_name}")
    print(f"  Label  : Δx={delta_m[0]:+.3f}m  Δy={delta_m[1]:+.3f}m")
    print(f"  Target : {n_reps} repetitions")
    print(f"  Output : {out_path}")
    print(f"  Fusion : {fusion_mode}")
    print()
    stroke_dist = (delta_m[0]**2 + delta_m[1]**2) ** 0.5
    window_s    = cfg.odometry.window_size / 180.0   # 180 Hz IMU rate
    min_speed   = stroke_dist / window_s
    print("  HOW TO RECORD:")
    print(f"    1. Touch pen to [{from_name}] tape marker.")
    print(f"    2. Draw to [{to_name}] tape marker  (any path is fine).")
    print(f"    3. Lift the pen.  Repeat {n_reps} times.")
    print()
    print(f"  SPEED GUIDE for this pair:")
    print(f"    Stroke distance : {stroke_dist:.2f} m")
    print(f"    Window duration : {window_s:.2f} s  ({cfg.odometry.window_size} frames @ ~170 Hz)")
    print(f"    Min speed needed: {min_speed:.2f} m/s  (to finish within one window)")
    print(f"    Recommended     : aim for {min_speed*1.2:.2f}+ m/s  — brisk, not rushed")
    print()
    print("  Press Ctrl+C at any time to stop and save what was collected.")
    print("=" * 62)

    try:
        while completed_strokes < n_reps:
            raw = streamer.read_new_packets()
            if not raw:
                time.sleep(0.005)
                continue

            evs         = norm.normalize(raw)
            aligner.add_events(evs)
            sorted_evs  = aligner.get_all_sorted()
            aligner.clear()

            for ev in sorted_evs:

                # ── IMU branch ──────────────────────────────────────────────
                if ev['sensor'] == 'IMU':
                    p = imu_prep.process_one(ev)
                    if not p:
                        continue
                    s = contact.process_one(p)
                    fused = fusion.process_event(s)
                    if not fused:
                        continue

                    # Detect stroke state first so the gate below is correct.
                    stroke_active_now = bool(fused.get('stroke_active', False))

                    # Only accumulate frames while the pen is actively drawing.
                    # Air frames between strokes are intentionally excluded —
                    # pushing them would make the next stroke appear to start
                    # mid-way through the window.
                    if stroke_active_now:
                        frame = _assemble_frame(s)
                        if frame is not None:
                            collector.on_imu_packet(
                                imu_frames=[frame],
                                timestamps=[s.get('ts_hw', 0)],
                            )
                    if not prev_stroke_active and stroke_active_now:
                        # Pen-down edge: guarantee the buffer is clean before
                        # the new stroke begins accumulating.
                        collector.reset_stroke()

                    if prev_stroke_active and not stroke_active_now:
                        saved, n_frames, status = collector.reset_stroke()
                        attempt_count += 1
                        need     = cfg.odometry.window_size
                        max_ok   = int(need * collector.OVERSHOOT_LIMIT)
                        duration = n_frames / 180.0
                        if saved:
                            completed_strokes += 1
                            print(
                                f"  [stroke {completed_strokes:>3}/{n_reps}]"
                                f"  frames={n_frames:>4} ({duration:.1f}s)  SAVED"
                                f"  (attempt {attempt_count})"
                            )
                        elif status == 'TOO_SHORT':
                            print(
                                f"  [attempt {attempt_count:>3}]"
                                f"  frames={n_frames:>4} ({duration:.1f}s)  TOO SHORT"
                                f"  — draw for at least {need/180:.1f}s  (need {need} frames)"
                            )
                        else:  # TOO_LONG
                            print(
                                f"  [attempt {attempt_count:>3}]"
                                f"  frames={n_frames:>4} ({duration:.1f}s)  TOO LONG"
                                f"  — lift pen before {max_ok/180:.1f}s  (max {max_ok} frames)"
                            )
                    prev_stroke_active = stroke_active_now

                # ── UWB branch ──────────────────────────────────────────────
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

                        # Feed UWB fix — label will be overridden by use_known_points().
                        uwb_pos = np.array(
                            [fused.get('uwb_x', 0.0), fused.get('uwb_y', 0.0)],
                            dtype=np.float32,
                        )
                        collector.on_uwb_fix(
                            uwb_pos=uwb_pos,
                            is_drawing=bool(fused.get('stroke_active', False)),
                        )

    except KeyboardInterrupt:
        print("\n  [STOP] Ctrl+C — saving collected data...")

    finally:
        streamer.close()
        _save(collector, from_name, to_name, out_path)


def _save(
    collector: object,
    from_name: str,
    to_name:   str,
    out_path:  str,
) -> None:
    from background.pipelines.fusion.learned_odometry import OdometryDataCollector
    collector: OdometryDataCollector

    if len(collector) == 0:
        print("  [WARN] No samples collected — nothing saved.")
        return

    collector.use_known_points(
        start=LABEL_POINTS[from_name],
        end=LABEL_POINTS[to_name],
    )
    n = collector.save(out_path)
    delta = (
        LABEL_POINTS[to_name][0] - LABEL_POINTS[from_name][0],
        LABEL_POINTS[to_name][1] - LABEL_POINTS[from_name][1],
    )
    print()
    print("=" * 62)
    print(f"  Saved {n} samples → {out_path}")
    print(f"  Label applied : {from_name}→{to_name}  "
          f"Δx={delta[0]:+.3f}m  Δy={delta[1]:+.3f}m")
    print("=" * 62)


# ── CLI entry point ──────────────────────────────────────────────────────────

def main() -> None:
    os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

    p = argparse.ArgumentParser(
        description='Collect odometry training data using known board points.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--from', dest='from_name', required=True,
                   help=f'Start point. Choices: {list(LABEL_POINTS)}')
    p.add_argument('--to',   dest='to_name',   required=True,
                   help=f'End point.   Choices: {list(LABEL_POINTS)}')
    p.add_argument('--reps', type=int, default=25,
                   help='Number of stroke repetitions (default: 25)')
    p.add_argument('--out',  default=None,
                   help='Output .npz path (default: data/odometry/<FROM>_to_<TO>.npz)')
    p.add_argument('--fusion', default='eskf', choices=['eskf', 'complementary'])
    args = p.parse_args()

    out = args.out or os.path.join(
        'data', 'odometry', f'{args.from_name}_to_{args.to_name}.npz'
    )

    collect(
        from_name=args.from_name,
        to_name=args.to_name,
        n_reps=args.reps,
        out_path=out,
        fusion_mode=args.fusion,
    )


if __name__ == '__main__':
    main()
