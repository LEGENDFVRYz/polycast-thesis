"""
Multi-point UWB offset calibrator for vertical whiteboards.

Collects range measurements at several known board positions and computes
per-anchor offsets that minimise the trilateration error across all points.
Hand tremor averages out because the systematic antenna delay is constant
while tremor is zero-mean random noise.

Usage:
    python -m background.pipelines.preprocess.uwb.calibrate_offsets

You will be prompted to hold the pen at each of 5 positions in sequence.
Each position is held for 8 seconds. The tool then computes and prints
the optimal offsets — copy them into config.py range_offsets_m.

Board positions used (in metres from bottom-left corner):
    P0 — centre          (0.625, 0.620)
    P1 — top-left        (0.100, 1.140)
    P2 — top-right       (1.150, 1.140)
    P3 — bottom-right    (1.150, 0.100)
    P4 — bottom-left     (0.100, 0.100)

Mark these 5 positions with tape on the board before running.
"""

import time
import math
import sys
import numpy as np

# ── Pipeline imports ──────────────────────────────────────────────────────────
from background.pipelines.config import cfg
from background.pipelines.cleaner.unpacker   import SerialStreamer
from background.pipelines.cleaner.normalizer import StreamNormalizer

# ── Calibration positions (x, y) on board surface in metres ──────────────────
POSITIONS = [
    (0.625, 0.600),   # P0 centre (1.25/2, 1.20/2)
    (0.100, 1.100),   # P1 top-left
    (1.150, 1.100),   # P2 top-right
    (1.150, 0.100),   # P3 bottom-right
    (0.100, 0.100),   # P4 bottom-left
]

COLLECT_S  = 8     # seconds per position
DISCARD_S  = 2     # discard first N seconds (pen settling)


def _expected_range(pos_xy: tuple, anchor_xyz: tuple) -> float:
    px, py = pos_xy
    ax, ay, az = anchor_xyz
    return math.sqrt((px - ax)**2 + (py - ay)**2 + az**2)


def collect_position(streamer: SerialStreamer, norm: StreamNormalizer,
                     label: str, duration_s: float,
                     discard_s: float) -> dict[int, list[float]]:
    """Collect raw range samples for one pen position."""
    samples: dict[int, list[float]] = {i: [] for i in range(4)}
    t_start = time.time()
    t_accept = t_start + discard_s

    print(f"  Collecting '{label}' for {duration_s}s "
          f"(discarding first {discard_s}s)...", end='', flush=True)

    while time.time() - t_start < duration_s:
        raw = streamer.read_new_packets()
        if raw:
            evs = norm.normalize(raw)
            for ev in evs:
                if ev.get('sensor') != 'UWB':
                    continue
                dists = ev.get('dists')
                if not dists or len(dists) != 4:
                    continue
                if time.time() < t_accept:
                    continue   # still in settling window
                for i, d in enumerate(dists):
                    if d is not None and not math.isnan(d) and d > 0.05:
                        samples[i].append(float(d))
        time.sleep(0.002)

    counts = [len(samples[i]) for i in range(4)]
    print(f" done. Samples: {counts}")
    return samples


def compute_offsets(all_samples: list[dict], positions: list[tuple],
                    anchors: list[tuple]) -> np.ndarray:
    """
    Compute per-anchor offset that minimises mean(raw + offset - expected)
    across all positions. Simple least-squares: offset_i = mean(expected - raw).
    """
    offsets = np.zeros(4)
    counts  = np.zeros(4)

    for pos_idx, (samples, pos_xy) in enumerate(zip(all_samples, positions)):
        for i, anchor_xyz in enumerate(anchors):
            expected = _expected_range(pos_xy, anchor_xyz)
            raw_vals = samples[i]
            if not raw_vals:
                continue
            raw_mean = float(np.mean(raw_vals))
            offsets[i] += (expected - raw_mean)
            counts[i]  += 1

    # Average over all positions
    for i in range(4):
        if counts[i] > 0:
            offsets[i] /= counts[i]

    return offsets


def verify_offsets(all_samples: list[dict], positions: list[tuple],
                   anchors: list[tuple], offsets: np.ndarray) -> None:
    """Print per-position residual after applying computed offsets."""
    print("\n=== Verification — residuals after offset correction ===")
    for pos_idx, (samples, pos_xy) in enumerate(zip(all_samples, positions)):
        residuals = []
        for i, anchor_xyz in enumerate(anchors):
            expected = _expected_range(pos_xy, anchor_xyz)
            raw_vals = samples[i]
            if not raw_vals:
                continue
            corrected = float(np.mean(raw_vals)) + offsets[i]
            residuals.append(abs(corrected - expected) * 1000)
        if residuals:
            print(f"  P{pos_idx} {pos_xy}: mean_err={np.mean(residuals):.1f}mm  "
                  f"max_err={max(residuals):.1f}mm")


def main():
    anchors = [
        cfg.anchors.a0,
        cfg.anchors.a1,
        cfg.anchors.a2,
        cfg.anchors.a3,
    ]

    print("=" * 60)
    print("  UWB Multi-Point Offset Calibrator")
    print(f"  Board: {cfg.anchors.board_size_x}m x {cfg.anchors.board_size_y}m")
    print(f"  Anchor depth: {cfg.anchors.a0[2]}m")
    print("=" * 60)
    print()
    print("Mark these 5 positions on your board with tape:")
    for idx, (x, y) in enumerate(POSITIONS):
        label = ['Centre', 'Top-left', 'Top-right',
                 'Bottom-right', 'Bottom-left'][idx]
        print(f"  P{idx} {label:15s}: ({x:.3f}m, {y:.3f}m)")
    print()
    input("Press Enter when ready to start calibration...")
    print()

    streamer = SerialStreamer(port=cfg.serial.port, baud=cfg.serial.baud)
    norm     = StreamNormalizer()

    all_samples = []
    labels = ['Centre', 'Top-left', 'Top-right', 'Bottom-right', 'Bottom-left']

    for idx, (pos_xy, label) in enumerate(zip(POSITIONS, labels)):
        print(f"\n[{idx+1}/{len(POSITIONS)}] Hold pen tip at {label} "
              f"({pos_xy[0]:.3f}m, {pos_xy[1]:.3f}m)")
        input("  Press Enter when pen is in position...")
        samples = collect_position(streamer, norm, label, COLLECT_S, DISCARD_S)
        all_samples.append(samples)

    streamer.close()

    # Compute offsets
    offsets = compute_offsets(all_samples, POSITIONS, anchors)

    print("\n" + "=" * 60)
    print("  COMPUTED OFFSETS")
    print("=" * 60)
    for i, o in enumerate(offsets):
        print(f"  A{i}: {o:+.4f}m")

    print()
    print("  Copy this line into config.py → UWBConfig.range_offsets_m:")
    vals = ", ".join(f"{o:.4f}" for o in offsets)
    print(f"  range_offsets_m: tuple = ({vals})")

    verify_offsets(all_samples, POSITIONS, anchors, offsets)

    print()
    print("  Previous offsets were:", cfg.uwb.range_offsets_m)


if __name__ == "__main__":
    main()
