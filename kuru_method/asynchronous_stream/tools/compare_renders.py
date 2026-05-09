"""
compare_renders.py — three-panel diagnostic for the live-vs-Layer-10 gap.

Replays one CSV through AsyncEKFFusionEngine and produces a single PNG with:
  (1) "before"  — pre-fix main_ekf.py behaviour:
                  • per-stroke snap-to-RTS (Pattern A) overwriting tip XY
                  • full-history green RTS overlay (legacy _run_rts_smoother)
                  • MAX_TRAIL=1000 trim on draw_x/draw_y
                  • final in-progress stroke never snapped at EOF
  (2) "after"   — current main_ekf.py behaviour after the three landed edits:
                  • Pattern A snap-to-RTS on every fall edge
                  • per-stroke green RTS overlay (Layer-10-style slicing)
                  • no MAX_TRAIL trim in CSV mode
                  • EOF snap for the in-progress stroke
                  Tip projection is still applied (the only difference vs
                  Layer 10), which lets us verify that tip-vs-tag is
                  visually irrelevant for the current datasets.
  (3) "layer10" — verification/layer10_legibility.py reference render:
                  per-stroke RTS over tag XY, no overlay, no trim, no tip
                  projection.

Outputs to verification/out/compare_renders/<dataset_stem>.png.
This is a read-only diagnostic — it does not modify any algorithm code.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

_ASYNC = Path(__file__).resolve().parent.parent
if str(_ASYNC) not in sys.path:
    sys.path.insert(0, str(_ASYNC))

from kuru_method.asynchronous_stream.data_parser   import AsyncDataParser
from kuru_method.asynchronous_stream.preprocessor  import UWBPreprocessor
from kuru_method.asynchronous_stream.ekf_fusion    import AsyncEKFFusionEngine
from kuru_method.asynchronous_stream.note_smoother import NoteSmoother
from kuru_method.asynchronous_stream.trail_smoother import OnlineTrailSmoother
from kuru_method.asynchronous_stream.config        import ANCHORS, UWB_OFFSETS

TRAIL_SUBSAMPLE = 3
MAX_TRAIL       = 1000
MARGIN          = 0.05


def _trim(lst, maxlen):
    if len(lst) > maxlen:
        del lst[:len(lst) - maxlen]


def _replay(csv_path: Path):
    """Run the engine over a dataset, producing the artefacts needed by the
    three panels:
      - BEFORE: tip-projected, trimmed, full-history green overlay,
                no EOF snap of the in-progress stroke.
      - AFTER : tip-projected, NO trim, per-stroke green overlay,
                Pattern A on every fall edge, plus EOF snap of the
                final in-progress stroke.
      - LAYER10: per-stroke RTS over tag XY, clean axes.
    """

    engine = AsyncEKFFusionEngine()
    engine.ekf.record_history = True
    pre    = UWBPreprocessor(offsets=tuple(UWB_OFFSETS))
    parser = AsyncDataParser(csv_path=str(csv_path))
    if not parser.connect():
        raise RuntimeError(f'cannot open {csv_path}')

    smoother_b   = OnlineTrailSmoother()   # BEFORE
    smoother_a   = OnlineTrailSmoother()   # AFTER
    note_smooth  = NoteSmoother()

    before_x, before_y = [], []
    after_x,  after_y  = [], []

    # Per-IMU streams (Layer 10-style, also used by AFTER's per-stroke green).
    is_writing_stream:   list[bool] = []
    history_idx_at_step: list[int]  = []

    _imu_sub = 0
    _prev_w  = False
    _b_hist_start = None; _b_draw_start = None
    _a_hist_start = None; _a_draw_start = None

    while True:
        pkt = parser.get_packet()
        if pkt == 'EOF':
            break
        if pkt is None:
            continue

        if pkt['type'] == 'imu':
            pos, vel, is_writing = engine.process_imu(pkt)
            if pos is None:
                continue

            tip = engine.tip_position
            draw_pt = tip if tip is not None else pos

            if engine.ekf.initialized:
                is_writing_stream.append(bool(is_writing))
                history_idx_at_step.append(len(engine.ekf._history) - 1)

            _imu_sub += 1
            if _imu_sub >= TRAIL_SUBSAMPLE:
                _imu_sub = 0
                rising  = is_writing and not _prev_w
                falling = (not is_writing) and _prev_w

                if rising:
                    _b_hist_start = len(engine.ekf._history)
                    _b_draw_start = len(before_x)
                    _a_hist_start = len(engine.ekf._history)
                    _a_draw_start = len(after_x)

                if is_writing:
                    smoother_b.push(draw_pt[0], draw_pt[1])
                    sxb, syb = smoother_b.get()
                    before_x.append(sxb); before_y.append(syb)
                    _trim(before_x, MAX_TRAIL); _trim(before_y, MAX_TRAIL)

                    smoother_a.push(draw_pt[0], draw_pt[1])
                    sxa, sya = smoother_a.get()
                    after_x.append(sxa); after_y.append(sya)

                else:
                    if falling:
                        # BEFORE: same Pattern A snap on fall edge
                        if _b_hist_start is not None and 0 <= _b_hist_start < len(engine.ekf._history):
                            slc = engine.ekf._history[_b_hist_start:]
                            if len(slc) >= 2:
                                sm  = note_smooth.smooth_stroke(slc)
                                sub = sm[::TRAIL_SUBSAMPLE]
                                n_replace = len(before_x) - _b_draw_start
                                n_use = min(len(sub), n_replace)
                                for i in range(n_use):
                                    before_x[_b_draw_start + i] = float(sub[i, 0])
                                    before_y[_b_draw_start + i] = float(sub[i, 1])
                        # AFTER: same Pattern A snap, no trim, no full-history overlay
                        if _a_hist_start is not None and 0 <= _a_hist_start < len(engine.ekf._history):
                            slc = engine.ekf._history[_a_hist_start:]
                            if len(slc) >= 2:
                                sm  = note_smooth.smooth_stroke(slc)
                                sub = sm[::TRAIL_SUBSAMPLE]
                                n_replace = len(after_x) - _a_draw_start
                                n_use = min(len(sub), n_replace)
                                for i in range(n_use):
                                    after_x[_a_draw_start + i] = float(sub[i, 0])
                                    after_y[_a_draw_start + i] = float(sub[i, 1])

                        before_x.append(float('nan')); before_y.append(float('nan'))
                        after_x.append(float('nan'));  after_y.append(float('nan'))
                        smoother_b.reset(); smoother_a.reset()
                        _b_hist_start = None; _b_draw_start = None
                        _a_hist_start = None; _a_draw_start = None

                _prev_w = is_writing

        else:
            _, weights, ekf_dists = pre.process(*pkt['dists'])
            engine.process_uwb(ekf_dists, weights, ts=pkt.get('ts'))

    parser.close()

    # AFTER (edit 2): if the CSV ended mid-stroke, snap that final stroke
    # so the displayed trail matches what main_ekf now does at EOF.
    if _a_hist_start is not None and 0 <= _a_hist_start < len(engine.ekf._history):
        slc = engine.ekf._history[_a_hist_start:]
        if len(slc) >= 2:
            sm  = note_smooth.smooth_stroke(slc)
            sub = sm[::TRAIL_SUBSAMPLE]
            n_replace = len(after_x) - _a_draw_start
            n_use = min(len(sub), n_replace)
            for i in range(n_use):
                after_x[_a_draw_start + i] = float(sub[i, 0])
                after_y[_a_draw_start + i] = float(sub[i, 1])

    # BEFORE: full-history green RTS overlay (the legacy _run_rts_smoother).
    green_full = engine.ekf.rts_smooth() if engine.ekf._history else np.zeros((0, 2))

    # Layer 10 logic: slice history by is_writing, smooth per stroke.
    history = engine.ekf._history
    strokes_hist: list[list[dict]] = []
    cur: list[dict] = []
    for w, idx in zip(is_writing_stream, history_idx_at_step):
        if 0 <= idx < len(history):
            if w:
                cur.append(history[idx])
            elif cur:
                strokes_hist.append(cur); cur = []
    if cur:
        strokes_hist.append(cur)

    layer10_strokes = [note_smooth.smooth_stroke(s)
                       for s in strokes_hist if len(s) >= 2]
    # AFTER's green overlay reuses the per-stroke RTS — same content as
    # Layer 10, just rendered as a green overlay on the live axes.
    after_green_strokes = layer10_strokes

    return (before_x, before_y, green_full,
            after_x,  after_y,  after_green_strokes,
            layer10_strokes)


def _draw_anchors(ax):
    ax.scatter(ANCHORS[:, 0], ANCHORS[:, 1], s=80, c='red', marker='s', zorder=10)
    for i, a in enumerate(ANCHORS):
        ax.annotate(f'A{i}', (a[0], a[1]),
                    textcoords='offset points', xytext=(0, 8),
                    ha='center', fontsize=8, color='red')


def render(csv_path: Path, out_dir: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    (before_x, before_y, green_full,
     after_x,  after_y,  after_green,
     layer10_strokes) = _replay(csv_path)

    fig, axes = plt.subplots(1, 3, figsize=(15, 5.5))

    titles = [
        '(1) BEFORE — pre-fix main_ekf\n(tip + green full-history RTS + MAX_TRAIL=1000)',
        '(2) AFTER — current main_ekf\n(tip + Pattern A + per-stroke green + EOF snap, no trim)',
        '(3) Layer 10 reference\n(tag + per-stroke RTS, clean axes)',
    ]

    # Panel 1 — BEFORE
    ax = axes[0]
    if len(green_full):
        ax.plot(green_full[:, 0], green_full[:, 1], '-',
                color='limegreen', lw=1.2, alpha=0.8,
                label='RTS smoothed (full history)')
    ax.plot(before_x, before_y, '-', color='royalblue', lw=1.6,
            label='Pen tip (writing)')
    ax.legend(fontsize=7, loc='lower left')

    # Panel 2 — AFTER
    ax = axes[1]
    first = True
    for s in after_green:
        ax.plot(s[:, 0], s[:, 1], '-', color='limegreen', lw=1.2, alpha=0.85,
                label='RTS smoothed (per-stroke)' if first else None)
        first = False
    ax.plot(after_x, after_y, '-', color='royalblue', lw=1.6,
            label='Pen tip (writing)')
    ax.legend(fontsize=7, loc='lower left')

    # Panel 3 — Layer 10
    ax = axes[2]
    for s in layer10_strokes:
        ax.plot(s[:, 0], s[:, 1], '-', color='black', lw=2.0)

    for ax, t in zip(axes, titles):
        _draw_anchors(ax)
        ax.set_xlim(-MARGIN, float(np.max(ANCHORS[:, 0])) + MARGIN)
        ax.set_ylim(-MARGIN, float(np.max(ANCHORS[:, 1])) + MARGIN)
        ax.set_aspect('equal')
        ax.set_title(t, fontsize=10)
        ax.grid(True, alpha=0.25)

    fig.suptitle(f'Render comparison — {csv_path.stem}', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out_path = out_dir / f'{csv_path.stem}.png'
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    return out_path


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('datasets', nargs='*', default=['ABC_b1'],
                    help='dataset stems (without .csv)')
    args = ap.parse_args()

    dataset_dir = _ASYNC / 'datasets_str_50hz'
    out_dir = _ASYNC / 'verification' / 'out' / 'compare_renders'

    for stem in args.datasets:
        csv_path = dataset_dir / f'{stem}.csv'
        if not csv_path.exists():
            print(f'  [skip] {csv_path}')
            continue
        out = render(csv_path, out_dir)
        print(f'  {stem} -> {out}')


if __name__ == '__main__':
    main()
