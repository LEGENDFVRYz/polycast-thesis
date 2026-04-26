"""
layer10_legibility.py -- offline-rendered note legibility (Tier C).

Renders each dataset's offline-smoothed (NoteSmoother) per-stroke output
as a binary PNG. If pytesseract is available, runs OCR and reports
character count / non-empty status. Without ground-truth labels CER/WER
cannot be computed automatically; the layer passes whenever it produces
a non-empty render and (optionally) any OCR output at all.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from _common import (ensure_out, replay, LayerResult, list_datasets,
                     DATASET_DIR)
from preprocessor  import UWBPreprocessor
from ekf_fusion    import AsyncEKFFusionEngine
from note_smoother import NoteSmoother
from config        import ANCHORS, UWB_OFFSETS


LAYER = 'layer10_legibility'


def analyse(csv_path: Path) -> LayerResult:
    engine = AsyncEKFFusionEngine()
    engine.ekf.record_history = True
    pre = UWBPreprocessor(offsets=tuple(UWB_OFFSETS))

    is_writing_stream: list[bool] = []
    history_idx_at_step: list[int] = []   # parallel to history list

    for pkt in replay(csv_path):
        if pkt['type'] == 'imu':
            _, _, w = engine.process_imu(pkt)
            if engine.ekf.initialized:
                is_writing_stream.append(bool(w))
                history_idx_at_step.append(len(engine.ekf._history) - 1)
        else:
            _, weights, ekf_dists = pre.process(*pkt['dists'])
            engine.process_uwb(ekf_dists, weights, ts=pkt.get('ts'))

    history = engine.ekf._history
    if not history:
        return LayerResult(LAYER, csv_path.stem, False,
                           {'strokes': 0}, ['EKF never initialised'])

    # Slice history into per-stroke segments using is_writing_stream.
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

    smoother = NoteSmoother()
    smoothed = [smoother.smooth_stroke(s) for s in strokes_hist if len(s) >= 2]

    out_dir = ensure_out(LAYER)
    fig, ax = plt.subplots(figsize=(6, 6))
    for s in smoothed:
        ax.plot(s[:, 0], s[:, 1], color='black', lw=2.0)
    ax.set_xlim(-0.05, float(np.max(ANCHORS[:, 0])) + 0.05)
    ax.set_ylim(-0.05, float(np.max(ANCHORS[:, 1])) + 0.05)
    ax.set_aspect('equal'); ax.axis('off')
    plot = out_dir / f'{csv_path.stem}.png'
    fig.savefig(plot, dpi=200, bbox_inches='tight', facecolor='white')
    plt.close(fig)

    metrics = {'strokes_rendered': len(smoothed)}
    notes: list[str] = []
    try:
        import pytesseract                          # type: ignore
        from PIL import Image                       # type: ignore
        text = pytesseract.image_to_string(Image.open(plot))
        metrics['ocr_chars']       = len(text.strip())
        metrics['ocr_words']       = len(text.split())
        metrics['ocr_text_sample'] = text.strip()[:60]
    except Exception as e:
        notes.append(f'OCR skipped: {e!r}')
        metrics['ocr_chars'] = -1

    passed = metrics['strokes_rendered'] > 0
    return LayerResult(LAYER, csv_path.stem, passed, metrics, notes,
                       plot=str(plot.relative_to(Path(__file__).resolve().parent)))


def run_all() -> list[LayerResult]:
    return [analyse(p) for p in list_datasets()]


if __name__ == '__main__':
    for p in list_datasets():
        r = analyse(p)
        print(r.summary_line())
        for k, v in r.metrics.items():
            print(f'    {k:24s} {v}')
