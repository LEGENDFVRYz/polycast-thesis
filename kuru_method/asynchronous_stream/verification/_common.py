"""
_common.py -- shared replay, plotting, and result helpers for the verification layer.
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Iterator, Optional

import matplotlib
matplotlib.use('Agg')   # headless backend -- layer scripts save PNGs, never show

# Make parent (asynchronous_stream) importable when running layer scripts directly.
_PARENT = Path(__file__).resolve().parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from data_parser import AsyncDataParser   # noqa: E402
from config      import ANCHORS            # noqa: E402


# -- Paths ----------------------------------------------------------------
ASYNC_DIR    = _PARENT
DATASET_DIR  = ASYNC_DIR / 'datasets_a3_ls'
OUT_DIR      = Path(__file__).resolve().parent / 'out'


def ensure_out(layer: str) -> Path:
    p = OUT_DIR / layer
    p.mkdir(parents=True, exist_ok=True)
    return p


# -- Dataset discovery ----------------------------------------------------
def list_datasets(predicate=None) -> list[Path]:
    """Return sorted list of .csv files under datasets_a3_ls/ matching predicate."""
    files = sorted(DATASET_DIR.glob('*.csv'))
    if predicate is not None:
        files = [f for f in files if predicate(f.stem)]
    return files


def stems_matching(*needles: str) -> list[str]:
    """Return dataset stems whose name (case-insensitive) contains any needle."""
    out = []
    for f in DATASET_DIR.glob('*.csv'):
        low = f.stem.lower()
        if any(n.lower() in low for n in needles):
            out.append(f.stem)
    return sorted(out)


# -- Replay ---------------------------------------------------------------
def replay(csv_path: Path) -> Iterator[dict]:
    """Yield every parsed packet dict from a CSV, in file order."""
    parser = AsyncDataParser(csv_path=str(csv_path))
    if not parser.connect():
        return
    try:
        while True:
            pkt = parser.get_packet()
            if pkt == 'EOF':
                return
            if pkt is None:
                continue
            yield pkt
    finally:
        parser.close()


def collect(csv_path: Path) -> tuple[list[dict], list[dict]]:
    """Return (imu_packets, uwb_packets) for a single CSV."""
    imu, uwb = [], []
    for pkt in replay(csv_path):
        (imu if pkt['type'] == 'imu' else uwb).append(pkt)
    return imu, uwb


# -- Results --------------------------------------------------------------
@dataclass
class LayerResult:
    layer:    str
    dataset:  str
    passed:   bool
    metrics:  dict = field(default_factory=dict)
    notes:    list[str] = field(default_factory=list)
    plot:     Optional[str] = None      # path to saved figure, relative to verification/

    def summary_line(self) -> str:
        flag = 'PASS' if self.passed else 'FAIL'
        return f'[{flag}] {self.layer:28s} {self.dataset}'


def write_report(results: Iterable[LayerResult], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    results = list(results)
    lines: list[str] = []
    lines.append('# Async-stream verification report\n')
    lines.append(f'Datasets root: `{DATASET_DIR}`\n')
    lines.append('')

    by_dataset: dict[str, list[LayerResult]] = {}
    for r in results:
        by_dataset.setdefault(r.dataset, []).append(r)

    lines.append('## Per-dataset summary')
    lines.append('')
    lines.append('| Dataset | First fail | Layers run |')
    lines.append('|---|---|---|')
    for ds, rs in sorted(by_dataset.items()):
        fail = next((r.layer for r in rs if not r.passed), '--')
        lines.append(f'| {ds} | {fail} | {len(rs)} |')
    lines.append('')

    lines.append('## Detailed results')
    lines.append('')
    for r in results:
        lines.append(f'### {r.summary_line()}')
        for k, v in r.metrics.items():
            if isinstance(v, float):
                lines.append(f'- **{k}**: {v:.4g}')
            else:
                lines.append(f'- **{k}**: {v}')
        for n in r.notes:
            lines.append(f'> {n}')
        if r.plot:
            lines.append(f'![{r.layer} {r.dataset}]({r.plot})')
        lines.append('')

    path.write_text('\n'.join(lines), encoding='utf-8')


# -- Plot helpers ---------------------------------------------------------
def draw_board(ax, margin: float = 0.30) -> None:
    """Draw anchors + board outline on the given axes."""
    ax.scatter(ANCHORS[:, 0], ANCHORS[:, 1],
               s=120, c='red', marker='s', zorder=10)
    for i, a in enumerate(ANCHORS):
        ax.annotate(f'A{i}', (a[0], a[1]),
                    textcoords='offset points', xytext=(0, 10),
                    ha='center', fontsize=9, color='red',
                    fontweight='bold')
    xmin = float(ANCHORS[:, 0].min()) - margin
    xmax = float(ANCHORS[:, 0].max()) + margin
    ymin = float(ANCHORS[:, 1].min()) - margin
    ymax = float(ANCHORS[:, 1].max()) + margin
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.3)
