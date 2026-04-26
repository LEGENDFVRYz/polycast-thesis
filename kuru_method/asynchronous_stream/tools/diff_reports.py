"""
diff_reports.py — compare two verification/out/report.md files.

Phase 8 helper: after running the baseline verifier, applying calibration
changes, and re-running, this script tabulates per-dataset deltas in
the headline metrics so we can decide if the change helped or hurt.

Usage
-----
    python tools/diff_reports.py baseline.md post.md
    python tools/diff_reports.py baseline.md post.md --layer layer6_ekf_end_to_end

Each report.md is parsed into a {(layer, dataset) -> {metric: value}} map.
The diff prints, per layer, the metrics that changed and by how much.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from collections import defaultdict


_LAYER_RE = re.compile(r'^### (\w+)\s+\((.+?)\)\s*\[(\w+)\]')
_BULLET_RE = re.compile(r'^\s*-\s+\*\*([^*]+)\*\*:\s*(.+)$')


def parse_report(path: Path) -> dict:
    """Return {(layer, dataset): {metric: value}}.

    A v.lower-cased pass/fail status is included as metric '__status__'.
    """
    out: dict[tuple[str, str], dict] = {}
    cur_key = None
    text = path.read_text(encoding='utf-8', errors='replace')
    for line in text.splitlines():
        m = _LAYER_RE.match(line)
        if m:
            layer, dataset, status = m.group(1), m.group(2), m.group(3)
            cur_key = (layer, dataset)
            out[cur_key] = {'__status__': status.lower()}
            continue
        if cur_key is None:
            continue
        m = _BULLET_RE.match(line)
        if m:
            metric, raw = m.group(1).strip(), m.group(2).strip()
            try:
                v = float(raw)
            except ValueError:
                v = raw
            out[cur_key][metric] = v
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('baseline')
    ap.add_argument('post')
    ap.add_argument('--layer', default=None,
                    help='only show diffs for this layer (e.g. layer6_ekf_end_to_end)')
    args = ap.parse_args()

    a = parse_report(Path(args.baseline))
    b = parse_report(Path(args.post))

    keys = sorted(set(a.keys()) | set(b.keys()))
    if args.layer:
        keys = [k for k in keys if k[0] == args.layer]

    # Group by layer for readable output
    by_layer: dict[str, list] = defaultdict(list)
    for k in keys:
        by_layer[k[0]].append(k[1])

    for layer, datasets in by_layer.items():
        print(f'\n## {layer}')
        for ds in datasets:
            ka = a.get((layer, ds), {})
            kb = b.get((layer, ds), {})
            metric_keys = sorted(set(ka.keys()) | set(kb.keys()))
            row_diffs = []
            for mk in metric_keys:
                if mk == '__status__':
                    sa, sb = ka.get(mk, '?'), kb.get(mk, '?')
                    if sa != sb:
                        row_diffs.append((mk, sa, sb, '!'))
                    continue
                va, vb = ka.get(mk), kb.get(mk)
                if isinstance(va, float) and isinstance(vb, float):
                    delta = vb - va
                    if abs(delta) > max(1e-9, 0.001 * max(abs(va), abs(vb))):
                        row_diffs.append((mk, va, vb, delta))
                elif va != vb:
                    row_diffs.append((mk, va, vb, '!'))
            if row_diffs:
                print(f'  {ds}')
                for m, va, vb, delta in row_diffs:
                    if isinstance(delta, float):
                        # Highlight Layer-6/8/9 metrics where lower is better
                        better = ''
                        lower_is_better = ('rmse' in m.lower() or 'error' in m.lower()
                                           or 'p95' in m.lower() or 'median' in m.lower()
                                           or 'false' in m.lower() or 'reject' in m.lower())
                        higher_is_better = ('accept' in m.lower())
                        if lower_is_better:
                            better = '  ↓ better' if delta < 0 else '  ↑ worse'
                        elif higher_is_better:
                            better = '  ↑ better' if delta > 0 else '  ↓ worse'
                        print(f'    {m:32s} {va:10.4g} -> {vb:10.4g}  '
                              f'(Δ {delta:+.4g}){better}')
                    else:
                        print(f'    {m:32s} {va} -> {vb}  ({delta})')


if __name__ == '__main__':
    main()
