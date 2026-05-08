"""trace_quality_diagnostics.py — PolyCast fused-trace quality audit.

This module is intentionally diagnostic-only.  It does not alter EKF,
contact, smoothing, or recognition behavior.  It measures whether the trace
that reaches the recognition preprocessor is faithful to the live display and
where the remaining distortion likely enters the pipeline.

Main checks:
  * tag ↔ tip offset statistics       (tip-offset / heading sanity)
  * blue ↔ cyan statistics            (recognition trace post-filter movement)
  * black/extraction ↔ cyan statistics (export/extraction faithfulness)
  * inter-stroke gaps and jumps       (false lifts / missed contact sections)
  * stroke shape/orientation stats    (generic geometry, no letter assumptions)

The CLI mode can analyze a recognition-export JSON by itself.  When used from
main_ekf.py it has richer live samples, so the exported report also includes
blue/purple/cyan consistency metrics.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import json
import math
import time
from typing import Any, Iterable, Optional

import numpy as np


@dataclass
class DistanceStats:
    count: int = 0
    mean_m: float = 0.0
    median_m: float = 0.0
    p90_m: float = 0.0
    p95_m: float = 0.0
    max_m: float = 0.0
    rms_m: float = 0.0

    def as_dict(self) -> dict[str, float | int]:
        return {
            'count': self.count,
            'mean_m': self.mean_m,
            'median_m': self.median_m,
            'p90_m': self.p90_m,
            'p95_m': self.p95_m,
            'max_m': self.max_m,
            'rms_m': self.rms_m,
        }


def _finite_xy(xy: Any) -> Optional[np.ndarray]:
    try:
        arr = np.asarray(xy, dtype=float).reshape(-1)
        if len(arr) < 2:
            return None
        arr = arr[:2]
        if not np.all(np.isfinite(arr)):
            return None
        return arr.astype(float)
    except Exception:
        return None


def _stats(values: Iterable[float]) -> DistanceStats:
    arr = np.asarray([float(v) for v in values if math.isfinite(float(v))], dtype=float)
    if len(arr) == 0:
        return DistanceStats()
    return DistanceStats(
        count=int(len(arr)),
        mean_m=float(np.mean(arr)),
        median_m=float(np.median(arr)),
        p90_m=float(np.percentile(arr, 90)),
        p95_m=float(np.percentile(arr, 95)),
        max_m=float(np.max(arr)),
        rms_m=float(math.sqrt(np.mean(arr * arr))),
    )


def _path_length(points: np.ndarray) -> float:
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def _bbox(points: np.ndarray) -> list[float]:
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return [float('nan')] * 4
    mn = np.nanmin(pts, axis=0)
    mx = np.nanmax(pts, axis=0)
    return [float(mn[0]), float(mn[1]), float(mx[0]), float(mx[1])]


def _max_step(points: np.ndarray) -> float:
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) < 2:
        return 0.0
    return float(np.max(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def _median_step(points: np.ndarray) -> float:
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) < 2:
        return 0.0
    return float(np.median(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def _principal_angle_deg(points: np.ndarray) -> float:
    """Principal-axis angle in degrees from +X, range [-180, 180)."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) < 2:
        return float('nan')
    centered = pts - np.mean(pts, axis=0)
    if float(np.max(np.linalg.norm(centered, axis=1))) <= 1e-9:
        return float('nan')
    try:
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        v = vh[0]
        return float(math.degrees(math.atan2(v[1], v[0])))
    except Exception:
        return float('nan')


def _distance_point_to_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-12:
        return float(np.linalg.norm(p - a))
    t = max(0.0, min(1.0, float(np.dot(p - a, ab) / denom)))
    proj = a + t * ab
    return float(np.linalg.norm(p - proj))


def _distance_to_polyline(points: np.ndarray, polyline: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    poly = np.asarray(polyline, dtype=float).reshape(-1, 2)
    if len(pts) == 0 or len(poly) == 0:
        return np.asarray([], dtype=float)
    if len(poly) == 1:
        return np.linalg.norm(pts - poly[0], axis=1)
    out = []
    seg_a = poly[:-1]
    seg_b = poly[1:]
    for p in pts:
        dmin = min(_distance_point_to_segment(p, a, b) for a, b in zip(seg_a, seg_b))
        out.append(dmin)
    return np.asarray(out, dtype=float)


class TraceQualityDiagnostics:
    """Collect and export trace-quality metrics.

    Use ``add_tip_tag`` and ``add_recognition_pair`` from main_ekf.py.  At
    export time pass the NormalizedStrokeExtractor instance to ``export``.
    """

    def __init__(self, max_samples: int = 25000) -> None:
        self.max_samples = int(max_samples)
        self.tip_tag_dist_m: list[float] = []
        self.rec_blue_dist_m: list[float] = []
        self.blue_points: list[list[float]] = []
        self.tag_points: list[list[float]] = []
        self.rec_points: list[list[float]] = []
        self.last_export_paths: dict[str, str] = {}

    def clear(self) -> None:
        self.tip_tag_dist_m.clear()
        self.rec_blue_dist_m.clear()
        self.blue_points.clear()
        self.tag_points.clear()
        self.rec_points.clear()
        self.last_export_paths = {}

    def _trim(self) -> None:
        n = self.max_samples
        if n <= 0:
            return
        for name in ('tip_tag_dist_m', 'rec_blue_dist_m', 'blue_points', 'tag_points', 'rec_points'):
            seq = getattr(self, name)
            if len(seq) > n:
                del seq[:len(seq) - n]

    def add_tip_tag(self, tip_xy: Any, tag_xy: Any, ts: Any = None) -> None:
        tip = _finite_xy(tip_xy)
        tag = _finite_xy(tag_xy)
        if tip is None or tag is None:
            return
        self.blue_points.append([float(tip[0]), float(tip[1])])
        self.tag_points.append([float(tag[0]), float(tag[1])])
        self.tip_tag_dist_m.append(float(np.linalg.norm(tip - tag)))
        self._trim()

    def add_recognition_pair(self, blue_xy: Any, rec_xy: Any, ts: Any = None) -> None:
        blue = _finite_xy(blue_xy)
        rec = _finite_xy(rec_xy)
        if blue is None or rec is None:
            return
        self.rec_points.append([float(rec[0]), float(rec[1])])
        self.rec_blue_dist_m.append(float(np.linalg.norm(rec - blue)))
        self._trim()

    def quick_summary(self) -> str:
        tt = _stats(self.tip_tag_dist_m)
        rb = _stats(self.rec_blue_dist_m)
        return (f'traceQ tip-tag μ/max={tt.mean_m*100:.1f}/{tt.max_m*100:.1f}cm '
                f'rec-blue μ/max={rb.mean_m*100:.1f}/{rb.max_m*100:.1f}cm')

    def analyze(self, normalized_strokes: Any = None, recognition_json_path: str | Path | None = None) -> dict[str, Any]:
        strokes = []
        line_groups = []
        character_groups = []
        if recognition_json_path is not None:
            payload = json.loads(Path(recognition_json_path).read_text(encoding='utf-8'))
            strokes = payload.get('strokes', [])
            line_groups = payload.get('line_groups', [])
            character_groups = payload.get('character_groups', [])
        elif normalized_strokes is not None:
            try:
                strokes = [self._stroke_to_dict(s) for s in normalized_strokes.strokes]
                line_groups = normalized_strokes.line_groups()
                character_groups = normalized_strokes.character_groups()
            except Exception:
                strokes = []
                line_groups = []
                character_groups = []

        line_map: dict[int, int] = {}
        for g in line_groups or []:
            for sid in g.get('stroke_ids', []):
                line_map[int(sid)] = int(g.get('line_id', 0))
        char_map: dict[int, int] = {}
        for ch in character_groups or []:
            for sid in ch.get('stroke_ids', []):
                char_map[int(sid)] = int(ch.get('char_id', 0))

        stroke_rows = []
        for s in strokes:
            sid = int(s.get('stroke_id', 0))
            pts = np.asarray(s.get('resampled_points') or s.get('raw_points') or [], dtype=float).reshape(-1, 2)
            if len(pts) == 0:
                continue
            box = s.get('raw_bbox_m') or _bbox(pts)
            x0, y0, x1, y1 = [float(v) for v in box]
            w = max(0.0, x1 - x0)
            h = max(0.0, y1 - y0)
            path_m = float(s.get('raw_path_m') or _path_length(pts))
            chord_m = float(np.linalg.norm(pts[-1] - pts[0])) if len(pts) >= 2 else 0.0
            diag_m = float(math.hypot(w, h))
            angle = _principal_angle_deg(pts)
            vertical_slant = abs(90.0 - abs(angle)) if math.isfinite(angle) else float('nan')
            stroke_rows.append({
                'stroke_id': sid,
                'line_id': int(line_map.get(sid, 0)),
                'char_id': int(char_map.get(sid, 0)),
                'point_count': int(len(pts)),
                'path_m': path_m,
                'bbox_w_m': w,
                'bbox_h_m': h,
                'bbox_diag_m': diag_m,
                'chord_m': chord_m,
                'straightness_chord_over_path': float(chord_m / max(path_m, 1e-9)),
                'path_over_bbox_diag': float(path_m / max(diag_m, 1e-9)),
                'max_step_m': _max_step(pts),
                'median_step_m': _median_step(pts),
                'principal_angle_deg': angle,
                'vertical_slant_deg_if_tall': vertical_slant if h > 1.8 * max(w, 1e-9) and h > 0.045 else float('nan'),
            })

        gap_rows = []
        sorted_strokes = sorted(strokes, key=lambda s: int(s.get('stroke_id', 0)))
        for prev, cur in zip(sorted_strokes[:-1], sorted_strokes[1:]):
            prev_pts = np.asarray(prev.get('resampled_points') or prev.get('raw_points') or [], dtype=float).reshape(-1, 2)
            cur_pts = np.asarray(cur.get('resampled_points') or cur.get('raw_points') or [], dtype=float).reshape(-1, 2)
            if len(prev_pts) == 0 or len(cur_pts) == 0:
                continue
            sid0 = int(prev.get('stroke_id', 0)); sid1 = int(cur.get('stroke_id', 0))
            dxy = cur_pts[0] - prev_pts[-1]
            gap_rows.append({
                'from_stroke_id': sid0,
                'to_stroke_id': sid1,
                'gap_m': float(np.linalg.norm(dxy)),
                'dx_m': float(dxy[0]),
                'dy_m': float(dxy[1]),
                'same_line': bool(line_map.get(sid0, -1) == line_map.get(sid1, -2)),
                'same_char': bool(char_map.get(sid0, -1) == char_map.get(sid1, -2)),
            })

        # Extraction/cyan faithfulness: compare exported/extracted resampled points
        # to the cyan recognition polyline if main_ekf provided it.
        extraction_to_rec_dists: list[float] = []
        if len(self.rec_points) >= 2:
            rec_poly = np.asarray(self.rec_points, dtype=float).reshape(-1, 2)
            for s in strokes:
                pts = np.asarray(s.get('resampled_points') or [], dtype=float).reshape(-1, 2)
                if len(pts):
                    extraction_to_rec_dists.extend(_distance_to_polyline(pts, rec_poly).astype(float).tolist())

        tall_slants = [r['vertical_slant_deg_if_tall'] for r in stroke_rows if math.isfinite(float(r['vertical_slant_deg_if_tall']))]
        same_line_gaps = [g['gap_m'] for g in gap_rows if g['same_line']]
        all_gaps = [g['gap_m'] for g in gap_rows]
        warnings = self._make_warnings(
            tip_tag=_stats(self.tip_tag_dist_m),
            rec_blue=_stats(self.rec_blue_dist_m),
            extract_rec=_stats(extraction_to_rec_dists),
            same_line_gaps=same_line_gaps,
            stroke_rows=stroke_rows,
            tall_slants=tall_slants,
        )

        return {
            'created_unix_s': time.time(),
            'purpose': 'diagnostic-only; no EKF/contact/recognition behavior was changed',
            'counts': {
                'strokes': len(strokes),
                'lines': len(line_groups or []),
                'characters': len(character_groups or []),
                'tip_tag_samples': len(self.tip_tag_dist_m),
                'rec_blue_samples': len(self.rec_blue_dist_m),
                'rec_points': len(self.rec_points),
            },
            'distance_metrics_m': {
                'tip_tag': _stats(self.tip_tag_dist_m).as_dict(),
                'recognition_vs_blue': _stats(self.rec_blue_dist_m).as_dict(),
                'extraction_vs_recognition_polyline': _stats(extraction_to_rec_dists).as_dict(),
                'inter_stroke_gap_all': _stats(all_gaps).as_dict(),
                'inter_stroke_gap_same_line': _stats(same_line_gaps).as_dict(),
            },
            'generic_orientation_metrics': {
                'tall_narrow_stroke_count': int(len(tall_slants)),
                'tall_narrow_slant_deg_median': float(np.median(tall_slants)) if tall_slants else None,
                'tall_narrow_slant_deg_max': float(np.max(tall_slants)) if tall_slants else None,
                'note': 'slant is reported only; do not auto-straighten unless caused by calibration or handled by training augmentation',
            },
            'stroke_rows': stroke_rows,
            'gap_rows': gap_rows,
            'line_groups': line_groups or [],
            'character_groups': character_groups or [],
            'warnings': warnings,
            'summary_text': self._summary_text_from_parts(len(strokes), len(line_groups or []), len(character_groups or []), warnings),
        }

    @staticmethod
    def _stroke_to_dict(s: Any) -> dict[str, Any]:
        # Works for dataclass StrokeRecord without importing it here.
        return {
            'stroke_id': int(getattr(s, 'stroke_id', 0)),
            'raw_points': getattr(s, 'raw_points', None),
            'resampled_points': getattr(s, 'resampled_points', None),
            'normalized_points': getattr(s, 'normalized_points', None),
            'raw_path_m': float(getattr(s, 'raw_path_m', 0.0) or 0.0),
            'raw_bbox_m': getattr(s, 'raw_bbox_m', None),
            'reason': getattr(s, 'reason', ''),
        }

    @staticmethod
    def _make_warnings(
        tip_tag: DistanceStats,
        rec_blue: DistanceStats,
        extract_rec: DistanceStats,
        same_line_gaps: list[float],
        stroke_rows: list[dict[str, Any]],
        tall_slants: list[float],
    ) -> list[str]:
        warnings: list[str] = []
        if tip_tag.count and tip_tag.p95_m > 0.025:
            warnings.append(f'Tip/tag offset p95 is {tip_tag.p95_m*100:.1f} cm; inspect heading/tip-offset calibration.')
        if rec_blue.count and rec_blue.p95_m > 0.012:
            warnings.append(f'Recognition trace moves blue by p95={rec_blue.p95_m*100:.1f} cm; smoothing may be altering geometry.')
        if extract_rec.count and extract_rec.p95_m > 0.012:
            warnings.append(f'Extraction overlay differs from cyan by p95={extract_rec.p95_m*100:.1f} cm; inspect resampling/simplification.')
        if same_line_gaps and max(same_line_gaps) > 0.065:
            warnings.append(f'Large same-line stroke gap {max(same_line_gaps)*100:.1f} cm; check false lifts/missed contact or intended pen lift.')
        jumpy = [r for r in stroke_rows if float(r.get('max_step_m', 0.0)) > 0.020]
        if jumpy:
            worst = max(jumpy, key=lambda r: float(r.get('max_step_m', 0.0)))
            warnings.append(f'Stroke #{worst["stroke_id"]} has max resampled step {worst["max_step_m"]*100:.1f} cm; may contain a jump or too few points.')
        if tall_slants and float(np.median(tall_slants)) > 15.0:
            warnings.append(f'Tall-stroke median slant is {np.median(tall_slants):.0f}° from vertical. Report only; fix via calibration/training, not letter rules.')
        if not warnings:
            warnings.append('No large trace-quality alarms; remaining imperfections likely require calibration or recognizer training.')
        return warnings

    @staticmethod
    def _summary_text_from_parts(strokes: int, lines: int, chars: int, warnings: list[str]) -> str:
        head = f'Trace-quality report: strokes={strokes}, lines={lines}, chars={chars}'
        return head + '\n' + '\n'.join(f'- {w}' for w in warnings)

    def export(self, base_path: str | Path, normalized_strokes: Any = None) -> dict[str, str]:
        base = Path(base_path)
        base.parent.mkdir(parents=True, exist_ok=True)
        json_path = base.with_name(base.name + '_trace_quality.json')
        csv_path = base.with_name(base.name + '_trace_quality.csv')
        txt_path = base.with_name(base.name + '_trace_quality.txt')

        payload = self.analyze(normalized_strokes=normalized_strokes)
        json_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
        txt_path.write_text(payload.get('summary_text', ''), encoding='utf-8')

        with csv_path.open('w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['row_type', 'stroke_id', 'line_id', 'char_id', 'metric', 'value_m_or_deg', 'extra'])
            for r in payload.get('stroke_rows', []):
                sid = r.get('stroke_id', 0)
                line_id = r.get('line_id', 0)
                char_id = r.get('char_id', 0)
                for key in ('path_m', 'bbox_w_m', 'bbox_h_m', 'bbox_diag_m', 'chord_m', 'straightness_chord_over_path', 'path_over_bbox_diag', 'max_step_m', 'median_step_m', 'principal_angle_deg', 'vertical_slant_deg_if_tall'):
                    w.writerow(['stroke', sid, line_id, char_id, key, r.get(key, ''), ''])
            for g in payload.get('gap_rows', []):
                w.writerow(['gap', f"{g.get('from_stroke_id')}->{g.get('to_stroke_id')}", '', '', 'gap_m', g.get('gap_m', ''), f"same_line={g.get('same_line')} same_char={g.get('same_char')}"])
            for family, stats in payload.get('distance_metrics_m', {}).items():
                for key, val in stats.items():
                    w.writerow(['distance', '', '', '', f'{family}.{key}', val, ''])

        self.last_export_paths = {'quality_json': str(json_path), 'quality_csv': str(csv_path), 'quality_txt': str(txt_path)}
        return dict(self.last_export_paths)


# ---------------------------------------------------------------------------
# CLI utility for exported recognition JSONs.
# ---------------------------------------------------------------------------

def main(argv: Optional[list[str]] = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description='Analyze a PolyCast recognition export JSON.')
    ap.add_argument('recognition_json', help='Path to live_..._recognition_....json')
    args = ap.parse_args(argv)

    path = Path(args.recognition_json)
    if not path.exists():
        raise SystemExit(f'File not found: {path}')
    diag = TraceQualityDiagnostics()
    payload = diag.analyze(recognition_json_path=path)
    out_json = path.with_name(path.stem + '_trace_quality.json')
    out_txt = path.with_name(path.stem + '_trace_quality.txt')
    out_json.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    out_txt.write_text(payload.get('summary_text', ''), encoding='utf-8')
    print(payload.get('summary_text', ''))
    print(f'Saved: {out_json}')
    print(f'Saved: {out_txt}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
