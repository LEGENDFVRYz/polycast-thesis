"""recognition_preprocessor.py — normalized stroke extraction/export for PolyCast.

This module is intentionally independent from the EKF.  It consumes the
completed pen-tip strokes that main_ekf.py already draws and produces a
recognition-friendly representation:

    raw stroke points in board metres
      -> light duplicate/jitter removal
      -> optional Ramer-Douglas-Peucker simplification
      -> arc-length resampling
      -> session-level normalization to a unit canvas

The normalized output can be viewed/exported in three ways:

* full session normalization (one bbox for all strokes)
* line-group normalization (separate recognition canvas per writing line)
* character/cluster normalization (each likely character cell gets its own bbox)
* stroke-grid normalization (each stroke normalized in its own mini-cell)

This is still geometry derived from the EKF trace; it does not infer labels.
Character groups are heuristic stroke clusters meant for debugging/export, not OCR labels.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from pathlib import Path
import csv
import json
import math
import time
from typing import Iterable, Optional

import numpy as np


@dataclass
class StrokeRecord:
    """One completed stroke in both raw and normalized spaces."""
    stroke_id: int
    raw_points: list[list[float]]
    ts_start: Optional[int] = None
    ts_end: Optional[int] = None
    reason: str = "lift"
    raw_path_m: float = 0.0
    raw_bbox_m: list[float] | None = None  # [min_x, min_y, max_x, max_y]
    resampled_points: list[list[float]] | None = None
    normalized_points: list[list[float]] | None = None


class NormalizedStrokeExtractor:
    """Collect, normalize, preview, and export completed writing strokes.

    Parameters are conservative by default.  The output is meant as a clean
    recognition input, not as a replacement for the raw diagnostic EKF trace.
    """

    def __init__(
        self,
        canvas_size: int = 64,
        margin: float = 0.08,
        resample_points: int = 48,
        simplify_epsilon_m: float = 0.0025,
        min_point_step_m: float = 0.0015,
        min_stroke_path_m: float = 0.004,
        min_stroke_points: int = 4,
    ) -> None:
        self.canvas_size = int(canvas_size)
        self.margin = float(margin)
        self.resample_points = int(resample_points)
        self.simplify_epsilon_m = float(simplify_epsilon_m)
        self.min_point_step_m = float(min_point_step_m)
        self.min_stroke_path_m = float(min_stroke_path_m)
        self.min_stroke_points = int(min_stroke_points)
        self.strokes: list[StrokeRecord] = []
        self.active_points: list[list[float]] = []
        self.active_ts_start: Optional[int] = None
        self.active_ts_last: Optional[int] = None
        self._next_id = 1
        self.last_export_paths: dict[str, str] = {}
        self.last_drop_reason = ""

    # ------------------------------------------------------------------
    # Stroke collection
    # ------------------------------------------------------------------
    def begin_stroke(self, ts: Optional[int] = None) -> None:
        self.active_points = []
        self.active_ts_start = ts
        self.active_ts_last = ts
        self.last_drop_reason = ""

    def add_point(self, x: float, y: float, ts: Optional[int] = None) -> None:
        if not (math.isfinite(float(x)) and math.isfinite(float(y))):
            return
        p = [float(x), float(y)]
        if self.active_points:
            prev = np.asarray(self.active_points[-1], dtype=float)
            if float(np.linalg.norm(np.asarray(p, dtype=float) - prev)) < self.min_point_step_m:
                self.active_ts_last = ts
                return
        self.active_points.append(p)
        self.active_ts_last = ts

    def end_stroke(self, ts: Optional[int] = None, reason: str = "lift") -> Optional[StrokeRecord]:
        pts = np.asarray(self.active_points, dtype=float)
        self.active_points = []
        ts_end = ts if ts is not None else self.active_ts_last
        ts_start = self.active_ts_start
        self.active_ts_start = None
        self.active_ts_last = None

        if len(pts) < self.min_stroke_points:
            self.last_drop_reason = f"dropped short stroke: n={len(pts)}"
            return None

        path_m = self._path_length(pts)
        if path_m < self.min_stroke_path_m:
            self.last_drop_reason = f"dropped tiny stroke: path={path_m*100:.1f}cm"
            return None

        pts = self._rdp(pts, self.simplify_epsilon_m)
        if len(pts) < 2:
            self.last_drop_reason = "dropped degenerate stroke after simplify"
            return None

        resampled = self._resample_arclength(pts, self.resample_points)
        mn = np.nanmin(pts, axis=0)
        mx = np.nanmax(pts, axis=0)
        stroke = StrokeRecord(
            stroke_id=self._next_id,
            raw_points=pts.tolist(),
            ts_start=ts_start,
            ts_end=ts_end,
            reason=reason,
            raw_path_m=float(path_m),
            raw_bbox_m=[float(mn[0]), float(mn[1]), float(mx[0]), float(mx[1])],
            resampled_points=resampled.tolist(),
            normalized_points=None,
        )
        self._next_id += 1
        self.strokes.append(stroke)
        self._renormalize_all()
        return stroke

    def force_finish_active(self, reason: str = "forced") -> Optional[StrokeRecord]:
        return self.end_stroke(reason=reason) if self.active_points else None

    def clear(self) -> None:
        self.strokes.clear()
        self.active_points = []
        self.active_ts_start = None
        self.active_ts_last = None
        self._next_id = 1
        self.last_export_paths = {}
        self.last_drop_reason = ""

    # ------------------------------------------------------------------
    # Preview helpers
    # ------------------------------------------------------------------
    @property
    def completed_count(self) -> int:
        return len(self.strokes)

    @property
    def active_count(self) -> int:
        return len(self.active_points)

    def summary(self) -> str:
        if not self.strokes:
            return f"normalized strokes=0 active={self.active_count}"
        paths = [s.raw_path_m for s in self.strokes]
        pts = sum(len(s.normalized_points or []) for s in self.strokes)
        return (f"normalized strokes={len(self.strokes)} lines={len(self.line_groups())} chars={len(self.character_groups())} active={self.active_count} "
                f"points={pts} path_med={np.median(paths)*100:.1f}cm")

    # ------------------------------------------------------------------
    # Grouping helpers
    # ------------------------------------------------------------------
    def line_groups(self, y_gap_m: Optional[float] = None) -> list[dict]:
        """Return stroke groups that likely correspond to writing lines.

        Strokes are grouped by vertical center.  This deliberately avoids
        character segmentation; it only separates rows such as "abcde" vs
        "hello".  The returned line order is top-to-bottom, and strokes inside
        each line are left-to-right.
        """
        if not self.strokes:
            return []

        rows = []
        heights = []
        for s in self.strokes:
            if not s.raw_bbox_m:
                continue
            x0, y0, x1, y1 = [float(v) for v in s.raw_bbox_m]
            cy = 0.5 * (y0 + y1)
            heights.append(max(1e-6, y1 - y0))
            rows.append({
                'stroke': s,
                'stroke_id': s.stroke_id,
                'x0': x0, 'y0': y0, 'x1': x1, 'y1': y1,
                'cx': 0.5 * (x0 + x1),
                'cy': cy,
            })
        if not rows:
            return []

        if y_gap_m is None:
            med_h = float(np.median(heights)) if heights else 0.08
            # Keep this slightly below one median letter height so the two
            # lines in the current test split cleanly while same-line tall
            # strokes stay together.
            y_gap_m = max(0.055, 0.70 * med_h)

        clusters: list[list[dict]] = []
        for item in sorted(rows, key=lambda r: r['cy'], reverse=True):
            placed = False
            for cluster in clusters:
                c_cy = float(np.mean([r['cy'] for r in cluster]))
                if abs(item['cy'] - c_cy) <= y_gap_m:
                    cluster.append(item)
                    placed = True
                    break
            if not placed:
                clusters.append([item])

        # Sort clusters top-to-bottom and strokes left-to-right.
        clusters.sort(key=lambda c: float(np.mean([r['cy'] for r in c])), reverse=True)
        out = []
        for line_idx, cluster in enumerate(clusters, start=1):
            cluster.sort(key=lambda r: (r['x0'], r['cx']))
            x0 = min(r['x0'] for r in cluster); y0 = min(r['y0'] for r in cluster)
            x1 = max(r['x1'] for r in cluster); y1 = max(r['y1'] for r in cluster)
            out.append({
                'line_id': line_idx,
                'stroke_ids': [int(r['stroke_id']) for r in cluster],
                'bbox_m': [float(x0), float(y0), float(x1), float(y1)],
                'center_y_m': float(np.mean([r['cy'] for r in cluster])),
                'stroke_count': len(cluster),
            })
        return out


    def _strokes_by_id(self) -> dict[int, StrokeRecord]:
        return {int(s.stroke_id): s for s in self.strokes}

    def character_groups(self, merge_small_marks: bool = True, merge_multistroke: bool = True) -> list[dict]:
        """Return heuristic character/cluster groups within each detected line.

        This groups adjacent strokes that most likely belong to the same
        written character.  It does not infer or repair character labels;  The previous version was intentionally too
        conservative and treated nearly every pen-down stroke as a different
        character; that breaks letters such as a handwritten ``d`` when the
        loop and stem are written as separate strokes.

        The grouping is still heuristic, but it now merges adjacent strokes
        when their board-space bounding boxes strongly overlap in X *and*
        overlap enough vertically, while avoiding common false merges such as
        a short rounded ``e`` followed by a tall narrow ``l``.

        The returned ``cell_bbox_m`` is now anchored to the actual cluster
        bbox in board metres.  A separate ``layout_bbox_m`` is kept for the old
        equal-cell view, but the live ``chars`` overlay no longer reflows the
        user writing into artificial spacing.
        """
        out: list[dict] = []
        by_id = self._strokes_by_id()
        char_id = 1
        for line in self.line_groups():
            items = []
            for sid in line.get('stroke_ids', []):
                s = by_id.get(int(sid))
                if s is None or not s.raw_bbox_m:
                    continue
                x0, y0, x1, y1 = [float(v) for v in s.raw_bbox_m]
                items.append({
                    'stroke': s,
                    'stroke_id': int(s.stroke_id),
                    'x0': x0, 'y0': y0, 'x1': x1, 'y1': y1,
                    'cx': 0.5 * (x0 + x1),
                    'cy': 0.5 * (y0 + y1),
                    'w': max(1e-6, x1 - x0),
                    'h': max(1e-6, y1 - y0),
                    'path': float(s.raw_path_m),
                    'ts_start': s.ts_start,
                    'ts_end': s.ts_end,
                })
            if not items:
                continue
            # Sort left-to-right for character order.  Stroke ids are still
            # preserved inside each cluster for export/debugging.
            items.sort(key=lambda r: (r['x0'], r['cx'], r['stroke_id']))
            line_x0, line_y0, line_x1, line_y1 = [float(v) for v in line.get('bbox_m', [0, 0, 1, 1])]
            line_w = max(1e-6, line_x1 - line_x0)
            line_h = max(1e-6, line_y1 - line_y0)
            med_w = float(np.median([max(1e-6, r['w']) for r in items]))
            med_h = float(np.median([max(1e-6, r['h']) for r in items]))
            med_path = float(np.median([max(1e-6, r['path']) for r in items]))

            def cluster_stats(cluster: list[dict]) -> dict:
                x0 = min(r['x0'] for r in cluster); y0 = min(r['y0'] for r in cluster)
                x1 = max(r['x1'] for r in cluster); y1 = max(r['y1'] for r in cluster)
                return {
                    'x0': x0, 'y0': y0, 'x1': x1, 'y1': y1,
                    'w': max(1e-6, x1 - x0),
                    'h': max(1e-6, y1 - y0),
                    'cx': 0.5 * (x0 + x1),
                    'cy': 0.5 * (y0 + y1),
                }

            def should_merge(prev: list[dict], item: dict) -> tuple[bool, str]:
                st = cluster_stats(prev)
                x_overlap = min(st['x1'], item['x1']) - max(st['x0'], item['x0'])
                y_overlap = min(st['y1'], item['y1']) - max(st['y0'], item['y0'])
                x_gap = item['x0'] - st['x1']
                min_w = max(1e-6, min(st['w'], item['w']))
                min_h = max(1e-6, min(st['h'], item['h']))
                x_overlap_ratio = x_overlap / min_w if x_overlap > 0 else 0.0
                y_overlap_ratio = y_overlap / min_h if y_overlap > 0 else 0.0
                width_ratio = item['w'] / max(1e-6, st['w'])
                height_ratio = item['h'] / max(1e-6, st['h'])
                center_dx = abs(item['cx'] - st['cx'])

                item_small = (item['w'] < 0.35 * med_w and item['path'] < 0.45 * med_path)
                item_inside_prev = item['x0'] >= st['x0'] - 0.008 and item['x1'] <= st['x1'] + 0.008
                if merge_small_marks and item_small and (x_overlap > 0 or item_inside_prev):
                    return True, 'small-mark-overlap'

                if not merge_multistroke:
                    return False, 'separate'

                # Strong same-character overlap, used for e.g. a two-stroke d
                # where loop/stem occupy the same horizontal region.  The
                # height ratio guard prevents merging a low rounded character
                # with a following tall l-like stroke.
                comparable_height = 0.42 <= height_ratio <= 2.35
                comparable_width = 0.28 <= width_ratio <= 3.2
                strong_overlap = (
                    x_overlap_ratio >= 0.38 and
                    y_overlap_ratio >= 0.28 and
                    comparable_height and
                    comparable_width
                )
                if strong_overlap:
                    return True, f'multistroke-overlap x={x_overlap_ratio:.2f} y={y_overlap_ratio:.2f}'

                # Do not merge merely because the horizontal gap is small.
                # In cursive-like or tight handwriting, separate letters such
                # as adjacent l/l can have tiny gaps and nearly complete
                # vertical overlap.  Multi-stroke same-character merging is
                # therefore driven primarily by real X-overlap above.

                return False, 'separate'

            clusters: list[list[dict]] = []
            merge_notes: list[str] = []
            for item in items:
                if not clusters:
                    clusters.append([item])
                    continue
                do_merge, reason = should_merge(clusters[-1], item)
                if do_merge:
                    clusters[-1].append(item)
                    merge_notes.append(f"{clusters[-1][0]['stroke_id']}+{item['stroke_id']}:{reason}")
                else:
                    clusters.append([item])

            n_chars = max(1, len(clusters))
            cell_gap = min(0.012, 0.04 * line_w / max(1, n_chars - 1)) if n_chars > 1 else 0.0
            usable_w = max(1e-6, line_w - cell_gap * max(0, n_chars - 1))
            layout_cell_w = usable_w / n_chars

            for idx, cluster in enumerate(clusters, start=1):
                x0 = min(r['x0'] for r in cluster); y0 = min(r['y0'] for r in cluster)
                x1 = max(r['x1'] for r in cluster); y1 = max(r['y1'] for r in cluster)

                # Actual anchored bbox: expand just enough to make the black
                # overlay visible, but do not reflow or equal-space characters.
                bw = max(1e-6, x1 - x0); bh = max(1e-6, y1 - y0)
                pad_x = min(0.020, max(0.004, 0.10 * bw))
                pad_y = min(0.020, max(0.004, 0.10 * bh))
                anchor_bbox = [
                    max(line_x0, x0 - pad_x),
                    max(line_y0, y0 - pad_y),
                    min(line_x1, x1 + pad_x),
                    min(line_y1, y1 + pad_y),
                ]

                # Old equal-cell layout is still exported and available as an
                # optional overlay mode, but it is no longer the default chars
                # view because it does not represent where the user wrote.
                layout_x0 = line_x0 + (idx - 1) * (layout_cell_w + cell_gap)
                layout_x1 = layout_x0 + layout_cell_w
                layout_bbox = [float(layout_x0), float(line_y0), float(layout_x1), float(line_y1)]

                out.append({
                    'char_id': char_id,
                    'line_id': int(line.get('line_id', 0)),
                    'char_index_in_line': idx,
                    'stroke_ids': [int(r['stroke_id']) for r in cluster],
                    'bbox_m': [float(x0), float(y0), float(x1), float(y1)],
                    'cell_bbox_m': [float(v) for v in anchor_bbox],
                    'layout_bbox_m': layout_bbox,
                    'stroke_count': len(cluster),
                    'merge_note': ';'.join(merge_notes) if len(cluster) > 1 else '',
                    'estimated_label': None,
                })
                char_id += 1
        return out

    def _normalized_for_stroke_ids(self, stroke_ids: list[int]) -> list[tuple[StrokeRecord, np.ndarray]]:
        idset = set(int(i) for i in stroke_ids)
        selected = [s for s in self.strokes if s.stroke_id in idset and s.resampled_points]
        point_sets = [np.asarray(s.resampled_points, dtype=float) for s in selected]
        normed = self._normalize_point_sets(point_sets)
        return list(zip(selected, normed))

    def preview_lines(self, include_active: bool = True, mode: str = 'lines') -> tuple[list[float], list[float]]:
        """Return NaN-separated unit-canvas coordinates for Matplotlib.

        mode='full'    : old behavior, one global bbox for all strokes.
        mode='lines'   : normalize each detected writing line independently,
                         then stack lines top-to-bottom in the preview.
        mode='strokes' : normalize each stroke into a small grid cell.  Useful
                         for verifying whether individual strokes are sane.
        """
        mode = (mode or 'lines').lower()
        if mode == 'full':
            return self._preview_full(include_active=include_active)
        if mode == 'strokes':
            return self._preview_stroke_grid(include_active=include_active)
        return self._preview_line_groups(include_active=include_active)

    def _preview_full(self, include_active: bool = True) -> tuple[list[float], list[float]]:
        xs: list[float] = []
        ys: list[float] = []
        for s in self.strokes:
            pts = np.asarray(s.normalized_points if s.normalized_points is not None else [], dtype=float)
            if len(pts) == 0:
                continue
            xs.extend(pts[:, 0].astype(float).tolist())
            ys.extend(pts[:, 1].astype(float).tolist())
            xs.append(float('nan'))
            ys.append(float('nan'))

        if include_active and len(self.active_points) >= 2:
            active = np.asarray(self.active_points, dtype=float)
            all_pts = [np.asarray(s.resampled_points, dtype=float) for s in self.strokes if s.resampled_points]
            all_pts.append(active)
            normed = self._normalize_point_sets(all_pts)
            pts = normed[-1]
            if len(pts):
                xs.extend(pts[:, 0].astype(float).tolist())
                ys.extend(pts[:, 1].astype(float).tolist())
                xs.append(float('nan'))
                ys.append(float('nan'))
        return xs, ys

    def _preview_line_groups(self, include_active: bool = True) -> tuple[list[float], list[float]]:
        xs: list[float] = []
        ys: list[float] = []
        groups = self.line_groups()
        n = max(1, len(groups))
        for row_idx, group in enumerate(groups):
            row_y0 = 1.0 - (row_idx + 1) / n
            row_h = 1.0 / n
            for _s, pts in self._normalized_for_stroke_ids(group['stroke_ids']):
                if len(pts) == 0:
                    continue
                qx = pts[:, 0]
                qy = row_y0 + pts[:, 1] * row_h
                xs.extend(qx.astype(float).tolist())
                ys.extend(qy.astype(float).tolist())
                xs.append(float('nan'))
                ys.append(float('nan'))

        # Active stroke is previewed in full-session coordinates so live drawing
        # remains visible without reassigning it to a line before lift.
        if include_active and len(self.active_points) >= 2:
            ax, ay = self._preview_full(include_active=True)
            # Only append the active part by taking from the end after last nan is
            # hard to do robustly; omit active here to keep line grouping stable.
            pass
        return xs, ys

    def _preview_stroke_grid(self, include_active: bool = True) -> tuple[list[float], list[float]]:
        strokes = [s for s in self.strokes if s.resampled_points]
        if include_active and len(self.active_points) >= 2:
            # Active stroke is not committed; previewing it as a temporary grid
            # item makes the grid jump around.  Skip it until lift/export.
            pass
        count = len(strokes)
        if count == 0:
            return [], []
        cols = int(math.ceil(math.sqrt(count)))
        rows = int(math.ceil(count / cols))
        pad = 0.08
        xs: list[float] = []
        ys: list[float] = []
        for idx, s in enumerate(strokes):
            r = idx // cols
            c = idx % cols
            cell_x0 = c / cols
            cell_y0 = 1.0 - (r + 1) / rows
            cell_w = 1.0 / cols
            cell_h = 1.0 / rows
            pts = np.asarray(s.resampled_points, dtype=float)
            norm = self._normalize_point_sets([pts])[0]
            qx = cell_x0 + (pad + norm[:, 0] * (1 - 2 * pad)) * cell_w
            qy = cell_y0 + (pad + norm[:, 1] * (1 - 2 * pad)) * cell_h
            xs.extend(qx.astype(float).tolist())
            ys.extend(qy.astype(float).tolist())
            xs.append(float('nan'))
            ys.append(float('nan'))
        return xs, ys


    def preview_overlay(self, include_active: bool = True, mode: str = 'raw') -> tuple[list[float], list[float]]:
        """Return NaN-separated preview points in *board metres*.

        The safest live-session debug view is ``raw``: it draws the extracted
        recognition strokes directly in board metres, without re-normalizing,
        re-centering, re-spacing, or straightening them.  That means the black
        overlay follows where the user actually wrote.

        The remaining modes intentionally show what normalization would do for
        model input/debugging.  They are useful, but they should not be treated
        as a physical reconstruction of the handwriting path.
        """
        mode = (mode or 'raw').lower()
        if mode in ('raw', 'board', 'actual', 'extract'):
            return self._preview_raw_overlay(include_active=include_active)
        if mode == 'full':
            return self._preview_full_overlay(include_active=include_active)
        if mode == 'strokes':
            return self._preview_stroke_overlay(include_active=include_active)
        if mode in ('chars', 'characters', 'char'):
            return self._preview_character_overlay(include_active=include_active, use_layout=False)
        if mode in ('cells', 'layout', 'charcells', 'char-cells'):
            return self._preview_character_overlay(include_active=include_active, use_layout=True)
        return self._preview_line_overlay(include_active=include_active)

    @staticmethod
    def _unit_to_bbox(pts: np.ndarray, bbox: list[float], pad_frac: float = 0.04) -> np.ndarray:
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        if len(pts) == 0:
            return pts
        x0, y0, x1, y1 = [float(v) for v in bbox]
        w = max(1e-9, x1 - x0)
        h = max(1e-9, y1 - y0)
        px = min(0.45, max(0.0, float(pad_frac))) * w
        py = min(0.45, max(0.0, float(pad_frac))) * h
        q = np.empty_like(pts, dtype=float)
        q[:, 0] = (x0 + px) + pts[:, 0] * max(1e-9, w - 2 * px)
        q[:, 1] = (y0 + py) + pts[:, 1] * max(1e-9, h - 2 * py)
        return q

    def _append_nan_path(self, xs: list[float], ys: list[float], pts: np.ndarray) -> None:
        if len(pts) == 0:
            return
        xs.extend(pts[:, 0].astype(float).tolist())
        ys.extend(pts[:, 1].astype(float).tolist())
        xs.append(float('nan'))
        ys.append(float('nan'))


    def _preview_raw_overlay(self, include_active: bool = True) -> tuple[list[float], list[float]]:
        """Return the recognition extractor path in board metres.

        This is deliberately not a normalization pass.  It uses the completed
        stroke resampled points, plus the active stroke if requested, so it is
        the most unbiased visual audit of what will be exported.
        """
        xs: list[float] = []
        ys: list[float] = []
        for s in self.strokes:
            if not s.resampled_points:
                continue
            pts = np.asarray(s.resampled_points, dtype=float).reshape(-1, 2)
            self._append_nan_path(xs, ys, pts)
        if include_active and self.active_points:
            pts = np.asarray(self.active_points, dtype=float).reshape(-1, 2)
            self._append_nan_path(xs, ys, pts)
        return xs, ys

    def _preview_full_overlay(self, include_active: bool = True) -> tuple[list[float], list[float]]:
        xs: list[float] = []
        ys: list[float] = []
        valid = [np.asarray(s.resampled_points, dtype=float) for s in self.strokes if s.resampled_points]
        if not valid:
            return xs, ys
        all_pts = np.vstack(valid)
        mn = np.nanmin(all_pts, axis=0); mx = np.nanmax(all_pts, axis=0)
        bbox = [float(mn[0]), float(mn[1]), float(mx[0]), float(mx[1])]
        for s, pts in zip([s for s in self.strokes if s.resampled_points], self._normalize_point_sets(valid)):
            self._append_nan_path(xs, ys, self._unit_to_bbox(pts, bbox, pad_frac=0.02))
        return xs, ys

    def _preview_line_overlay(self, include_active: bool = True) -> tuple[list[float], list[float]]:
        xs: list[float] = []
        ys: list[float] = []
        for group in self.line_groups():
            bbox = group.get('bbox_m')
            if not bbox:
                continue
            for _s, pts in self._normalized_for_stroke_ids(group['stroke_ids']):
                self._append_nan_path(xs, ys, self._unit_to_bbox(pts, bbox, pad_frac=0.02))
        return xs, ys

    def _preview_character_overlay(self, include_active: bool = True, use_layout: bool = False) -> tuple[list[float], list[float]]:
        xs: list[float] = []
        ys: list[float] = []
        for ch in self.character_groups():
            if use_layout:
                cell = ch.get('layout_bbox_m') or ch.get('cell_bbox_m') or ch.get('bbox_m')
                pad = 0.08
            else:
                # Default character overlay is anchored to the user's actual
                # board-space writing location.  It no longer redistributes
                # characters into equal-width cells across the line.
                cell = ch.get('cell_bbox_m') or ch.get('bbox_m')
                pad = 0.015
            if not cell:
                continue
            for _s, pts in self._normalized_for_stroke_ids(ch['stroke_ids']):
                self._append_nan_path(xs, ys, self._unit_to_bbox(pts, cell, pad_frac=pad))
        return xs, ys

    def _preview_stroke_overlay(self, include_active: bool = True) -> tuple[list[float], list[float]]:
        xs: list[float] = []
        ys: list[float] = []
        for s in self.strokes:
            if not s.resampled_points or not s.raw_bbox_m:
                continue
            pts = self._normalize_point_sets([np.asarray(s.resampled_points, dtype=float)])[0]
            self._append_nan_path(xs, ys, self._unit_to_bbox(pts, s.raw_bbox_m, pad_frac=0.05))
        return xs, ys

    # ------------------------------------------------------------------
    # Export
    # ------------------------------------------------------------------
    def export(self, base_path: str | Path, include_active: bool = False) -> dict[str, str]:
        """Export normalized strokes to JSON and CSV.

        base_path may omit an extension.  Returns created paths.
        """
        if include_active and self.active_points:
            self.force_finish_active(reason="export_active")
        base = Path(base_path)
        base.parent.mkdir(parents=True, exist_ok=True)
        json_path = base.with_suffix('.json')
        csv_path = base.with_suffix('.csv')

        payload = {
            'created_unix_s': time.time(),
            'canvas_size': self.canvas_size,
            'margin': self.margin,
            'resample_points_per_stroke': self.resample_points,
            'stroke_count': len(self.strokes),
            'line_count': len(self.line_groups()),
            'character_count': len(self.character_groups()),
            'coordinate_system': 'normalized_unit_canvas_0_to_1_y_up',
            'normalization_modes': ['full_session', 'line_groups', 'character_groups', 'stroke_grid_preview'],
            'overlay_modes': ['raw_board_space', 'character_anchor', 'character_cells', 'line_overlay', 'full_overlay', 'stroke_overlay'],
            'line_groups': self.line_groups(),
            'character_groups': self.character_groups(),
            'strokes': [asdict(s) for s in self.strokes],
        }
        json_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')

        with csv_path.open('w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['stroke_id', 'line_id', 'char_id', 'char_index_in_line', 'point_index',
                        'x_norm', 'y_norm', 'x_px', 'y_px', 'x_raw_m', 'y_raw_m', 'reason'])
            line_map = {}
            for g in self.line_groups():
                for sid in g.get('stroke_ids', []):
                    line_map[int(sid)] = int(g.get('line_id', 0))
            char_map = {}
            char_idx_map = {}
            for ch in self.character_groups():
                for sid in ch.get('stroke_ids', []):
                    char_map[int(sid)] = int(ch.get('char_id', 0))
                    char_idx_map[int(sid)] = int(ch.get('char_index_in_line', 0))
            for s in self.strokes:
                norm = np.asarray(s.normalized_points or [], dtype=float)
                raw = np.asarray(s.resampled_points or [], dtype=float)
                for i in range(len(norm)):
                    x = float(norm[i, 0]); y = float(norm[i, 1])
                    xr = float(raw[i, 0]) if i < len(raw) else float('nan')
                    yr = float(raw[i, 1]) if i < len(raw) else float('nan')
                    w.writerow([s.stroke_id, line_map.get(int(s.stroke_id), 0),
                                char_map.get(int(s.stroke_id), 0),
                                char_idx_map.get(int(s.stroke_id), 0),
                                i, x, y,
                                x * (self.canvas_size - 1),
                                y * (self.canvas_size - 1),
                                xr, yr, s.reason])

        self.last_export_paths = {'json': str(json_path), 'csv': str(csv_path)}
        return dict(self.last_export_paths)

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _path_length(points: np.ndarray) -> float:
        if len(points) < 2:
            return 0.0
        return float(np.sum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)))

    @staticmethod
    def _resample_arclength(points: np.ndarray, n: int) -> np.ndarray:
        pts = np.asarray(points, dtype=float)
        if len(pts) == 0:
            return pts.reshape(0, 2)
        if len(pts) == 1 or n <= 1:
            return pts[:1, :2]
        d = np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(d)])
        total = float(s[-1])
        if total <= 1e-12:
            return np.repeat(pts[:1, :2], n, axis=0)
        target = np.linspace(0.0, total, int(n))
        x = np.interp(target, s, pts[:, 0])
        y = np.interp(target, s, pts[:, 1])
        return np.column_stack([x, y])

    @classmethod
    def _rdp(cls, points: np.ndarray, epsilon: float) -> np.ndarray:
        pts = np.asarray(points, dtype=float)
        if len(pts) <= 2 or epsilon <= 0:
            return pts[:, :2]
        keep = cls._rdp_indices(pts[:, :2], float(epsilon))
        return pts[keep, :2]

    @staticmethod
    def _rdp_indices(points: np.ndarray, epsilon: float) -> list[int]:
        def dist_to_segment(p, a, b):
            ab = b - a
            denom = float(np.dot(ab, ab))
            if denom <= 1e-12:
                return float(np.linalg.norm(p - a))
            t = max(0.0, min(1.0, float(np.dot(p - a, ab) / denom)))
            proj = a + t * ab
            return float(np.linalg.norm(p - proj))

        n = len(points)
        keep = {0, n - 1}
        stack = [(0, n - 1)]
        while stack:
            i, j = stack.pop()
            if j <= i + 1:
                continue
            a, b = points[i], points[j]
            dmax = -1.0
            idx = i
            for k in range(i + 1, j):
                d = dist_to_segment(points[k], a, b)
                if d > dmax:
                    dmax = d
                    idx = k
            if dmax > epsilon:
                keep.add(idx)
                stack.append((i, idx))
                stack.append((idx, j))
        return sorted(keep)

    def _renormalize_all(self) -> None:
        sets = [np.asarray(s.resampled_points or [], dtype=float) for s in self.strokes]
        normed = self._normalize_point_sets(sets)
        for s, pts in zip(self.strokes, normed):
            s.normalized_points = pts.tolist()

    def _normalize_point_sets(self, point_sets: Iterable[np.ndarray]) -> list[np.ndarray]:
        sets = [np.asarray(p, dtype=float).reshape(-1, 2) for p in point_sets]
        valid = [p for p in sets if len(p) > 0]
        if not valid:
            return sets
        all_pts = np.vstack(valid)
        mn = np.nanmin(all_pts, axis=0)
        mx = np.nanmax(all_pts, axis=0)
        span = mx - mn
        scale = float(np.max(span))
        if not np.isfinite(scale) or scale < 1e-9:
            scale = 1.0
        usable = max(1e-6, 1.0 - 2.0 * self.margin)
        center = 0.5 * (mn + mx)
        out = []
        for pts in sets:
            if len(pts) == 0:
                out.append(pts)
                continue
            q = (pts - center) / scale * usable + 0.5
            q = np.clip(q, 0.0, 1.0)
            out.append(q)
        return out
