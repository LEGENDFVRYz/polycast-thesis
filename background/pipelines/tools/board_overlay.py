"""
Reusable board overlay: several traces of one recording on a single axis.

Every comparison tool in this package had grown its own copy of the same plot -
draw N polylines in board coordinates, crop, equal aspect, strip the ticks, put
the numbers in the legend. This is that plot, once, so the tools differ only in
what they measure.

Two decisions are baked in because getting them wrong makes the plot lie:

  * **Crop to the ink, not the board.** Writing occupies roughly a third of the
    1.90 x 1.20 m surface while the differences between traces are a few
    centimetres. Drawn to the full board those differences are a hairline. The
    crop is computed from the union of every trace so all layers stay in the
    same frame - cropping per layer would align them artificially and hide the
    placement differences the overlay exists to show.

  * **Equal aspect, always.** Letterform is the subject. A stretched axis
    changes the shape being judged, so the aspect ratio is never left to
    matplotlib's autoscaling.

Layers draw heaviest-first when a weight is supplied, so a noisy trace does not
bury a clean one underneath it.

Usage:
    from background.pipelines.tools.board_overlay import OverlayLayer, render_overlay

    render_overlay(
        layers=[OverlayLayer(label='OG', colour='#1f77d0', strokes=[...])],
        title='stage comparison | abc_1',
        out_path=Path('output/comparison.png'),
    )
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# Space left around the ink, in metres. Enough that strokes are not clipped at
# the frame, small enough that it does not shrink the differences on show.
_CROP_MARGIN_M = 0.04


@dataclass
class OverlayLayer:
    """One trace of a recording - a configuration, a filter, a sensor path."""

    label: str
    colour: str
    strokes: list = field(default_factory=list)

    # Appended to the label in the legend. The tool owns its own metrics, so
    # this module never computes or interprets them.
    summary: str = ''

    # Higher draws first, ending up underneath. Tools generally pass a
    # noisiness measure here so the busiest trace does not cover the rest.
    weight: float = 0.0

    linewidth: float = 1.7
    alpha: float = 0.8

    def points(self) -> list[np.ndarray]:
        """Strokes as (N, 2) arrays, skipping anything too short to draw."""

        out = []
        for stroke in self.strokes:
            array = np.asarray(stroke, dtype=float)
            if array.ndim == 2 and len(array) >= 2:
                out.append(array[:, :2])
        return out

    def legend_text(self) -> str:
        return f'{self.label}  -  {self.summary}' if self.summary else self.label


def crop_bounds(layers: list[OverlayLayer], margin: float = _CROP_MARGIN_M):
    """
    Bounding box of every point in every layer, padded by `margin`.

    Shared across layers on purpose: a per-layer crop would centre each trace in
    its own frame and make traces that disagree about placement look identical.

    Returns `((min_x, min_y), (max_x, max_y))`, or None when there is nothing
    to draw.
    """

    gathered = [array for layer in layers for array in layer.points()]
    if not gathered:
        return None

    stacked = np.vstack(gathered)
    if not np.all(np.isfinite(stacked)):
        stacked = stacked[np.all(np.isfinite(stacked), axis=1)]
        if len(stacked) == 0:
            return None

    return stacked.min(axis=0) - margin, stacked.max(axis=0) + margin


def draw_layers(axis, layers: list[OverlayLayer], margin: float = _CROP_MARGIN_M) -> bool:
    """
    Draw layers onto an existing axis and apply the crop.

    Split out from `render_overlay` so a caller assembling a multi-panel figure
    can reuse the overlay as one panel of its own layout.
    """

    bounds = crop_bounds(layers, margin)
    if bounds is None:
        return False

    for layer in sorted(layers, key=lambda item: -item.weight):
        arrays = layer.points()
        for index, array in enumerate(arrays):
            axis.plot(
                array[:, 0], array[:, 1],
                color=layer.colour,
                linewidth=layer.linewidth,
                alpha=layer.alpha,
                solid_capstyle='round',
                # Label once per layer, or the legend repeats it per stroke.
                label=layer.legend_text() if index == 0 else None,
            )

    (min_x, min_y), (max_x, max_y) = bounds
    axis.set_xlim(float(min_x), float(max_x))
    axis.set_ylim(float(min_y), float(max_y))
    axis.set_aspect('equal', adjustable='box')
    axis.set_xticks([])
    axis.set_yticks([])
    return True


def render_overlay(
    layers: list[OverlayLayer],
    title: str,
    out_path: Path,
    subtitle: str = '',
    figure_width: float = 15.0,
    show_legend: bool = True,
    margin: float = _CROP_MARGIN_M,
) -> bool:
    """
    Write a single-axis overlay PNG cropped to the ink.

    The figure height follows the ink's own aspect ratio, so a wide line of
    writing produces a wide image instead of being centred in wasted space.
    Returns False when matplotlib is unavailable or there is nothing to draw.
    """

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as error:
        print(f'  [plot] matplotlib unavailable ({error}); skipping PNG.')
        return False

    bounds = crop_bounds(layers, margin)
    if bounds is None:
        print('  [plot] nothing to draw.')
        return False

    (min_x, min_y), (max_x, max_y) = bounds
    span_x = max(float(max_x - min_x), 1e-6)
    span_y = max(float(max_y - min_y), 1e-6)

    # Height from the ink's aspect, with headroom for the title and legend, and
    # bounded so a nearly-flat trace cannot collapse the figure.
    figure_height = float(np.clip(figure_width * span_y / span_x, 3.0, 12.0)) + 1.6

    figure, axis = plt.subplots(figsize=(figure_width, figure_height))
    figure.suptitle(f'{title}\n{subtitle}' if subtitle else title, fontsize=13)

    if not draw_layers(axis, layers, margin):
        plt.close(figure)
        return False

    if show_legend:
        axis.legend(fontsize=9, loc='upper left', framealpha=0.9)

    figure.tight_layout(rect=(0, 0, 1, 0.93 if subtitle else 0.95))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(out_path, dpi=140)
    plt.close(figure)
    print(f'  [plot] wrote {out_path}')
    return True
