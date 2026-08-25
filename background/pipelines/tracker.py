"""
background.pipelines.tracker
============================

Headless ESKF stroke-coordinate provider for the background drawing thread.

Bridges the full ``background/pipelines/`` stack to the ``_prototype_thread.py``
interface.  The prototype thread owns the serial port and delivers one
``readline()`` result at a time; this module accepts raw bytes and runs them
through the same pipeline ``visualizer.py`` drives, without touching the serial
layer or any UI code.

    raw serial line -> unpacker -> normalizer -> time alignment
      -> IMU/contact + UWB range/trilateration/position preprocessors
      -> ESKF -> StrokeReconstructor (postprocess) -> (x_m, y_m, is_drawing, state)

This is the application-side entry point for the *ESKF* pipeline.  The other
pipeline has its own tracker with the same public interface:

    from background.pipelines.tracker    import StrokeTracker   # ESKF
    from background.pipeline_ekf.tracker import StrokeTracker   # 6-state EKF

Both expose ``get_bbox()`` / ``process_packet()`` / ``reset()``, so the app
selects a pipeline purely by which module it imports.

Postprocessed ink
-----------------
The fused position emitted per IMU sample is *live* ink: it is what the pen is
doing right now, and it is what this tracker streams back so the app can draw
without latency.  The postprocess stages (velocity detrend, IMU-degeneracy
fallback, two-point anchor, minimum-jerk) only run inside
``StrokeReconstructor._close_current`` when a stroke *ends*, because they need
the whole stroke to work on.

The earlier version of this bridge called the fusion engine directly and never
built a reconstructor, so none of those stages ever ran and its ink was raw
fused output.  This version keeps the reconstructor in the loop and exposes each
corrected stroke through ``drain_closed_strokes()``; the app can repaint the
finished stroke over the live one.  Ignoring that queue reproduces the old
behaviour minus the drift correction, so callers that want the corrected shape
must drain it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple, Union

from background.pipelines.cleaner.normalizer import StreamNormalizer
from background.pipelines.cleaner.time_alignment import TimeAlignLayer
from background.pipelines.cleaner.unpacker import parse_packet_line
from background.pipelines.config import cfg
from background.pipelines.fusion.eskf import ESKF
from background.pipelines.preprocess.contact import ContactStateDetector
from background.pipelines.preprocess.imu import IMUPreprocessor
from background.pipelines.preprocess.uwb.position import UWBPositionFilter
from background.pipelines.preprocess.uwb.range import UWBRangePreprocessor
from background.pipelines.preprocess.uwb.trilateration import UWBSolver
from background.pipelines.reconstruct import StrokeReconstructor


StrokeResult = Tuple[float, float, bool, str]

# Matches the teleport guard in visualizer.py.  A fused jump larger than this is
# a filter glitch rather than pen motion, and drawing through it leaves a long
# false line across the board.
_MAX_INK_JUMP_M = 0.15


@dataclass
class TrackerDiagnostics:
    last_packet_type: str = "none"
    last_stroke_state: str = "AIR_MOVE"
    last_error: str = ""
    imu_packets: int = 0
    uwb_packets: int = 0
    emitted_points: int = 0
    closed_strokes: int = 0
    dropped_lines: int = 0
    last_postprocess: Dict[str, Any] = field(default_factory=dict)


class StrokeTracker:
    """Convert raw serial packets into ESKF stroke coordinates.

    ``process_packet`` returns the 4-tuple the background thread expects:

        (meter_x, meter_y, is_drawing, stroke_state)

    UWB packets correct the filter but lay down no ink, so they return ``None``.
    """

    def __init__(self, *, max_ink_jump_m: float = _MAX_INK_JUMP_M) -> None:
        self._norm = StreamNormalizer()
        self._aligner = TimeAlignLayer(buffer_size=500)
        self._imu_prep = IMUPreprocessor()
        self._contact = ContactStateDetector()
        self._range_prep = UWBRangePreprocessor(offsets=cfg.uwb.range_offsets_m)
        self._trilat = UWBSolver()
        self._pos_filt = UWBPositionFilter()
        self._fusion = ESKF()
        self._reconstructor = StrokeReconstructor()

        self.max_ink_jump_m = float(max_ink_jump_m)

        # ContactStateDetector.__init__ does not initialise _was_drawing; only
        # reset() does.  Call it now to avoid AttributeError on the first event.
        self._contact.reset()

        self.last_position: Optional[Tuple[float, float]] = None
        self.last_uwb_position: Optional[Tuple[float, float]] = None
        self._last_ink_point: Optional[Tuple[float, float]] = None
        self._closed_strokes: List[Dict[str, Any]] = []
        self.diagnostics = TrackerDiagnostics()

    # ------------------------------------------------------------------
    # Public API expected by _prototype_thread.py
    # ------------------------------------------------------------------
    def get_bbox(self) -> Dict[str, float]:
        """Return the physical drawing bounds in metres.

        Independent x/y keys, with ``b_min``/``b_max`` kept for older code paths
        that assumed a square board.
        """
        x_max = float(cfg.anchors.board_size_x)
        y_max = float(cfg.anchors.board_size_y)
        return {
            "x_min": 0.0,
            "x_max": x_max,
            "y_min": 0.0,
            "y_max": y_max,
            "b_min": 0.0,
            "b_max": max(x_max, y_max),
        }

    def process_packet(
        self, raw_line: Union[bytes, str, Dict[str, Any]]
    ) -> Optional[StrokeResult]:
        """Feed one raw ``readline()`` result through the full pipeline."""
        try:
            packet = self._parse_line(raw_line)
            if packet is None:
                self.diagnostics.dropped_lines += 1
                return None

            events = self._norm.normalize([packet])
            if not events:
                return None

            # Events are buffered and re-sorted so an IMU sample that arrived
            # before an earlier-stamped UWB frame is still fused in hardware
            # time order.
            self._aligner.add_events(events)
            sorted_events = self._aligner.get_all_sorted()
            self._aligner.clear()

            result: Optional[StrokeResult] = None
            for event in sorted_events:
                emitted = self._process_event(event)
                # Keep the newest emission: one readline can unblock several
                # buffered events, and the app draws the most recent point.
                if emitted is not None:
                    result = emitted
            return result
        except Exception as exc:
            self.diagnostics.last_error = str(exc)
            raise

    def drain_closed_strokes(self) -> List[Dict[str, Any]]:
        """Pop the strokes that finished since the last call.

        Each entry is a ``StrokeReconstructor`` stroke dict whose ``points`` are
        ``(x, y, ts_hw)`` after every postprocess stage has run.  These are the
        corrected shapes; the points streamed from ``process_packet`` are the
        uncorrected live ink for the same stroke.
        """
        drained = self._closed_strokes
        self._closed_strokes = []
        return drained

    def reset(self) -> None:
        """Reset all stateful pipeline stages (call on serial reconnect)."""
        self._norm.reset_stats()
        self._aligner.clear()
        self._imu_prep.reset()
        self._contact.reset()
        self._range_prep.reset()
        self._trilat.reset()
        self._pos_filt.reset()
        self._fusion.reset()
        self._reconstructor.reset()

        self.last_position = None
        self.last_uwb_position = None
        self._last_ink_point = None
        self._closed_strokes = []
        self.diagnostics = TrackerDiagnostics()

    def flush(self) -> Optional[Dict[str, Any]]:
        """Close any stroke still open, e.g. at end of session."""
        tail = self._reconstructor.flush()
        if tail:
            self.diagnostics.closed_strokes += 1
            self._closed_strokes.append(tail)
        return tail

    def stats(self) -> Dict[str, Any]:
        """Diagnostic counters, including the reconstructor's own."""
        return {
            "imu_packets": self.diagnostics.imu_packets,
            "uwb_packets": self.diagnostics.uwb_packets,
            "emitted_points": self.diagnostics.emitted_points,
            "closed_strokes": self.diagnostics.closed_strokes,
            "dropped_lines": self.diagnostics.dropped_lines,
            "last_stroke_state": self.diagnostics.last_stroke_state,
            "last_error": self.diagnostics.last_error,
            "reconstructor": self._reconstructor.stats(),
        }

    # ------------------------------------------------------------------
    # Event processing: mirrors visualizer.py::_process_event, minus the UI
    # ------------------------------------------------------------------
    def _process_event(self, event: Dict[str, Any]) -> Optional[StrokeResult]:
        sensor = event.get("sensor")

        if sensor == "IMU":
            self.diagnostics.last_packet_type = "imu"
            self.diagnostics.imu_packets += 1
            return self._process_imu_event(event)

        if sensor == "UWB":
            self.diagnostics.last_packet_type = "uwb"
            self.diagnostics.uwb_packets += 1
            self._process_uwb_event(event)
            return None

        return None

    def _process_imu_event(self, event: Dict[str, Any]) -> Optional[StrokeResult]:
        preprocessed = self._imu_prep.process_one(event)
        if not preprocessed:
            return None

        with_contact = self._contact.process_one(preprocessed)
        fused = self._fusion.process_event(with_contact)
        if not fused:
            return None

        # Reconstruction runs on every fused event, not only on drawing ones:
        # it is the transition to an inactive event that closes a stroke and
        # triggers the postprocess chain.
        closed = self._reconstructor.process_event(fused)
        if closed:
            self.diagnostics.closed_strokes += 1
            self.diagnostics.last_postprocess = closed.get("postprocess", {})
            self._closed_strokes.append(closed)

        fused_x = fused.get("fused_x")
        fused_y = fused.get("fused_y")
        if fused_x is None or fused_y is None:
            return None
        fused_x = float(fused_x)
        fused_y = float(fused_y)
        if not (math.isfinite(fused_x) and math.isfinite(fused_y)):
            return None

        self.last_position = (fused_x, fused_y)

        is_drawing = bool(fused.get("stroke_active", False))
        stroke_state = str(fused.get("state", "AIR_MOVE"))

        if is_drawing:
            # Teleport guard: a filter glitch mid-stroke would otherwise draw a
            # long straight line across the board.  Suppress the drawing flag
            # for that one sample so the app breaks the polyline, then re-anchor
            # so only the glitch segment is dropped and not the rest behind it.
            if self._last_ink_point is not None:
                jump = math.hypot(
                    fused_x - self._last_ink_point[0],
                    fused_y - self._last_ink_point[1],
                )
                if jump > self.max_ink_jump_m:
                    is_drawing = False
            self._last_ink_point = (fused_x, fused_y)
        else:
            self._last_ink_point = None

        self.diagnostics.last_stroke_state = stroke_state
        self.diagnostics.emitted_points += 1
        return (fused_x, fused_y, is_drawing, stroke_state)

    def _process_uwb_event(self, event: Dict[str, Any]) -> None:
        for ranged in self._range_prep.feed([event]):
            solved = self._trilat.process_one(ranged)
            if not solved:
                continue
            clean = self._pos_filt.process_one(solved)
            if not clean:
                continue

            position = clean.get("pos_clean")
            if position is not None:
                self.last_uwb_position = (float(position[0]), float(position[1]))

            # UWB events correct the filter and feed the reconstructor UWB
            # centroid buffer, but never lay down ink.
            fused = self._fusion.process_event(clean)
            if fused:
                self._reconstructor.process_event(fused)

    # ------------------------------------------------------------------
    # Line parsing
    # ------------------------------------------------------------------
    @staticmethod
    def _parse_line(
        raw_line: Union[bytes, str, Dict[str, Any]]
    ) -> Optional[Dict[str, Any]]:
        if raw_line is None:
            return None
        if isinstance(raw_line, dict):
            return raw_line
        if isinstance(raw_line, (bytes, bytearray)):
            text = bytes(raw_line).decode("utf-8", errors="ignore").strip()
        else:
            text = str(raw_line).strip()
        if not text:
            return None
        return parse_packet_line(text)


# Legacy alias: the previous bridge exported this name.
PipelineTracker = StrokeTracker
