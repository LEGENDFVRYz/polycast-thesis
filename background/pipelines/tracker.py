"""
background.pipelines.tracker
============================

Headless EKF stroke-coordinate provider for the background drawing thread.

This module mirrors the coordinate path used by ``main_ekf.py`` for the blue
trace:

    raw serial line -> parser -> IMU/UWB preprocessors -> AsyncEKFFusionEngine
    -> calibrated pen-tip XY -> (x_m, y_m, is_drawing, stroke_state)

It intentionally contains no matplotlib/UI code.  Drop this file at:

    background/pipelines/tracker.py

and keep ``_prototype_thread.py`` importing:

    from background.pipelines.tracker import StrokeTracker
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple, Union

import numpy as np

# The EKF visualiser imports these as top-level modules.  In the app, this file
# may live under background/pipelines/, so support both project-root imports and
# package-relative imports.
try:  # project-root layout, same as main_ekf.py
    from preprocessor import UWBPreprocessor, IMUPreprocessor
    from ekf_fusion import AsyncEKFFusionEngine
except ImportError:  # package layout fallback
    from kuru_method.asynchronous_stream.preprocessor import UWBPreprocessor, IMUPreprocessor
    from kuru_method.asynchronous_stream.ekf_fusion import AsyncEKFFusionEngine

try:
    from config import UWB_OFFSETS, TIP_OFFSET_FROM_TAG_M
except Exception:  # keep import-time failures from hiding unrelated tests
    UWB_OFFSETS = [0.0, 0.0, 0.0, 0.0]
    TIP_OFFSET_FROM_TAG_M = 0.0


StrokeResult = Tuple[float, float, bool, str]


@dataclass
class TrackerDiagnostics:
    last_packet_type: str = "none"
    last_stroke_state: str = "AIR_MOVE"
    last_error: str = ""
    imu_packets: int = 0
    uwb_packets: int = 0
    emitted_points: int = 0


class StrokeTracker:
    """Convert raw serial packets into EKF pen-tip stroke coordinates.

    ``process_packet`` returns the same tuple shape expected by the legacy
    background thread:

        (meter_x, meter_y, is_drawing, stroke_state)

    where ``meter_x`` / ``meter_y`` are the calibrated pen-tip coordinates used
    as the blue trace in ``main_ekf.py``. UWB packets update the filter but do
    not emit drawable points, so they normally return ``None``.
    """

    # Match the board-space view used by main_ekf.py.
    DEFAULT_X_MIN = -0.30
    DEFAULT_X_MAX = 2.20
    DEFAULT_Y_MIN = -0.30
    DEFAULT_Y_MAX = 1.55

    # Small movement threshold to avoid drawing repeated zero-length segments.
    MIN_DRAW_STEP_M = 0.0005

    def __init__(
        self,
        *,
        uwb_offsets: Any = None,
        tip_offset_m: Optional[float] = None,
        min_draw_step_m: float = MIN_DRAW_STEP_M,
    ) -> None:
        self.engine = AsyncEKFFusionEngine()
        self.uwb_cleaner = UWBPreprocessor(offsets=UWB_OFFSETS if uwb_offsets is None else uwb_offsets)
        self.imu_cleaner = IMUPreprocessor()

        self.tip_offset_m = float(TIP_OFFSET_FROM_TAG_M if tip_offset_m is None else tip_offset_m)
        self.min_draw_step_m = float(min_draw_step_m)

        self.last_tip_pos: Optional[np.ndarray] = None
        self.last_tag_pos: Optional[np.ndarray] = None
        self.last_vel: Optional[np.ndarray] = None
        self._last_contact_tip: Optional[np.ndarray] = None
        self._prev_contact = False
        self.diagnostics = TrackerDiagnostics()

        # Keep EKF history enabled, matching main_ekf.py.  This is useful if the
        # engine uses history internally and harmless for the headless adapter.
        try:
            self.engine.ekf.record_history = True
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Public API expected by _prototype_thread.py
    # ------------------------------------------------------------------
    def get_bbox(self) -> Dict[str, float]:
        """Return the physical drawing bounds in metres.

        New callers should use independent x/y keys.  ``b_min``/``b_max`` are
        kept for older code paths that assumed a square board.
        """
        x_min, x_max = self.DEFAULT_X_MIN, self.DEFAULT_X_MAX
        y_min, y_max = self.DEFAULT_Y_MIN, self.DEFAULT_Y_MAX

        # If anchors are available and extend past the default view, include
        # them with a small margin.  This keeps the mapper robust to custom rigs.
        try:
            anchors = np.asarray(self.engine.anchors, dtype=float)
            if anchors.ndim == 2 and anchors.shape[1] >= 2 and len(anchors):
                x_min = min(x_min, float(np.nanmin(anchors[:, 0])) - 0.10)
                x_max = max(x_max, float(np.nanmax(anchors[:, 0])) + 0.10)
                y_min = min(y_min, float(np.nanmin(anchors[:, 1])) - 0.10)
                y_max = max(y_max, float(np.nanmax(anchors[:, 1])) + 0.10)
        except Exception:
            pass

        return {
            "x_min": float(x_min),
            "x_max": float(x_max),
            "y_min": float(y_min),
            "y_max": float(y_max),
            "b_min": float(min(x_min, y_min)),
            "b_max": float(max(x_max, y_max)),
        }

    def process_packet(self, raw_line: Union[bytes, str, Dict[str, Any]]) -> Optional[StrokeResult]:
        """Process one serial packet and maybe emit a stroke coordinate."""
        try:
            pkt = self._parse_packet(raw_line)
            if not pkt:
                return None

            self.diagnostics.last_packet_type = pkt["type"]

            if pkt["type"] == "imu":
                self.diagnostics.imu_packets += 1
                return self._process_imu(pkt)

            if pkt["type"] == "uwb":
                self.diagnostics.uwb_packets += 1
                self._process_uwb(pkt)
                return None

            return None
        except Exception as exc:
            self.diagnostics.last_error = str(exc)
            raise

    def reset(self) -> None:
        """Reset filter and stroke state."""
        self.__init__(tip_offset_m=self.tip_offset_m, min_draw_step_m=self.min_draw_step_m)

    # ------------------------------------------------------------------
    # EKF processing: mirrors the relevant non-UI path from main_ekf.py
    # ------------------------------------------------------------------
    def _process_imu(self, pkt: Dict[str, Any]) -> Optional[StrokeResult]:
        clean = self.imu_cleaner.process_sample(*pkt["quat"], *pkt["acc"])
        pkt["quat"] = clean[0:4]
        pkt["acc"] = clean[4:7]

        tag_pos, vel, is_writing = self.engine.process_imu(pkt)
        if tag_pos is None:
            return None

        tip_pos = self._calibrated_tip_from_tag(tag_pos)
        self.last_tag_pos = np.asarray(tag_pos, dtype=float).copy()
        self.last_tip_pos = np.asarray(tip_pos, dtype=float).copy()
        self.last_vel = None if vel is None else np.asarray(vel, dtype=float).copy()

        contact = bool(is_writing)
        if not contact:
            self._prev_contact = False
            self._last_contact_tip = None
            stroke_state = "AIR_MOVE"
            is_drawing = False
        else:
            if self._last_contact_tip is None:
                step_m = 0.0
            else:
                step_m = float(np.linalg.norm(tip_pos[:2] - self._last_contact_tip[:2]))

            # First contact sample anchors the stroke.  Subsequent samples draw
            # only when there is meaningful movement.
            is_drawing = bool(self._prev_contact and step_m >= self.min_draw_step_m)
            stroke_state = "CONTACT_DRAWING" if is_drawing else "CONTACT_STATIC"
            self._last_contact_tip = np.asarray(tip_pos, dtype=float).copy()
            self._prev_contact = True

        self.diagnostics.last_stroke_state = stroke_state
        self.diagnostics.emitted_points += 1
        return (float(tip_pos[0]), float(tip_pos[1]), bool(is_drawing), stroke_state)

    def _process_uwb(self, pkt: Dict[str, Any]) -> None:
        raw = pkt["dists"]
        _filtered, weights, ekf_dists = self.uwb_cleaner.process(*raw)
        tag_pos, vel, _accepted, _rejected = self.engine.process_uwb(
            ekf_dists, weights, ts=pkt.get("ts")
        )

        if tag_pos is not None:
            self.last_tag_pos = np.asarray(tag_pos, dtype=float).copy()
            self.last_tip_pos = self._calibrated_tip_from_tag(tag_pos)
            self.last_vel = None if vel is None else np.asarray(vel, dtype=float).copy()

            # Same prediction feedback as main_ekf.py, using tilt-aware tag z.
            try:
                self.uwb_cleaner.update_predictions(
                    np.append(tag_pos, self.engine.tag_z_wb), self.engine.anchors
                )
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Pen-tip calibration: copied from the blue-trace path in main_ekf.py
    # ------------------------------------------------------------------
    def _projected_tip_axis_2d(self) -> np.ndarray:
        try:
            e3 = np.asarray(self.engine.marker_axis_wb, dtype=float).reshape(-1)
            if len(e3) < 2 or not np.all(np.isfinite(e3[:2])):
                return np.zeros(2, dtype=float)
            e2 = np.asarray(e3[:2], dtype=float)
            n2 = float(np.linalg.norm(e2))
            if n2 > 1.0:
                e2 = e2 / n2
            return e2
        except Exception:
            return np.zeros(2, dtype=float)

    def _calibrated_tip_from_tag(self, tag_pos: Any) -> np.ndarray:
        tag = np.asarray(tag_pos, dtype=float)[:2]
        try:
            e2 = self._projected_tip_axis_2d()
            if float(np.linalg.norm(e2)) < 1e-9:
                fallback = getattr(self.engine, "tip_position", None)
                if fallback is not None:
                    return np.asarray(fallback, dtype=float)[:2]
                return tag
            return tag - self.tip_offset_m * e2
        except Exception:
            fallback = getattr(self.engine, "tip_position", None)
            if fallback is not None:
                return np.asarray(fallback, dtype=float)[:2]
            return tag

    # ------------------------------------------------------------------
    # Serial packet parsing
    # ------------------------------------------------------------------
    def _parse_packet(self, raw_line: Union[bytes, str, Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        if raw_line is None:
            return None
        if isinstance(raw_line, dict):
            return raw_line

        if isinstance(raw_line, bytes):
            text = raw_line.decode("utf-8", errors="ignore").strip()
        else:
            text = str(raw_line).strip()

        if not text:
            return None

        # Optional JSON form for tests/web bridges.
        if text.startswith("{"):
            data = json.loads(text)
            return self._normalise_json_packet(data)

        parts = [p.strip() for p in text.split(",")]
        kind = parts[0].upper() if parts else ""

        if kind == "I":
            return self._parse_imu_csv(parts)
        if kind == "U":
            return self._parse_uwb_csv(parts)
        return None

    def _parse_imu_csv(self, parts: list[str]) -> Optional[Dict[str, Any]]:
        # New format from main_ekf live recorder:
        # I,seq,qx,qy,qz,qw,ax,ay,az,gx,gy,gz,force,ts
        if len(parts) >= 14:
            return {
                "type": "imu",
                "seq": self._to_int(parts[1]),
                "quat": tuple(float(parts[i]) for i in range(2, 6)),
                "acc": tuple(float(parts[i]) for i in range(6, 9)),
                "gyro": tuple(float(parts[i]) for i in range(9, 12)),
                "gyro_valid": True,
                "force": float(parts[12]),
                "ts": self._to_int(parts[13]),
            }

        # Legacy format without gyro:
        # I,seq,qx,qy,qz,qw,ax,ay,az,force,ts
        if len(parts) >= 11:
            return {
                "type": "imu",
                "seq": self._to_int(parts[1]),
                "quat": tuple(float(parts[i]) for i in range(2, 6)),
                "acc": tuple(float(parts[i]) for i in range(6, 9)),
                "gyro": None,
                "gyro_valid": False,
                "force": float(parts[9]),
                "ts": self._to_int(parts[10]),
            }
        return None

    def _parse_uwb_csv(self, parts: list[str]) -> Optional[Dict[str, Any]]:
        # U,seq,d0,d1,d2,d3,ts
        if len(parts) < 7:
            return None
        return {
            "type": "uwb",
            "seq": self._to_int(parts[1]),
            "dists": tuple(float(parts[i]) for i in range(2, 6)),
            "ts": self._to_int(parts[6]),
        }

    def _normalise_json_packet(self, data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        typ = str(data.get("type", data.get("kind", ""))).lower()
        if typ == "imu":
            return {
                "type": "imu",
                "seq": self._to_int(data.get("seq", 0)),
                "quat": tuple(data["quat"]),
                "acc": tuple(data["acc"]),
                "gyro": tuple(data["gyro"]) if data.get("gyro") is not None else None,
                "gyro_valid": bool(data.get("gyro_valid", data.get("gyro") is not None)),
                "force": float(data.get("force", 0.0)),
                "ts": self._to_int(data.get("ts", 0)),
            }
        if typ == "uwb":
            return {
                "type": "uwb",
                "seq": self._to_int(data.get("seq", 0)),
                "dists": tuple(data.get("dists", data.get("ranges"))),
                "ts": self._to_int(data.get("ts", 0)),
            }
        return None

    @staticmethod
    def _to_int(value: Any) -> int:
        try:
            if isinstance(value, str) and value.strip() == "":
                return 0
            return int(float(value))
        except Exception:
            return 0
