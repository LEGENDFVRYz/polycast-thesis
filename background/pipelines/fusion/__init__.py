"""
Sensor fusion stage of the PolyCast pipeline.

Converts preprocessed IMU and UWB events into a single fused pen-tip position
per event. Two engines are available:

    ESKF          error-state Kalman filter, the production path
    FusionEngine  complementary filter, kept as a simpler baseline for A/B runs

Both expose the same `process_event(event) -> dict | None` interface, so callers
can swap engines without changing the surrounding pipeline.
"""

from background.pipelines.fusion.baseline import FusionEngine
from background.pipelines.fusion.eskf import ESKF
from background.pipelines.fusion.stroke_dead_reckoner import StrokeIMUDeadReckoner

__all__ = ['ESKF', 'FusionEngine', 'StrokeIMUDeadReckoner']
