"""
Sensor fusion stage of the PolyCast pipeline.

Converts preprocessed IMU and UWB events into a single fused pen-tip position
per event via ESKF, the error-state Kalman filter that is the sole fusion
engine in this pipeline.
"""

from background.pipelines.fusion.eskf import ESKF
from background.pipelines.fusion.stroke_dead_reckoner import StrokeIMUDeadReckoner

__all__ = ['ESKF', 'StrokeIMUDeadReckoner']
