"""
Centralized configuration for the PolyCast pipeline stages.

Read only by modules under background/pipelines/.

    IMU     short-term stroke detail / fast motion shape
    UWB     long-term absolute anchor / drift correction
    Force   stroke state / drawing mode selector

Split across `sensors`, `fusion` and `postprocess`; everything is re-exported
here, so `from background.pipelines.config import cfg` is unchanged.
"""

from dataclasses import dataclass, field

from background.pipelines.config._meta import inert
from background.pipelines.config.fusion import (
    FusionESKFConfig,
    FusionModeParams,
    FusionModeTable,
    PipelineConfig,
    StrokeDeadReckonerConfig,
)
from background.pipelines.config.postprocess import (
    CentroidAlignConfig,
    IMUDegeneracyConfig,
    MinJerkConfig,
    PostprocessConfig,
    StrokeCleanerConfig,
    TraceFilterConfig,
    TwoPointAnchorConfig,
    VelocityDetrendConfig,
)
from background.pipelines.config.sensors import (
    AnchorConfig,
    ContactConfig,
    IMUConfig,
    MarkerConfig,
    SerialConfig,
    UWBConfig,
)


@dataclass(frozen=True)
class Config:
    serial: SerialConfig = field(default_factory=SerialConfig)
    imu: IMUConfig = field(default_factory=IMUConfig)
    contact: ContactConfig = field(default_factory=ContactConfig)
    uwb: UWBConfig = field(default_factory=UWBConfig)
    anchors: AnchorConfig = field(default_factory=AnchorConfig)
    pipeline: PipelineConfig = field(default_factory=PipelineConfig)
    marker: MarkerConfig = field(default_factory=MarkerConfig)
    fusion_eskf: FusionESKFConfig = field(default_factory=FusionESKFConfig)
    stroke_cleaner: StrokeCleanerConfig = field(default_factory=StrokeCleanerConfig)
    postprocess: PostprocessConfig = field(default_factory=PostprocessConfig)
    velocity_detrend: VelocityDetrendConfig = field(default_factory=VelocityDetrendConfig)
    two_point_anchor: TwoPointAnchorConfig = field(default_factory=TwoPointAnchorConfig)
    imu_degeneracy: IMUDegeneracyConfig = field(default_factory=IMUDegeneracyConfig)
    trace_filter: TraceFilterConfig = field(default_factory=TraceFilterConfig)


# Module-level singleton every pipeline stage imports.
cfg = Config()


# `POLYCAST_MODE` applies a named configuration from modes.py; unset keeps the
# values above. Applying it here centralizes mode selection in the config.
#
# Kept at the bottom because modes.py imports `cfg`. The existing singleton
# resolves the cycle, while the guard preserves defaults if mode loading fails.
def _apply_environment_mode() -> str:
    try:
        from background.pipelines import modes
    except Exception:
        return ''
    return modes.apply_from_environment()


ACTIVE_MODE = _apply_environment_mode()
