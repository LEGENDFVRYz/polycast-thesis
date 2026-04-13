"""
[config] - Centralized Configuration for the "pipelines" of polycast

All of the configuration here are solely intended for main pipeline
modules, only to be used my the module inside this folder (pipelines)

Note: Main Pipelines - Transforming Marker Sensor to Stroke Coordinates

"""

from dataclasses import dataclass, field


# ------------------------------------------------------------------------
# SERIAL / HARDWARE
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class SerialConfig:
    port: str = "COM20"
    baud: int = 115200


# ------------------------------------------------------------------------
# IMU (Adafruit BNO085)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class IMUConfig:
    sample_rate_hz: float = 50.0

    # ZUPT
    zupt_acc_threshold: float = 1.15
    zupt_jerk_threshold: float = 22.5
    zupt_min_duration_s: float = 0.05

    # Contact / force
    force_contact_threshold: float = 1000.0

    # Physics
    gravity_ms2: float = 9.81
    quat_norm_epsilon: float = 1e-6

    # Board projection
    board_axes: tuple[str, str] = ("x", "z")
    smooth_alpha: float = 0.75

    acc_is_linear: bool = True


# ------------------------------------------------------------------------
# UWB (Ai Thinker BU03)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class UWBConfig:
    rate_hz: float = 9.0

    ema_alpha: float = 0.25
    max_range_jump_m: float = 0.20
    median_window: int = 5

    outlier_speed_limit_ms: float = 2.0
    num_anchors: int = 4


# ------------------------------------------------------------------------
# ANCHOR GEOMETRY (Target Whiteboard)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class AnchorConfig:
    board_size_m: float = 1.25

    a0: tuple[float, float, float] = (0.00, 0.00, 0.07)
    a1: tuple[float, float, float] = (1.25, 0.00, 0.07)
    a2: tuple[float, float, float] = (1.25, 1.24, 0.07)
    a3: tuple[float, float, float] = (0.00, 1.24, 0.07)

    positions: tuple[tuple[float, float, float], ...] = (
        (0.00, 0.00, 0.07),
        (1.25, 0.00, 0.07),
        (1.25, 1.24, 0.07),
        (0.00, 1.24, 0.07),
    )


# ------------------------------------------------------------------------
# PIPELINE (constraints)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class PipelineConfig:
    time_align_buffer: int = 2000
    imu_max_dt_ms: int = 100

    uwb_min_range_m: float = 0.05
    uwb_max_range_m: float = 6.00


# ------------------------------------------------------------------------
# ROOT CONFIG (wrapper)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class Config:
    serial: SerialConfig = field(default_factory=SerialConfig)
    imu: IMUConfig = field(default_factory=IMUConfig)
    uwb: UWBConfig = field(default_factory=UWBConfig)
    anchors: AnchorConfig = field(default_factory=AnchorConfig)
    pipeline: PipelineConfig = field(default_factory=PipelineConfig)


cfg = Config()


# ------------------------------------------------------------------------
# VALIDATION TESTING
# ------------------------------------------------------------------------
if __name__ == "__main__":
    print(cfg)

    assert cfg.imu.zupt_acc_threshold == 0.50
    assert cfg.uwb.ema_alpha == 0.25
    assert cfg.anchors.positions[0] == (0.00, 0.00, 0.07)

    try:
        cfg.imu.gravity_ms2 = 99
    except Exception:
        print("✓ Config is immutable")