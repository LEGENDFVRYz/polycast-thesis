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
    baud: int = 921600


# ------------------------------------------------------------------------
# IMU (Adafruit BNO085)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class IMUConfig:
    sample_rate_hz: float = 200.0

    # ZUPT
    zupt_acc_threshold: float = 0.15
    zupt_jerk_threshold: float = 8.5
    zupt_min_duration_s: float = 0.05

    # Contact / force
    force_contact_threshold: float = 100.0

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
    range_offsets_m: tuple = (-0.1752, -0.0466, -0.2227, -0.1620)
    # range_offsets_m: tuple = (-0.1752, -0.0466, -0.2227, -0.1220)
    # range_offsets_m: tuple = (-0.1232, -0.0146, -0.1919, -0.0965)

    rate_hz: float = 9.0

    ema_alpha: float = 0.25
    max_range_jump_m: float = 0.40
    median_window: int = 10

    outlier_speed_limit_ms: float = 2.0
    drop_speed_outliers: bool = True
    num_anchors: int = 4
    pos_ema_alpha: float = 0.75             # legacy — kept for reference, superseded by pos_alpha
    trilat_max_residual: float = 0.15

    # Alpha-Beta filter (replaces scalar EMA)
    pos_alpha: float = 0.60                 # position correction gain
    pos_beta: float = 0.10                  # velocity correction gain

    # Trilateration stale-guess recovery
    stale_guess_timeout_us: int = 1_000_000 # 1 second gap triggers centroid re-seed

    # Range filter time-aware EMA (tau = smoothing time constant)
    range_tau_s: float = 0.30               # at 9 Hz (dt≈0.11s): α ≈ 0.31; tune up to slow down

    # Weighted least squares (inverse distance weighting in trilateration)
    wls_power: float   = 2.0                # exponent: 2.0 = inverse-square, 1.0 = inverse-linear
    wls_epsilon: float = 0.01               # numerical guard prevents ÷0 at very-close anchors


# ------------------------------------------------------------------------
# ANCHOR GEOMETRY (Target Whiteboard)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class AnchorConfig:
    board_size_x: float = 1.25
    board_size_y: float = 1.24

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
# MARKER (physical layout)
#   Body frame: z_body = pen long axis, tip at origin, sensors stacked above.
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class MarkerConfig:
    r_imu_body_m: tuple[float, float, float] = (0.0, 0.0, 0.110)   # tip → IMU
    r_uwb_body_m: tuple[float, float, float] = (0.0, 0.0, 0.200)   # tip → UWB


# ------------------------------------------------------------------------
# FUSION (ESKF)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class FusionESKFConfig:
    # Process noise
    sigma_a: float           = 0.15      
    sigma_b_a: float         = 0.005     
    sigma_zupt: float        = 0.005     
    
    # Measurement noise 
    sigma_uwb: float         = 0.12      # Tightened for faster relocation
    sigma_trilat: float      = 0.05      
    k_nlos: float            = 0.00 
    r_scale_max: float       = 100.0
    
    # Raised to 6.0 to stop rejecting valid macro-movements
    hard_reject_mult: float  = 15.0       
    
    # Turn detection - Reactivated with high threshold
    turn_omega_threshold: float = 1.85    # ~140 dps (Ignores friction, catches corners)
    turn_k_q: float             = 3.0    # Forces a crisp corner
    turn_n_post: int            = 2      
    
    state_buffer_size: int      = 20
    # Initial covariance (diagonal, one value per block in m/s/m/s²)
    p0_pos: float    = 0.20
    p0_vel: float    = 0.10
    p0_bias: float   = 0.05


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
    marker: MarkerConfig = field(default_factory=MarkerConfig)
    fusion_eskf: FusionESKFConfig = field(default_factory=FusionESKFConfig)


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