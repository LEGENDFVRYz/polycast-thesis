"""
config.py  --  PolyCast shared configuration (Async Stream)
===========================================================
Single source of truth for anchor positions, UWB offsets,
marker geometry, and serial connection defaults.

After running calibrate.py, update UWB_OFFSETS here only.
"""

import json
from pathlib import Path

import numpy as np

# -- Serial defaults --------------------------------------------------------
SERIAL_PORT = 'COM5'
BAUD_RATE   = 921600

# -- Physical geometry ------------------------------------------------------
MARKER_LENGTH = 0.215   # m -- tip to UWB tag antenna

# Along the marker long axis from the pen tip (s=0) toward the rear:
#   s = 0                        -> pen tip (tactile button)
#   s = IMU_S_FROM_TIP_M = 0.13  -> BNO085 IMU
#   s = MARKER_LENGTH   = 0.21   -> UWB tag antenna (rear)
# Therefore (signed offsets measured from the UWB tag toward the tip):
#   p_imu = p_tag - IMU_OFFSET_FROM_TAG_M * e_w
#   p_tip = p_tag - TIP_OFFSET_FROM_TAG_M * e_w
# where e_w is the marker long-axis unit vector in world/whiteboard frame.
# Body frame convention (see imu_integrator.py docstring):
#   body X -> Z_wb (board normal). Marker axis in body frame = (1, 0, 0).
#   So e_w = R(q)[:, 0] (first column of the body->world rotation matrix).
IMU_S_FROM_TIP_M     = 0.125           # m -- IMU distance from tip
IMU_OFFSET_FROM_TAG_M = 0.09           # m -- tag to IMU along the marker axis
TIP_OFFSET_FROM_TAG_M = MARKER_LENGTH   # m -- tag to tip along the marker axis

# Anchor antenna positions (metres), measured antenna-to-antenna from A0.
# Z = mount height above whiteboard surface.
ANCHORS = np.array([
    [0.00, 0.00, 0.01],   # A0 -- bottom-left (origin)
    [1.25, 0.00, 0.01],   # A1 -- bottom-right
    [1.25, 1.20, 0.01],   # A2 -- top-right
    [0.00, 1.20, 0.01],   # A3 -- top-left
], dtype=float)

# -- UWB calibration offsets (from calibrate.py) ----------------------------
# Per-anchor distance correction applied to raw UWB measurements.
# Re-run calibrate.py and paste the new tuple here after each recalibration.
# UWB_OFFSETS = (-0.1024, -0.0080, -0.1415, -0.0756)
UWB_OFFSETS = (-0.1585, -0.0339, -0.1984, -0.1201)

# -- EKF feature flags ------------------------------------------------------
# Item C (Zou 2023, §3.3): feed the EKF posterior range back into each
# per-anchor 1-D range Kalman as a soft prior. Disabled by default — in
# low-multipath whiteboard testing it introduces state-measurement
# correlation that regresses closed-loop stability on curved strokes.
# Re-enable per A/B test if a regression is observed on open shapes.
ENABLE_ITEM_C_FEEDBACK = False

# -- Priority 5: pen-up hybrid A→B policy ----------------------------------
# Pen-down  → normal tightly-coupled raw-range EKF.
# HOVER_SHORT (hover < T_SHORT_S) → keep UWB but multiply R by ALPHA_R_HOVER.
# HOVER_LONG (hover ≥ T_SHORT_S) → skip UWB updates entirely.
# HOVER_H    (optional, feature-flagged) → 7-state hover-height EKF.
T_SHORT_S              = 0.150   # short-hover cutoff (also EKF7 enable time)
T_H_S                  = 0.200   # additional hold before HOVER_H is allowed
Q_SHORT                = 0.35    # mean anchor quality required for HOVER_SHORT
Q_H                    = 0.70    # mean anchor quality required for HOVER_H
ALPHA_R_HOVER          = 16.0    # R multiplier in HOVER_SHORT
HOVER_MIN_VALID_ANCHORS = 2      # required for HOVER_SHORT update
HOVER_H_MIN_VALID_ANCHORS = 3    # required for HOVER_H update
R_TOUCHDOWN_SCALE      = 4.0     # R inflation immediately after long-hover touchdown
TOUCHDOWN_REACQ_UPDATES = 2      # number of UWB updates the inflation lasts
GATE_CHI2_TOUCHDOWN    = 6.63    # widened χ² gate during touchdown reacq (99% CL)
NU_CLIP_M              = 0.25    # innovation clip used in adaptive R variance
LAMBDA_R_ADAPT         = 0.80    # EMA λ for adaptive R innovation variance
SIGMA_TOUCH_H_M        = 0.005   # 5 mm — pseudo-measurement σ for h→0 collapse
Q_H_RATE_MS            = 0.10    # m/√s — random-walk noise on hover height
H_INIT_M               = 0.01    # 1 cm — initial hover height when EKF7 spawns
P_H_INIT               = 0.03    # initial σ_h (m) on EKF7 spawn

# EKF7 hover-height branch — research extension, OFF by default per audit.
ENABLE_HOVER_H         = False


# -- Rate profile (Priority 1 — measured dt, not nominal Hz) ---------------
# Layer 0 (verification/layer0_stream_health.py) writes
# `rate_profile.json` next to this file after a verifier run. Every
# downstream module that needs a "rate" or a "samples per X seconds"
# must read it from RATE_PROFILE — never hardcode 50/100/200 Hz.
#
# Fallback values reflect the latest 50 Hz UWB capture (datasets_str_50hz)
# as observed in layer0 PNGs (BNO085 paired @ ~216 Hz, UWB @ ~50 Hz, FSR
# rides on the IMU stream so FSR_HZ = IMU_HZ).
_RATE_PROFILE_PATH = Path(__file__).resolve().parent / 'rate_profile.json'

_RATE_PROFILE_DEFAULTS = {
    'imu_hz':          216.0,
    'uwb_hz':           50.0,
    'fsr_hz':          216.0,        # FSR sampled per IMU packet
    'imu_dt_nom_s':      1.0 / 216.0,
    'uwb_dt_nom_s':      1.0 / 50.0,
    'fsr_dt_nom_s':      1.0 / 216.0,
    'imu_dt_p99_s':      0.020,      # 20 ms — anything above this is a spike
    'uwb_dt_p99_s':      0.050,      # 50 ms — same
    'fsr_dt_p99_s':      0.020,
    'source':            'defaults',
}

def _load_rate_profile() -> dict:
    """Load rate_profile.json if present; otherwise return defaults."""
    try:
        if _RATE_PROFILE_PATH.is_file():
            data = json.loads(_RATE_PROFILE_PATH.read_text(encoding='utf-8'))
            merged = dict(_RATE_PROFILE_DEFAULTS)
            merged.update(data)
            merged['source'] = str(_RATE_PROFILE_PATH)
            return merged
    except (OSError, ValueError):
        pass
    return dict(_RATE_PROFILE_DEFAULTS)

RATE_PROFILE = _load_rate_profile()

# Convenience scalars (read at import; modules cache as needed)
IMU_HZ        = float(RATE_PROFILE['imu_hz'])
UWB_HZ        = float(RATE_PROFILE['uwb_hz'])
FSR_HZ        = float(RATE_PROFILE['fsr_hz'])
IMU_DT_NOM_S  = float(RATE_PROFILE['imu_dt_nom_s'])
UWB_DT_NOM_S  = float(RATE_PROFILE['uwb_dt_nom_s'])
FSR_DT_NOM_S  = float(RATE_PROFILE['fsr_dt_nom_s'])


# -- Calibration profile (Phase 8 Step 2 — derived from real datasets) -----
# `tools/derive_calibration.py` writes `calibration_profile.json` here.
# Modules consume the per-key getters below with explicit fallbacks; if the
# JSON is missing or a key is absent, the legacy hardcoded constant in the
# corresponding module wins. NEVER mutate this dict at runtime.
_CALIBRATION_PATH = Path(__file__).resolve().parent / 'calibration_profile.json'

def _load_calibration_profile() -> dict:
    try:
        if _CALIBRATION_PATH.is_file():
            data = json.loads(_CALIBRATION_PATH.read_text(encoding='utf-8'))
            data.setdefault('source', str(_CALIBRATION_PATH))
            return data
    except (OSError, ValueError):
        pass
    return {'source': 'defaults (calibration_profile.json missing)'}

CALIBRATION_PROFILE = _load_calibration_profile()


def calibration_get(key: str, fallback):
    """Return CALIBRATION_PROFILE[key] if present and finite, else fallback.
    Lists pass through unchanged when present."""
    v = CALIBRATION_PROFILE.get(key)
    if v is None:
        return fallback
    if isinstance(v, list):
        return v
    try:
        f = float(v)
        if not (f == f) or f == float('inf'):  # NaN / Inf
            return fallback
        return f
    except (TypeError, ValueError):
        return fallback
