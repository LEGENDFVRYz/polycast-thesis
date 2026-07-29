"""
Fusion mode resolution.

The filter runs with different trust balances depending on what the pen is
physically doing. Each mode supplies its own UWB sigma scale, velocity drag,
gain cap and IMU acceleration scale from the config mode table:

    CONTACT_DRAWING  normal ink        IMU owns shape, UWB nudges placement
    DRAWING_FAST     fast ink burst    brief extra IMU authority
    AIR_MOVE         pen lifted        UWB re-anchors, IMU suppressed
    IDLE             held still        UWB re-anchors hard, IMU silent

This module also owns the DRAWING_FAST arming state machine and the
stroke-age gain-cap ramp, keeping that bookkeeping out of the filter.
"""

from background.pipelines.config import cfg

MODE_CONTACT_DRAWING = 'CONTACT_DRAWING'
MODE_DRAWING_FAST = 'DRAWING_FAST'
MODE_AIR_MOVE = 'AIR_MOVE'
MODE_IDLE = 'IDLE'
MODE_CONTACT_STATIC_LOCK = 'CONTACT_STATIC_LOCK'

# Contact substates from contact.py that mean "planted or not moving", as
# opposed to AIR_MOVE which still deserves the air mode's UWB re-anchoring.
_STATIONARY_STATES = ('IDLE', 'CONTACT_STATIC')


class FusionModeTracker:
    """Tracks which fusion mode is active and supplies its tuning parameters."""

    def __init__(self):
        self.stroke_active = False
        self.stroke_state = 'UNKNOWN'
        self.in_fast_mode = False

        # Surfaced in diagnostics to make fast-mode triggering observable in tuning.
        self.fast_arm_count = 0
        self.fast_burst_count = 0

        self.stroke_start_ts: int | None = None

    # -------------------------------------------------------------------------
    # Fast-mode arming
    # -------------------------------------------------------------------------

    def update_fast_mode(self, speed_ms: float):
        """
        Advance the DRAWING_FAST state machine and return the active mode params.

        Call exactly once per IMU frame - this mutates the arming counters.
        Fast mode arms only after sustained fast motion, then holds for a bounded
        burst so a short fast stroke gets IMU authority without letting the
        filter dead-reckon indefinitely.
        """

        eskf_cfg = cfg.fusion_eskf

        if not self.stroke_active:
            self.fast_arm_count = 0
            self.fast_burst_count = 0
            self.in_fast_mode = False
            return eskf_cfg.modes.air

        if speed_ms >= eskf_cfg.drawing_fast_speed_thresh:
            self.fast_arm_count += 1
        else:
            # Decay rather than reset: a single slow frame mid-burst should not
            # disarm a genuinely fast stroke.
            self.fast_arm_count = max(0, self.fast_arm_count - 1)

        # Re-arm the hold-down on every frame the trigger condition still holds.
        if self.fast_arm_count >= eskf_cfg.drawing_fast_min_frames:
            self.fast_burst_count = eskf_cfg.drawing_fast_burst_frames
        elif self.fast_burst_count > 0:
            self.fast_burst_count -= 1

        self.in_fast_mode = self.fast_burst_count > 0
        return eskf_cfg.modes.drawing_fast if self.in_fast_mode else eskf_cfg.modes.drawing

    def force_stationary(self):
        """Drop fast-mode arming when the tip is detected as planted."""

        self.in_fast_mode = False
        self.fast_arm_count = 0
        self.fast_burst_count = 0

    # -------------------------------------------------------------------------
    # Pure reads
    # -------------------------------------------------------------------------

    def current_parameters(self):
        """
        Return the active mode's parameters without side effects.

        Safe to call from the UWB path and covariance helpers, which must not
        disturb the arming counters owned by the IMU path.
        """

        eskf_cfg = cfg.fusion_eskf
        if self.stroke_active:
            return eskf_cfg.modes.drawing_fast if self.in_fast_mode else eskf_cfg.modes.drawing
        if self.stroke_state in _STATIONARY_STATES:
            return eskf_cfg.modes.static
        return eskf_cfg.modes.air

    def current_name(self, tip_locked: bool = False) -> str:
        """Human-readable mode name matching current_parameters()."""

        if tip_locked:
            return MODE_CONTACT_STATIC_LOCK
        if self.stroke_active:
            return MODE_DRAWING_FAST if self.in_fast_mode else MODE_CONTACT_DRAWING
        if self.stroke_state in _STATIONARY_STATES:
            return MODE_IDLE
        return MODE_AIR_MOVE

    def stroke_age_gain_multiplier(self, ts: int) -> float:
        """
        Gain-cap multiplier that grows with the age of the active stroke.

        Long strokes accumulate more IMU drift than short ones, so relaxing the
        cap lets UWB pull harder the longer the pen stays down. Returns 1.0 when
        no stroke is open so air and idle caps are unaffected.

        Currently inert: age_ramp_mult_max is 1.0 in config because measured abc
        handwriting stroke durations overlapped geometric-shape durations, making
        the ramp fire on letters it was not meant to touch. Retained so it can be
        re-enabled per dataset.
        """

        if self.stroke_start_ts is None:
            return 1.0

        eskf_cfg = cfg.fusion_eskf
        age_s = (ts - self.stroke_start_ts) * 1e-6

        if age_s <= eskf_cfg.age_ramp_start_s:
            return 1.0
        if age_s >= eskf_cfg.age_ramp_end_s:
            return eskf_cfg.age_ramp_mult_max

        ramp_span = eskf_cfg.age_ramp_end_s - eskf_cfg.age_ramp_start_s
        ramp_fraction = (age_s - eskf_cfg.age_ramp_start_s) / ramp_span
        return 1.0 + ramp_fraction * (eskf_cfg.age_ramp_mult_max - 1.0)

    def stroke_age_s(self, ts: int) -> float:
        if self.stroke_start_ts is None:
            return 0.0
        return (ts - self.stroke_start_ts) * 1e-6
