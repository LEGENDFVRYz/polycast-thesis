"""
Error-state Kalman filter state container.

Holds the nominal state that is propagated directly (position, velocity,
accelerometer bias, attitude) alongside the 6x6 error-state covariance, and
owns every operation that keeps those numbers physically and numerically valid:
covariance floors/caps, symmetry enforcement, and NaN recovery.

Error-state layout (6 elements, 2 per block):
    [0:2]  position error      dp
    [2:4]  velocity error      dv
    [4:6]  accel bias error    db_a

The visible output is `position + position_bias`. Splitting placement from
shape is deliberate: `position` carries the relative IMU stroke shape while
`position_bias` absorbs the slow global UWB correction, so letters keep their
form while still landing at the right board location.
"""

import numpy as np

from background.pipelines.config import cfg

STATE_DIM = 6
POSITION_SLICE = slice(0, 2)
VELOCITY_SLICE = slice(2, 4)
BIAS_SLICE = slice(4, 6)

# Physically meaningless for a ~1.25 x 1.20 m board; exists purely to stop the
# position-velocity cross terms from overflowing the Joseph-form matmul.
_COVARIANCE_ABS_LIMIT = 1e6

# The bias block has no other constraint when sigma_b_a is tiny, so it is pinned
# to a sane band to stop it silently absorbing all UWB/IMU disagreement.
_BIAS_VARIANCE_MIN = 1e-10
_BIAS_VARIANCE_MAX = 0.25


class ESKFState:
    """Nominal state, error covariance, and their validity invariants."""

    def __init__(self, board_width: float, board_height: float, position_floor_source):
        self.board_width = board_width
        self.board_height = board_height

        # Resolved live on every floor application rather than captured once per
        # frame: the active mode can change mid-frame (a planted tip forces
        # static mode), and the floor must track that change immediately.
        self._position_floor_source = position_floor_source

        self.position = np.array([board_width * 0.5, board_height * 0.5], dtype=float)
        self.velocity = np.zeros(2, dtype=float)
        self.accel_bias = np.zeros(2, dtype=float)
        self.position_bias = np.zeros(2, dtype=float)
        self.attitude = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

        self.covariance = self._initial_covariance()
        self.covariance_resets = 0

        # Most recent floor/cap actually applied, surfaced in diagnostics.
        self.last_position_floor = 0.025 ** 2
        self.last_position_cap = (0.025 * 6.0) ** 2

        # Set for exactly one emit after a NaN recovery so the caller can break
        # the rendered polyline instead of drawing a line to the snapped position.
        self.suppress_next_emit = False

        # Last-known good UWB tip, used as the re-anchor target on NaN recovery.
        self.last_uwb_tip = self.position.copy()

    # -------------------------------------------------------------------------
    # Construction helpers
    # -------------------------------------------------------------------------

    @staticmethod
    def _initial_covariance() -> np.ndarray:
        """Build the block-diagonal covariance from configured initial sigmas."""

        eskf_cfg = cfg.fusion_eskf
        variances = np.array([
            eskf_cfg.p0_pos,  eskf_cfg.p0_pos,
            eskf_cfg.p0_vel,  eskf_cfg.p0_vel,
            eskf_cfg.p0_bias, eskf_cfg.p0_bias,
        ]) ** 2
        return np.diag(variances)

    @property
    def visible_position(self) -> np.ndarray:
        """Position actually rendered as ink: nominal position plus global bias."""

        return self.position + self.position_bias

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.velocity))

    # -------------------------------------------------------------------------
    # Error-state application
    # -------------------------------------------------------------------------

    def apply_error_state(self, error_state: np.ndarray) -> None:
        """Inject a computed error-state correction into the nominal state."""

        self.position += error_state[POSITION_SLICE]
        self.velocity += error_state[VELOCITY_SLICE]
        self.accel_bias += error_state[BIAS_SLICE]
        self.clamp_accel_bias()

    def apply_joseph_covariance_update(
        self,
        kalman_gain: np.ndarray,
        observation_matrix: np.ndarray,
        measurement_noise: np.ndarray,
    ) -> None:
        """
        Update the covariance in Joseph form and restore its invariants.

        Joseph form is used over the shorter (I - KH)P because it stays positive
        semi-definite under floating-point error, which matters here since the
        gain is often hard-clipped and therefore not the optimal gain.
        """

        identity = np.eye(STATE_DIM)
        residual_transform = identity - kalman_gain @ observation_matrix
        self.covariance = (
            residual_transform @ self.covariance @ residual_transform.T
            + kalman_gain @ measurement_noise @ kalman_gain.T
        )
        self.sanitize_covariance()
        self.apply_covariance_floor()
        self.sanitize_nominal_state()

    # -------------------------------------------------------------------------
    # Numerical hygiene
    # -------------------------------------------------------------------------

    def apply_covariance_floor(self) -> None:
        """
        Clamp covariance diagonals into their configured working band.

        The position floor keeps the Kalman gain alive so UWB never stops being
        able to correct; the cap stops a long IMU-only stretch from inflating
        uncertainty to the point where a single UWB fix teleports the state.
        """

        eskf_cfg = cfg.fusion_eskf

        mode_position_floor = self._position_floor_source()
        position_floor = mode_position_floor ** 2
        position_cap = (mode_position_floor * eskf_cfg.pos_cap_mult) ** 2
        velocity_floor = eskf_cfg.vel_floor ** 2

        self.last_position_floor = position_floor
        self.last_position_cap = position_cap

        for index in (0, 1):
            bounded = max(self.covariance[index, index], position_floor)
            self.covariance[index, index] = min(bounded, position_cap)

        for index in (2, 3):
            self.covariance[index, index] = max(self.covariance[index, index], velocity_floor)

        np.clip(self.covariance, -_COVARIANCE_ABS_LIMIT, _COVARIANCE_ABS_LIMIT, out=self.covariance)

    def sanitize_covariance(self) -> None:
        """Recover from non-finite covariance, then re-impose symmetry and bias bounds."""

        if not np.all(np.isfinite(self.covariance)):
            self.reset_covariance()
            return

        # Repeated Joseph-form products accumulate asymmetry in the last bits;
        # left unchecked this eventually breaks the covariance inverse.
        self.covariance = 0.5 * (self.covariance + self.covariance.T)

        for index in (4, 5):
            bounded = max(_BIAS_VARIANCE_MIN, self.covariance[index, index])
            self.covariance[index, index] = min(bounded, _BIAS_VARIANCE_MAX)

    def reset_covariance(self) -> None:
        """
        Re-initialize after a non-finite covariance and re-anchor position to UWB.

        A finite-but-wrong position surviving the reset would be emitted as a
        teleport on the next frame, so it is snapped to the last good UWB tip.
        """

        self.covariance = self._initial_covariance()

        if self.last_uwb_tip is not None and np.all(np.isfinite(self.last_uwb_tip)):
            self.position[:] = self.last_uwb_tip
        else:
            self.position[:] = np.array([self.board_width * 0.5, self.board_height * 0.5])

        self.velocity[:] = 0.0
        self.accel_bias[:] = 0.0
        self.position_bias[:] = 0.0
        self.suppress_next_emit = True
        self.covariance_resets += 1
        print(f"[ESKF] P became non-finite - covariance reset (total: {self.covariance_resets})")

    def sanitize_nominal_state(self) -> None:
        """Replace any non-finite nominal state component with a safe default."""

        if not np.all(np.isfinite(self.accel_bias)):
            self.accel_bias[:] = 0.0
        if not np.all(np.isfinite(self.velocity)):
            self.velocity[:] = 0.0
        if not np.all(np.isfinite(self.position)):
            self.position[:] = np.array([self.board_width * 0.5, self.board_height * 0.5])
        if not np.all(np.isfinite(self.position_bias)):
            self.position_bias[:] = 0.0

    def clamp_accel_bias(self) -> None:
        """
        Hold the accelerometer bias inside a physically plausible band.

        `sigma_b_a` already asserts the bias is very nearly constant, but nothing
        enforced a magnitude, so a run of large innovations could walk it far
        past anything the sensor could actually exhibit. On abc_extralarge it
        reached 0.71 m/s^2 - integrating to 0.36 m/s of position error per second
        and driving the state off the board, after which every UWB fix failed the
        board-margin gate and no correction path was left.

        A BNO085 does not exhibit a bias this large; anything approaching it is an
        estimation failure, so it is bounded rather than trusted.
        """

        limit = cfg.fusion_eskf.accel_bias_max_ms2
        if limit <= 0.0:
            return

        # Bounded by norm rather than per axis: the physical claim is about the
        # magnitude of the bias vector, and a per-axis clamp would admit
        # limit*sqrt(2) along a diagonal. Scaling preserves the estimated
        # direction, which per-axis clipping would skew toward the axes.
        magnitude = float(np.linalg.norm(self.accel_bias))
        if magnitude > limit:
            self.accel_bias *= limit / magnitude

    def clamp_to_board(self) -> None:
        self.position[0] = max(0.0, min(self.board_width, self.position[0]))
        self.position[1] = max(0.0, min(self.board_height, self.position[1]))

    def snap_to_uwb(self, measurement: np.ndarray) -> None:
        """
        Re-localize onto a UWB fix after the innovation gate deadlocks.

        Position and velocity covariance return to their initial widths, but the
        bias estimate is kept: it converged over earlier good epochs and is not
        what went wrong.
        """

        eskf_cfg = cfg.fusion_eskf
        self.position[:] = measurement
        self.velocity[:] = 0.0
        self.covariance[0, 0] = self.covariance[1, 1] = eskf_cfg.p0_pos ** 2
        self.covariance[2, 2] = self.covariance[3, 3] = eskf_cfg.p0_vel ** 2

        # Cross terms tied the old, wrong position to the bias estimate; keeping
        # them would let the stale correlation immediately re-corrupt position.
        self.covariance[0:4, 4:6] = 0.0
        self.covariance[4:6, 0:4] = 0.0
        self.apply_covariance_floor()
