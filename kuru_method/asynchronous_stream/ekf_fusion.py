"""
ekf_fusion.py  —  PolyCast UWB + IMU Extended Kalman Filter (Async Stream)
==========================================================================
Architecture: tightly coupled, 6-state EKF with event-driven processing.

State vector
------------
    x = [px, py, vx, vy, bax, bay]

    px, py   — 2D position on whiteboard (m)
    vx, vy   — velocity (m/s)
    bax, bay — accelerometer bias in whiteboard frame (m/s^2)

Event-driven processing (replaces batched update)
---------------------------------------------------
    IMU packet arrives (~200 Hz)  ->  EKF predict step
    UWB packet arrives (~10 Hz)   ->  EKF measurement update

    Each packet carries its own micros() timestamp for exact dt computation.
    No stale UWB detection needed — each UWB packet is inherently fresh.

Predict step  (IMU-driven, ~200 Hz, dt from timestamps)
-------------------------------------------------------
    Input: a_wb = [ax_wb, ay_wb]  from IMUIntegrator
    Same constant-acceleration model as batched version.

Update step  (UWB-driven, ~10 Hz, per anchor, scalar measurement)
------------------------------------------------------------------
    Tightly coupled: each raw UWB range is one scalar measurement.
    2D-projected range: d_2d_meas = sqrt( max(0, d_raw^2 - dz_i^2) )
    Sequential per-anchor update with chi^2 gating.
"""

from collections import deque

import numpy as np
from config import (ANCHORS, MARKER_LENGTH, TIP_OFFSET_FROM_TAG_M,
                    ENABLE_ITEM_C_FEEDBACK, IMU_HZ,
                    ALPHA_R_HOVER, R_TOUCHDOWN_SCALE, GATE_CHI2_TOUCHDOWN,
                    NU_CLIP_M, LAMBDA_R_ADAPT,
                    calibration_get)
from pen_mode import PenMode
from imu_integrator import IMUIntegrator, quat_to_rotmat
from force_detector import ForceContactDetector
from range_kf       import PerAnchorRangeKFBank


# Neutral-pose marker axis in whiteboard frame: board normal (wb-Z = +1).
# Used as a fallback when no quaternion-derived axis is supplied.
_NEUTRAL_MARKER_AXIS_WB = np.array([0.0, 0.0, 1.0])


# -------------------------------------------------------------------------
#  TIGHTLY COUPLED EKF
# -------------------------------------------------------------------------
class TightlyCoupledEKF:
    """
    Core 6-state EKF.  Called from AsyncEKFFusionEngine — do not call directly.
    """

    # -- Tuning -------------------------------------------------------------

    # Process noise
    SIGMA_ACC  = 2.0     # m/s^2  — IMU DR diagnostic on 50 Hz datasets shows
                         # 0.4–1.1 m drift in 2 s on curves/characters (imu_validator
                         # PNGs + layer4 metrics). The IMU's effective accel noise
                         # on motion is larger than 1.5 m/s^2; raising Q here lets
                         # the EKF downweight noisy IMU predictions and trust UWB
                         # more during strokes. Stationary behavior is unaffected
                         # because ZUPT + bias estimation dominate there.
                         # Kept as constant — see derive_calibration.py docstring
                         # for why the measured stationary σ is *not* used here
                         # (this is process noise, not sensor noise).
    # Phase 8 Step 2: σ_bias is *not* taken from the calibration profile
    # because the simple Allan-style estimator over-estimates the random
    # walk rate when the dataset includes hand handling / placement noise.
    # The legacy 0.0005 m/s²/√s remains the default; revisit when a clean
    # stationary dataset (no marker handling) is available for σ_bias.
    SIGMA_BIAS = 0.0005

    # Measurement noise
    SIGMA_UWB      = 0.08   # m — Kalman-update R (quality-weighted, was 0.15)
    GATE_SIGMA_UWB = 0.12   # m — gate-only R (tight, fixed, not quality-weighted)

    # Chi-squared gate threshold (1 degree of freedom, scalar range measurement)
    GATE_CHI2  = 3.841   # 95% CL  (was 6.635 / 99% — tighter outlier rejection)

    # Hard absolute innovation clamp (state-independent safety net)
    # Lower hard innovation clamp: reject any |innov| beyond 18 cm. A tighter
    # bound prevents rare large UWB spikes from yanking the state, which was
    # observed to cause "teleporting" artefacts when running live at 50 Hz.
    MAX_INNOV_M = 0.18   # m — reject any |innov| > 18 cm regardless of P

    # Per-anchor NLOS muting
    NLOS_MUTE_THRESH = 25    # consecutive rejections before muting
    NLOS_MUTE_CYCLES = 50   # UWB cycles to skip while muted
    NLOS_MAX_MUTED   = 2    # never mute more than this many anchors at once

    # Prolonged UWB outage handling (all anchors rejected N consecutive packets)
    MAX_CONSEC_OUTAGE = 8
    VELOCITY_DAMP     = 0.85  # per-predict multiplier when UWB is blacked out

    # Lifted-pen velocity damping (applied every predict step when contact = 0)
    LIFTED_VEL_DAMP   = 0.80

    # Slow-motion velocity decay
    SLOW_SPEED_THRESH = 0.04   # m/s
    SLOW_VEL_DECAY    = 0.85

    # Hard velocity cap. Natural pen-on-board writing peaks at 0.5–1.0 m/s
    # even on big characters. 1.50 let pen-lift/relocate velocities leak into
    # the EKF state and triggered runaway re-inits mid-session.
    MAX_WRITING_SPEED = 1.00   # m/s

    # Zero-velocity update (ZUPT) — Fix B: bias-independent raw-accel variance
    # Priority 1: window expressed in seconds, not samples. The deque length
    # is computed from RATE_PROFILE.IMU_HZ at construction time so the same
    # threshold means the same wall-clock duration whether the stream is
    # 100 Hz or 200 Hz.
    ZUPT_WINDOW_S      = 0.030      # 30 ms of stillness
    # Phase 8 Step 2: ZUPT_VAR_THRESH is *not* read from the calibration
    # profile. The measured p99 of 30 ms-window stationary acc variance
    # came out at 0.18 (m/s²)² — too loose: it includes hand handling
    # spikes and overlaps with real writing motion, so ZUPT fires during
    # strokes and crushes the rendered letter shapes. The legacy 0.0025
    # (m/s²)² is conservative but rejects writing motion correctly.
    ZUPT_VAR_THRESH    = 0.0025
    ZUPT_R_VEL         = 0.0005     # (m/s)^2

    # Bias runaway safeguard: if ZUPT has not fired for this long while pen
    # is in contact, the bias state is suspect. Reset to zero and inflate
    # P[4:6] to let UWB re-learn it.
    ZUPT_STALL_S       = 5.0        # seconds
    BIAS_RESET_P       = 0.04       # (m/s^2)^2 — matches initial P_bias

    # Fix C: observability-aware bias freeze. Freezing only on raw speed
    # ignores that a short straight stroke still leaves bias unobservable.
    # We freeze when the velocity direction has been persistent over the
    # last ~0.5 s (cos of direction change > threshold) AND speed > limit.
    # Any curved / changing-direction motion unfreezes bias learning.
    BIAS_FREEZE_SPEED    = 0.20     # m/s — speed floor for freeze gate
    BIAS_FREEZE_DIR_WIN  = 0.50     # s   — direction-persistence window
    BIAS_FREEZE_DIR_COS  = 0.90     # cos(angle between old/new v) threshold
    BIAS_FREEZE_MIN_SPD  = 0.05     # m/s — ignore noise below this for dir.

    # Fix I: runaway-drift escape. If the EKF position disagrees with the
    # fresh IRLS fix by more than this distance for RUNAWAY_HOLD UWB cycles,
    # force a soft re-init (snap to IRLS, zero v/b, inflate P). This breaks
    # the feedback loop where large innovations -> all rejected -> NLOS mute
    # -> IMU-only drift -> even larger innovations.
    RUNAWAY_DIST_M   = 0.50   # m — EKF vs IRLS discrepancy
    RUNAWAY_HOLD     = 10     # consecutive UWB cycles to confirm
    # Strictness of the IRLS sanity check for runaway re-init: IRLS must be
    # within this distance of the anchor-defined board to be used as reset
    # target. Tighter than COLD_MARGIN (0.25) because mid-session we know
    # the pen has been on the board; an off-board IRLS is garbage, not a
    # fresh start. 0.10 m tolerates small overshoot (pen at board edge,
    # slight IRLS bias) without accepting wild jumps below y=0.
    RUNAWAY_MARGIN_M = 0.10   # m

    # -- init ---------------------------------------------------------------

    def __init__(self, anchors: np.ndarray, tag_z: float = MARKER_LENGTH):
        self.anchors  = np.asarray(anchors, dtype=float)  # (N, 3)
        # _tag_z is kept only as a legacy fallback (pre-quaternion cold-start,
        # IRLS, NLOS prediction). The real tag Z is computed per-update from
        # the marker-axis unit vector in whiteboard frame.
        self._tag_z   = tag_z

        self._x    = None   # state vector [px, py, vx, vy, bax, bay]
        self._P    = None   # 6x6 covariance

        # Diagnostics
        self._consec_outage   = 0
        self._zupt_count      = 0
        self._n_imu_steps     = 0
        self._n_uwb_accepted  = 0
        self._n_uwb_rejected  = 0

        # IRLS speed for velocity damping gating (fed from AsyncEKFFusionEngine)
        self._irls_speed = 0.0

        # Innovation-based adaptive measurement noise (per anchor).
        # alpha=0.20 gives ~10-sample response time (≈ 200ms @ 50 Hz UWB),
        # half of the previous 0.10 (≈ 400ms). The faster adaptation lets
        # R_eff inflate quickly on motion (so UWB influence drops) and
        # re-tighten on stationary segments.
        self._innov_var   = np.ones(len(anchors)) * 0.005
        self._innov_alpha = 0.20   # EMA smoothing for innovation variance

        # Phase 8 Step 2 attempted to use per-anchor R_i,0 from
        # calibration_profile.json (audit recommendation R = max(σ_LOS²,
        # 0.05²)). On the post-Step-4b verifier sweep the per-anchor
        # tightening produced 21–45% WORSE Layer-6 RMSE across all 40
        # datasets — the conservative global SIGMA_UWB was masking a real
        # bias in the UWB anchor offsets / positions. Calibration profile
        # is preserved on disk for diagnostics, but the EKF reads from the
        # global SIGMA_UWB until the underlying bias is fixed.
        # See verification/out/report_baseline.md for the comparison data.
        self._R_per_anchor = np.full(len(anchors), self.SIGMA_UWB ** 2)

        # Per-anchor NLOS muting state
        self._consec_reject = np.zeros(len(anchors), dtype=int)
        self._mute_remain   = np.zeros(len(anchors), dtype=int)

        # Priority 2: per-anchor dropout counter (samples where the raw
        # measurement was missing/NaN). Surfaced via verifier metrics so
        # Layer 0 / Layer 6 can audit transport health.
        self._dropout_count = np.zeros(len(anchors), dtype=int)
        self._uwb_cycles    = 0

        # RTS smoother history (offline CSV mode only)
        self.record_history   = False
        self._history         = []

        # Item B: per-anchor 1D range Kalman pre-filter (Zou §3.1).
        # Item C: same bank also accepts EKF posterior range as feedback.
        self._range_bank      = PerAnchorRangeKFBank(
            n_anchors      = len(self.anchors),
            sigma_meas     = self.SIGMA_UWB,
            sigma_feedback = 2.0 * self.SIGMA_UWB,
        )
        self._last_uwb_ts     = None   # micros() of previous UWB packet
        # Latest (raw, filtered) ranges for verification logging.
        self._last_raw_ranges      = np.full(len(self.anchors), np.nan)
        self._last_filtered_ranges = np.full(len(self.anchors), np.nan)

        # Fix B: raw-accel ring buffer for variance-based ZUPT.  Stores
        # whiteboard-frame acceleration *before* bias subtraction so a
        # drifting bias cannot keep ZUPT silent (self-referential failure).
        # Priority 1: maxlen sized from RATE_PROFILE so 30 ms is 30 ms.
        zupt_maxlen = max(2, int(round(self.ZUPT_WINDOW_S * IMU_HZ)))
        self._zupt_accel_buf   = deque(maxlen=zupt_maxlen)
        # Bias-stall safeguard: cumulative seconds since ZUPT last fired
        # while the pen is in contact. If it runs > ZUPT_STALL_S, velocity is
        # assumed non-trivial and bias is reset so UWB can re-learn.
        self._zupt_stall_s     = 0.0

        # Fix C: short velocity-direction window for observability-aware
        # bias freeze. Unit vectors of velocity over BIAS_FREEZE_DIR_WIN.
        vel_dir_maxlen = max(10,
                             int(round(self.BIAS_FREEZE_DIR_WIN * IMU_HZ)))
        self._vel_dir_buf      = deque(maxlen=vel_dir_maxlen)

        # Cached bias-freeze flag — updated every predict so that both
        # Q_bias (Fix H) and the UWB Kalman gain (Fix C) share one truth.
        self._bias_freeze      = False

    # -- predict step (IMU) -------------------------------------------------

    def predict(self, a_wb: np.ndarray, dt: float, is_contact: bool = True,
                mode: 'PenMode' = PenMode.PEN_DOWN):
        """
        IMU-driven predict step.

        Parameters
        ----------
        a_wb       : np.ndarray (2,)  whiteboard-frame linear acceleration (m/s^2)
        dt         : float            time step (s), typically 0.010
        is_contact : bool             True = pen touching board
        """
        if self._x is None:
            return

        ax_wb, ay_wb = float(a_wb[0]), float(a_wb[1])

        # Subtract current bias estimate
        ax = ax_wb - self._x[4]
        ay = ay_wb - self._x[5]

        # Velocity damping
        if self._consec_outage > self.MAX_CONSEC_OUTAGE:
            self._x[2] *= self.VELOCITY_DAMP
            self._x[3] *= self.VELOCITY_DAMP
        elif not is_contact and self._irls_speed < 0.03:
            self._x[2] *= self.LIFTED_VEL_DAMP
            self._x[3] *= self.LIFTED_VEL_DAMP
        else:
            # Slow-motion decay (pen down, gentle creep suppression)
            spd = np.sqrt(self._x[2]**2 + self._x[3]**2)
            if spd < self.SLOW_SPEED_THRESH:
                self._x[2] *= self.SLOW_VEL_DECAY
                self._x[3] *= self.SLOW_VEL_DECAY

        # State transition (constant-acceleration model)
        dt2 = dt * dt
        self._x[0] += self._x[2] * dt + 0.5 * ax * dt2
        self._x[1] += self._x[3] * dt + 0.5 * ay * dt2
        self._x[2] += ax * dt
        self._x[3] += ay * dt

        # Hard velocity cap
        spd = np.sqrt(self._x[2]**2 + self._x[3]**2)
        if spd > self.MAX_WRITING_SPEED:
            scale = self.MAX_WRITING_SPEED / spd
            self._x[2] *= scale
            self._x[3] *= scale

        # Jacobian F
        F = np.array([
            [1, 0, dt,  0, -0.5*dt2, 0        ],
            [0, 1,  0, dt,  0,       -0.5*dt2 ],
            [0, 0,  1,  0, -dt,       0       ],
            [0, 0,  0,  1,  0,       -dt      ],
            [0, 0,  0,  0,  1,        0       ],
            [0, 0,  0,  0,  0,        1       ],
        ], dtype=float)

        # Process noise Q
        sa2 = self.SIGMA_ACC  ** 2
        sb2 = (self.SIGMA_BIAS * np.sqrt(dt)) ** 2
        Q   = np.zeros((6, 6))
        Q[0, 0] = sa2 * dt2*dt2 / 4
        Q[1, 1] = sa2 * dt2*dt2 / 4
        Q[0, 2] = Q[2, 0] = sa2 * dt2 * dt / 2
        Q[1, 3] = Q[3, 1] = sa2 * dt2 * dt / 2
        Q[2, 2] = sa2 * dt2
        Q[3, 3] = sa2 * dt2
        # Fix H: don't grow P_bias when bias is frozen.  Adding Q_bias while
        # the gain is zero just inflates covariance so that the first post-
        # unfreeze innovation snaps bias to a noisy value.
        if not self._bias_freeze:
            Q[4, 4] = sb2
            Q[5, 5] = sb2

        # Covariance predict
        self._P = F @ self._P @ F.T + Q
        self._P = 0.5 * (self._P + self._P.T)   # enforce symmetry
        self._n_imu_steps += 1

        # Fix C: maintain short velocity-direction window for observability.
        # Only include samples above the ignore-noise floor — otherwise the
        # direction is dominated by numerical jitter at near-zero speed.
        spd_now = np.sqrt(self._x[2]**2 + self._x[3]**2)
        if spd_now > self.BIAS_FREEZE_MIN_SPD:
            self._vel_dir_buf.append(
                (self._x[2] / spd_now, self._x[3] / spd_now))

        # Refresh bias-freeze decision for the next predict / UWB update.
        self._bias_freeze = self._compute_bias_freeze()

        # Fix B: variance-based ZUPT on RAW whiteboard accel (pre-bias).
        # This keeps ZUPT detection independent of the bias estimate, so a
        # drifting bias cannot silence the trigger.
        self._zupt_accel_buf.append((ax_wb, ay_wb))
        # Priority 5: ZUPT and bias-stall safeguard run in PEN_DOWN only.
        # In any hover mode the marker is moving in 3D and acceleration
        # variance no longer means "stationary on board", so ZUPT would
        # fire spuriously and freeze hover velocity.
        run_zupt = is_contact and (mode == PenMode.PEN_DOWN)
        if run_zupt:
            self.check_zupt()
            # Runaway safeguard: if ZUPT has not fired for ZUPT_STALL_S in a
            # row while in contact, the bias is suspect — reset it and
            # re-inflate the bias covariance so UWB can drive re-learning.
            self._zupt_stall_s += dt
            if self._zupt_stall_s >= self.ZUPT_STALL_S:
                self._x[4] = 0.0
                self._x[5] = 0.0
                self._P[4, 4] = self.BIAS_RESET_P
                self._P[5, 5] = self.BIAS_RESET_P
                self._P[4, 5] = 0.0
                self._P[5, 4] = 0.0
                self._zupt_stall_s = 0.0
                print("[EKF] Bias-stall safeguard fired — reset bias to 0")
        else:
            self._zupt_count   = 0
            self._zupt_stall_s = 0.0

        # RTS history: save state after each predict step
        if self.record_history:
            self._history.append({
                'x': self._x.copy(),
                'P': self._P.copy(),
                'F': F.copy(),
                'Q': Q.copy(),
            })

    # -- zero-velocity update (ZUPT) ----------------------------------------

    def _zupt_update(self):
        """Apply zero-velocity pseudo-measurement [vx, vy] = [0, 0]."""
        if self._x is None:
            return
        H = np.array([
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
        ], dtype=float)
        y = -self._x[2:4]                        # innovation = 0 - [vx, vy]
        R = np.eye(2) * self.ZUPT_R_VEL
        S = H @ self._P @ H.T + R
        K = self._P @ H.T @ np.linalg.inv(S)
        self._x = self._x + K @ y
        I_KH = np.eye(6) - K @ H
        self._P = I_KH @ self._P @ I_KH.T + K @ R @ K.T
        self._P = 0.5 * (self._P + self._P.T)

    def check_zupt(self):
        """
        Fix B: fire ZUPT when the *variance* of the raw (pre-bias) whiteboard
        acceleration across the last ZUPT_WINDOW samples falls below
        ZUPT_VAR_THRESH.

        Variance is bias-independent: a drifting DC offset (stale bias, tilt-
        induced gravity leakage, sensor warm-up drift) does not raise it.
        This is the SHOE-family detector recommended in the ZUPT literature
        (Wahlström & Skog 2021; Ren et al. 2018) for pedestrian / hand-held
        dead reckoning where the IMU bias is unknown at rest.
        """
        if len(self._zupt_accel_buf) < self._zupt_accel_buf.maxlen:
            return
        arr = np.asarray(self._zupt_accel_buf, dtype=float)
        var_total = float(np.var(arr[:, 0]) + np.var(arr[:, 1]))
        if var_total < self.ZUPT_VAR_THRESH:
            self._zupt_count += 1
            self._zupt_update()
            self._zupt_stall_s = 0.0
        else:
            self._zupt_count = 0

    def _compute_bias_freeze(self) -> bool:
        """
        Fix C: observability-aware bias freeze.  Accelerometer biases are
        unobservable during straight constant-velocity motion — freezing is
        right there.  For curved strokes (where observability is restored),
        unfreeze so the bias can learn from the direction-change geometry.

        Returns True iff we should zero K[4:6] on the next UWB update.
        """
        if self._x is None:
            return False
        spd = float(np.sqrt(self._x[2]**2 + self._x[3]**2))
        if spd <= self.BIAS_FREEZE_SPEED:
            return False
        # Insufficient direction history — fall back to raw-speed gate.
        if len(self._vel_dir_buf) < 10:
            return True
        v0 = self._vel_dir_buf[0]
        v1 = self._vel_dir_buf[-1]
        cos_change = float(v0[0] * v1[0] + v0[1] * v1[1])
        # Straight motion  -> cos ≈ 1   -> freeze (bias unobservable)
        # Curved motion    -> cos < thr -> unfreeze (bias observable)
        return cos_change > self.BIAS_FREEZE_DIR_COS

    # -- update step (UWB) --------------------------------------------------

    def update_uwb(self, raw_dists, quality_weights=None, marker_axis_wb=None,
                   ts=None, mode: 'PenMode' = PenMode.PEN_DOWN,
                   touchdown_reacq: bool = False):
        """
        UWB range measurement update (tightly coupled, one anchor at a time).

        Parameters
        ----------
        raw_dists       : array-like (4,)  UWB distances after offset correction (m)
        quality_weights : array-like (4,)  per-anchor quality [0.1, 1.0] or None
        marker_axis_wb  : array-like (3,)  unit vector of the marker long axis
                                           in WHITEBOARD frame (tip -> rear).
        mode            : PenMode          Priority 5 — drives R inflation and
                                           skip semantics:
                                             PEN_DOWN     -> normal R, normal gate
                                             HOVER_SHORT  -> R *= ALPHA_R_HOVER
                                             HOVER_LONG   -> hard skip; returns ([],[])
                                             HOVER_H      -> currently same path as
                                               HOVER_SHORT (full 7-state branch
                                               is feature-flagged stub for now)
        touchdown_reacq : bool             True for the first few updates after a
                                           long-hover → pen-down transition; widens
                                           the χ² gate and inflates R by
                                           R_TOUCHDOWN_SCALE so a stale state can
                                           catch the first valid range.

        Returns
        -------
        accepted : list[int]  anchor indices that passed the chi^2 gate
        rejected : list[int]  anchor indices that were rejected
        """
        if self._x is None:
            return [], []

        # Priority 5: HOVER_LONG skips the entire UWB path. The IMU predict
        # has already advanced state and inflated covariance; that is the
        # intended behaviour for sustained hover.
        if mode == PenMode.HOVER_LONG:
            return [], []

        # Per-mode R / gate scales — combined later with adaptive R.
        r_mode_scale = 1.0
        gate_chi2    = self.GATE_CHI2
        if mode == PenMode.HOVER_SHORT or mode == PenMode.HOVER_H:
            r_mode_scale = float(ALPHA_R_HOVER)
        if touchdown_reacq:
            r_mode_scale = max(r_mode_scale, float(R_TOUCHDOWN_SCALE))
            gate_chi2    = float(GATE_CHI2_TOUCHDOWN)

        n_anchors = len(self.anchors)
        if quality_weights is None:
            quality_weights = [1.0] * n_anchors

        # Marker axis in whiteboard frame -> current tag z (on-board tip).
        if marker_axis_wb is None:
            e_wb = _NEUTRAL_MARKER_AXIS_WB
        else:
            e_wb = np.asarray(marker_axis_wb, dtype=float)
        tag_z_wb = TIP_OFFSET_FROM_TAG_M * float(e_wb[2])

        # -- Item B: per-anchor 1D Kalman pre-filter on the raw ranges ----
        # Compute dt between successive UWB packets; default to 0.02 s on
        # the first packet (~50 Hz nominal cycle).
        if ts is not None and self._last_uwb_ts is not None:
            dt_us = ts - self._last_uwb_ts
            dt = dt_us / 1_000_000.0 if 0 < dt_us < 2_000_000 else 0.02
        else:
            dt = 0.02
        if ts is not None:
            self._last_uwb_ts = ts

        raw_arr = np.asarray(raw_dists, dtype=float)
        filtered_ranges = self._range_bank.step_all(dt, raw_arr)
        # Cache for verification / logging consumers.
        self._last_raw_ranges      = raw_arr.copy()
        self._last_filtered_ranges = filtered_ranges.copy()

        accepted, rejected = [], []
        px, py = self._x[0], self._x[1]
        n_currently_muted = int(np.sum(self._mute_remain > 0))
        self._uwb_cycles += 1

        for i in range(min(n_anchors, len(raw_dists))):
            # -- Priority 2: hard skip on missing/NaN ----------------------
            # No innovation, no chi^2 gate, no NLOS counter, no Joseph
            # update — a missing measurement must look missing to the EKF.
            raw_in = float(raw_dists[i]) if i < len(raw_dists) else float('nan')
            if not np.isfinite(raw_in):
                self._dropout_count[i] += 1
                continue

            # -- NLOS muting: skip anchors in mute cooldown ----------------
            if self._mute_remain[i] > 0:
                self._mute_remain[i] -= 1
                rejected.append(i)
                continue

            # Use the Item-B-filtered range for the EKF measurement.  Raw
            # is preserved in self._last_raw_ranges for verification logs.
            d_raw = float(filtered_ranges[i])
            qw    = float(quality_weights[i])

            # filtered_ranges[i] is also NaN if the per-anchor KF skipped
            # update this cycle — same hard skip applies.
            if not np.isfinite(d_raw) or d_raw < 0.05:
                self._dropout_count[i] += 1
                continue

            ax_i, ay_i, az_i = (self.anchors[i, 0],
                                self.anchors[i, 1],
                                self.anchors[i, 2])

            # Predicted 3D range from current state (tilt-aware):
            #   p_tag_wb = [px, py, 0.21 * e_wb_z]
            #   r3d      = || p_tag_wb - a_i ||
            dx = px - ax_i
            dy = py - ay_i
            dz = tag_z_wb - az_i
            r3d_pred = np.sqrt(dx*dx + dy*dy + dz*dz)
            if r3d_pred < 1e-6:
                continue

            # Innovation against the raw 3D range (no more 2D projection).
            innov = d_raw - r3d_pred

            # -- Hard innovation clamp (state-independent safety net) ------
            if abs(innov) > self.MAX_INNOV_M:
                self._n_uwb_rejected += 1
                self._consec_reject[i] += 1
                if (self._consec_reject[i] >= self.NLOS_MUTE_THRESH
                        and n_currently_muted < self.NLOS_MAX_MUTED):
                    self._mute_remain[i] = self.NLOS_MUTE_CYCLES
                    self._consec_reject[i] = 0
                    n_currently_muted += 1
                self._innov_var[i] *= 0.98
                rejected.append(i)
                continue

            # Measurement Jacobian H (1x6)
            # h(x) = sqrt(dx^2 + dy^2 + dz^2); dz is a known input per update.
            H = np.array([[dx / r3d_pred, dy / r3d_pred,
                           0.0, 0.0, 0.0, 0.0]])

            # -- Chi-squared gate: use TIGHT fixed R (not quality-weighted) -
            HPHT = float((H @ self._P @ H.T).item())
            R_gate = self.GATE_SIGMA_UWB ** 2
            S_gate = HPHT + R_gate
            mahal_sq = (innov * innov) / S_gate

            if mahal_sq > gate_chi2:
                self._n_uwb_rejected += 1
                self._consec_reject[i] += 1
                if (self._consec_reject[i] >= self.NLOS_MUTE_THRESH
                        and n_currently_muted < self.NLOS_MAX_MUTED):
                    self._mute_remain[i] = self.NLOS_MUTE_CYCLES
                    self._consec_reject[i] = 0
                    n_currently_muted += 1
                self._innov_var[i] *= 0.98
                rejected.append(i)
                continue

            # -- Accepted: update adaptive noise from clean innovation -----
            # Phase 8 Step 3 attempted the audit's capped-innovation EMA
            # (clip ν² to NU_CLIP_M² before the EMA, λ=0.80). Combined
            # with the per-anchor R it regressed Layer-6 RMSE; the legacy
            # uncapped EMA at α=0.20 wins until the underlying bias is
            # addressed. Keeping the constants imported in case a future
            # experiment wants them.
            self._innov_var[i] = ((1 - self._innov_alpha) * self._innov_var[i]
                                  + self._innov_alpha * innov * innov)
            _ = NU_CLIP_M, LAMBDA_R_ADAPT   # noqa — preserved for future use
            self._consec_reject[i] = 0

            # -- Kalman update uses quality-weighted R, scaled per pen mode
            # (Priority 5: ALPHA_R_HOVER for short hover; R_TOUCHDOWN_SCALE
            # for the first few updates after a long-hover touchdown).
            R_base = (self.SIGMA_UWB / max(qw, 0.1)) ** 2
            R_eff  = r_mode_scale * max(R_base, self._innov_var[i])
            S = HPHT + R_eff

            # Kalman gain (6x1)
            K = (self._P @ H.T) / S

            # Bias freeze — applied during pen-down only. In any hover
            # mode, freeze the bias unconditionally (the tag is moving in
            # 3D, not just XY, so persistence cannot grant observability).
            if self._bias_freeze or mode != PenMode.PEN_DOWN:
                K[4] = 0.0
                K[5] = 0.0

            # State update
            self._x = self._x + K.ravel() * innov

            # Clamp bias to physically realistic bounds (BNO085)
            self._x[4] = np.clip(self._x[4], -0.10, 0.10)
            self._x[5] = np.clip(self._x[5], -0.10, 0.10)

            px, py = self._x[0], self._x[1]

            # Covariance update — Joseph form for numerical stability
            I_KH    = np.eye(6) - K @ H
            self._P = I_KH @ self._P @ I_KH.T + (R_eff * K) @ K.T
            self._P = 0.5 * (self._P + self._P.T)

            accepted.append(i)
            self._n_uwb_accepted += 1

        # -- Item C: feedback correction (Zou §3.3) -----------------------
        # Push the EKF posterior 3D range back into each accepted anchor's
        # 1D Kalman as a soft prior.  Guardrails: no anchor in active mute,
        # no current outage, speed below the writing limit.  The whole path
        # is gated by config.ENABLE_ITEM_C_FEEDBACK so we can A/B test:
        # the closed-loop correlation it introduces regresses curved-stroke
        # accuracy in our low-multipath setup.
        speed = float(np.linalg.norm(self._x[2:4]))
        feedback_ok = (
            ENABLE_ITEM_C_FEEDBACK
            and accepted
            and int(np.sum(self._mute_remain > 0)) == 0
            and self._consec_outage == 0
            and speed < self.MAX_WRITING_SPEED
        )
        if feedback_ok:
            px_post, py_post = self._x[0], self._x[1]
            for i in accepted:
                ax_i, ay_i, az_i = (self.anchors[i, 0],
                                    self.anchors[i, 1],
                                    self.anchors[i, 2])
                dx = px_post - ax_i
                dy = py_post - ay_i
                dz = tag_z_wb - az_i
                r_post = float(np.sqrt(dx*dx + dy*dy + dz*dz))
                self._range_bank[i].update_feedback(r_post)

        # Update outage counter
        if accepted:
            self._consec_outage = 0
        else:
            self._consec_outage += 1

        return accepted, rejected

    # -- initialisation -----------------------------------------------------

    def initialize(self, px: float, py: float):
        """
        Cold-start: set initial position from IRLS fix.
        Velocity and bias start at zero.
        """
        self._x = np.array([px, py, 0.0, 0.0, 0.0, 0.0])
        self._P = np.diag([0.25, 0.25, 2.0, 2.0, 0.04, 0.04])
        print(f"[EKF] Initialised at ({px:.3f}, {py:.3f})")

    # -- properties ---------------------------------------------------------

    @property
    def position(self) -> np.ndarray:
        """2D UWB tag position on the whiteboard (m). See get_tip_position()
        for the pen-tip position, which is what matters for reconstruction."""
        return self._x[:2].copy() if self._x is not None else None

    def get_tip_position(self, marker_axis_wb=None) -> np.ndarray:
        """
        Pen-tip 2D position on the whiteboard.

        Geometry: p_tip = p_tag - TIP_OFFSET_FROM_TAG_M * e_w (tip-to-rear unit
        vector), so subtract the 2D component of the marker axis projected
        onto the whiteboard face.

        Parameters
        ----------
        marker_axis_wb : array-like (3,) or None
            Marker long-axis unit vector in whiteboard frame (from
            IMUIntegrator.get_marker_axis_wb).  If None, uses neutral pose
            (tip directly beneath tag on the board face).

        Returns
        -------
        np.ndarray (2,) tip position, or None if EKF not initialised.
        """
        if self._x is None:
            return None
        e_wb = (_NEUTRAL_MARKER_AXIS_WB if marker_axis_wb is None
                else np.asarray(marker_axis_wb, dtype=float))
        return self._x[:2] - TIP_OFFSET_FROM_TAG_M * e_wb[:2]

    def get_tag_z_wb(self, marker_axis_wb=None) -> float:
        """Whiteboard-frame z of the UWB tag, assuming the tip is on the board."""
        e_wb = (_NEUTRAL_MARKER_AXIS_WB if marker_axis_wb is None
                else np.asarray(marker_axis_wb, dtype=float))
        return TIP_OFFSET_FROM_TAG_M * float(e_wb[2])

    @property
    def velocity(self) -> np.ndarray:
        return self._x[2:4].copy() if self._x is not None else None

    @property
    def bias(self) -> np.ndarray:
        return self._x[4:6].copy() if self._x is not None else None

    @property
    def initialized(self) -> bool:
        return self._x is not None

    @property
    def consecutive_outage(self) -> int:
        return self._consec_outage

    @property
    def diagnostics(self) -> dict:
        return {
            'imu_steps'    : self._n_imu_steps,
            'uwb_accepted' : self._n_uwb_accepted,
            'uwb_rejected' : self._n_uwb_rejected,
            'consec_outage': self._consec_outage,
            'bias'         : self.bias,
            'velocity'     : self.velocity,
        }

    # -- RTS backward smoother (offline only) --------------------------------

    def rts_smooth(self):
        """
        Rauch-Tung-Striebel backward smoother over recorded history.

        Returns
        -------
        smoothed_positions : np.ndarray (N, 2)
        """
        return rts_smooth_history(self._history)


def rts_smooth_history(history):
    """
    Free-function RTS smoother — runs over an arbitrary slice of EKF
    history records (each {'x', 'P', 'F', 'Q'} from predict()).

    Priority 6: extracted so `note_smoother.NoteSmoother` can run RTS on
    per-stroke slices without depending on a live EKF instance.

    Returns
    -------
    smoothed_positions : np.ndarray shape (N, 2)
    """
    n = len(history)
    if n == 0:
        return np.zeros((0, 2))
    if n < 2:
        return np.array([history[0]['x'][:2]])

    xs = [h['x'].copy() for h in history]
    Ps = [h['P'].copy() for h in history]

    for k in range(n - 2, -1, -1):
        F = history[k + 1]['F']
        Q = history[k + 1]['Q']
        P_pred = F @ Ps[k] @ F.T + Q
        try:
            G = Ps[k] @ F.T @ np.linalg.inv(P_pred)
        except np.linalg.LinAlgError:
            continue
        xs[k] = xs[k] + G @ (xs[k + 1] - F @ xs[k])
        Ps[k] = Ps[k] + G @ (Ps[k + 1] - P_pred) @ G.T

    return np.array([x[:2] for x in xs])


# -------------------------------------------------------------------------
#  ASYNC FUSION ENGINE — event-driven interface for main_ekf.py
# -------------------------------------------------------------------------
class AsyncEKFFusionEngine:
    """
    Event-driven fusion engine for the asynchronous sensor stream.

    Two separate entry points replace the batched update():
        process_imu(pkt)  — single IMU sample -> EKF predict
        process_uwb(dists, weights) — UWB ranges -> EKF update

    Cold-start
    ----------
    The first UWB packets are fed to the IRLS solver to get an initial
    position estimate.  The EKF is initialised at the median of the first
    _COLD_N valid IRLS solutions.

    process_imu() return values
    ----------------------------
        pos        np.ndarray (2,) or None — EKF position
        vel        np.ndarray (2,) or None — EKF velocity
        is_writing bool                    — contact state

    process_uwb() return values
    ----------------------------
        pos        np.ndarray (2,) or None — EKF position
        vel        np.ndarray (2,) or None — EKF velocity
        accepted   list[int]               — anchor indices that passed gate
        rejected   list[int]               — anchor indices rejected
    """

    def __init__(self):
        self.anchors = ANCHORS.copy()

        self._ekf     = TightlyCoupledEKF(self.anchors, tag_z=MARKER_LENGTH)
        self._imu     = IMUIntegrator()
        self._contact = ForceContactDetector()

        # Priority 5: pen-up state machine (hybrid A→B). Drives R inflation,
        # update skipping, and touchdown reacquisition.
        from pen_mode import PenModeManager
        self._pen_mode = PenModeManager()

        self._last_imu_ts = None
        self._is_writing  = False
        self._prev_writing = False

        # Latest marker-axis unit vector in whiteboard frame (tip -> rear).
        # Updated every IMU packet; consumed by UWB update and tip-position
        # readout so the 3D measurement residual stays tilt-correct.
        #
        # Two copies: the raw per-IMU-sample value is used by the UWB update
        # path (slerp-interpolated, accurate per-timestamp). A low-pass-filtered
        # copy is exposed for display / pen-tip readout so BNO085 quaternion
        # sample noise doesn't translate into a wobbly pen-tip trail. Natural
        # pen tilt changes are <10 Hz; 100 ms EMA smooths sensor noise without
        # lagging real tilt changes.
        self._latest_marker_axis_wb = _NEUTRAL_MARKER_AXIS_WB.copy()
        self._display_marker_axis_wb = _NEUTRAL_MARKER_AXIS_WB.copy()
        self._marker_axis_alpha      = 0.05   # ~20-sample EMA @ 200 Hz ≈ 100 ms
        self._latest_quat           = None

        # Fix D: short ring of (ts_us, quat) pairs from IMU packets so UWB
        # updates can slerp-interpolate attitude to the UWB timestamp rather
        # than using the freshest (future) quaternion — eliminates the ~50
        # ms attitude/range misalignment on tilted markers.
        self._quat_hist = deque(maxlen=40)   # ≈200 ms @ 200 Hz

        # Cold-start IRLS (reuses existing solver)
        from fusion_engine import IRLSTrilateration
        _bmin = [-0.30, -0.30, -0.50]
        _bmax = [ 1.55,  1.55,  1.00]
        self._irls = IRLSTrilateration(
            self.anchors, _bmin, _bmax, tag_z=MARKER_LENGTH)

        # Robust cold-start: collect several IRLS solutions and use their
        # median.  A single IRLS packet can land way outside the board.
        self._cold_buf       = []
        self._COLD_N         = 7     # ≥3-anchor IRLS solves to median-average
                                     # (Priority 4: bumped from 5 because some
                                     # raw cycles will now be skipped if any
                                     # anchor is missing/NaN)
        self._COLD_MARGIN    = 0.25  # m — reject IRLS fixes outside board+margin

        # IRLS speed tracking (for LIFTED_VEL_DAMP gating)
        self._prev_irls_pos  = None
        self._prev_uwb_ts    = None

        # Fix I: runaway-drift escape.  If the EKF disagrees with a fresh
        # IRLS fix by > RUNAWAY_DIST_M for RUNAWAY_HOLD consecutive UWB
        # cycles, soft-reset the filter to the IRLS position.  This breaks
        # the loop in which every innovation exceeds MAX_INNOV_M, all
        # anchors get muted, and IMU dead-reckoning drifts unbounded.
        self._runaway_count = 0

    # -- IMU predict --------------------------------------------------------

    def process_imu(self, imu_pkt: dict):
        """
        Single IMU predict step.

        Parameters
        ----------
        imu_pkt : dict with keys 'quat', 'acc', 'force', 'ts'

        Returns
        -------
        pos        : np.ndarray (2,) or None
        vel        : np.ndarray (2,) or None
        is_writing : bool
        """
        if not self._ekf.initialized:
            # Still in cold-start — process contact but skip predict.  Keep
            # caching attitude history so the first warm UWB update has a
            # populated quaternion ring for slerp interpolation (Fix D).
            state, _ = self._contact.process(imu_pkt['force'], ts=imu_pkt.get('ts'))
            self._is_writing = bool(state)
            q = np.asarray(imu_pkt['quat'], dtype=float)
            self._latest_quat           = q
            self._latest_marker_axis_wb = self._imu.get_marker_axis_wb(q)
            # During cold-start, seed the display axis directly (no smoothing)
            # so the very first frame after init shows correct pen tilt.
            self._display_marker_axis_wb = self._latest_marker_axis_wb.copy()
            ts_cold = imu_pkt.get('ts')
            if ts_cold is not None:
                self._quat_hist.append((int(ts_cold), q.copy()))
            return None, None, self._is_writing

        # Compute dt from per-packet microsecond timestamps.  Default to
        # nominal 5 ms (~200 Hz) when no timestamp is provided.  Guard
        # against negative or zero dt_us (out-of-order or duplicate
        # packets) by skipping the predict step entirely.  Large dt
        # anomalies (often due to serial buffering) are clamped to 10 ms
        # to avoid huge dead‑reckoning jumps.
        dt = 0.005
        ts = imu_pkt.get('ts')
        # Flag indicating whether to bypass the predict for this sample.
        skip_predict = False
        dt_us = None
        if ts is not None and self._last_imu_ts is not None:
            dt_us = ts - self._last_imu_ts
            # Negative or zero delta means this packet is stale or
            # duplicate.  Update the last timestamp but do not advance
            # the state.
            if dt_us <= 0:
                skip_predict = True
                # update last ts to the new value so future samples use it as base
                self._last_imu_ts = ts
            else:
                # Convert microseconds to seconds
                dt = dt_us / 1_000_000.0
                # Clamp abnormally large dt to 10 ms (twice nominal period)
                if dt > 0.010:
                    print(f"[EKF] Warning: IMU dt {dt:.3f}s too large; clamping to 0.010s")
                    dt = 0.010
        # Always record the timestamp for next iteration if available
        if ts is not None:
            self._last_imu_ts = ts

        # Body -> whiteboard acceleration (with lever-arm correction when
        # |omega| crosses the threshold; ts enables omega estimation).
        a_wb = self._imu.get_wb_acceleration(
            imu_pkt['quat'], imu_pkt['acc'], ts=imu_pkt.get('ts'))

        # Cache latest quaternion and marker-axis direction so the UWB update
        # (arriving asynchronously, ~10x slower) can reuse the freshest attitude.
        self._latest_quat           = np.asarray(imu_pkt['quat'], dtype=float)
        self._latest_marker_axis_wb = self._imu.get_marker_axis_wb(
            self._latest_quat)

        # Low-pass the marker axis for display / pen-tip readout. Natural pen
        # tilt varies slowly (<10 Hz); BNO085 quaternion sample noise gets
        # amplified 21 cm by the tip projection. Smoothing here keeps the
        # displayed pen-tip trail stable without lagging real tilt changes.
        a = self._marker_axis_alpha
        blended = ((1.0 - a) * self._display_marker_axis_wb
                   + a * self._latest_marker_axis_wb)
        norm = float(np.linalg.norm(blended))
        if norm > 1e-9:
            self._display_marker_axis_wb = blended / norm

        # Fix D: accumulate attitude history so UWB updates can interpolate
        # to their own timestamp instead of using the freshest quaternion.
        if ts is not None:
            self._quat_hist.append((int(ts), self._latest_quat.copy()))

        # Contact detection
        state, _ = self._contact.process(imu_pkt['force'], ts=imu_pkt.get('ts'))
        self._is_writing = bool(state)

        # Priority 5: drive the pen-mode state machine (touchdowns, lifts,
        # short→long hover promotions).
        cur_mode = self._pen_mode.update_imu(self._is_writing,
                                             imu_pkt.get('ts'))

        # Zero velocity on pen touchdown (transition lifting -> writing)
        if self._is_writing and not self._prev_writing:
            self._ekf._x[2] = 0.0
            self._ekf._x[3] = 0.0

        self._prev_writing = self._is_writing

        # EKF predict (mode-aware: ZUPT and bias-stall safeguard fire only
        # in PEN_DOWN; LIFTED_VEL_DAMP / hover behaviour is handled by the
        # existing is_contact branch).  When skip_predict is True (dt_us
        # <= 0), we bypass the state propagation but still return the
        # current state.  This prevents out‑of‑order or duplicate IMU
        # packets from corrupting the EKF history.
        if not skip_predict:
            self._ekf.predict(a_wb, dt,
                              is_contact=self._is_writing, mode=cur_mode)

        return self._ekf.position, self._ekf.velocity, self._is_writing

    # -- UWB update ---------------------------------------------------------

    def process_uwb(self, filtered_dists, quality_weights=None, ts=None):
        """
        UWB measurement update.

        Parameters
        ----------
        filtered_dists  : tuple (d0, d1, d2, d3) — preprocessed (despiked) distances
        quality_weights : tuple (w0, w1, w2, w3) — per-anchor quality or None
        ts              : int or None — micros() timestamp from the UWB
                          packet.  Used by the Item-B per-anchor 1D range
                          Kalman to compute dt between UWB cycles.

        Returns
        -------
        pos      : np.ndarray (2,) or None
        vel      : np.ndarray (2,) or None
        accepted : list[int]
        rejected : list[int]
        """
        # -- Cold-start: buffer IRLS solutions until stable fix ------------
        # Priority 4: every buffered solution must be a true ≥3-anchor solve
        # (`last_solve_ok=True`). A 2-anchor 2D solve is geometrically
        # ambiguous — refuse it, even at the cost of a slightly slower init.
        if not self._ekf.initialized:
            pos_xyz, _ = self._irls.solve(filtered_dists)
            n_valid = self._irls.last_solve_n_valid
            ok      = self._irls.last_solve_ok
            px, py = float(pos_xyz[0]), float(pos_xyz[1])

            board_max = float(np.max(self.anchors[:, :2])) + self._COLD_MARGIN
            in_bounds = (-self._COLD_MARGIN <= px <= board_max and
                         -self._COLD_MARGIN <= py <= board_max)
            if ok and in_bounds:
                self._cold_buf.append((px, py))

            if len(self._cold_buf) >= self._COLD_N:
                init_x = float(np.median([p[0] for p in self._cold_buf]))
                init_y = float(np.median([p[1] for p in self._cold_buf]))
                self._ekf.initialize(init_x, init_y)
                self._cold_buf.clear()
                # Priority 5: pen-mode leaves COLD_START on first valid init.
                self._pen_mode.mark_initialized()
            else:
                n = len(self._cold_buf)
                if not ok:
                    print(f"[EKF] Cold-start: insufficient anchors "
                          f"({n_valid}<3) — waiting  buffered={n}/{self._COLD_N}")
                else:
                    print(f"[EKF] Cold-start: buffering IRLS solution "
                          f"{n}/{self._COLD_N}  raw=({px:.3f},{py:.3f})")
                return (np.array([px, py]), np.zeros(2), [], [])

        # -- Fix D: time-align attitude to the UWB packet timestamp --------
        # The freshest quaternion is ~50 ms *newer* than the UWB range
        # measurement.  For a tilted marker, applying "now" attitude to a
        # "50 ms ago" range biases the 3D projection.  Slerp the attitude
        # history back to the UWB ts before computing the marker axis.
        marker_axis_for_update = self._latest_marker_axis_wb
        if ts is not None and len(self._quat_hist) > 0:
            q_at_ts = self._slerp_quat_at(int(ts))
            marker_axis_for_update = self._imu.get_marker_axis_wb(q_at_ts)

        # -- Priority 5: per-mode UWB policy --------------------------------
        # Resolve the effective mode using current anchor count + quality
        # gate. The manager may downgrade HOVER_SHORT → HOVER_LONG (skip)
        # when quality is too low, or promote HOVER_LONG → HOVER_H when
        # the feature flag is on and anchors are clean.
        n_valid = sum(1 for d in filtered_dists if np.isfinite(d) and d >= 0.05)
        if quality_weights is None:
            q_mean = 1.0
        else:
            qs = [float(quality_weights[i]) for i, d in enumerate(filtered_dists)
                  if np.isfinite(d) and d >= 0.05]
            q_mean = float(np.mean(qs)) if qs else 0.0
        eff_mode = self._pen_mode.decide_uwb_mode(n_valid, q_mean)
        touchdown_credit = (eff_mode == PenMode.PEN_DOWN
                            and self._pen_mode.consume_touchdown_credit())

        # -- Warm: EKF update -----------------------------------------------
        accepted, rejected = self._ekf.update_uwb(
            filtered_dists,
            quality_weights,
            marker_axis_wb=marker_axis_for_update,
            ts=ts,
            mode=eff_mode,
            touchdown_reacq=touchdown_credit,
        )

        # Update IRLS speed for LIFTED_VEL_DAMP gating, and run Fix I
        # runaway-drift escape in the same pass.
        pos = self._ekf.position
        if pos is not None and self._prev_irls_pos is not None:
            # Dynamically calculate UWB time delta
            dt_uwb = 0.02  # default to 50 Hz
            if ts is not None and self._prev_uwb_ts is not None:
                dt_us = ts - self._prev_uwb_ts
                if 0 < dt_us < 2_000_000:
                    dt_uwb = dt_us / 1_000_000.0
            self._prev_uwb_ts = ts
            
            # ---> DO NOT FORGET THESE TWO LINES <---
            disp = float(np.linalg.norm(pos - self._prev_irls_pos))
            self._ekf._irls_speed = disp / dt_uwb
            
        if pos is not None:
            self._prev_irls_pos = pos.copy()

        # -- Fix I: EKF-vs-IRLS runaway detection --------------------------
        if pos is not None:
            self._check_runaway(pos, filtered_dists)

        # Warn on prolonged outage
        n_out = self._ekf.consecutive_outage
        if n_out == self._ekf.MAX_CONSEC_OUTAGE:
            print(f"[EKF] WARNING: {n_out} consecutive UWB outage packets "
                  f"— velocity damping active")

        return self._ekf.position, self._ekf.velocity, accepted, rejected

    # -- Fix D helper: attitude slerp to a specific timestamp --------------

    def _slerp_quat_at(self, ts_us: int) -> np.ndarray:
        """
        Linear-interpolate (nlerp) the stored (ts, quat) history to ts_us.

        Nlerp instead of true slerp: the angular delta between adjacent IMU
        samples at 200 Hz is <1° in practice, so the constant-rate slerp /
        linear-blend-renormalise distinction is sub-noise.  Shortest-path
        correction (negate one quat if the dot product is negative) keeps
        us on the correct hemisphere.
        """
        hist = self._quat_hist
        if not hist:
            return (self._latest_quat if self._latest_quat is not None
                    else np.array([0.0, 0.0, 0.0, 1.0]))
        if ts_us <= hist[0][0]:
            return hist[0][1]
        if ts_us >= hist[-1][0]:
            return hist[-1][1]
        # Linear scan is fine — maxlen is ~40.
        prev_ts, prev_q = hist[0]
        for t, q in list(hist)[1:]:
            if prev_ts <= ts_us <= t:
                span = float(t - prev_ts)
                if span <= 0.0:
                    return q
                alpha = (ts_us - prev_ts) / span
                q0 = prev_q
                q1 = q
                if float(np.dot(q0, q1)) < 0.0:
                    q1 = -q1
                blended = (1.0 - alpha) * q0 + alpha * q1
                n = float(np.linalg.norm(blended))
                return blended / n if n > 1e-9 else q0
            prev_ts, prev_q = t, q
        return self._latest_quat

    # -- Fix I helper: EKF vs IRLS runaway escape --------------------------

    def _check_runaway(self, pos_ekf: np.ndarray, dists) -> None:
        """
        Compare the EKF position against a fresh IRLS fix.  When the two
        disagree by more than RUNAWAY_DIST_M for RUNAWAY_HOLD consecutive
        UWB cycles AND the IRLS fix itself is on-board (so we don't re-init
        on an IRLS outlier), soft-reset the EKF to the IRLS fix and flush
        the per-anchor muting / range-KF state.
        """
        try:
            pos_irls_xyz, _ = self._irls.solve(dists)
        except Exception:
            return
        irls_2d = np.array([float(pos_irls_xyz[0]), float(pos_irls_xyz[1])])
        if not np.all(np.isfinite(irls_2d)):
            return

        # Mid-session sanity check — stricter than COLD_MARGIN because we
        # know the pen has already been on the board. An off-board IRLS fix
        # is a bad trilateration, not a legitimate reset target.
        margin = self._ekf.RUNAWAY_MARGIN_M
        board_max_x = float(np.max(self.anchors[:, 0])) + margin
        board_max_y = float(np.max(self.anchors[:, 1])) + margin
        irls_sane = (-margin <= irls_2d[0] <= board_max_x and
                     -margin <= irls_2d[1] <= board_max_y)
        if not irls_sane:
            # IRLS itself is garbage — don't use it as a reset target.
            # Hold the runaway counter so we don't immediately fire once a
            # sane IRLS arrives; reset it so we re-confirm from scratch.
            self._runaway_count = 0
            return

        dist_mismatch = float(np.linalg.norm(pos_ekf - irls_2d))
        if dist_mismatch <= self._ekf.RUNAWAY_DIST_M:
            self._runaway_count = 0
            return

        self._runaway_count += 1
        if self._runaway_count < self._ekf.RUNAWAY_HOLD:
            return

        print(f"[EKF] Runaway escape: EKF ({pos_ekf[0]:.2f}, {pos_ekf[1]:.2f}) "
              f"vs IRLS ({irls_2d[0]:.2f}, {irls_2d[1]:.2f}) = "
              f"{dist_mismatch:.2f} m — soft re-init")
        self._ekf.initialize(irls_2d[0], irls_2d[1])
        self._ekf._range_bank.reset()
        self._ekf._mute_remain[:]   = 0
        self._ekf._consec_reject[:] = 0
        self._ekf._zupt_stall_s     = 0.0
        self._ekf._zupt_accel_buf.clear()
        self._ekf._vel_dir_buf.clear()
        self._ekf._bias_freeze      = False
        self._runaway_count         = 0

    # -- properties ---------------------------------------------------------

    @property
    def ekf(self) -> TightlyCoupledEKF:
        return self._ekf

    @property
    def imu_integrator(self) -> IMUIntegrator:
        return self._imu

    @property
    def is_writing(self) -> bool:
        return self._is_writing

    @property
    def marker_axis_wb(self) -> np.ndarray:
        """Latest marker long-axis unit vector in whiteboard frame (tip->rear)."""
        return self._latest_marker_axis_wb.copy()

    @property
    def tip_position(self) -> np.ndarray:
        """Pen-tip 2D position on the whiteboard (None until EKF initialised).
        Uses the EMA-smoothed marker axis so the displayed trail is stable."""
        return self._ekf.get_tip_position(self._display_marker_axis_wb)

    @property
    def tag_z_wb(self) -> float:
        """Latest whiteboard-frame z of the UWB tag (tip-on-board assumption)."""
        return self._ekf.get_tag_z_wb(self._latest_marker_axis_wb)

    @property
    def last_raw_ranges(self) -> np.ndarray:
        """Latest raw UWB ranges fed into the EKF (Item B verification)."""
        return self._ekf._last_raw_ranges.copy()

    @property
    def last_filtered_ranges(self) -> np.ndarray:
        """Latest Item-B-filtered UWB ranges actually used in the EKF."""
        return self._ekf._last_filtered_ranges.copy()

    @property
    def diagnostics(self) -> dict:
        d = self._ekf.diagnostics
        d['heading_locked'] = self._imu.heading_locked
        d['heading_vec']    = self._imu.heading_vec
        return d
