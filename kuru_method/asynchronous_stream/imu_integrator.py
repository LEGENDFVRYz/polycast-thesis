"""
imu_integrator.py  —  PolyCast IMU Integration Layer
=====================================================
Converts BNO085 body-frame measurements into whiteboard 2D acceleration
for the EKF predict step.

Physical setup
--------------
    Marker (21 cm):
        Rear end (z = 21 cm from board)  <- UWB tag antenna
        Middle   (z = 13 cm from board)  <- BNO085 IMU  (8 cm from rear)
        Front tip (z =  0 cm from board) <- tactile button

    Whiteboard: vertical, wall-mounted.

Body-frame axis mapping to whiteboard frame
-------------------------------------------
    body X  ->  Z_wb  (board normal — outward from board surface)
    body Y  ->  X_wb  (horizontal across whiteboard, positive right)
    body Z  ->  Y_wb  (vertical on whiteboard, positive up)

Lever arm note (Item A.2)
-------------------------
    The 8 cm offset (UWB tag -> IMU) lies along the marker's long axis.
    In neutral pose this is exactly Z_wb (board normal) and contributes
    zero XY error.  For a tilted marker the offset rotates into the
    whiteboard XY plane, so a centripetal correction
        a_tag = a_imu + omega x (omega x r_tag/imu)
    (Groves Eq 14.59) is applied when |omega| exceeds
    LEVER_ARM_OMEGA_THRESH (default 1 rad/s).  Below that threshold the
    correction is sub-noise and the legacy a_imu is returned unchanged.

BNO085 reports used
-------------------
    ROTATION_VECTOR  -> quaternion (body -> world), absolute orientation
    LINEAR_ACCEL     -> gravity-compensated acceleration in body frame (m/s^2)

Conversion strategy for a vertical whiteboard
----------------------------------------------
    BNO085 world frame: Z = up (anti-gravity).

    Whiteboard Y (vertical) = World Z  ->  a_wb_y = (R @ a_body)[2]
        Always correct regardless of board compass heading.

    Whiteboard X (horizontal along board face):
        Estimated once from the first HEADING_INIT_SAMPLES by projecting
        the body-Y axis (= wb-X by user mapping) into the world horizontal
        plane and averaging.  The resulting unit vector (heading_vec) is
        then used for every subsequent packet.

    Before heading is locked a direct body-axis fallback is used:
        a_wb_x = body_ay,  a_wb_y = body_az
    This is exact when the marker is in its neutral pose.
"""

import numpy as np


# -- Physical constants -----------------------------------------------------
# Tag (s=0.21 m) -> IMU (s=0.13 m) along marker axis = 0.08 m magnitude.
# (See LEVER_ARM_R_M on the class for the value actually used by the
# centripetal correction.)
LEVER_ARM_M = 0.08   # m — |UWB tag - IMU| along the marker long axis.


def quat_to_rotmat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """
    Unit quaternion -> 3x3 rotation matrix R  where  v_world = R @ v_body.
    Handles un-normalised input gracefully.
    """
    n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-10:
        return np.eye(3, dtype=float)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ], dtype=float)


def _quat_mul(qa, qb):
    """Hamilton quaternion product: returns qa * qb in [x, y, z, w] order."""
    ax, ay, az, aw = qa[0], qa[1], qa[2], qa[3]
    bx, by, bz, bw = qb[0], qb[1], qb[2], qb[3]
    return np.array([
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    ])


def _quat_conj(q):
    """Conjugate of a unit quaternion in [x, y, z, w] order."""
    return np.array([-q[0], -q[1], -q[2], q[3]])


class IMUIntegrator:
    """
    Converts BNO085 (quaternion + linear accel) -> whiteboard 2D acceleration.

    Usage
    -----
        integrator = IMUIntegrator()
        a_wb = integrator.get_wb_acceleration(pkt['quat'], pkt['acc'])
        # a_wb -> np.ndarray([ax_wb, ay_wb]) in m/s^2

    Tuning guide
    ------------
    HEADING_INIT_SAMPLES (default 50):
        Samples averaged before locking the board heading.
        50 samples @ 200 Hz = 250 ms.  Increase to 80 if the first
        packets have erratic marker orientation.
        Call reset_heading() to re-estimate (e.g. after a board move).

    ACC_DEADBAND_MS2 (default 0.02 m/s^2):
        Acceleration magnitudes below this are zeroed out.
        Prevents sub-noise thermal / vibration drift from accumulating.
        Raise if the EKF drifts when stationary; lower if slow-writing
        accelerations are being clipped.

    ACC_CLAMP_MS2 (default 20.0 m/s^2):
        Hard clamp per sample.  Rejects sensor glitches.
        Never set below the maximum expected stroke acceleration (~8 m/s^2).
    """

    # -- Tuning -------------------------------------------------------------
    HEADING_INIT_SAMPLES = 50    # samples before heading lock (250ms at 200Hz)
    ACC_DEADBAND_MS2     = 0.08  # m/s^2 — sub-noise floor (was 0.02)
    ACC_CLAMP_MS2        = 20.0  # m/s^2 — glitch hard-clamp

    # Lever-arm correction (Item A.2, Groves Eq 14.59)
    # a_tag = a_imu + omega x (omega x r_tag/imu)   [centripetal, 1st order]
    # Only applied when |omega| exceeds this threshold; for writing motion
    # omega is small and the correction is sub-noise.  Toggling by threshold
    # keeps the data path identical to pre-upgrade behaviour during normal
    # writing, while still catching aggressive twists/rotations.
    LEVER_ARM_OMEGA_THRESH = 1.0   # rad/s — below this, correction is skipped
    LEVER_ARM_R_M          = 0.08  # m — |r_tag/imu| along marker axis (tag > IMU)

    # Heading drift leash: the heading EMA is bounded within this many
    # degrees of the initial lock direction.  During rotation-in-place the
    # body-Y axis sweeps 360° and the EMA (alpha=0.01) would track it,
    # destroying the heading lock.  The leash prevents more than ±20° of
    # drift from the initial estimate.
    #
    # Physical justification: the board is fixed, so the true heading
    # cannot change.  Any large heading drift is either sensor noise or
    # body rotation — both should be rejected.  For genuine reorientation
    # (e.g. board moved), call reset_heading().
    HEADING_MAX_DRIFT_DEG = 15.0  # max heading drift from initial lock

    def __init__(self):
        self._heading_locked = False
        self._heading_vec    = np.array([1.0, 0.0])  # wb-X direction in world horiz.
        self._locked_heading = np.array([1.0, 0.0])  # snapshot at lock time
        self._init_headings  = []

        # Lever-arm / angular-rate tracking.  ω is estimated from quaternion
        # differentiation.  When the user does not call with a timestamp, the
        # correction is skipped.
        self._prev_q          = None   # quaternion from the last call
        self._prev_ts         = None   # micros() timestamp from the last call
        self._last_omega_mag  = 0.0    # |ω| in rad/s, exposed for verification

    # -- public API ---------------------------------------------------------

    def get_wb_acceleration(self, q, a_body_raw, ts=None):
        """
        Parameters
        ----------
        q          : array-like (4,) [qx, qy, qz, qw]
        a_body_raw : array-like (3,) [ax, ay, az]  body-frame linear accel (m/s^2)
        ts         : int or None  micros() timestamp (Item A.2 lever-arm).
                     Required to estimate ω and apply the centripetal
                     correction.  When None, the lever arm is ignored
                     (legacy pre-Item-A behaviour).

        Returns
        -------
        a_wb : np.ndarray (2,) [a_wb_x, a_wb_y]  whiteboard 2D accel (m/s^2)
        """
        q = np.asarray(q, dtype=float)
        a = np.asarray(a_body_raw, dtype=float)

        # Reject identity quaternion — BNO085 boot artifact (first 1-2 samples)
        if abs(q[3] - 1.0) < 0.01 and np.linalg.norm(q[:3]) < 0.01:
            return np.zeros(2)

        a = np.clip(a, -self.ACC_CLAMP_MS2, self.ACC_CLAMP_MS2)
        if np.linalg.norm(a) < self.ACC_DEADBAND_MS2:
            self._update_omega_history(q, ts)
            return np.zeros(2)

        R = quat_to_rotmat(*q)

        if not self._heading_locked:
            # Fix G: before the heading is locked, the body -> whiteboard
            # mapping is only exact in the neutral pose.  The legacy
            # fallback (body_Y, body_Z) silently injected wrong-frame
            # acceleration for any non-neutral tilt, corrupting the bias
            # estimate during the first ~250 ms.  Returning zero keeps the
            # predict step well-formed (dt still advances) and matches
            # layer 4's verification expectation of a <400 ms dead zone.
            self._accumulate_heading(R)
            self._update_omega_history(q, ts)
            return np.zeros(2)

        # Heading adaptation (EMA, alpha=0.01 ~ 100-sample time constant)
        # with drift leash: heading cannot drift more than
        # HEADING_MAX_DRIFT_DEG from the initial lock direction.
        body_y_world = R[:, 1]
        h_xy = body_y_world[:2]
        h_norm = float(np.linalg.norm(h_xy))
        if h_norm > 0.3:
            new_h = h_xy / h_norm
            candidate = 0.99 * self._heading_vec + 0.01 * new_h
            candidate /= float(np.linalg.norm(candidate))
            # Enforce drift leash: reject update if it exceeds max drift
            cos_drift = float(np.clip(
                np.dot(candidate, self._locked_heading), -1.0, 1.0))
            drift_deg = np.degrees(np.arccos(cos_drift))
            if drift_deg <= self.HEADING_MAX_DRIFT_DEG:
                self._heading_vec = candidate

        a_world = R @ a
        wb_ay   = float(a_world[2])                              # world Z = up = wb Y
        wb_ax   = float(np.dot(a_world[:2], self._heading_vec))  # horizontal board axis

        # -- Lever-arm centripetal correction (Item A.2, Groves Eq 14.59) ---
        # a_tag = a_imu + omega x (omega x r_tag/imu).  The first-order
        # angular-acceleration term (alpha x r) is omitted: alpha is even
        # smaller than omega for writing motion and would just add noise.
        omega_world = self._estimate_omega_world(q, ts)
        if omega_world is not None and self._last_omega_mag >= self.LEVER_ARM_OMEGA_THRESH:
            r_world = self.LEVER_ARM_R_M * R[:, 0]   # IMU -> tag in world
            a_centrip_world = np.cross(omega_world, np.cross(omega_world, r_world))
            wb_ax += float(np.dot(a_centrip_world[:2], self._heading_vec))
            wb_ay += float(a_centrip_world[2])
            
        # TEMPORARY POLARITY TEST
        # if np.linalg.norm([wb_ax, wb_ay]) > 0.5: # Only print distinct movements
        #     print(f"Accel -> X: {wb_ax:+.2f} | Y: {wb_ay:+.2f}")

        return np.array([wb_ax, wb_ay])

    def get_marker_axis_wb(self, q) -> np.ndarray:
        """
        Return the marker long-axis unit vector (tip -> rear) in WHITEBOARD
        frame, computed from the current quaternion and the locked heading.

        Body-frame convention (see module docstring): body-X maps to Z_wb
        (board normal). So the marker long axis in body frame is e_b = (1,0,0).

        The vector in world frame is the first column of R(q).  We then
        project it onto the whiteboard axes:
            wb-X direction in world horiz. plane = self._heading_vec
            wb-Y direction in world frame        = (0, 0, 1)  (up)
            wb-Z direction (board normal)        = heading rotated 90 deg

        Before heading is locked, we return the neutral-pose value (0, 0, 1)
        (board normal).  This matches the pre-Item-A legacy assumption that
        tag_z = MARKER_LENGTH and keeps cold-start behaviour unchanged.
        """
        if not self._heading_locked:
            return np.array([0.0, 0.0, 1.0])

        q = np.asarray(q, dtype=float)
        if abs(q[3] - 1.0) < 0.01 and np.linalg.norm(q[:3]) < 0.01:
            return np.array([0.0, 0.0, 1.0])

        R        = quat_to_rotmat(*q)
        e_world  = R[:, 0]            # body-X in world frame
        h        = self._heading_vec  # wb-X in world horizontal plane
        perp     = np.array([-h[1], h[0]])  # wb-Z in world horizontal plane

        e_wb_x = e_world[0] * h[0]    + e_world[1] * h[1]
        e_wb_y = e_world[2]
        e_wb_z = e_world[0] * perp[0] + e_world[1] * perp[1]
        v      = np.array([e_wb_x, e_wb_y, e_wb_z])
        n      = float(np.linalg.norm(v))
        return v / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])

    def reset_heading(self):
        """Re-trigger heading estimation after a board or sensor reposition."""
        self._heading_locked  = False
        self._heading_vec     = np.array([1.0, 0.0])
        self._locked_heading  = np.array([1.0, 0.0])
        self._init_headings   = []
        print("[IMUIntegrator] Heading reset — will re-accumulate.")

    @property
    def heading_locked(self) -> bool:
        return self._heading_locked

    @property
    def heading_vec(self) -> np.ndarray:
        return self._heading_vec.copy()

    @property
    def last_omega_mag(self) -> float:
        """|ω| in rad/s from the most recent quaternion pair (Item A.2)."""
        return float(self._last_omega_mag)

    # -- internal -----------------------------------------------------------

    def _update_omega_history(self, q: np.ndarray, ts):
        """Refresh the previous-quaternion / -timestamp baseline only."""
        if ts is None:
            return
        self._prev_q  = q.copy()
        self._prev_ts = ts

    def _estimate_omega_world(self, q: np.ndarray, ts):
        """
        Estimate angular velocity in WORLD frame (rad/s) from the quaternion
        delta over the last sample interval, then update the baseline.

        Returns
        -------
        omega_world : np.ndarray (3,) or None  (None if no valid baseline)
        """
        if ts is None or self._prev_q is None or self._prev_ts is None:
            self._update_omega_history(q, ts)
            return None

        dt_us = ts - self._prev_ts
        if not (0 < dt_us < 200_000):   # > 200 ms gap = treat as restart
            self._update_omega_history(q, ts)
            return None

        dt = dt_us / 1_000_000.0
        # World-frame relative rotation: dq_w = q * conj(q_prev)
        dq = _quat_mul(q, _quat_conj(self._prev_q))
        # Choose the shortest path (avoid the antipodal flip)
        if dq[3] < 0.0:
            dq = -dq
        omega_world = 2.0 * dq[:3] / dt
        self._last_omega_mag = float(np.linalg.norm(omega_world))
        self._update_omega_history(q, ts)
        return omega_world

    def _accumulate_heading(self, R: np.ndarray):
        # Body Y roughly points "Right-ish" in a normal human grip
        body_y_approx = R[:2, 1]
        
        # Body X (R[:, 0]) is the marker barrel pointing into/out of the board
        rx_world = R[0, 0]
        ry_world = R[1, 0]
        
        # UP x Barrel generates a perfectly horizontal vector on the board surface
        # (This is 100% immune to how you roll the pen in your fingers)
        vec_a = np.array([-ry_world, rx_world])
        vec_b = np.array([ry_world, -rx_world])
        
        # Automatically pick the vector that points in the direction of your natural grip
        if np.dot(vec_a, body_y_approx) > 0:
            right_vec = vec_a
        else:
            right_vec = vec_b
            
        norm = float(np.linalg.norm(right_vec))
        if norm > 0.15:
            self._init_headings.append(right_vec / norm)
            
        # Lock once enough heading samples have accumulated. Use the class
        # constant, not a hard-coded 50, and snapshot the locked heading so
        # the drift leash is relative to the actual board direction.
        if len(self._init_headings) >= self.HEADING_INIT_SAMPLES:
            avg_vec = np.mean(self._init_headings, axis=0)
            avg_norm = float(np.linalg.norm(avg_vec))
            if avg_norm > 1e-9:
                self._heading_vec = avg_vec / avg_norm
                self._locked_heading = self._heading_vec.copy()
                self._heading_locked = True
                print(f"[IMUIntegrator] Heading locked: {self._heading_vec}")
