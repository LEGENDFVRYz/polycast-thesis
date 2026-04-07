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

Lever arm note
--------------
    The 8 cm offset (UWB tag -> IMU) is entirely along the marker's long
    axis = Z_wb (board normal).  For 2D (X, Y) tracking this contributes
    zero XY error — no lever arm correction is required.

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
LEVER_ARM_M = 0.09   # m — UWB tag to IMU offset. Zero XY effect.


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
        50 samples @ 100 Hz = 500 ms.  Increase to 80 if the first
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
    HEADING_INIT_SAMPLES = 50    # samples before heading lock (500ms at 100Hz)
    ACC_DEADBAND_MS2     = 0.02  # m/s^2 — sub-noise floor
    ACC_CLAMP_MS2        = 20.0  # m/s^2 — glitch hard-clamp

    def __init__(self):
        self._heading_locked = False
        self._heading_vec    = np.array([1.0, 0.0])  # wb-X direction in world horiz.
        self._init_headings  = []

    # -- public API ---------------------------------------------------------

    def get_wb_acceleration(self, q, a_body_raw):
        """
        Parameters
        ----------
        q          : array-like (4,) [qx, qy, qz, qw]
        a_body_raw : array-like (3,) [ax, ay, az]  body-frame linear accel (m/s^2)

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
            return np.zeros(2)

        R = quat_to_rotmat(*q)

        if not self._heading_locked:
            self._accumulate_heading(R)
            return np.array([float(a[1]), float(a[2])])  # fallback: body_Y, body_Z

        a_world = R @ a
        wb_ay   = float(a_world[2])                              # world Z = up = wb Y
        wb_ax   = float(np.dot(a_world[:2], self._heading_vec))  # horizontal board axis
        return np.array([wb_ax, wb_ay])

    def reset_heading(self):
        """Re-trigger heading estimation after a board or sensor reposition."""
        self._heading_locked = False
        self._heading_vec    = np.array([1.0, 0.0])
        self._init_headings  = []
        print("[IMUIntegrator] Heading reset — will re-accumulate.")

    @property
    def heading_locked(self) -> bool:
        return self._heading_locked

    @property
    def heading_vec(self) -> np.ndarray:
        return self._heading_vec.copy()

    # -- internal -----------------------------------------------------------

    def _accumulate_heading(self, R: np.ndarray):
        body_y_world = R[:, 1]         # body-Y direction in world frame
        h_xy         = body_y_world[:2]
        norm         = float(np.linalg.norm(h_xy))
        if norm > 0.15:
            self._init_headings.append(h_xy / norm)

        if len(self._init_headings) >= self.HEADING_INIT_SAMPLES:
            mean_h = np.mean(self._init_headings, axis=0)
            n2     = float(np.linalg.norm(mean_h))
            self._heading_vec    = mean_h / n2 if n2 > 0.1 else np.array([1.0, 0.0])
            self._heading_locked = True
            print(f"[IMUIntegrator] Heading locked: {self._heading_vec.round(4)}")
