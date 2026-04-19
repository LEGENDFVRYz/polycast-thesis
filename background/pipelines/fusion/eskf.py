"""
Module 7b — Error-State Kalman Filter (ESKF) Fusion

Upgrade path from baseline.py's complementary filter. Differences:
  - Error-state formulation (linearizes about a nominal state), not full-state.
  - UWB is a weighted correction via Kalman gain, not a fixed-alpha mix.
  - NLOS-adaptive R (consumes trilateration solve_error).
  - UWB↔IMU timestamp interpolation via a short ring buffer.
  - Lever-arm compensation (tip ≠ UWB tag ≠ IMU).
  - Sharp-stroke / turn-aware Q inflation from quaternion-derived ω.

Build is staged — this file grows Step-by-Step per the plan. The current
implementation is Step 1 (skeleton): IMU dead-reckoning + UWB passthrough
with the final output schema wired. Filter math is added in Steps 2–7.

Input events (identical to baseline.py):
    IMU:      from preprocess/imu.py → preprocess/contact.py
    POSITION: from preprocess/uwb/{range, trilateration, position}.py

Output event:
    {
      'ts_hw': int, 'source': 'IMU' | 'POSITION',
      'fused_x': float, 'fused_y': float,
      'uwb_x': float, 'uwb_y': float,
      'state': str, 'stroke_id': int, 'stroke_active': bool,
      'eskf': {
        'P_pos_trace': float,        # √(P[0,0]+P[1,1]), 2D position std proxy
        'innovation_norm': float,    # |y| on last UWB update
        'r_scale': float,            # NLOS gain scaler on last UWB update
        'omega_in_plane': float,     # rad/s  (Step 7)
        'turn_flag': bool,           # (Step 7)
        'b_a': (float, float),       # current accel-bias estimate
      }
    }
"""

from collections import deque

import numpy as np

from background.pipelines.config import cfg


class ESKF:
    def __init__(self):
        ecfg = cfg.fusion_eskf

        # ── Nominal state (propagated directly, not in the filter) ──────────
        bx = cfg.anchors.board_size_x
        by = cfg.anchors.board_size_y
        self.p   = np.array([bx * 0.5, by * 0.5], dtype=float)   # position
        self.v   = np.zeros(2, dtype=float)                      # velocity
        self.b_a = np.zeros(2, dtype=float)                      # accel bias
        self.q   = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)   # identity

        # ── Error-state covariance (6×6 block-diag init) ────────────────────
        # Blocks: δp (2), δv (2), δb_a (2)
        diag = np.array([
            ecfg.p0_pos,  ecfg.p0_pos,
            ecfg.p0_vel,  ecfg.p0_vel,
            ecfg.p0_bias, ecfg.p0_bias,
        ]) ** 2
        self.P = np.diag(diag)

        # ── Ring buffer of (ts, p, v, q) for UWB↔IMU time interpolation ─────
        self._state_buf: deque[tuple[int, np.ndarray, np.ndarray, np.ndarray]] = (
            deque(maxlen=ecfg.state_buffer_size)
        )

        # ── Step-7 turn tracking (dormant until Step 7) ─────────────────────
        self._prev_quat: np.ndarray | None = None
        self._turn_cooldown = 0
        self._omega_in_plane_last = 0.0
        self._turn_flag_last = False

        # ── Book-keeping ────────────────────────────────────────────────────
        self.last_ts: int | None = None
        self.last_uwb = self.p.copy()
        self._last_innovation_norm = 0.0
        self._last_r_scale = 1.0

        self._board_w = bx
        self._board_h = by

    # ────────────────────────────────────────────────────────────────────────
    # Public API
    # ────────────────────────────────────────────────────────────────────────
    def process_event(self, ev: dict) -> dict | None:
        ts = ev.get('ts_hw')
        if ts is None:
            return None

        sensor = ev.get('sensor')
        if sensor == 'IMU':
            return self._on_imu(ev, ts)
        if sensor == 'POSITION':
            return self._on_uwb(ev, ts)
        return None

    def reset(self):
        self.__init__()

    # ────────────────────────────────────────────────────────────────────────
    # IMU path — prediction + ZUPT update (Step 2)
    # ────────────────────────────────────────────────────────────────────────
    def _on_imu(self, ev: dict, ts: int) -> dict:
        dt_s = self._advance_clock(ts)

        # 1. Nominal state propagation (mid-point integration).
        acc = np.asarray(ev.get('acc_board', (0.0, 0.0)), dtype=float)
        a   = acc - self.b_a
        self.p += self.v * dt_s + 0.5 * a * dt_s * dt_s
        self.v += a * dt_s

        # 2. Error-state covariance propagation:   P ← F·P·Fᵀ + Q
        F = self._build_F(dt_s)
        Q = self._build_Q(dt_s)
        self.P = F @ self.P @ F.T + Q

        # 3. ZUPT pseudo-measurement (v = 0) when the IMU preprocessor
        #    flags the pen as still. Proper Kalman update — replaces the
        #    hard `v := 0` assignment of the baseline filter.
        if ev.get('is_static', False):
            self._zupt_update()

        # Track quaternion for the lever-arm rotation (Step 6) and
        # ω-derivation (Step 7).
        q = ev.get('quat')
        if q is not None:
            self.q = np.asarray(q, dtype=float)

        # Snapshot for UWB time interpolation (Step 4 consumes this).
        self._state_buf.append((ts, self.p.copy(), self.v.copy(), self.q.copy()))

        self._clamp_to_board()

        return self._emit(
            ts       = ts,
            source   = 'IMU',
            state    = ev.get('stroke_state', 'UNKNOWN'),
            sid      = ev.get('stroke_id', 0),
            active   = ev.get('stroke_active', False),
        )

    # ────────────────────────────────────────────────────────────────────────
    # UWB path — Kalman correction (Step 3, static R)
    #   Step 5 adds NLOS-adaptive R. Step 4 adds time interpolation.
    #   Step 6 adds lever-arm compensation on the measurement.
    # ────────────────────────────────────────────────────────────────────────
    def _on_uwb(self, ev: dict, ts: int) -> dict:
        self._advance_clock(ts)

        # pos_clean from position.py is EMA-smoothed + clamped. We'll swap
        # to pos_raw + speed_flag once Step 5 relaxes position.py's gate.
        uwb = ev.get('pos_clean') or ev.get('pos_raw')
        if uwb is None:
            return self._emit(ts, 'POSITION', 'UWB_DROPPED', 0, False)

        z = np.asarray(uwb, dtype=float)
        self.last_uwb = z.copy()

        self._uwb_update(z)
        self._clamp_to_board()

        return self._emit(
            ts       = ts,
            source   = 'POSITION',
            state    = 'UWB_CORRECTION',
            sid      = 0,
            active   = False,
        )

    def _uwb_update(self, z: np.ndarray):
        """Standard Kalman update with  H = [I 0 0]  (direct position obs).
        Static R in Step 3; Step 5 swaps R for the NLOS-adaptive form."""
        H = np.zeros((2, 6))
        H[0, 0] = 1.0
        H[1, 1] = 1.0

        sigma = cfg.fusion_eskf.sigma_uwb
        R = (sigma * sigma) * np.eye(2)

        # Innovation  y = z − H·x_nom = z − p
        y = z - self.p
        self._last_innovation_norm = float(np.linalg.norm(y))
        self._last_r_scale = 1.0

        S = H @ self.P @ H.T + R                  # 2×2
        K = self.P @ H.T @ np.linalg.inv(S)       # 6×2

        dx = K @ y
        self.p   += dx[0:2]
        self.v   += dx[2:4]
        self.b_a += dx[4:6]

        I = np.eye(6)
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T

    # ────────────────────────────────────────────────────────────────────────
    # Filter math
    # ────────────────────────────────────────────────────────────────────────
    def _build_F(self, dt: float) -> np.ndarray:
        """Error-state transition. Blocks (δp, δv, δb_a) of 2 each.

        Since  a_true = acc_board − b_a,  p̈ = a_true,  v̇ = a_true:
            ∂δp/∂δv   =  dt · I
            ∂δp/∂δb_a = −0.5·dt² · I
            ∂δv/∂δb_a = −dt · I
        """
        F = np.eye(6)
        I2 = np.eye(2)
        F[0:2, 2:4] = dt * I2
        F[0:2, 4:6] = -0.5 * dt * dt * I2
        F[2:4, 4:6] = -dt * I2
        return F

    def _build_Q(self, dt: float) -> np.ndarray:
        """Discrete-time process noise.

        Accel white-noise (σ_a) enters via the kinematic chain — it couples
        δp and δv with cross-covariance. Bias random walk (σ_b_a) drives
        only the δb_a block. Turn-aware inflation of σ_a lands in Step 7.
        """
        ecfg = cfg.fusion_eskf
        sa2 = ecfg.sigma_a * ecfg.sigma_a
        sb2 = ecfg.sigma_b_a * ecfg.sigma_b_a

        I2 = np.eye(2)
        Q = np.zeros((6, 6))
        # δp–δp
        Q[0:2, 0:2] = 0.25 * sa2 * dt ** 4 * I2
        # δp–δv (cross, both signs)
        Q[0:2, 2:4] = 0.50 * sa2 * dt ** 3 * I2
        Q[2:4, 0:2] = 0.50 * sa2 * dt ** 3 * I2
        # δv–δv
        Q[2:4, 2:4] = sa2 * dt * dt * I2
        # δb_a–δb_a
        Q[4:6, 4:6] = sb2 * dt * I2
        return Q

    def _zupt_update(self):
        """Kalman update for the zero-velocity pseudo-measurement (z = 0)."""
        H = np.zeros((2, 6))
        H[0, 2] = 1.0
        H[1, 3] = 1.0

        R = (cfg.fusion_eskf.sigma_zupt ** 2) * np.eye(2)

        # Innovation y = z − H·x_nom  with z = 0  →  y = −v_nom
        y = -self.v.copy()

        S = H @ self.P @ H.T + R                  # 2×2
        K = self.P @ H.T @ np.linalg.inv(S)       # 6×2

        dx = K @ y                                # 6-vec
        self.p   += dx[0:2]
        self.v   += dx[2:4]
        self.b_a += dx[4:6]

        # Joseph-form covariance update (numerically stable).
        I  = np.eye(6)
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T

    # ────────────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────────────
    def _advance_clock(self, ts: int) -> float:
        """Returns dt (s) since the previous event; handles gaps and init."""
        if self.last_ts is None:
            self.last_ts = ts
            return 1.0 / cfg.imu.sample_rate_hz

        dt_s = (ts - self.last_ts) / 1_000_000.0
        self.last_ts = ts
        if dt_s <= 0.0 or dt_s > 0.5:
            # Monotonicity / long-gap guard: treat as nominal step.
            return 1.0 / cfg.imu.sample_rate_hz
        return dt_s

    def _clamp_to_board(self):
        self.p[0] = max(0.0, min(self._board_w, self.p[0]))
        self.p[1] = max(0.0, min(self._board_h, self.p[1]))

    def _emit(self, ts: int, source: str, state: str, sid: int, active: bool) -> dict:
        return {
            'ts_hw': ts,
            'source': source,
            'fused_x': float(self.p[0]),
            'fused_y': float(self.p[1]),
            'uwb_x': float(self.last_uwb[0]),
            'uwb_y': float(self.last_uwb[1]),
            'state': state,
            'stroke_id': sid,
            'stroke_active': active,
            'eskf': {
                'P_pos_trace':     float(np.sqrt(self.P[0, 0] + self.P[1, 1])),
                'innovation_norm': self._last_innovation_norm,
                'r_scale':         self._last_r_scale,
                'omega_in_plane':  self._omega_in_plane_last,
                'turn_flag':       self._turn_flag_last,
                'b_a':             (float(self.b_a[0]), float(self.b_a[1])),
            },
        }


# ==============================================================================
# LIVE HARDWARE SELF-TEST
#   SerialStreamer → Normalizer → TimeAlign → (IMU | UWB) → ESKF
#   Prints a dashboard; Ctrl+C stops and prints a terse summary.
# ==============================================================================
if __name__ == '__main__':
    import time
    import os

    os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

    from background.pipelines.cleaner.unpacker       import SerialStreamer
    from background.pipelines.cleaner.normalizer     import StreamNormalizer
    from background.pipelines.cleaner.time_alignment import TimeAlignLayer
    from background.pipelines.preprocess.imu         import IMUPreprocessor
    from background.pipelines.preprocess.contact     import ContactStateDetector
    from background.pipelines.preprocess.uwb.range         import UWBRangePreprocessor
    from background.pipelines.preprocess.uwb.trilateration import UWBSolver
    from background.pipelines.preprocess.uwb.position      import UWBPositionFilter

    SERIAL_PORT  = getattr(cfg.serial, 'port', 'COM20')
    BAUD_RATE    = getattr(cfg.serial, 'baud', 115200)
    DISPLAY_RATE = 0.1

    streamer   = SerialStreamer(port=SERIAL_PORT, baud=BAUD_RATE)
    norm       = StreamNormalizer()
    aligner    = TimeAlignLayer(buffer_size=500)

    imu_prep   = IMUPreprocessor()
    contact    = ContactStateDetector()

    uwb_offs   = getattr(cfg.uwb, 'range_offsets_m', (0.0, 0.0, 0.0, 0.0))
    range_prep = UWBRangePreprocessor(offsets=uwb_offs)
    trilat     = UWBSolver()
    pos_filter = UWBPositionFilter()

    eskf = ESKF()

    print("=" * 60)
    print(f"  [TEST] MODULE 7b LIVE: ESKF (skeleton) on {SERIAL_PORT}")
    print("  Draw strokes. Ctrl+C to stop.")
    print("=" * 60)

    last_print_time = 0.0
    latest = None
    imu_count = 0
    uwb_count = 0

    try:
        while True:
            raw = streamer.read_new_packets()
            if raw:
                evs = norm.normalize(raw)
                aligner.add_events(evs)
                sorted_evs = aligner.get_all_sorted()
                aligner.clear()

                for ev in sorted_evs:
                    if ev['sensor'] == 'IMU':
                        p = imu_prep.process_one(ev)
                        if p:
                            s = contact.process_one(p)
                            fused = eskf.process_event(s)
                            if fused:
                                imu_count += 1
                                latest = fused
                    elif ev['sensor'] == 'UWB':
                        for r in range_prep.feed([ev]):
                            raw_pos = trilat.process_one(r)
                            if raw_pos:
                                clean = pos_filter.process_one(raw_pos)
                                if clean:
                                    fused = eskf.process_event(clean)
                                    if fused:
                                        uwb_count += 1
                                        latest = fused

            now = time.time()
            if latest and (now - last_print_time) >= DISPLAY_RATE:
                os.system('cls' if os.name == 'nt' else 'clear')
                e = latest['eskf']
                print(f"========= LIVE ESKF (skeleton, {DISPLAY_RATE}s) =========")
                print(f"  State      : {latest['state']}")
                print(f"  Stroke ID  : {latest['stroke_id']} (Active: {latest['stroke_active']})")
                print(f"  Source     : {latest['source']}")
                print("-" * 50)
                print(f"  Fused Pos  : X: {latest['fused_x']:6.3f} m | Y: {latest['fused_y']:6.3f} m")
                print(f"  UWB Anchor : X: {latest['uwb_x']:6.3f} m | Y: {latest['uwb_y']:6.3f} m")
                print(f"  P_pos_trace: {e['P_pos_trace']:.4f} m")
                print(f"  Bias b_a   : ({e['b_a'][0]:+.4f}, {e['b_a'][1]:+.4f}) m/s²")
                print(f"  Last |y|   : {e['innovation_norm']:.4f} m  (UWB innovation)")
                print(f"  IMU / UWB  : {imu_count} / {uwb_count}")
                print("=" * 52)
                last_print_time = now

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[STOP] Halting ESKF skeleton.")
        streamer.close()
        print("-" * 60)
        print(f"  IMU events processed : {imu_count}")
        print(f"  UWB events processed : {uwb_count}")
        if latest:
            print(f"  Final fused position : ({latest['fused_x']:.3f}, {latest['fused_y']:.3f}) m")
        print("=" * 60)
