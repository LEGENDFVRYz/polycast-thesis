"""
[postprocess/rts_smoother] — Rauch-Tung-Striebel backward pass

Pure function — no state, no side effects. Takes the forward-pass ESKF
history for one completed stroke and returns smoothed state estimates.

Input: list of per-step dicts (oldest → newest), each containing:
    ts        : int            hardware timestamp (µs)
    x_post    : ndarray (6,)   posterior state  [δp_x, δp_y, δv_x, δv_y, δb_a_x, δb_a_y]
    P_post    : ndarray (6,6)  posterior covariance
    F         : ndarray (6,6)  state-transition Jacobian used in this step
    Q         : ndarray (6,6)  process-noise used in this step

Output: list of dicts (same length, same order):
    ts         : int
    x_smoothed : ndarray (6,)
    P_smoothed : ndarray (6,6)

Notes:
  - The last sample's smoothed state == its posterior (nothing to pull from).
  - np.linalg.solve is used for G_k instead of explicit matrix inversion
    (numerically stable, ~2x faster on 6x6 systems).
  - A small ridge (1e-9 * I) is added to P_pred before solve to guard
    against singular matrices from near-zero-dt steps or numerical underflow.
"""

from __future__ import annotations

import numpy as np


_RIDGE = 1e-9 * np.eye(6)


def rts_smooth_history(history: list[dict]) -> list[dict]:
    """Run the RTS backward pass over one stroke's forward-pass history.

    Returns a list of equal length with smoothed estimates. Returns the
    input unchanged (with x_smoothed == x_post) if fewer than 2 samples.
    """
    n = len(history)

    if n < 2:
        return [
            {'ts': h['ts'], 'x_smoothed': h['x_post'].copy(), 'P_smoothed': h['P_post'].copy()}
            for h in history
        ]

    # Pre-allocate output arrays for speed — avoids per-step dict allocation
    # during the tight loop; we'll pack into dicts at the end.
    xs = np.empty((n, 6), dtype=float)
    Ps = np.empty((n, 6, 6), dtype=float)

    # Seed from the last (most recent) forward estimate — no smoothing there.
    xs[n - 1] = history[n - 1]['x_post']
    Ps[n - 1] = history[n - 1]['P_post']

    # Backward sweep: k runs from N-2 down to 0.
    for k in range(n - 2, -1, -1):
        h  = history[k]
        Fk = h['F']           # (6,6)
        Qk = h['Q']           # (6,6)
        xk = h['x_post']     # (6,)
        Pk = h['P_post']     # (6,6)

        # Prior covariance predicted from step k → k+1
        P_pred = Fk @ Pk @ Fk.T + Qk  # (6,6)

        # Smoother gain: G_k = P_k * F_k^T * P_pred^{-1}
        # Solved as (P_pred^T \ (F_k * P_k^T)^T)^T to avoid explicit inv.
        # G_k^T = solve(P_pred, F_k @ P_k)  →  shape (6,6)
        P_pred_reg = P_pred + _RIDGE
        G_k = np.linalg.solve(P_pred_reg.T, (Fk @ Pk).T).T  # (6,6)

        # Smoothed state and covariance
        xs[k] = xk + G_k @ (xs[k + 1] - Fk @ xk)
        dP    = Ps[k + 1] - P_pred
        Ps[k] = Pk + G_k @ dP @ G_k.T

    return [
        {'ts': history[i]['ts'], 'x_smoothed': xs[i], 'P_smoothed': Ps[i]}
        for i in range(n)
    ]


# ==============================================================================
# SELF-TEST  (synthetic 1-D constant-velocity track)
# ==============================================================================
if __name__ == '__main__':
    import math

    np.random.seed(42)
    dt      = 0.005          # 200 Hz
    n_steps = 400            # 2-second stroke
    sigma_a = 1.1            # match eskf.py default
    sigma_b = 0.0001

    # Ground truth: constant velocity in x, stationary in y
    v_true = 0.15            # m/s
    p_true = np.zeros((n_steps, 2))
    for k in range(1, n_steps):
        p_true[k, 0] = p_true[k - 1, 0] + v_true * dt

    # Build synthetic forward-pass history with noisy integration
    history = []
    p  = np.array([0.0, 0.0])
    v  = np.array([v_true, 0.0])
    ba = np.zeros(2)
    x  = np.concatenate([p, v, ba])

    # Initial covariance
    P = np.diag([0.04, 0.04, 0.01, 0.01, 0.0025, 0.0025])

    I2 = np.eye(2)

    def build_F(dt):
        F = np.eye(6)
        F[0:2, 2:4] = dt * I2
        F[0:2, 4:6] = -0.5 * dt * dt * I2
        F[2:4, 4:6] = -dt * I2
        return F

    def build_Q(dt):
        sa2 = sigma_a ** 2
        sb2 = sigma_b ** 2
        Q   = np.zeros((6, 6))
        Q[0:2, 0:2] = 0.25 * sa2 * dt**4 * I2
        Q[0:2, 2:4] = 0.50 * sa2 * dt**3 * I2
        Q[2:4, 0:2] = 0.50 * sa2 * dt**3 * I2
        Q[2:4, 2:4] =        sa2 * dt**2 * I2
        Q[4:6, 4:6] =        sb2 * dt    * I2
        return Q

    for k in range(n_steps):
        F = build_F(dt)
        Q = build_Q(dt)

        # Noisy acceleration input
        a_noise = np.random.randn(2) * sigma_a * math.sqrt(dt)
        a_in    = np.array([0.0, 0.0]) + a_noise

        x[0:2] += x[2:4] * dt + 0.5 * a_in * dt * dt
        x[2:4] += a_in * dt

        P = F @ P @ F.T + Q

        history.append({
            'ts':     int(k * dt * 1e6),
            'x_post': x.copy(),
            'P_post': P.copy(),
            'F':      F,
            'Q':      Q,
        })

    # Run RTS
    smoothed = rts_smooth_history(history)

    # Compare position RMSE: forward vs smoothed vs truth
    fwd_err = np.array([h['x_post'][0:2]    for h in history])    - p_true
    rts_err = np.array([s['x_smoothed'][0:2] for s in smoothed])  - p_true

    fwd_rmse = math.sqrt(np.mean(fwd_err ** 2))
    rts_rmse = math.sqrt(np.mean(rts_err ** 2))

    print("=" * 56)
    print("  RTS Smoother Self-Test (synthetic 200 Hz, 2 s stroke)")
    print("=" * 56)
    print(f"  Forward RMSE  : {fwd_rmse * 1000:.3f} mm")
    print(f"  RTS RMSE      : {rts_rmse * 1000:.3f} mm")

    # Last sample must coincide (smoother seeded from forward at N-1)
    last_diff = np.linalg.norm(
        smoothed[-1]['x_smoothed'] - history[-1]['x_post']
    )
    print(f"  Last-sample Δ : {last_diff:.2e}  (expect ≈ 0)")

    ok = rts_rmse < fwd_rmse and last_diff < 1e-10
    print(f"  Result        : {'PASS' if ok else 'FAIL'}")
    print("=" * 56)
