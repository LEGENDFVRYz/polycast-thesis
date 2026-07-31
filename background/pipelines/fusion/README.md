# Fusion Stage

Module 7 of the pipeline. Takes preprocessed IMU events (from
`preprocess/imu.py` -> `preprocess/contact.py`) and UWB position events (from
`preprocess/uwb/position.py`) and produces a single fused pen-tip position per
event, on the board plane.

`ESKF` (`eskf.py`) is the sole fusion engine, exposing
`process_event(event) -> dict | None`.

Run directly against live hardware:

```
python -m background.pipelines.fusion.eskf
```

Exports a session CSV to `test/module_runs/fusion_eskf/` on Ctrl+C - see
`test/module_runs/README.md`.

## Package layout

`eskf.py` is the coordinator of the system. Instead of containing all the logic, it delegates quaternion math, Kalman filtering, UWB validation, constraints, tuning, and telemetry formatting to separate modules. The `ESKF` class connects these pieces in `process_event()`.

```
eskf.py                      (ESKF class: orchestrator)
 |-- state.py                   ESKFState: nominal state + covariance, numerical hygiene
 |-- quaternion.py              quaternion/vector math, no state
 |-- modes.py                   FusionModeTracker: which tuning applies right now
 |-- gates.py                   UWB admission: accept or reject a fix, and why
 |-- updates.py                 Kalman predict/correct primitives
 |-- constraints.py             StationaryContactLock, ActiveStrokeBoundaryGuard
 |-- diagnostics.py             builds the emitted telemetry dict
 `-- stroke_dead_reckoner.py    IMU-only dead-reckoning fallback
```

Every file here only imports `config.py` (and each other) - nothing in
`fusion/` depends on `preprocess/` or `cleaner/`, so this stage is
self-contained and can be tuned in isolation.

### `state.py` - ESKFState

Holds the nominal state that is propagated directly (position, velocity,
accelerometer bias, attitude) and the 6x6 error-state covariance:

```
[0:2]  position error   dp
[2:4]  velocity error   dv
[4:6]  accel bias error db_a
```

Owns covariance floors/caps, symmetry enforcement, and NaN recovery (resets
and reports rather than propagating garbage). The visible output is
`position + position_bias` - shape and placement are deliberately split so
IMU stroke shape and slow UWB placement correction don't fight each other.

### `quaternion.py` - math primitives

`[x, y, z, w]`-order quaternion multiply, conjugate, slerp, and
quaternion-to-rotation-matrix, plus `clip_vector_norm` for magnitude-based 2D
clipping. Pure functions, no state, shared by `eskf.py` and `updates.py`.

Note: `preprocess/imu.py` has its own quaternion helpers, and that's
intentional rather than duplication - the two stages use them for different
jobs. Preprocess does a one-shot body-to-world rotation of the current
sample. This module additionally tracks attitude *change* between samples
(`quaternion_multiply` + `quaternion_conjugate` to get `omega_world` for
turn detection) and interpolates attitude at an arbitrary timestamp
(`slerp`, used to align IMU attitude to a UWB fix's exact arrival time) -
neither of which preprocess needs to do.

### `modes.py` - FusionModeTracker

Resolves which trust balance is active right now and supplies its tuning
(UWB sigma scale, velocity drag, gain cap, IMU acceleration scale) from the
config mode table:

| Mode | When | Effect |
| --- | --- | --- |
| `CONTACT_DRAWING` | normal ink | IMU owns shape, UWB nudges placement |
| `DRAWING_FAST` | fast ink burst | brief extra IMU authority |
| `AIR_MOVE` | pen lifted | UWB re-anchors, IMU suppressed |
| `IDLE` | held still | UWB re-anchors hard, IMU silent |

Also owns the `DRAWING_FAST` arming state machine and the stroke-age
gain-cap ramp.

### `gates.py` - UWB admission

A UWB fix must clear every gate here before it can correct the filter:
speed-implied jump rejection, low geometric confidence, off-board margin,
awaiting-IMU-init, and NLOS residual. Gates run cheapest-first so an
obviously bad fix never reaches the Kalman update, and each rejection carries
its own label so the reason survives into diagnostics.

### `updates.py` - Kalman primitives

`apply_measurement_update` computes the gain, clips the resulting error state,
and folds the covariance update in Joseph form (numerically stable under
hard-clipped gains). Callers differ only in their observation matrix and
`ErrorStateClipLimits` - the clip is the real safety net, since a
mathematically optimal gain can still teleport the visible tip when the
covariance is briefly wrong.

### `constraints.py` - physical constraints

Applied on top of the statistical estimate, not part of the filter math
itself:

- **`StationaryContactLock`** - while the tip is planted but not moving, hold
  the visible position still instead of integrating IMU noise into a visible
  smear (pen-down settling, mid-stroke micro-pauses, end-of-stroke hold).
- **`ActiveStrokeBoundaryGuard`** - a safety net. If live IMU integration has
  drifted far outside the neighborhood of the last trusted UWB fix, pull it
  back and kill the outward velocity driving the excursion.

### `diagnostics.py` - emitted payload

Builds the `dict` returned by `ESKF.process_event()`. Consumed by
`reconstruct.py`, `visualizer.py`, and `test/tuning/scorer.py` - the schema is
a stable contract, so key names and nesting don't change without updating all
three. Counters are cumulative over the filter's lifetime; instantaneous
values latch on frames where their producing path didn't run.

## Event flow

1. **IMU event** drives the clock: `updates.py` builds the transition matrix
   and process noise, and `state.py`'s nominal state + covariance are
   propagated forward. IMU never corrects, only predicts.
2. **UWB event** never advances time, only corrects, and only after clearing
   every `gates.py` admission check. Accepted fixes are interpolated against
   the IMU state history to the UWB timestamp, then folded in through
   `updates.apply_measurement_update`.
3. **`constraints.py`** can override the result afterward - the stationary
   lock and boundary guard both act on the nominal state post-update.
4. **`modes.py`** is consulted throughout so the same update code runs with
   different trust and gain settings depending on what the pen is doing.
5. **`diagnostics.py`** packages the final state into the event dict handed
   back to the caller.