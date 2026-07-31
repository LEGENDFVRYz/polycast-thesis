# ESKF Pipelines

Converts raw marker prototype sensor data (IMU + UWB + force) arriving over serial 
from the ESP32 WROOM bridge into finished stroke polylines (capturedf notes) ready 
for the web server to render. Everything under `background/pipelines/` is one linear
pipeline, stage by stage:

```
cleaner(fetcher)/  ->  preprocess/  ->  fusion/  ->  reconstruct.py  ->  postprocess/
```

**Note:** `visualizer.py` is a standalone debug dashboard that wires the same stages
together for live viewing; it is not part of the served pipeline but for testing.

## Important: Development-Friendly and Production-Ready

The `fusion/eskf.py` module (`ESKF`) implements the fusion pipeline documented below. It can be developed, tested, and validated independently using `reconstruct.py --live` and `visualizer.py`, making it suitable for standalone verification before integration.

**For production use,** `background/pipelines/tracker.py` (`StrokeTracker`) serves as the integration layer between the serial communication thread and the application. It imports and uses `ESKF` directly and is, in turn, imported by `background/_prototype_thread.py`.

## Stage map

| Stage | Folder / file | Job |
| --- | --- | --- |
| 1-3 | `cleaner/` | Unpack serial packets, normalize schema, sort/align by timestamp |
| 4-6 | `preprocess/` | Per-sensor cleanup: IMU kinematics, UWB ranging/trilateration/position, contact state |
| 7 | `fusion/` | ESKF - fuses IMU + UWB into one pen-tip position per event |
| 8 | `reconstruct.py` | Assembles fused events into closed stroke polylines |
| 9 | `postprocess/` | Per-stroke correction after pen-up: centroid alignment + minimum-jerk smoothing |
| - | `config.py` | Single source of truth for every tuning value used above |
| - | `visualizer.py` | Live PyQtGraph dashboard driving the same stages for debugging |
| - | `module_output.py` | Shared helper routing every module's testing output to `test/module_runs/` |
| - | `tracker.py` | Live-server bridge for application layer |

Every stage-owning file can be run directly for a self-test:

```
python -m background.pipelines.cleaner.unpacker
python -m background.pipelines.preprocess.imu
python -m background.pipelines.preprocess.uwb.range
python -m background.pipelines.fusion.eskf
python -m background.pipelines.reconstruct [--live]
python -m background.pipelines.postprocess.centroid_align
python -m background.pipelines.visualizer
```

**Note:** See `test/module_runs/README.md` for what each one exports and how to read
the output.

## Stages 1-3: `cleaner/`

| File | Role |
| --- | --- |
| `unpacker.py` | `SerialStreamer` - reads raw lines off the serial port, parses IMU/UWB packets |
| `normalizer.py` | `StreamNormalizer` - schema boundary; normalizes an absent `gyro` field to explicit `None` |
| `time_alignment.py` | `TimeAlignLayer` - buffers and sorts IMU/UWB events into one timestamp-ordered stream |

Smallest, least eventful stage - a thin, well-tested boundary between raw
serial bytes and the rest of the pipeline.

## Stages 4-6: `preprocess/`

| File | Role |
| --- | --- |
| `imu.py` | `IMUPreprocessor` - rigid-body lever-arm tip correction, three parallel smoothing paths (light EMA / heavy EMA for ZUPT / high-pass), ZUPT stillness detection, angular kinematics |
| `contact.py` | `ContactStateDetector` - three-layer state machine: physical force latch, kinematic substate, to logical stroke session |
| `uwb/range.py` | `UWBRangePreprocessor` - per-anchor range filtering, jump/blind-spot detection |
| `uwb/trilateration.py` | `UWBSolver` - weighted least-squares trilateration (`soft_l1` loss, IDW anchor weighting, warm-start) |
| `uwb/position.py` | `UWBPositionFilter` - alpha-beta position smoothing, board clamp, speed-outlier gate |

IMU and UWB are processed on independent branches here; they only meet in `fusion/`.

## Stage 7 (ESKF Method): `fusion/`

Error-state Kalman filter fusing high-rate IMU motion with low-rate UWB
absolute position.

| File | Role |
| --- | --- |
| `eskf.py` | `ESKF` class - orchestrates the modules below in `process_event()` |
| `state.py` | Nominal state + 6x6 error-state covariance, numerical hygiene (floors/caps, NaN recovery) |
| `quaternion.py` | Quaternion/vector math, no state |
| `modes.py` | `FusionModeTracker` - resolves CONTACT_DRAWING / DRAWING_FAST / AIR_MOVE / IDLE tuning |
| `gates.py` | UWB admission gates: accept or reject a fix, and why |
| `updates.py` | Kalman predict/correct primitives, Joseph-form covariance update |
| `constraints.py` | `StationaryContactLock`, `ActiveStrokeBoundaryGuard`: physical plausibility on top of the statistical estimate |
| `diagnostics.py` | Builds the emitted `eskf` telemetry sub-dict |
| `stroke_dead_reckoner.py` | Stroke-local IMU-only dead reckoning, diagnostic only, never touches ESKF state |

**Note:** Full data-flow detail and Fusion layout overview is in `fusion/README.md`.

## Stage 8: `reconstruct.py`

`StrokeReconstructor` turns the flat stream of ESKF fused events into closed
stroke dicts. Only `source == 'IMU'` events with `stroke_active=True` become
ink; UWB-originated corrections move the fused position but never start,
extend, or close a stroke 

**Note:** they're picked up separately by `postprocess/uwb_buffer.py` for the
 centroid-alignment pass.

Also contains `StrokeFinalizationIMUCleaner`, a batch/offline drift-removal
pass (`removeAccErr` + `zupt`-style re-integration) that runs on every closed 
stroke before `postprocess/` does. Gated by `cfg.stroke_cleaner.enabled` (**`False` by
default**).

## Stage 9: `postprocess/`

Runs once per closed stroke, after pen-up - the only stage that looks at a
whole finished stroke at once instead of one event at a time.

| File | Role |
| --- | --- |
| `corrector.py` | `StrokePostprocessor`: orchestrates the two passes below |
| `centroid_align.py` | Pass A: one global rigid/similarity transform (translate, optional scale/rotate) matching the IMU polyline's centroid to the buffered UWB centroid |
| `minimum_jerk.py` | Pass B: detects curvature waypoints, replaces noisy in-between samples with a 5th-order minimum-jerk curve |
| `uwb_buffer.py` | `UWBStrokeBuffer` plain per-stroke UWB point accumulator: no math, just remembers which UWB fixes arrived during the open stroke, deduplicated |

Gated by `cfg.postprocess.enabled` (**`False` by default**).

## Stage 0: `config.py`

One `Config` dataclass tree (`cfg = Config()`), frozen, holding every tuned
parameter for every stage above: serial/hardware, IMU, contact detector,
UWB, anchor geometry, marker lever-arm geometry, the ESKF's per-mode
parameter table, the stroke dead reckoner, the stroke cleaner, and
postprocess. 

**Note (Strictly):** Nothing outside `background/pipelines/` should read this file.

## Stages with No Activity

Two stages are fully wired into the live pipeline but **disabled by
default** in `config.py`:

| Stages | Affected Features |
| --- | --- |
| `cfg.stroke_cleaner.enabled = False` | `StrokeFinalizationIMUCleaner` in `reconstruct.py` |
| `cfg.postprocess.enabled = False` | the whole `postprocess/` package |

When disabled, both short-circuit cleanly to a pass-through, so this
pipeline currently emits the raw ESKF-fused polyline with no post-stroke
correction applied. Flip either flag on to activate that stage; no code
changes are needed.
