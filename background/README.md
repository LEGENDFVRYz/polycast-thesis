# `background/`

Everything that turns marker sensor data into drawable strokes, plus the
thread that carries them to the web layer. The Flask app in `app/` never
touches sensors directly - it reads the canvas this package maintains.

```
serial bytes -> StrokeTracker (one of two pipelines) -> image_generator -> MJPEG / WebSocket
```

## The two pipelines

Both implement the same interface - a `StrokeTracker` class - so they are
interchangeable. `_prototype_thread.py` picks one at import time from the
`POLYCAST_PIPELINE` environment variable, and an unknown value is a hard
error rather than a silent fallback.

| Directory | Fusion | Selected by |
| --- | --- | --- |
| `pipeline_ekf/` | 6-state tightly-coupled EKF | `POLYCAST_PIPELINE=ekf` **(default)** |
| `pipelines/` | ESKF with per-stroke post-processing | `POLYCAST_PIPELINE=eskf` |


### `pipeline_ekf/` - the stable trace

A tightly-coupled EKF that corrects on each individual UWB range rather than
on a trilaterated position. Flat module layout, no sub-packages. It is the
default because it is the settled, known-good path.


### `pipelines/` - the current research line

An ESKF split into explicit stages (`cleaner` -> `preprocess` -> `fusion` ->
`reconstruct` -> `postprocess`), each runnable standalone against recorded
data for self-testing. Adds per-stroke correction after pen-up, and a set of
post-processing designs selectable at runtime via `POLYCAST_MODE`.

This is where new work belongs.

**See [`pipelines/README.md`](pipelines/README.md) for the stage-by-stage
documentation** - start there if you came for the fusion work.

> Both packages carry their own `config`, and they share no symbols. Keep
> them straight: `pipeline_ekf` is the 6-state EKF, `pipelines` is the ESKF.

## Shared runtime

| File | Job |
| --- | --- |
| `_prototype_thread.py` | Serial reader thread; selects the pipeline and feeds the canvas |
| `prototype_manager.py` | Start/stop lifecycle the Flask routes call |
| `image_generator.py` | PIL canvas the MJPEG stream renders from |
| `trail_smoother.py` | Causal display-time smoothing of the live trail; never edits stored stroke geometry |
