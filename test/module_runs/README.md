# Module Run Outputs

Artifacts produced by running a pipeline module directly. Every module in
`background/pipelines/` can be executed on its own to validate that stage
against live hardware; the CSVs and plots those runs export land here instead
of the working directory.

Nothing in this folder is an input to the pipeline, and nothing here is tracked
by git. Delete any of it freely.

## Layout

Each module gets its own folder, and each run gets its own timestamped
subfolder inside it:

```
test/module_runs/
└── preprocess_imu/
    ├── 20260729-142530/          <- a run
    │   ├── imu_stationary.csv
    │   └── imu_stationary.png
    ├── 20260729-151204/          <- a later run
    │   ├── imu_stationary.csv
    │   └── imu_stationary.png
    └── latest/                   <- copy of the newest run
        ├── imu_stationary.csv
        └── imu_stationary.png
```

Runs are kept rather than overwritten so a session before a config change can
be compared against one after it. `latest/` is a plain copy of the most recent
run, so scripts and notes can reference a stable path.

Folders appear only once their module has been run at least once.

## Folder to module map

| Folder | Produced by | Stage | Outputs |
| --- | --- | --- | --- |
| `cleaner_unpacker` | `python -m background.pipelines.cleaner.unpacker` | 1 - Stream unpacker | console only |
| `cleaner_normalizer` | `python -m background.pipelines.cleaner.normalizer` | 2 - Stream normalizer | console only |
| `cleaner_time_alignment` | `python -m background.pipelines.cleaner.time_alignment` | 3 - Time alignment | console only |
| `preprocess_imu` | `python -m background.pipelines.preprocess.imu` | 4 - IMU preprocessor | `imu_stationary.csv`, `imu_stationary.png` |
| `preprocess_uwb_range` | `python -m background.pipelines.preprocess.uwb.range` | 5 - UWB range preprocessor | `uwb_range_report.csv`, `uwb_range_report.png` |
| `preprocess_uwb_trilateration` | `python -m background.pipelines.preprocess.uwb.trilateration` | 5b - Trilateration solver | `trilateration_math_report.csv`, `trilateration_math_report.png` |
| `preprocess_uwb_position` | `python -m background.pipelines.preprocess.uwb.position` | 5c - Position filter | `position_filter_report.csv`, `position_filter_report.png` |
| `preprocess_contact` | `python -m background.pipelines.preprocess.contact` | 6 - Contact / stroke state | `state_detector_report.csv`, `state_detector_report.png` |
| `fusion_eskf` | `python -m background.pipelines.fusion.eskf` | 7 - ESKF fusion | `eskf_session.csv` |
| `visualizer` | `python -m background.pipelines.visualizer` | Full pipeline dashboard | `visualizer_log.csv`, `visualizer_output.png` |

The three `cleaner_*` modules print to the console and write no files, so their
folders stay empty unless a future change adds an export.

## What each run tells you

Work down the stages: a problem at one stage makes every later stage's output
meaningless, so start from the earliest one that looks wrong.

- **`cleaner_unpacker`** - packet rate and loss. IMU should sit near
  `cfg.imu.sample_rate_hz` (~170 Hz), UWB near `cfg.uwb.rate_hz` (~50 Hz).
  Sustained loss above a few percent points at the ESP-NOW link.
- **`cleaner_time_alignment`** - sensor timing and the UWB-to-IMU offset, which
  bounds the interpolation error the fusion stage inherits.
- **`preprocess_imu`** - the 2D stroke here is naive double integration with no
  UWB correction, so it shows raw IMU drift before fusion hides it.
- **`preprocess_uwb_range`** - raw versus filtered distance per anchor. One
  anchor noisy while the rest are clean usually means a blocked or badly placed
  anchor, not a filter problem.
- **`preprocess_uwb_trilateration`** - expect visible scatter; this is the
  unfiltered geometric solve. The residual plot shows whether the ranges
  actually intersect cleanly.
- **`preprocess_uwb_position`** - the filtered trace should stay smooth and
  strictly inside the board outline even where the raw trace exits it.
- **`preprocess_contact`** - force against the detected stroke session. Confirms
  micro-pauses do not fragment a stroke.
- **`fusion_eskf`** - fused output with filter health.
- **`visualizer`** - all layers at once: fused ink, air movement, raw UWB, and
  IMU-only dead reckoning.

## Related test folders

- `test/_datasets/` - recorded sensor logs used for replay
- `test/batch/outputs/` - batch replay results
- `test/tuning/results/` - parameter tuning results
