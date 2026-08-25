"""
background.pipeline_ekf
=======================
Self-contained copy of the tightly-coupled UWB+IMU EKF that `main_ekf.py`
runs, packaged for the background drawing thread.

Provenance
----------
Copied from `kuru_method/asynchronous_stream/` rather than moved: roughly a
dozen modules there (`main_ekf.py`, `note_smoother.py`, all of
`verification/`) still import the originals. The two trees are therefore
independent, and a tuning change in one does NOT propagate to the other.
Port deliberate changes across by hand.

Distinct from `background.pipelines`
------------------------------------
`background/pipelines/` is the other pipeline and carries its own,
unrelated `config` package plus an ESKF (`pipelines/fusion/eskf.py`). This
package's `config` is kuru's and shares no symbols with it. Keep the two
straight: `pipeline_ekf` is the 6-state EKF, `pipelines` is the ESKF.

Runtime data files
------------------
`config.py` resolves `rate_profile.json` and `calibration_profile.json`
relative to its own `__file__`, so both live in this directory. They fail
soft -- a missing file silently falls back to defaults and yields subtly
wrong numbers rather than an error -- so keep them alongside `config.py`.
"""

from background.pipeline_ekf.ekf_fusion import (
    AsyncEKFFusionEngine,
    TightlyCoupledEKF,
    rts_smooth_history,
)
from background.pipeline_ekf.pen_mode import PenMode, PenModeManager
from background.pipeline_ekf.imu_integrator import IMUIntegrator, quat_to_rotmat
from background.pipeline_ekf.range_kf import PerAnchorRangeKFBank
from background.pipeline_ekf.contact import ForceContactDetector
from background.pipeline_ekf.preprocessor import UWBPreprocessor, IMUPreprocessor

__all__ = [
    'AsyncEKFFusionEngine',
    'TightlyCoupledEKF',
    'rts_smooth_history',
    'PenMode',
    'PenModeManager',
    'IMUIntegrator',
    'quat_to_rotmat',
    'PerAnchorRangeKFBank',
    'ForceContactDetector',
    'UWBPreprocessor',
    'IMUPreprocessor',
]
