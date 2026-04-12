"""
config.py  --  PolyCast shared configuration (Async Stream)
===========================================================
Single source of truth for anchor positions, UWB offsets,
marker geometry, and serial connection defaults.

After running calibrate.py, update UWB_OFFSETS here only.
"""

import numpy as np

# -- Serial defaults --------------------------------------------------------
SERIAL_PORT = 'COM5'
BAUD_RATE   = 115200

# -- Physical geometry ------------------------------------------------------
MARKER_LENGTH = 0.21   # m -- tip to UWB tag antenna

# Anchor antenna positions (metres), measured antenna-to-antenna from A0.
# Z = mount height above whiteboard surface.
ANCHORS = np.array([
    [0.00, 0.00, 0.05],   # A0 -- bottom-left (origin)
    [1.25, 0.00, 0.05],   # A1 -- bottom-right
    [1.25, 1.24, 0.05],   # A2 -- top-right
    [0.00, 1.24, 0.05],   # A3 -- top-left
], dtype=float)

# -- UWB calibration offsets (from calibrate.py) ----------------------------
# Per-anchor distance correction applied to raw UWB measurements.
# Re-run calibrate.py and paste the new tuple here after each recalibration.
# UWB_OFFSETS = (-0.1326, -0.0332, -0.1785, -0.1318)
UWB_OFFSETS = (-0.1752, -0.0466, -0.2227, -0.1220)
