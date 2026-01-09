# tv1_config.py
import numpy as np

class Config:
    # --- PHYSICAL SETUP ---
    # The Anchor coordinates on your wall
    ANCHORS = np.array([
        [1.21, 0.00, 0.00], 
        [1.21, 1.21, 0.00], 
        [0.00, 1.21, 0.00]
    ])
    
    # --- CALIBRATION MAP ---
    # Format: [Raw_X, Raw_Y]
    # Based on your observations:
    CALIB_OBSERVED = np.array([
        [0.25, 0.25],  # Bottom Left (Raw)
        [1.50, 0.25],  # Bottom Right (Raw)
        [0.25, 1.25],  # Top Left (Raw)
        [1.50, 1.55]   # Top Right (Raw)
    ])

    # Format: [Real_X, Real_Y]
    # Where they SHOULD be:
    CALIB_REAL = np.array([
        [0.00, 0.00],  # Bottom Left (Real)
        [1.20, 0.00],  # Bottom Right (Real)
        [0.00, 1.20],  # Top Left (Real)
        [1.20, 1.20]   # Top Right (Real)
    ])
    

    # --- PHASE 1: IMU SETTINGS ---
    IMU_ZUPT_THRESH = 0.15     # m/s^2. Below this, we force 0.0 (noise floor)
    ZUPT_ACC_THRESH  = 0.075   # m/s²
    ZUPT_JERK_THRESH = 7.839   # m/s³
    ZUPT_TIME_THRESH = 0.080   # seconds
    
    # --- PHASE 2: UWB PHYSICS GATES ---
    MAX_HUMAN_SPEED   = 6.0   # m/s (Hard limit for "Superman" jumps)
    GHOST_SPEED_LIMIT = 0.1   # m/s (Limit allowed when IMU says "Static")
    
    # Geometry Solver Constraints (X, Y, Z bounds)
    GEO_BOUNDS_MIN = [-10, -10, -0.1]
    GEO_BOUNDS_MAX = [ 10,  10,  0.1]
    
    # --- SYSTEM SETTINGS ---
    SYNC_BUFFER_SIZE  = 300   # Store last 3 second of IMU data for sync
    
    
    # --- FORCE CONFIG ---
    FORCE_CONTACT_THRESH = 3.1