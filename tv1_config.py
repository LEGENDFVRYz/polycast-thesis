# tv1_config.py
import numpy as np

class Config:
    # --- PHYSICAL SETUP ---
    # The Anchor coordinates on your wall
    ANCHORS = np.array([
        [1.23, 0.00, 0.00],   # A0 – bottom-left
        [1.23, 1.23, 0.00],   # A1 – bottom-right
        [0.00, 1.23, 0.00],   # A2 – top-right
    ])

    # --- PHASE 1: IMU SETTINGS ---
    IMU_ZUPT_THRESH  = 0.15    # m/s^2. Below this, we force 0.0 (noise floor)
    ZUPT_ACC_THRESH  = 0.097   # m/s² (Auto-calibrated)
    ZUPT_JERK_THRESH = 2.975   # m/s³ (Auto-calibrated)
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