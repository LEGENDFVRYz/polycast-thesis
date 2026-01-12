# tv1_config.py
import numpy as np

class Config:
    # --- PHYSICAL SETUP ---
    # The Anchor coordinates on your wall
    ANCHORS = np.array([
        [1.25, 0.00, 0.00], 
        [1.25, 1.25, 0.00], 
        [0.00, 1.25, 0.00]
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