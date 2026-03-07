import numpy as np
from collections import deque

class UWBPreprocessor:
    def __init__(self, window_size=5, max_range=5.0):
        self.max_range = max_range
        # Median Filter Buffers (Rejects Spikes)
        self.med_buffers = {0: deque(maxlen=window_size), 1: deque(maxlen=window_size), 
                            2: deque(maxlen=window_size), 3: deque(maxlen=window_size)}
        
        self.avg_buffers = {0: deque(maxlen=5), 1: deque(maxlen=5), 
                            2: deque(maxlen=5), 3: deque(maxlen=5)}

    def process(self, d0, d1, d2, d3):
        filtered_results = []
        for i, raw_dist in enumerate([d0, d1, d2, d3]):
            # --- STEP 1: VALIDITY GATE ---
            # If invalid, reuse last known good value (or 0 if startup)
            if raw_dist <= 0 or raw_dist > self.max_range:
                if len(self.avg_buffers[i]) > 0:
                    raw_dist = self.avg_buffers[i][-1] # Use last filtered value
                else:
                    raw_dist = 0.0

            # --- STEP 2: MEDIAN FILTER (Despiking) ---
            self.med_buffers[i].append(raw_dist)
            median_val = np.median(self.med_buffers[i])

            # --- STEP 3: MOVING AVERAGE (Smoothing) ---
            self.avg_buffers[i].append(median_val)
            smooth_val = np.mean(self.avg_buffers[i])
            
            filtered_results.append(smooth_val)
            
        return tuple(filtered_results)

class IMUPreprocessor:
    def process_sample(self, qx, qy, qz, qw, ax, ay, az):
        # 1. Normalize Quaternion
        norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm == 0: 
            return (0, 0, 0, 1, ax, ay, az)
        
        # 2. Return cleaned tuple
        return (qx/norm, qy/norm, qz/norm, qw/norm, ax, ay, az)