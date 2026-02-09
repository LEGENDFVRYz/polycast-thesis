import numpy as np
from scipy.optimize import least_squares

class FusionEngine:
    def __init__(self):
        # --- PHYSICAL SETUP (XY Plane Strategy) ---
        # We treat the wall as X (width) and Y (height). 
        # Z is depth (distance from wall).
        self.anchors = np.array([
            [0.615, 0.00, 0.00], 
            [1.23, 1.23, 0.00], 
            [0.00, 1.23, 0.00]
        ])

        # Bounds: 
        self.bounds_min = [-0.5,  -0.5,  -0.5]
        self.bounds_max = [2,  2,  1]

        # Start guess in the middle (calculated automatically)
        self.last_pos = np.mean(self.anchors, axis=0)

    def _residuals(self, guess, anchors, measured_dists):
        # 1. Calculate distance from guess to all 3 anchors at once (Vectorized)
        dist_calc = np.linalg.norm(anchors - guess, axis=1)
        
        # 2. Return vector of errors
        return np.nan_to_num(dist_calc - measured_dists)

    def trilaterate(self, dists):
        # Convert list to numpy array
        dists = np.array(dists)

        # Sanity check: If all distances are bad/zero, return last position
        if np.all(dists <= 0):
            return self.last_pos

        # The 'soft_l1' loss function automatically ignores spikes (outliers)
        res = least_squares(
            self._residuals, 
            self.last_pos, 
            bounds=(self.bounds_min, self.bounds_max), 
            args=(self.anchors, dists), 
            loss='soft_l1',  # <--- This prevents teleporting!
            f_scale=0.5,    # Scale of the outlier rejection
            method='trf'
        )

        if res.success:
            self.last_pos = res.x
            return res.x
        else:
            return self.last_pos

    def update(self, raw_dists, filtered_dists):
        # Returns [x, y, z] for both
        pos_raw = self.trilaterate(raw_dists)
        pos_clean = self.trilaterate(filtered_dists)
        return pos_raw, pos_clean