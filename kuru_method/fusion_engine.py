import numpy as np
from scipy.optimize import least_squares

class FusionEngine:
    def __init__(self):
        # --- PHYSICAL SETUP (XY Plane Strategy) ---
        # We treat the wall as X (width) and Y (height). 
        # Z is depth (distance from wall).
        self.anchors = np.array([
            # [0.00, 0.00, 0.00],
            [1.23, 0.00, 0.00], 
            [1.23, 1.23, 0.00], 
            [0.00, 1.23, 0.00], # Anchor 3 (D3)
        ])

        # Bounds: 
        self.bounds_min = [-0.5,  -0.5,  -0.5]
        self.bounds_max = [2,  2,  1]

        # Start guess in the middle (calculated automatically)
        self.last_pos = np.mean(self.anchors, axis=0)

    def _residuals(self, guess, anchors, measured_dists):
        # Calculate distance from guess to all valid anchors at once
        dist_calc = np.linalg.norm(anchors - guess, axis=1)
        
        # Return vector of errors
        return np.nan_to_num(dist_calc - measured_dists)

    def trilaterate(self, dists):
        # --- DYNAMIC ANCHOR FILTERING ---
        valid_anchors = []
        valid_dists = []

        # Safely pair the distances with however many anchors you defined.
        # min() prevents crashes if you have 3 anchors defined but 4 distances arrive.
        for i in range(min(len(self.anchors), len(dists))):
            # Ignore missing anchors (-0.01) or dead preprocessor values (0.0)
            if dists[i] > 0.01:  
                valid_anchors.append(self.anchors[i])
                valid_dists.append(dists[i])

        valid_anchors = np.array(valid_anchors)
        valid_dists = np.array(valid_dists)

        # We need at least 3 valid anchors to calculate a stable 3D position
        if len(valid_dists) < 3:
            return self.last_pos

        # --- STANDARD UNWEIGHTED LEAST SQUARES ---
        # loss='linear' forces standard least squares with no outlier weights/softening
        res = least_squares(
            self._residuals, 
            self.last_pos, 
            bounds=(self.bounds_min, self.bounds_max), 
            args=(valid_anchors, valid_dists), 
            loss='linear',  
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