"""
ground_truth.py -- known (x, y) whiteboard coordinates for stationary datasets.

Fill in the true tip position for each static-hold dataset in datasets_a3_ls/.
Coordinates are in metres, with the board origin = anchor A0 (bottom-left)
and +Y = up / +X = right.  Anchor A1 sits at (1.25, 0.00),
A2 at (1.25, 1.24), A3 at (0.00, 1.24) -- see config.py.

Any dataset whose value is None is treated as "ground truth not known" and
the bias-vs-truth metrics are skipped for it.
"""

STATIC_TRUTH = {
    '0s':  (0.625, 0.620),   # e.g. (0.625, 0.620) for board centre
    '0s-': (0.625, 0.620),
    '1s':  (0.030, 0.000),
    '1s-': (0.030, 0.000),
    '2s':  (1.220, 0.000),
    '2s-': (1.220, 0.000),
    '3s':  (1.220, 1.240),
    '3s-': (1.220, 1.240),
    '4s':  (0.030, 1.240),
    '4s-': (0.030, 1.240),
}


def get_truth(dataset_stem: str):
    """Return (x, y) tuple or None.  `dataset_stem` is the filename without .csv."""
    return STATIC_TRUTH.get(dataset_stem)
