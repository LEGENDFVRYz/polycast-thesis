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
    # Orientation tests (all stationary at center)
    'NorthS':  (0.625, 0.620),
    'NorthS-': (0.625, 0.620),
    'SouthS':  (0.625, 0.620),
    'SouthS-': (0.625, 0.620),
    'EastS':   (0.625, 0.620),
    'EastS-':  (0.625, 0.620),
    'WestS':   (0.625, 0.620),
    'WestS-':  (0.625, 0.620),
    # Rotation tests (all stationary at center)
    'clockwiseM':      (0.625, 0.620),
    'clockwiseM-':     (0.625, 0.620),
    'revclockwiseM':   (0.625, 0.620),
    'revclockwiseM-':  (0.625, 0.620),
    'mix-mix_method':  (0.625, 0.620),
    'mix-mix_method-': (0.625, 0.620),

    # datasets_standard (all anchors portrait)
    # Stationary middle-hold (with-/no-contact pair). Other pt_*/ct_*/rt_*
    # entries are character/shape writes, not static holds — no truth here.
    'pt_middle':           (0.625, 0.620),
    'pt_middle-':          (0.625, 0.620),
    # Rotation in place at board center
    'pt_middle_r':         (0.625, 0.620),
    'pt_middle_rotation':  (0.625, 0.620),
    'pt_middle_rr':        (0.625, 0.620),

    # datasets_str_50hz (50 Hz UWB capture, renamed middle/rotation files)
    'middle-':             (0.625, 0.620),
    'middleCCW_rot-':      (0.625, 0.620),
    'middleCW_rot-':       (0.625, 0.620),
}


def get_truth(dataset_stem: str):
    """Return (x, y) tuple or None.  `dataset_stem` is the filename without .csv."""
    return STATIC_TRUTH.get(dataset_stem)
