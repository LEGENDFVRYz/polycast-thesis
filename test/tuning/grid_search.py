"""
Grid search strategies for systematic ESKF tuning.

Phase A — Sensitivity scan:
    Sweep each parameter independently ±range in 5 steps.
    Identifies which knobs affect the score most. ~25 trials per stage.

Phase B — Grid search on top-N parameters:
    Full combinatorial grid over the N most sensitive params.
    n_values per param derived from the --trials budget.

Stage definitions map dot-paths (into config.py) to search ranges.
"""

import itertools
import numpy as np


# ---------------------------------------------------------------------------
# Parameter spaces per stage
# ---------------------------------------------------------------------------

STAGE_PARAMS: dict[int, dict] = {
    # Stage 1 — Normal drawing mode (use pen-down datasets: abc, square, letters)
    1: {
        'fusion_eskf.modes.drawing.sigma_scale': {
            'default': 0.82, 'range': (0.50, 1.20), 'steps': 5,
        },
        'fusion_eskf.modes.drawing.drag_inv_s': {
            'default': 0.65, 'range': (0.30, 1.10), 'steps': 5,
        },
        'fusion_eskf.modes.drawing.pos_floor': {
            'default': 0.014, 'range': (0.005, 0.030), 'steps': 5,
        },
        'fusion_eskf.modes.drawing.acc_scale': {
            'default': 1.00, 'range': (0.60, 1.50), 'steps': 5,
        },
    },
    # Stage 2 — Fast mode (use quick-stroke datasets: abc fast, hello)
    2: {
        'fusion_eskf.drawing_fast_speed_thresh': {
            'default': 0.10, 'range': (0.06, 0.18), 'steps': 5,
        },
        'fusion_eskf.drawing_fast_min_frames': {
            'default': 2, 'range': (1, 5), 'steps': 5, 'dtype': int,
        },
        'fusion_eskf.drawing_fast_burst_frames': {
            'default': 4, 'range': (2, 8), 'steps': 5, 'dtype': int,
        },
        'fusion_eskf.modes.drawing_fast.sigma_scale': {
            'default': 1.45, 'range': (0.90, 2.20), 'steps': 5,
        },
        'fusion_eskf.modes.drawing_fast.acc_scale': {
            'default': 1.55, 'range': (1.00, 2.20), 'steps': 5,
        },
    },
    # Stage 3 — Air mode (use datasets with lots of pen lifts)
    3: {
        'fusion_eskf.modes.air.sigma_scale': {
            'default': 1.45, 'range': (0.80, 2.50), 'steps': 5,
        },
        'fusion_eskf.modes.air.drag_inv_s': {
            'default': 2.40, 'range': (1.00, 4.00), 'steps': 5,
        },
    },
    # Stage 4 — NLOS / rejection gates (tune after core modes stabilize)
    4: {
        'uwb.trilat_max_residual': {
            'default': 0.12, 'range': (0.06, 0.20), 'steps': 5,
        },
        'fusion_eskf.k_nlos': {
            'default': 1.50, 'range': (0.50, 3.50), 'steps': 5,
        },
        'fusion_eskf.r_scale_max': {
            'default': 25.0, 'range': (10.0, 50.0), 'steps': 5,
        },
        'fusion_eskf.hard_reject_mult': {
            'default': 7.50, 'range': (3.0, 15.0), 'steps': 5,
        },
    },
    # Stage 0 — Core ESKF noise covariances (tune last, after modes are locked)
    0: {
        'fusion_eskf.sigma_a': {
            'default': 2.40, 'range': (1.00, 5.00), 'steps': 5,
        },
        'fusion_eskf.sigma_b_a': {
            'default': 0.0001, 'range': (0.00005, 0.001), 'steps': 5,
        },
        'fusion_eskf.sigma_zupt': {
            'default': 0.005, 'range': (0.001, 0.020), 'steps': 5,
        },
        'fusion_eskf.sigma_uwb': {
            'default': 0.060, 'range': (0.020, 0.120), 'steps': 5,
        },
    },
}


def _linspace(spec: dict) -> list:
    lo, hi = spec['range']
    vals = np.linspace(lo, hi, spec['steps'])
    if spec.get('dtype') == int:
        vals = [int(round(v)) for v in vals]
    else:
        vals = [float(v) for v in vals]
    return vals


# ---------------------------------------------------------------------------
# Phase A: Sensitivity scan
# ---------------------------------------------------------------------------

def sensitivity_scan(stage: int, dataset_paths: list[str]) -> list[dict]:
    """
    For each parameter in the stage, sweep its full range in 5 steps with
    all others at default. Score each trial. Return rows sorted by |delta|
    descending so the most impactful parameters appear first.

    Returns list of {'param', 'value', 'score', 'delta'} dicts.
    """
    from test.tuning.scorer import score_datasets

    params = STAGE_PARAMS[stage]
    baseline = score_datasets(dataset_paths, overrides=None)
    baseline_total = baseline['total']

    rows = []
    for param_path, spec in params.items():
        for v in _linspace(spec):
            s = score_datasets(dataset_paths, overrides={param_path: v})
            rows.append({
                'param': param_path,
                'value': v,
                'score': s['total'],
                'delta': s['total'] - baseline_total,
            })

    rows.sort(key=lambda r: abs(r['delta']), reverse=True)
    return rows


def top_sensitive_params(sensitivity_rows: list[dict], n: int) -> list[str]:
    """
    Return the top-N most sensitive parameter dot-paths (no duplicates).
    """
    seen = {}
    result = []
    for row in sensitivity_rows:
        p = row['param']
        if p not in seen:
            seen[p] = True
            result.append(p)
        if len(result) >= n:
            break
    return result


# ---------------------------------------------------------------------------
# Phase B: Grid search
# ---------------------------------------------------------------------------

def grid_search(
    stage: int,
    dataset_paths: list[str],
    param_paths: list[str],
    n_values: int = 3,
) -> list[dict]:
    """
    Full combinatorial grid search over the specified parameters.

    param_paths: dot-paths to search (must exist in STAGE_PARAMS[stage])
    n_values: number of evenly-spaced values per parameter

    Returns list of trial result dicts sorted by total score descending.
    Each dict has 'overrides', 'total', 'filter_health', 'smoothness',
    'stability', 'n_frames', 'uwb_accepted', 'uwb_rejected'.
    """
    from test.tuning.scorer import score_datasets

    params = STAGE_PARAMS[stage]
    grids = []
    for p in param_paths:
        if p not in params:
            raise ValueError(f"Parameter '{p}' not found in stage {stage} params")
        spec = params[p]
        lo, hi = spec['range']
        vals = np.linspace(lo, hi, n_values)
        if spec.get('dtype') == int:
            vals = [int(round(v)) for v in vals]
        else:
            vals = [float(v) for v in vals]
        grids.append((p, vals))

    param_names  = [g[0] for g in grids]
    param_values = [g[1] for g in grids]

    trial_results = []
    total_combos  = 1
    for v in param_values:
        total_combos *= len(v)

    print(f"  Grid search: {len(param_names)} params × {n_values} values = {total_combos} trials")

    for i, combo in enumerate(itertools.product(*param_values), 1):
        overrides = dict(zip(param_names, combo))
        s = score_datasets(dataset_paths, overrides=overrides)
        s['overrides'] = overrides
        trial_results.append(s)
        if i % 10 == 0 or i == total_combos:
            print(f"    [{i}/{total_combos}] best so far: "
                  f"{max(r['total'] for r in trial_results):.2f}")

    trial_results.sort(key=lambda r: r['total'], reverse=True)
    return trial_results
