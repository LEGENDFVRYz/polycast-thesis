"""
In-memory config patcher for tuning trials.

All dataclasses in config.py are frozen=True, so we cannot assign fields
directly. Instead we use dataclasses.replace() to build a new frozen instance
with the single changed leaf, then write it back to the live cfg singleton
via object.__setattr__ (bypasses frozen protection at the CPython C level).

Usage:
    from test.tuning.config_patcher import patch_cfg, reset_cfg

    patch_cfg({'fusion_eskf.modes.drawing.sigma_scale': 0.9,
               'fusion_eskf.sigma_a': 2.8})
    results = run_dataset(...)
    reset_cfg()
"""

import dataclasses
import background.pipelines.config as _cfg_module
from background.pipelines.config import cfg


def patch_cfg(overrides: dict) -> None:
    """
    Apply {dot-path: value} overrides to the live cfg singleton in-place.

    Supported path depths:
        'fusion_eskf.sigma_a'                       — 2 levels
        'fusion_eskf.modes.drawing.sigma_scale'     — 4 levels
        'uwb.trilat_max_residual'                   — 2 levels

    All pipeline objects must be re-instantiated after this call because
    several preprocessors cache config values at __init__ time.
    """

    for dotpath, value in overrides.items():
        _apply_dotpath(cfg, dotpath, value)


def reset_cfg() -> None:
    """
    Restore cfg to factory defaults by rebuilding all sub-configs from scratch.
    Call this between tuning trials to guarantee a clean slate.
    """

    defaults = _cfg_module.Config()
    for f in dataclasses.fields(defaults):
        object.__setattr__(cfg, f.name, getattr(defaults, f.name))


def _apply_dotpath(root_cfg, dotpath: str, value) -> None:
    """
    Traverse dotpath, build dataclasses.replace() calls bottom-up, then
    patch the top-level field on root_cfg with object.__setattr__.

    For 'fusion_eskf.modes.drawing.sigma_scale':
        1. new_drawing = replace(cfg.fusion_eskf.modes.drawing, sigma_scale=value)
        2. new_modes   = replace(cfg.fusion_eskf.modes, drawing=new_drawing)
        3. new_eskf    = replace(cfg.fusion_eskf, modes=new_modes)
        4. object.__setattr__(cfg, 'fusion_eskf', new_eskf)
    """

    parts = dotpath.split('.')

    # Build chain:
    #   chain[0] = root_cfg
    #   chain[1] = cfg.<parts[0]>          (top-level sub-config)
    #   chain[k] = chain[k-1].<parts[k-1]> (nested sub-config)
    # chain has len(parts) entries; the leaf value lives at chain[-1].<parts[-1]>
    chain = [root_cfg]
    for attr in parts[:-1]:
        chain.append(getattr(chain[-1], attr))

    # Replace bottom-up:
    #   Start with the deepest frozen object (chain[-1]) and replace its leaf field.
    #   Then walk up the chain, replacing each parent's field with the rebuilt child.
    leaf_field = parts[-1]
    new_obj = dataclasses.replace(chain[-1], **{leaf_field: value})

    # i goes from len(chain)-2 down to 0.
    # chain[i] is the parent; parts[i] is the field name on chain[i] that holds chain[i+1].
    # We rebuild chain[i] with parts[i] = new_obj (the freshly replaced child).
    for i in range(len(chain) - 2, -1, -1):
        new_obj = dataclasses.replace(chain[i], **{parts[i]: new_obj})

    # new_obj is now a rebuilt root_cfg with the single leaf changed.
    # Extract the updated top-level field and patch it onto the live cfg singleton.
    object.__setattr__(root_cfg, parts[0], getattr(new_obj, parts[0]))
