"""
Named pipeline configurations, and the switch that selects one.

The pipeline has shipped several distinct fusion / post-processing designs. Each
is fully determined by a handful of flags in `config.py`, so a design can be
reproduced by setting flags rather than by checking out an old commit. That is
deliberate: checking out `b7b6aec` would also drag in a different range
pre-filter, a different renderer and a different contact detector, and the
comparison would stop being about fusion.

Select one with the `POLYCAST_MODE` environment variable:

    POLYCAST_MODE=fused python -m background.pipelines.visualizer

Unset means `config.py`'s own values, which is exactly the behaviour this
package had before this module existed. An unknown key is a hard error rather
than a silent fallback, because a typo that quietly runs the wrong stage would
invalidate whatever it was used to measure.

The most important property here is that `active_summary()` reads `cfg` and
never the preset that was requested. A hand-edited config must report the edit,
not the label it no longer matches - otherwise the banner and the window title
would confidently lie about what produced an output.
"""

from __future__ import annotations

import dataclasses
import os
from dataclasses import dataclass, field

# NOT imported at module scope. `config.py` imports this module at the bottom of
# its own body to honour POLYCAST_MODE, so a module-scope import here would be a
# cycle: importing `modes` first would re-enter a half-built `config`. Resolved
# by fetching the singleton per call - it is a module attribute, so this is a
# dict lookup, and every pipeline stage already reads `cfg` at call time.
def _config():
    from background.pipelines import config
    return config.cfg

# Environment variable, read once at config import time.
MODE_ENV_VAR = 'POLYCAST_MODE'

# Every pen-up stage that can be switched independently, in the order
# `reconstruct.py::_close_current` runs them. `active_summary()` walks this
# list, so a new stage only has to be named here to become visible.
POSTPROCESS_STAGES = (
    'velocity_detrend',
    'stroke_cleaner',
    'imu_degeneracy',
    'two_point_anchor',
    'postprocess',
    'trace_filter',
)


@dataclass(frozen=True)
class PipelineMode:
    """One named configuration, expressed as flag overrides."""

    key: str
    label: str
    description: str

    # The commit this configuration reproduces, for traceability. Reproducing a
    # commit's *flags* on today's code is not the same as checking that commit
    # out; see the module docstring for why that is the intended trade.
    commit: str = ''

    # Plot colour, so every comparison tool draws a given mode the same way.
    colour: str = '#444444'

    # Overrides applied to cfg.fusion_eskf.
    fusion: dict = field(default_factory=dict)

    # Pen-up stage name -> enabled. Names come from POSTPROCESS_STAGES.
    # A dict rather than one boolean field per stage: there are six already and
    # the list grows, so named fields would need editing in three places each
    # time one is added.
    postprocess: dict = field(default_factory=dict)


def _stages(**enabled) -> dict:
    """All pen-up stages off, except the ones named."""

    base = {name: False for name in POSTPROCESS_STAGES}
    base.update(enabled)
    return base


MODES: dict[str, PipelineMode] = {
    'fused': PipelineMode(
        key='fused',
        label='FUSED (blended ESKF)',
        description='UWB corrects continuously through the Kalman update while '
                    'ink is drawn. Prediction blends bias-corrected and '
                    'high-passed acceleration; drag and the velocity cap are live.',
        commit='8883b96',
        colour='#1f77d0',
        fusion={'shape_mode': False, 'ink_from_dead_reckoner': False,
                'imu_only_mode': False},
        postprocess=_stages(),
    ),
    'imu-shape': PipelineMode(
        key='imu-shape',
        label='IMU-SHAPE (UWB places, IMU draws)',
        description='UWB is severed during ink and places the stroke once at '
                    'pen-down. The letter is plain double integration of '
                    'acc_board_tip - no bias subtraction, no drag, no cap. The '
                    'display trace filter is on, but nothing corrects geometry '
                    'at pen-up.',
        commit='5e6acae',
        colour='#2ca02c',
        fusion={'shape_mode': True, 'ink_from_dead_reckoner': True,
                'imu_only_mode': False},
        postprocess=_stages(trace_filter=True),
    ),
    'imu-shape+pp': PipelineMode(
        key='imu-shape+pp',
        label='IMU-SHAPE + two-point anchor',
        description='First post-stroke generation: both endpoints of the '
                    'finished stroke are pinned to UWB, removing the t^2 drift '
                    'parabola the dead reckoner accumulates. Note this commit '
                    'left centroid/min-jerk off - postprocess.enabled is False '
                    'there - so the anchor is the whole pass.',
        commit='b7b6aec',
        colour='#ff7f0e',
        fusion={'shape_mode': True, 'ink_from_dead_reckoner': True,
                'imu_only_mode': False},
        postprocess=_stages(two_point_anchor=True),
    ),
    'imu-shape+pp2': PipelineMode(
        key='imu-shape+pp2',
        label='IMU-SHAPE + detrend/trace filter',
        description='Second post-stroke generation, and the current default. '
                    'Closes each stroke velocity integration at pen-up, then '
                    'smooths for display with a corner-preserving causal EMA. '
                    'Deliberately excludes centroid/min-jerk, which flatten '
                    'letterforms.',
        commit='30ad840',
        colour='#d62728',
        fusion={'shape_mode': True, 'ink_from_dead_reckoner': True,
                'imu_only_mode': False},
        postprocess=_stages(velocity_detrend=True, trace_filter=True),
    ),
    'imu-only': PipelineMode(
        key='imu-only',
        label='IMU-ONLY (diagnostic)',
        colour='#9467bd',
        description='UWB is recorded and seeds the initial anchor but never '
                    'corrects the state. Shows what the inertial signal carries '
                    'on its own.',
        fusion={'shape_mode': True, 'ink_from_dead_reckoner': True,
                'imu_only_mode': True},
        postprocess=_stages(),
    ),
}

DEFAULT_MODE = 'imu-shape+pp2'


class UnknownModeError(ValueError):
    """Raised for a mode key that is not in MODES."""


def get(key: str) -> PipelineMode:
    """Look up a mode, raising with the valid keys listed if it is unknown."""

    try:
        return MODES[key]
    except KeyError:
        valid = ', '.join(sorted(MODES))
        raise UnknownModeError(
            f'unknown {MODE_ENV_VAR}={key!r}. Valid modes: {valid}'
        ) from None


def apply(mode: PipelineMode):
    """
    Apply a mode's flags to the frozen config. Returns a restore callback.

    `object.__setattr__` is the only way past a frozen dataclass, and every
    pipeline stage reads `cfg` at call time rather than caching it, so the
    override takes effect without reimporting anything.
    """

    cfg = _config()
    touched = ['fusion_eskf'] + [
        name for name in mode.postprocess if hasattr(cfg, name)
    ]
    saved = {name: getattr(cfg, name) for name in touched}

    if mode.fusion:
        object.__setattr__(
            cfg, 'fusion_eskf', dataclasses.replace(cfg.fusion_eskf, **mode.fusion)
        )

    for name, enabled in mode.postprocess.items():
        section = getattr(cfg, name, None)
        if section is None:
            # A stage this build does not carry. Skipped rather than raising:
            # modes are shared across branches that differ in which stages exist.
            continue
        object.__setattr__(cfg, name, dataclasses.replace(section, enabled=enabled))

    def restore():
        for name, value in saved.items():
            object.__setattr__(cfg, name, value)

    return restore


def _fusion_name(shape_mode: bool, ink_dr: bool, imu_only: bool) -> str:
    """Collapse the three fusion flags into one name."""

    if imu_only:
        return 'imu-only'
    if shape_mode and ink_dr:
        return 'imu-shape'
    if not shape_mode and not ink_dr:
        return 'fused'
    return 'mixed'


def active_summary() -> dict:
    """
    Report what is actually switched on, read from `cfg`.

    Deliberately does not consult the requested mode. If someone edits
    `config.py` by hand, this reports the edit - so the banner cannot claim a
    preset the configuration no longer matches.
    """

    cfg = _config()
    eskf = cfg.fusion_eskf
    stages = [
        name for name in POSTPROCESS_STAGES
        if getattr(getattr(cfg, name, None), 'enabled', False)
    ]
    fusion = _fusion_name(
        eskf.shape_mode, eskf.ink_from_dead_reckoner, eskf.imu_only_mode
    )

    return {
        'fusion': fusion,
        'shape_mode': eskf.shape_mode,
        'ink_from_dead_reckoner': eskf.ink_from_dead_reckoner,
        'imu_only_mode': eskf.imu_only_mode,
        'stages': stages,
        'matches': _matching_key(fusion, stages),
    }


def _matching_key(fusion: str, stages: list[str]) -> str:
    """Name of the mode whose flags equal the live config, or '' if none does."""

    for key, mode in MODES.items():
        enabled = {name for name, on in mode.postprocess.items() if on}
        if set(stages) != enabled:
            continue
        spec = mode.fusion
        expected = _fusion_name(
            bool(spec.get('shape_mode')),
            bool(spec.get('ink_from_dead_reckoner')),
            bool(spec.get('imu_only_mode')),
        )
        if expected == fusion:
            return key
    return ''


def banner() -> str:
    """One-line description of the live configuration, for logs and titles."""

    summary = active_summary()
    stages = ', '.join(summary['stages']) if summary['stages'] else 'none'
    key = summary['matches']
    name = MODES[key].label if key else 'CUSTOM (no preset matches)'
    return f'[MODE] {name}  |  fusion: {summary["fusion"]}  |  pen-up: {stages}'


def _load_dotenv_once() -> None:
    """
    Pull `.env` into the environment, if python-dotenv is installed.

    The pipeline is launched as `python -m background.pipelines.visualizer`,
    which never imports the project-root `config.py` - so the `load_dotenv()`
    call that serves the Flask app does not run, and `.env` would be invisible
    to the pipeline. Loaded here instead of at import so it happens once, on the
    single call that actually reads the variable.

    `override=False` is dotenv's default and is kept deliberately: a variable
    set in the shell must beat the file, so a one-off
    `$env:POLYCAST_MODE="..."` still wins over a checked-in default.
    """

    try:
        from dotenv import load_dotenv
    except ImportError:
        return
    load_dotenv()


def apply_from_environment() -> str:
    """
    Honour `POLYCAST_MODE` if it is set. Returns the key applied, or ''.

    Called once from `config.py` after `cfg` is constructed. The restore
    callback is discarded on purpose - this is a process-wide selection, not a
    scoped override.
    """

    _load_dotenv_once()

    # Surrounding quotes are stripped by dotenv but survive a shell `set`, so a
    # value written as POLYCAST_MODE="fused" outside dotenv would otherwise fail
    # the lookup against an unquoted key.
    key = os.environ.get(MODE_ENV_VAR, "").strip().strip("\"'")
    if not key:
        return ''
    apply(get(key))
    return key


# =============================================================================
# MODULE TESTING
#   Every mode must round-trip - apply it, and active_summary() must name it
#   back. That is what proves the registry and the reporter agree; if they ever
#   disagree the banner is lying, which is the one failure this module cannot
#   be allowed to have.
#
#   Run:  python -m background.pipelines.modes
# =============================================================================
if __name__ == '__main__':
    print('=' * 74)
    print('  Pipeline modes')
    print('=' * 74)
    print(f'  {"key":16s}{"commit":10s}{"fusion":12s}pen-up stages')
    print('  ' + '-' * 70)
    for key, mode in MODES.items():
        spec = mode.fusion
        fusion = _fusion_name(
            bool(spec.get('shape_mode')),
            bool(spec.get('ink_from_dead_reckoner')),
            bool(spec.get('imu_only_mode')),
        )
        stages = ', '.join(n for n, on in mode.postprocess.items() if on) or '-'
        print(f'  {key:16s}{mode.commit or "-":10s}{fusion:12s}{stages}')

    print()
    print(f'  live: {banner()}')

    failures = []

    for key, mode in MODES.items():
        restore = apply(mode)
        try:
            got = active_summary()['matches']
            if got != key:
                failures.append(f'{key} reported back as {got or "CUSTOM"}')
        finally:
            restore()

    before = active_summary()
    restore = apply(MODES['fused'])
    restore()
    if active_summary() != before:
        failures.append('restore did not return the config to its prior state')

    try:
        get('nope')
        failures.append('unknown key did not raise')
    except UnknownModeError as error:
        if 'Valid modes' not in str(error):
            failures.append('error message does not list the valid modes')

    print()
    for line in failures:
        print(f'  [FAIL] {line}')
    print(f'  [{"PASS" if not failures else "FAIL"}] '
          f'{len(MODES)} modes round-trip, restore is clean, unknown key raises')
    print('=' * 74)
