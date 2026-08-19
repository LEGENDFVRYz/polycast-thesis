"""
Check that every mode reproduces the commit it claims.

A `PipelineMode` carrying a `commit` is making a historical claim: "set these
flags on today's code and you get the configuration that commit shipped". The
claim is what makes a rendered comparison meaningful, and nothing in `modes.py`
can check it - the module deliberately knows nothing about git.

Two modes were wrong when this was first run: `imu-shape+pp` had
`postprocess=True, two_point_anchor=False` where `b7b6aec` has the reverse, and
`imu-shape` had the display trace filter off where `5e6acae` has it on. Both
produced plausible-looking output under the wrong label, which is worse than an
obvious failure - the mode name made the render look trustworthy.

Not part of `modes.py`'s own self-test on purpose: that module is imported by
`config.py` on every pipeline start, and shelling out to git there would make
startup depend on the working tree being a repository.

Usage:
    python -m background.pipelines.tools.verify_modes
"""

from __future__ import annotations

import re
import subprocess
import sys

from background.pipelines import modes

# Config section name -> the dataclass that carries its `enabled` flag.
_SECTION_CLASS = {
    'postprocess': 'PostprocessConfig',
    'stroke_cleaner': 'StrokeCleanerConfig',
    'two_point_anchor': 'TwoPointAnchorConfig',
    'imu_degeneracy': 'IMUDegeneracyConfig',
    'trace_filter': 'TraceFilterConfig',
    'velocity_detrend': 'VelocityDetrendConfig',
}

_FUSION_FLAGS = ('shape_mode', 'ink_from_dead_reckoner', 'imu_only_mode')


def commit_flags(commit: str) -> dict | None:
    """
    Read the stage flags out of a commit's `config.py`.

    Parsed with regex rather than imported: the file at an old commit may not be
    importable against today's package, and only these flags are needed.
    A section that does not exist yet counts as off, which is what the pipeline
    does at runtime too.
    """

    result = subprocess.run(
        ['git', 'show', f'{commit}:background/pipelines/config.py'],
        capture_output=True, text=True, encoding='utf-8',
    )
    if result.returncode != 0:
        return None

    text = result.stdout
    flags: dict[str, bool] = {}

    for name in _FUSION_FLAGS:
        match = re.search(r'^\s*' + name + r': bool = (\w+)', text, re.M)
        flags[name] = bool(match and match.group(1) == 'True')

    for name, class_name in _SECTION_CLASS.items():
        block = re.search(
            r'class ' + class_name + r':(.*?)(?=\n@dataclass|\nclass |\Z)', text, re.S
        )
        if not block:
            flags[name] = False
            continue
        match = re.search(r'enabled: bool = (\w+)', block.group(1))
        flags[name] = bool(match and match.group(1) == 'True')

    return flags


def report_reachability() -> int:
    """
    List fields marked `inert(...)` in config.py, grouped by reason.

    These are not dead - each has a live consumer - but the branch that reads
    them does not execute under the shipped default, so tuning them changes
    nothing. Printing them is the whole point: the failure this guards against
    is someone spending an afternoon on a value that was never read.

    Returns 0 always. This reports, it does not judge - a marked field is a
    documented state, not an error.
    """

    import collections
    import dataclasses

    from background.pipelines import config

    by_reason = collections.defaultdict(list)

    def walk(obj, path):
        cls = type(obj)
        if not dataclasses.is_dataclass(cls):
            return
        for f in dataclasses.fields(cls):
            reason = f.metadata.get('inert')
            if reason:
                by_reason[reason].append(path + '.' + f.name)
            value = getattr(obj, f.name, None)
            if dataclasses.is_dataclass(type(value)):
                walk(value, path + '.' + f.name)

    walk(config.cfg, 'cfg')

    print()
    print('=' * 74)
    print('  Reachability: fields whose reader the default config never reaches')
    print('=' * 74)

    if not by_reason:
        print('  none marked')
        return 0

    total = sum(len(v) for v in by_reason.values())
    for reason, names in sorted(by_reason.items(), key=lambda kv: -len(kv[1])):
        print()
        print('  ' + str(len(names)) + ' field(s) - ' + reason)
        for name in sorted(names):
            print('      ' + name)

    print()
    print('  ' + '-' * 70)
    print('  ' + str(total) + ' field(s) marked inert. Editing these under the')
    print('  shipped default changes nothing; other modes make them live.')
    return 0


def main() -> int:
    print('=' * 74)
    print('  Mode fidelity: does each preset reproduce the commit it names?')
    print('=' * 74)

    mismatches = 0

    for key, mode in modes.MODES.items():
        if not mode.commit:
            print(f'  ----     {key:16s} no commit reference')
            continue

        source = commit_flags(mode.commit)
        if source is None:
            print(f'  SKIP     {key:16s} commit {mode.commit} not reachable')
            continue

        claimed = {**mode.fusion, **mode.postprocess}
        differences = [
            (name, want, source.get(name, False))
            for name, want in claimed.items()
            if source.get(name, False) != want
        ]

        if not differences:
            print(f'  OK       {key:16s} matches {mode.commit}')
            continue

        mismatches += 1
        print(f'  MISMATCH {key:16s} vs {mode.commit}')
        for name, want, actual in differences:
            print(f'               {name:24s} mode={want!s:6s} commit={actual}')

    print('  ' + '-' * 70)
    if mismatches:
        print(f'  [FAIL] {mismatches} mode(s) do not match the commit they name')
        return 1
    print('  [PASS] every mode reproduces the commit it names')
    return 0


if __name__ == '__main__':
    if '--reachability' in sys.argv:
        sys.exit(report_reachability())
    sys.exit(main())
