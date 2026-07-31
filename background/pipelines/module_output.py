"""
Output directory helper for pipeline module self-tests.

Every pipeline module can be run directly to validate it against live hardware,
and most of those runs export a CSV and a plot. This routes those artifacts into
one predictable place instead of the working directory:

    test/module_runs/<module>/<YYYYMMDD-HHMMSS>/   the run that just finished
    test/module_runs/<module>/latest/              a copy of the newest run

Timestamped folders make before/after comparison possible, which is the usual
reason to re-run a module after changing config. The 'latest' copy exists so
scripts and docs can point at a stable path.

Used only by the __main__ blocks; nothing in the live pipeline touches it.

Usage inside a module self-test:

    from background.pipelines.module_output import ModuleRunOutput

    output = ModuleRunOutput('preprocess/imu')
    ...
    output.save_csv('imu_session.csv', rows, header=[...])
    figure.savefig(output.path('imu_session.png'), dpi=150)
    output.finish()
"""

import csv
import os
import shutil
from datetime import datetime

# Resolved from this file so a self-test writes to the same place regardless of
# the working directory it was launched from.
_PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MODULE_RUNS_ROOT = os.path.join(_PACKAGE_ROOT, 'test', 'module_runs')

LATEST_DIR_NAME = 'latest'
_RUN_STAMP_FORMAT = '%Y%m%d-%H%M%S'


class ModuleRunOutput:
    """Per-run output directory for one pipeline module's self-test."""

    def __init__(self, module_name: str, run_stamp: str | None = None):
        """
        Args:
            module_name: Pipeline-relative module path, e.g. 'preprocess/uwb/range'.
                         Becomes the folder name with separators flattened, so
                         every module lands in its own directory under
                         test/module_runs/.
            run_stamp:   Override the timestamp; defaults to the current time.
        """

        self.module_name = module_name
        self.folder_name = module_name.replace('/', '_').replace('\\', '_')
        self.run_stamp = run_stamp or datetime.now().strftime(_RUN_STAMP_FORMAT)

        self.module_dir = os.path.join(MODULE_RUNS_ROOT, self.folder_name)
        self.run_dir = os.path.join(self.module_dir, self.run_stamp)
        self.latest_dir = os.path.join(self.module_dir, LATEST_DIR_NAME)

        os.makedirs(self.run_dir, exist_ok=True)
        self._saved_files: list[str] = []

    def path(self, filename: str) -> str:
        """Absolute path for an output file inside this run's directory."""

        return os.path.join(self.run_dir, filename)

    def save_csv(self, filename: str, rows, header=None) -> str:
        """Write rows to a CSV in this run's directory and return its path."""

        destination = self.path(filename)
        with open(destination, mode='w', newline='') as handle:
            writer = csv.writer(handle)
            if header:
                writer.writerow(header)
            writer.writerows(rows)
        self.record(filename)
        return destination

    def record(self, filename: str) -> None:
        """
        Note a file written directly to path().

        Kept so callers can name their artifacts explicitly; finish() reports
        whatever is actually on disk, so a missed call costs nothing.
        """

        if filename not in self._saved_files:
            self._saved_files.append(filename)

    def finish(self, announce: bool = True) -> str:
        """
        Mirror this run into 'latest' and optionally print where things landed.

        Returns the run directory. Safe to call even if nothing was written.
        """

        self._refresh_latest()

        if announce:
            relative = os.path.relpath(self.run_dir, _PACKAGE_ROOT)
            print(f"[OUTPUT] {self.module_name} -> {relative}")
            for filename in self._written_files():
                print(f"[OUTPUT]   {filename}")
            print(f"[OUTPUT] latest copy -> "
                  f"{os.path.relpath(self.latest_dir, _PACKAGE_ROOT)}")

        return self.run_dir

    def _written_files(self) -> list[str]:
        """
        Filenames actually present in the run directory.

        Read from disk rather than from the recorded list so the report stays
        correct even when a caller saves a figure without calling record().
        """

        try:
            return sorted(
                name for name in os.listdir(self.run_dir)
                if os.path.isfile(os.path.join(self.run_dir, name))
            )
        except OSError:
            return list(self._saved_files)

    def _refresh_latest(self) -> None:
        """
        Replace the 'latest' directory with a copy of this run.

        A copy rather than a symlink: Windows needs elevated privileges or
        developer mode to create symlinks, which is not worth requiring here.
        """

        if not os.path.isdir(self.run_dir):
            return

        try:
            if os.path.isdir(self.latest_dir):
                shutil.rmtree(self.latest_dir)
            shutil.copytree(self.run_dir, self.latest_dir)
        except OSError as error:
            # The run's own output is already safe on disk; a failed convenience
            # copy must not take down the self-test that produced it.
            print(f"[OUTPUT] Could not refresh 'latest' copy: {error}")
