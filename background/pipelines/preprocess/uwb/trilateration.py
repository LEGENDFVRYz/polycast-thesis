"""
Module 5b - UWB Trilateration Solver

Solves a 2D board position from the cleaned anchor ranges by finding the point
whose distances to the anchors best match what was measured.

The solver is a weighted least-squares fit rather than exact sphere intersection,
because four noisy anchor ranges rarely meet at a single point. Three deliberate
choices shape it:

    Weighting     nearer anchors are trusted more, since a shorter path has less
                  opportunity for multipath reflection
    Robust loss   soft_l1 keeps one bad range from dragging the whole solution
    Warm start    each solve begins from the previous result, which converges
                  faster and keeps successive fixes from hopping between minima

The residual is reported unweighted so its threshold stays a real distance in
metres, and it doubles as the quality signal the ESKF uses to scale UWB trust.

Input:  cleaned ranges from uwb "range" module
Output: {'sensor': 'POSITION', 'ts_hw', 'packet_id', 'pos_raw',
         'solve_error', 'low_confidence'}, or None if the solve is unusable

Usage (import as a stage, or run directly for a live dashboard):
    python -m background.pipelines.preprocess.uwb.trilateration
"""

import os

os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

import numpy as np
from scipy.optimize import least_squares

from background.pipelines.config import cfg

# Trilateration is underdetermined with fewer than three ranges.
_MIN_ANCHORS_FOR_SOLVE = 3

# Search bounds extend slightly past the board so a tag held at the very edge
_BOUNDS_MARGIN_M = 0.10

# Residual band between "clean" and the hard rejection threshold.
_LOW_CONFIDENCE_RESIDUAL_M = 0.08

# The pen tip is on the board plane; anchors sit slightly proud of it, and the
# 3D distance math accounts for that offset on its own.
_PEN_PLANE_Z_M = 0.0


class UWBSolver:
    """Weighted least-squares position solver over the cleaned anchor ranges."""

    def __init__(self):
        self.anchors = np.array(cfg.anchors.positions)
        self.board_width = cfg.anchors.board_size_x
        self.board_height = getattr(cfg.anchors, 'board_size_y', 1.24)

        self.pen_z = _PEN_PLANE_Z_M

        self.bounds_min = [-_BOUNDS_MARGIN_M, -_BOUNDS_MARGIN_M]
        self.bounds_max = [
            self.board_width + _BOUNDS_MARGIN_M,
            self.board_height + _BOUNDS_MARGIN_M,
        ]
        self._guess = np.array([self.board_width / 2.0, self.board_height / 2.0])

        self._last_valid_ts: int | None = None
        self._stale_timeout_us: int = cfg.uwb.stale_guess_timeout_us

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------

    def process_one(self, event: dict) -> dict | None:
        """
        Solve one position fix from cleaned ranges.

        Returns None when too few anchors are valid, the solver raises, or the
        residual shows it converged to a geometrically impossible point.
        """

        if event.get('sensor') != 'UWB' or 'clean_dists' not in event:
            return None

        clean_distances = np.array(event['clean_dists'])
        valid_mask = np.array(event.get('valid_mask', [True] * 4), dtype=bool)

        valid_distances = clean_distances[valid_mask]
        valid_anchors = self.anchors[valid_mask]

        if len(valid_distances) < _MIN_ANCHORS_FOR_SOLVE:
            return None

        ts = event['ts_hw']
        self._reseed_guess_if_stale(ts, valid_anchors)

        try:
            solution = least_squares(
                self._residuals,
                self._guess,
                bounds=(self.bounds_min, self.bounds_max),
                args=(valid_distances, valid_anchors, self._anchor_weights(valid_distances)),
                loss='soft_l1',
                f_scale=0.1,
            )
        except Exception:
            return None

        raw_x, raw_y = solution.x
        rms_error = self._geometric_rms(valid_anchors, valid_distances, raw_x, raw_y)

        # The optimizer always converges to something; a large residual means it
        # converged to a point no combination of real ranges could describe.
        if rms_error > cfg.uwb.trilat_max_residual:
            return None

        self._guess = solution.x
        self._last_valid_ts = ts

        return {
            'sensor': 'POSITION',
            'ts_hw': ts,
            'packet_id': event['packet_id'],
            'pos_raw': (round(raw_x, 4), round(raw_y, 4)),
            'solve_error': round(rms_error, 6),
            'low_confidence': rms_error > _LOW_CONFIDENCE_RESIDUAL_M,
        }

    def reset(self):
        """Return the warm-start guess to board centre and clear staleness."""

        self._guess = np.array([self.board_width / 2.0, self.board_height / 2.0])
        self._last_valid_ts = None

    # -------------------------------------------------------------------------
    # Solver internals
    # -------------------------------------------------------------------------

    def _residuals(self, guess_xy, distances, valid_anchors, weights):
        """
        Weighted distance residuals for the least-squares fit.

        Scaling each residual by sqrt(weight) makes least_squares minimize the
        weighted sum of squares, which is what the IDW weighting intends.
        """

        guess_xyz = np.array([guess_xy[0], guess_xy[1], self.pen_z])
        predicted = np.linalg.norm(valid_anchors - guess_xyz, axis=1)
        return (predicted - distances) * np.sqrt(weights)

    @staticmethod
    def _anchor_weights(valid_distances: np.ndarray) -> np.ndarray:
        """
        Inverse-distance weights, normalized so the mean weight stays 1.

        Normalizing keeps the effective residual scale constant regardless of how
        many anchors are valid, so f_scale means the same thing in every solve.
        """

        raw_weights = 1.0 / (valid_distances ** cfg.uwb.wls_power + cfg.uwb.wls_epsilon)
        return raw_weights * (len(raw_weights) / raw_weights.sum())

    def _geometric_rms(self, valid_anchors, valid_distances, raw_x, raw_y) -> float:
        """
        RMS of the unweighted range errors, in metres.

        Deliberately unweighted: the solver's own residuals carry the IDW
        weighting, which would make the rejection threshold a unitless quantity
        rather than a physical distance.
        """

        solved_point = np.array([raw_x, raw_y, self.pen_z])
        errors = np.linalg.norm(valid_anchors - solved_point, axis=1) - valid_distances
        return float(np.sqrt(np.mean(errors ** 2)))

    def _reseed_guess_if_stale(self, ts: int, valid_anchors: np.ndarray):
        """
        Restart from the anchor centroid after a long gap in valid solves.

        A stale guess is worse than no guess: the tag may have been lifted and
        moved anywhere, and warm-starting from where it used to be can pull the
        solver into a local minimum near the old position.
        """

        if (self._last_valid_ts is None
                or (ts - self._last_valid_ts) > self._stale_timeout_us):
            self._guess = np.mean(valid_anchors[:, :2], axis=0)


# =============================================================================
# MODULE TESTING
#   Live solver dashboard: raw solved coordinates and the least-squares residual,
#   with optional accuracy measurement against a known coordinate.
#
#   Expect visible scatter - this is the unfiltered geometric solve, so the
#   spread here is the raw UWB noise the later stages exist to suppress. The
#   residual plot shows whether the ranges actually intersect cleanly.
#
#   Run:  python -m background.pipelines.preprocess.uwb.trilateration
# =============================================================================
if __name__ == '__main__':
    import csv
    import math
    import time

    import matplotlib.pyplot as plt

    from background.pipelines.cleaner.normalizer import StreamNormalizer
    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.module_output import ModuleRunOutput
    from background.pipelines.preprocess.uwb.range import UWBRangePreprocessor

    DISPLAY_RATE_S = 0.2
    CSV_FILENAME = 'trilateration_math_report.csv'
    PLOT_FILENAME = 'trilateration_math_report.png'

    streamer = SerialStreamer(port=cfg.serial.port, baud=cfg.serial.baud)
    normalizer = StreamNormalizer()
    range_preprocessor = UWBRangePreprocessor(offsets=cfg.uwb.range_offsets_m)
    solver = UWBSolver()

    print("=" * 60)
    print(f"  [TEST] PURE MATH: Trilateration Solver: {cfg.serial.port}")
    print("  Press Ctrl+C to stop and generate raw mathematical reports.")
    print("=" * 60)

    ground_truth = None
    print("Do you want to test accuracy against a specific known coordinate? (y/n)")
    if input().strip().lower() == 'y':
        try:
            ground_truth = (
                float(input("  Enter expected X coordinate (m): ")),
                float(input("  Enter expected Y coordinate (m): ")),
            )
            print(f"  [SET] Target ground truth: X={ground_truth[0]:.3f}, Y={ground_truth[1]:.3f}")
        except ValueError:
            print("  [ERROR] Invalid input. Proceeding without ground truth.")
    print("=" * 60)

    event_log = []
    last_print_time = 0.0

    try:
        while True:
            raw_packets = streamer.read_new_packets()
            if raw_packets:
                for cleaned in range_preprocessor.feed(normalizer.normalize(raw_packets)):
                    position = solver.process_one(cleaned)
                    if not position:
                        continue

                    event_log.append(position)

                    now = time.time()
                    if now - last_print_time >= DISPLAY_RATE_S:
                        os.system('cls' if os.name == 'nt' else 'clear')
                        print(f"========= RAW TRILATERATION MATH ({DISPLAY_RATE_S}s) =========")
                        print(f"  Pkt ID     : {position['packet_id']}")
                        print(f"  Solve Cost : {position['solve_error']:.6f} (Math Confidence)")
                        print(f"  RAW POS    : X: {position['pos_raw'][0]:6.3f} m  |  "
                              f"Y: {position['pos_raw'][1]:6.3f} m")

                        if ground_truth:
                            error_m = math.hypot(
                                position['pos_raw'][0] - ground_truth[0],
                                position['pos_raw'][1] - ground_truth[1],
                            )
                            print(f"  POS ERROR  : {error_m:.4f} m from target")

                        print("==========================================================")
                        last_print_time = now

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[STOP] Data collection halted.")
        streamer.close()

        if not event_log:
            print("No data collected. Exiting.")
            raise SystemExit(0)

        if ground_truth:
            errors = [
                math.hypot(row['pos_raw'][0] - ground_truth[0],
                           row['pos_raw'][1] - ground_truth[1])
                for row in event_log
            ]
            average_error = sum(errors) / len(errors)
            print("\n" + "=" * 60)
            print("  [ACCURACY REPORT]")
            print(f"  Target Coordinate : X={ground_truth[0]:.3f}, Y={ground_truth[1]:.3f}")
            print(f"  Samples Evaluated : {len(errors)}")
            print(f"  Average Error     : {average_error:.4f} meters "
                  f"({average_error * 100:.2f} cm)")
            print("=" * 60 + "\n")

        output = ModuleRunOutput('preprocess/uwb/trilateration')
        output.save_csv(
            CSV_FILENAME,
            [[row['ts_hw'], row['packet_id'],
              row['pos_raw'][0], row['pos_raw'][1], row['solve_error']]
             for row in event_log],
            header=['ts_hw', 'packet_id', 'raw_x', 'raw_y', 'solve_error'],
        )

        print("[PLOT] Rendering Math Report...")
        try:
            solved_x = [row['pos_raw'][0] for row in event_log]
            solved_y = [row['pos_raw'][1] for row in event_log]
            residuals = [row['solve_error'] for row in event_log]
            start_ts = event_log[0]['ts_hw']
            seconds = [(row['ts_hw'] - start_ts) / 1_000_000.0 for row in event_log]

            figure, (axis_board, axis_error) = plt.subplots(1, 2, figsize=(14, 6))
            figure.suptitle('Module 5b: Pure Trilateration Math',
                            fontsize=16, fontweight='bold')

            board_width = cfg.anchors.board_size_x
            board_height = cfg.anchors.board_size_y

            axis_board.add_patch(plt.Rectangle(
                (0, 0), board_width, board_height,
                fill=False, edgecolor='black', linestyle='--', lw=2,
            ))
            axis_board.scatter(
                [0, board_width, board_width, 0], [0, 0, board_height, board_height],
                c='red', s=100, marker='s', label='Anchors',
            )
            axis_board.plot(solved_x, solved_y, label='Raw Solved Coordinates',
                            color='red', alpha=0.5, marker='.', linestyle='none')

            if ground_truth:
                axis_board.plot(ground_truth[0], ground_truth[1], marker='X',
                                color='blue', markersize=12, label='Ground Truth Target')

            axis_board.set_xlim(-0.2, board_width + 0.2)
            axis_board.set_ylim(-0.2, board_height + 0.2)
            axis_board.set_aspect('equal')
            axis_board.set_title('Raw Geometric Intersections')
            axis_board.grid(True, linestyle=':', alpha=0.7)
            axis_board.legend()

            axis_error.plot(seconds, residuals, color='purple', linewidth=1.5)
            axis_error.axhline(y=cfg.uwb.trilat_max_residual, color='red',
                               linestyle='--', label='Rejection Threshold')
            axis_error.set_title('Least-Squares Optimization Cost (Confidence)')
            axis_error.set_xlabel('Time (s)')
            axis_error.set_ylabel('Residual Error (Lower = Better geometric fit)')
            axis_error.grid(True, linestyle='--', alpha=0.5)
            axis_error.legend()

            plt.tight_layout()
            # Saved before show() so a GUI crash cannot lose the report.
            plt.savefig(output.path(PLOT_FILENAME), dpi=300)
            output.record(PLOT_FILENAME)
            output.finish()
            plt.show(block=True)

        except KeyboardInterrupt:
            print("\n[INFO] Matplotlib UI interrupted. Image was saved to disk.")
        except Exception as error:
            print(f"\n[ERROR] Plotting failed: {error}")
