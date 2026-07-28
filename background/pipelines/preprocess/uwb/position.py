"""
Module 5c - UWB Position Filter

Applies physical constraints to raw solved positions and publishes the three
position signals the rest of the pipeline consumes.

Filtering order:
    1. Speed gate     reject fixes implying motion no pen could produce
    2. Board clamp    pull positions back inside the physical board (cutoff)
    3. Alpha-beta     smooth position while tracking velocity

The output is deliberately split rather than reduced to one "best" value,
because the fusion stage and the display want opposite things:

    pos_for_fusion          clamped only, no smoothing. The ESKF already models UWB
                            noise, so smoothing here would only add lag to the
                            Kalman update it feeds.
    pos_clean_for_display   alpha-beta smoothed, for plots and replay tools.
    pos_clean               backwards-compatible alias of the display signal.

Input:  solved positions from uwb "trilateration" module
Output: the same event with the position signals, 'mapped_position' for the
        ESKF, and 'uwb_quality' metadata added

Usage (import as a stage, or run directly for a live dashboard):
    python -m background.pipelines.preprocess.uwb.position
"""

import math
import os

os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

from background.pipelines.config import cfg

MICROSECONDS_PER_SECOND = 1_000_000

# Velocity retained after a fix the filter should not have trusted
# Near-total decay stops the tracker coasting on a trajectory it learned from bad geometry.
_UNTRUSTED_VELOCITY_RETENTION = 0.2


class UWBPositionFilter:
    """Constrains and smooths solved UWB positions into fusion-ready signals."""

    def __init__(self):
        self.board_width = cfg.anchors.board_size_x
        self.board_height = cfg.anchors.board_size_y
        self.max_speed_ms = cfg.uwb.outlier_speed_limit_ms

        self._alpha = cfg.uwb.pos_alpha
        self._beta = cfg.uwb.pos_beta

        self._est_x: float | None = None
        self._est_y: float | None = None
        self._vel_x: float = 0.0
        self._vel_y: float = 0.0

        # Held separately from the alpha-beta estimate: 
        # the speed gate must test the raw measurement sequence, not the smoothed one.
        self._prev_pos = None
        self._prev_ts = None

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------

    def process_one(self, event: dict) -> dict | None:
        """
        Constrain and smooth one solved position.

        Returns None when the fix implies impossible motion and the config asks
        for such outliers to be dropped rather than flagged.
        """

        if event.get('sensor') != 'POSITION' or 'pos_raw' not in event:
            return None

        raw_x, raw_y = event['pos_raw']
        ts = event['ts_hw']

        speed_flag = self._exceeds_speed_limit(raw_x, raw_y, ts)
        if speed_flag and cfg.uwb.drop_speed_outliers:
            return None

        clamped_x = max(0.0, min(self.board_width, raw_x))
        clamped_y = max(0.0, min(self.board_height, raw_y))
        was_clamped = (clamped_x != raw_x) or (clamped_y != raw_y)

        dt_s = self._elapsed_seconds(ts)
        self._update_alpha_beta(clamped_x, clamped_y, dt_s)

        low_confidence = event.get('low_confidence', False)
        if low_confidence or was_clamped:
            self._decay_untrusted_velocity()

        self._prev_pos = (clamped_x, clamped_y)
        self._prev_ts = ts

        self._attach_outputs(
            event, clamped_x, clamped_y, speed_flag, was_clamped, low_confidence
        )
        return event

    def reset(self):
        """Clear the alpha-beta estimate and speed-gate history."""

        self._est_x = None
        self._est_y = None
        self._vel_x = 0.0
        self._vel_y = 0.0
        self._prev_pos = None
        self._prev_ts = None

    # -------------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------------

    def _exceeds_speed_limit(self, raw_x: float, raw_y: float, ts: int) -> bool:
        """True when the step from the previous fix implies impossible pen speed."""

        if self._prev_pos is None or self._prev_ts is None:
            return False

        dt_s = (ts - self._prev_ts) / MICROSECONDS_PER_SECOND
        if dt_s <= 0:
            return False

        distance = math.hypot(raw_x - self._prev_pos[0], raw_y - self._prev_pos[1])
        return (distance / dt_s) > self.max_speed_ms

    def _elapsed_seconds(self, ts: int) -> float:
        """Seconds since the previous fix, falling back to the nominal UWB rate."""

        nominal_dt_s = 1.0 / cfg.uwb.rate_hz
        if self._prev_ts is None:
            return nominal_dt_s

        dt_s = (ts - self._prev_ts) / MICROSECONDS_PER_SECOND
        return dt_s if dt_s > 0 else nominal_dt_s

    # -------------------------------------------------------------------------
    # Alpha-beta tracking
    # -------------------------------------------------------------------------

    def _update_alpha_beta(self, measured_x: float, measured_y: float, dt_s: float):
        """
        Advance the alpha-beta tracker toward the measurement.

        Alpha corrects position from the residual; beta turns the same residual
        into a velocity estimate, which lets the tracker predict through the
        gaps between UWB fixes instead of stepping between them.
        """

        if self._est_x is None:
            self._est_x, self._est_y = measured_x, measured_y
            return

        predicted_x = self._est_x + self._vel_x * dt_s
        predicted_y = self._est_y + self._vel_y * dt_s

        residual_x = measured_x - predicted_x
        residual_y = measured_y - predicted_y

        self._est_x = predicted_x + self._alpha * residual_x
        self._est_y = predicted_y + self._alpha * residual_y
        self._vel_x += (self._beta * residual_x) / dt_s
        self._vel_y += (self._beta * residual_y) / dt_s

    def _decay_untrusted_velocity(self):
        """
        Shrink the velocity estimate after a fix the tracker should not learn from.

        A clamped or low-confidence measurement produces a residual that reflects
        solver error rather than motion, so the velocity it implies is phantom.
        """

        self._vel_x *= _UNTRUSTED_VELOCITY_RETENTION
        self._vel_y *= _UNTRUSTED_VELOCITY_RETENTION

    # -------------------------------------------------------------------------
    # Output assembly
    # -------------------------------------------------------------------------

    def _attach_outputs(self, event, clamped_x, clamped_y,
                        speed_flag, was_clamped, low_confidence):
        """Write the position signals and quality metadata onto the event."""

        event['pos_for_fusion'] = (round(clamped_x, 4), round(clamped_y, 4))
        event['pos_clean_for_display'] = (round(self._est_x, 4), round(self._est_y, 4))
        event['pos_clean'] = event['pos_clean_for_display']
        event['speed_flag'] = speed_flag

        # The ESKF reads this, so it carries the clamped-only signal.
        event['mapped_position'] = {
            'board_width_x': event['pos_for_fusion'][0],
            'board_height_y': event['pos_for_fusion'][1],
            'depth_z': cfg.anchors.a0[2],
        }
        event['coordinate_frame'] = 'UWB_BOARD_XY'

        # Drives adaptive measurement-noise inflation in the fusion stage.
        event['uwb_quality'] = {
            'solve_error': event.get('solve_error', 0.0),
            'speed_flag': speed_flag,
            'was_clamped': was_clamped,
            'low_confidence': low_confidence,
        }


# =============================================================================
# MODULE TESTING
#   Live trajectory dashboard: raw solver output versus the clamped and smoothed
#   result, with optional accuracy measurement against a known coordinate.
#
#   The filtered trace should stay smooth and strictly inside the board outline
#   even where the raw trace jumps outside it.
#
#   Run:  python -m background.pipelines.preprocess.uwb.position
# =============================================================================
if __name__ == '__main__':
    import csv
    import time

    import matplotlib.pyplot as plt

    from background.pipelines.cleaner.normalizer import StreamNormalizer
    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.module_output import ModuleRunOutput
    from background.pipelines.preprocess.uwb.range import UWBRangePreprocessor
    from background.pipelines.preprocess.uwb.trilateration import UWBSolver

    DISPLAY_RATE_S = 0.2
    CSV_FILENAME = 'position_filter_report.csv'
    PLOT_FILENAME = 'position_filter_report.png'

    streamer = SerialStreamer(port=cfg.serial.port, baud=cfg.serial.baud)
    normalizer = StreamNormalizer()
    range_preprocessor = UWBRangePreprocessor(offsets=cfg.uwb.range_offsets_m)
    solver = UWBSolver()
    position_filter = UWBPositionFilter()

    print("=" * 60)
    print(f"  [TEST] PHYSICS POLICE: Position Filter: {cfg.serial.port}")
    print("  Press Ctrl+C to stop and generate Trajectory Smoothing reports.")
    print("=" * 60)

    ground_truth = None
    print("Do you want to test clean position accuracy against a known coordinate? (y/n)")
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
                    solved = solver.process_one(cleaned)
                    if not solved:
                        continue

                    filtered = position_filter.process_one(solved)
                    if not filtered:
                        continue

                    event_log.append(filtered)

                    now = time.time()
                    if now - last_print_time >= DISPLAY_RATE_S:
                        os.system('cls' if os.name == 'nt' else 'clear')
                        print(f"========= TRAJECTORY SMOOTHER ({DISPLAY_RATE_S}s) =========")
                        print(f"  Pkt ID     : {filtered['packet_id']}")
                        print(f"  Raw Input  : X: {filtered['pos_raw'][0]:6.3f} m  |  "
                              f"Y: {filtered['pos_raw'][1]:6.3f} m")
                        print(f"  CLEAN OUT  : X: {filtered['pos_clean'][0]:6.3f} m  |  "
                              f"Y: {filtered['pos_clean'][1]:6.3f} m")

                        if ground_truth:
                            error_m = math.hypot(
                                filtered['pos_clean'][0] - ground_truth[0],
                                filtered['pos_clean'][1] - ground_truth[1],
                            )
                            print(f"  POS ERROR  : {error_m:.4f} m from target (Cleaned)")

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
                math.hypot(row['pos_clean'][0] - ground_truth[0],
                           row['pos_clean'][1] - ground_truth[1])
                for row in event_log
            ]
            average_error = sum(errors) / len(errors)
            print("\n" + "=" * 60)
            print("  [ACCURACY REPORT (CLEANED POSITION)]")
            print(f"  Target Coordinate : X={ground_truth[0]:.3f}, Y={ground_truth[1]:.3f}")
            print(f"  Samples Evaluated : {len(errors)}")
            print(f"  Average Error     : {average_error:.4f} meters "
                  f"({average_error * 100:.2f} cm)")
            print("=" * 60 + "\n")

        output = ModuleRunOutput('preprocess/uwb/position')
        output.save_csv(
            CSV_FILENAME,
            [[row['ts_hw'], row['packet_id'],
              row['pos_raw'][0], row['pos_raw'][1],
              row['pos_clean'][0], row['pos_clean'][1]]
             for row in event_log],
            header=['ts_hw', 'packet_id', 'raw_x', 'raw_y', 'clean_x', 'clean_y'],
        )

        print("[PLOT] Rendering Trajectory Report...")
        raw_x = [row['pos_raw'][0] for row in event_log]
        raw_y = [row['pos_raw'][1] for row in event_log]
        clean_x = [row['pos_clean'][0] for row in event_log]
        clean_y = [row['pos_clean'][1] for row in event_log]
        start_ts = event_log[0]['ts_hw']
        seconds = [(row['ts_hw'] - start_ts) / MICROSECONDS_PER_SECOND for row in event_log]

        board_width = cfg.anchors.board_size_x
        board_height = cfg.anchors.board_size_y

        figure = plt.figure(figsize=(14, 10))
        figure.suptitle('Module 5c: Position Filtering & Boundary Clamping',
                        fontsize=16, fontweight='bold')

        axis_board = plt.subplot(2, 1, 1)
        axis_board.add_patch(plt.Rectangle(
            (0, 0), board_width, board_height,
            fill=False, edgecolor='black', linestyle='--', lw=2,
        ))
        axis_board.scatter(
            [0, board_width, board_width, 0], [0, 0, board_height, board_height],
            c='red', s=100, marker='s', label='Anchors',
        )
        axis_board.plot(raw_x, raw_y, label='Raw Math Trajectory (Can exit bounds)',
                        color='red', alpha=0.3, linewidth=1, marker='.')
        axis_board.plot(clean_x, clean_y, label='Filtered Trajectory (Clamped & Smoothed)',
                        color='blue', linewidth=2)

        if ground_truth:
            axis_board.plot(ground_truth[0], ground_truth[1], marker='X',
                            color='green', markersize=12, label='Ground Truth Target')

        axis_board.set_xlim(-0.2, board_width + 0.2)
        axis_board.set_ylim(-0.2, board_height + 0.2)
        axis_board.set_aspect('equal')
        axis_board.set_title('2D Board Tracking')
        axis_board.grid(True, linestyle=':', alpha=0.7)
        axis_board.legend()

        axis_x = plt.subplot(2, 2, 3)
        axis_x.plot(seconds, raw_x, color='red', alpha=0.3, label='Raw X')
        axis_x.plot(seconds, clean_x, color='blue', linewidth=2, label='Clean X')
        axis_x.axhline(y=0, color='black', linestyle='--')
        axis_x.axhline(y=board_width, color='black', linestyle='--')
        axis_x.set_title('X-Axis Smoothing & Clamping')
        axis_x.set_xlabel('Time (s)')
        axis_x.set_ylabel('X Position (m)')
        axis_x.grid(True, linestyle='--', alpha=0.5)

        axis_y = plt.subplot(2, 2, 4, sharex=axis_x)
        axis_y.plot(seconds, raw_y, color='red', alpha=0.3, label='Raw Y')
        axis_y.plot(seconds, clean_y, color='blue', linewidth=2, label='Clean Y')
        axis_y.axhline(y=0, color='black', linestyle='--')
        axis_y.axhline(y=board_height, color='black', linestyle='--')
        axis_y.set_title('Y-Axis Smoothing & Clamping')
        axis_y.set_xlabel('Time (s)')
        axis_y.set_ylabel('Y Position (m)')
        axis_y.grid(True, linestyle='--', alpha=0.5)

        plt.tight_layout()
        plt.savefig(output.path(PLOT_FILENAME), dpi=300)
        output.record(PLOT_FILENAME)
        output.finish()
        plt.show()
