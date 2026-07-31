"""
Module 6 - Contact / Stroke State Detector

Decides when the marker is actually drawing, turning force and motion into the
stroke session that the fusion and reconstruction stages gate on.

Three layers, each answering a different question:

    Layer 1 - Physical contact latch
        Is the tip touching the board right now?
        The pen only goes down when enough pressure is applied consistently, and only lifts
        when the pressure stays low. This filters out accidental bumps and shaky micro-movements.

    Layer 2 - Kinematic substate (diagnostic)
        What kind of contact is it? IDLE, AIR_MOVE, CONTACT_STATIC, or
        CONTACT_DRAWING, debounced over consecutive samples.

    Layer 3 - Logical stroke session
        Is this one continuous stroke?
        Opens once enough cumulative drawing time accrues within a single contact
        latch, and stays open across CONTACT_STATIC micro-pauses. 

Input:  processed IMU motion events from "imu" module
Output: the same event with 'stroke_state', 'stroke_active', and 'stroke_id' added

Usage (import as a stage, or run directly for a live dashboard):
    python -m background.pipelines.preprocess.contact
"""

from collections import deque

from background.pipelines.config import cfg

MICROSECONDS_PER_MILLISECOND = 1000

IDLE = 'IDLE'
AIR_MOVE = 'AIR_MOVE'
CONTACT_STATIC = 'CONTACT_STATIC'
CONTACT_DRAWING = 'CONTACT_DRAWING'

_ALL_STATES = (IDLE, AIR_MOVE, CONTACT_STATIC, CONTACT_DRAWING)

_STATE_HISTORY_SAMPLES = 200


class ContactStateDetector:
    """Derives physical contact, kinematic substate, and stroke sessions."""

    def __init__(self):
        contact_cfg = cfg.contact
        imu_cfg = cfg.imu

        self._force_enter = imu_cfg.force_contact_threshold
        # Releasing below the entry threshold gives the hysteresis band that
        # stops a force hovering at the threshold from chattering the latch.
        self._force_exit = imu_cfg.force_contact_threshold * contact_cfg.force_exit_ratio

        self._pen_down_debounce_us = contact_cfg.pen_down_debounce_ms * MICROSECONDS_PER_MILLISECOND
        self._pen_up_debounce_us = contact_cfg.pen_up_debounce_ms * MICROSECONDS_PER_MILLISECOND
        self._min_draw_us = contact_cfg.min_draw_ms * MICROSECONDS_PER_MILLISECOND
        self._debounce_samples = max(1, contact_cfg.state_debounce_n)

        # Layer 1
        self._contact_active = False
        self._pen_down_ts: int | None = None
        self._pen_up_ts: int | None = None

        # Layer 2
        self._state = IDLE
        self._candidate = IDLE
        self._candidate_count = 0

        # Layer 3
        self._stroke_live = False
        self._stroke_id = 0
        self._cumulative_draw_us = 0.0
        self._last_ts: int | None = None

        self._state_history: deque[str] = deque(maxlen=_STATE_HISTORY_SAMPLES)
        self._transitions: list[tuple] = []

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------

    def feed(self, events: list[dict]) -> list[dict]:
        """Annotate a batch, passing non-IMU events through with neutral state."""

        annotated = []
        for event in events:
            if event.get('sensor') != 'IMU':
                event['stroke_state'] = None
                event['stroke_active'] = False
                event['stroke_id'] = self._stroke_id
                annotated.append(event)
                continue
            annotated.append(self.process_one(event))
        return annotated

    def process_one(self, event: dict) -> dict:
        """Annotate one IMU event with contact substate and stroke session fields."""

        force = event.get('force', 0.0)
        is_static = event.get('is_static', True)
        ts = event.get('ts_hw', 0)

        elapsed_us = (
            ts - self._last_ts
            if self._last_ts is not None and ts > self._last_ts
            else 0.0
        )
        self._last_ts = ts

        was_in_contact = self._contact_active
        self._update_contact_latch(force, ts)
        self._update_kinematic_substate(is_static, ts)
        self._update_stroke_session(was_in_contact, elapsed_us)

        self._state_history.append(self._state)
        event['stroke_state'] = self._state
        event['stroke_active'] = self._stroke_live
        # Holds the last completed id between sessions, and 0 before the first.
        event['stroke_id'] = self._stroke_id
        return event

    @property
    def state(self) -> str:
        return self._state

    def reset(self):
        """Clear all latch, substate, and session state."""

        self.__init__()

    def state_summary(self) -> dict:
        """Counts and percentages of each substate across the retained history."""

        counts = {state: 0 for state in _ALL_STATES}
        for state in self._state_history:
            if state in counts:
                counts[state] += 1

        total = len(self._state_history)
        percentages = {
            state: round(counts[state] / total * 100, 1) if total else 0.0
            for state in _ALL_STATES
        }
        return {'counts': counts, 'pcts': percentages, 'total': total}

    def transition_log(self) -> list[tuple]:
        return list(self._transitions)

    # -------------------------------------------------------------------------
    # Layer 1 - Physical contact latch
    # -------------------------------------------------------------------------

    def _update_contact_latch(self, force: float, ts: int):
        """
        Latch physical contact using hysteresis plus a time debounce on both edges.

        Each edge requires its threshold to hold continuously for the debounce
        window; a force excursion that recovers within the window cancels the
        pending transition rather than committing it.
        """

        if not self._contact_active:
            if force >= self._force_enter:
                if self._pen_down_ts is None:
                    self._pen_down_ts = ts
                if (ts - self._pen_down_ts) >= self._pen_down_debounce_us:
                    self._contact_active = True
                    self._pen_up_ts = None
            else:
                self._pen_down_ts = None
            return

        if force < self._force_exit:
            if self._pen_up_ts is None:
                self._pen_up_ts = ts
            if (ts - self._pen_up_ts) >= self._pen_up_debounce_us:
                self._contact_active = False
                self._pen_down_ts = None
        else:
            # Force recovered inside the window, so this was a tremor.
            self._pen_up_ts = None

    # -------------------------------------------------------------------------
    # Layer 2 - Kinematic substate
    # -------------------------------------------------------------------------

    def _update_kinematic_substate(self, is_static: bool, ts: int):
        """
        Classify contact and motion into a substate, debounced over N samples.

        Diagnostic only: downstream stages gate on stroke_active, not on this.
        """

        moving = not is_static

        if self._contact_active:
            observed = CONTACT_DRAWING if moving else CONTACT_STATIC
        else:
            observed = AIR_MOVE if moving else IDLE

        if observed == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = observed
            self._candidate_count = 1

        if self._candidate_count >= self._debounce_samples:
            if self._state != self._candidate:
                self._transitions.append((self._state, self._candidate, ts))
            self._state = self._candidate

    # -------------------------------------------------------------------------
    # Layer 3 - Logical stroke session
    # -------------------------------------------------------------------------

    def _update_stroke_session(self, was_in_contact: bool, elapsed_us: float):
        """
        Open and close the logical stroke that downstream stages treat as ink.

        A session needs a minimum of accumulated drawing time before it opens, so
        a brief graze that never becomes a stroke does not claim a stroke id.
        Once open it survives CONTACT_STATIC pauses and closes only on pen-up.
        """

        if was_in_contact and not self._contact_active:
            self._stroke_live = False
            self._cumulative_draw_us = 0.0

        if not self._contact_active:
            return

        if self._state == CONTACT_DRAWING:
            self._cumulative_draw_us += elapsed_us

        if not self._stroke_live and self._cumulative_draw_us >= self._min_draw_us:
            self._stroke_live = True
            self._stroke_id += 1


# =============================================================================
# MODULE TESTING
#   Live state-machine validator: prints the force reading, stillness flag, and
#   resulting substate/session as you draw. Exports a CSV and a timeline plot on
#   Ctrl+C, with the active stroke session shaded green.
#
#   Draw a few strokes, hover between them, and hold still to see all four
#   substates and confirm micro-pauses do not fragment a stroke.
#
#   Run:  python -m background.pipelines.preprocess.contact
# =============================================================================
if __name__ == '__main__':
    import csv
    import os
    import time

    os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

    import matplotlib.pyplot as plt

    from background.pipelines.cleaner.normalizer import StreamNormalizer
    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.module_output import ModuleRunOutput
    from background.pipelines.preprocess.imu import IMUPreprocessor

    DISPLAY_RATE_S = 0.1
    CSV_FILENAME = 'state_detector_report.csv'
    PLOT_FILENAME = 'state_detector_report.png'

    streamer = SerialStreamer(port=cfg.serial.port, baud=cfg.serial.baud)
    normalizer = StreamNormalizer()
    imu_preprocessor = IMUPreprocessor()
    detector = ContactStateDetector()

    print("=" * 60)
    print(f"  [TEST] MODULE 6: State Machine Validator: {cfg.serial.port}")
    print("  Draw a few strokes, hover, and hold still. Press Ctrl+C to stop.")
    print("=" * 60)

    event_log = []
    last_print_time = 0.0

    try:
        while True:
            raw_packets = streamer.read_new_packets()
            for event in normalizer.normalize(raw_packets):
                if event['sensor'] != 'IMU':
                    continue

                motion = imu_preprocessor.process_one(event)
                if not motion:
                    continue

                annotated = detector.process_one(motion)
                event_log.append(annotated)

                now = time.time()
                if now - last_print_time >= DISPLAY_RATE_S:
                    os.system('cls' if os.name == 'nt' else 'clear')
                    print(f"========= STATE DETECTOR ({DISPLAY_RATE_S}s) =========")
                    print(f"  Force     : {annotated['force']:6.1f} "
                          f"(Contact: {annotated['contact']})")
                    print(f"  Jerk      : {annotated['jerk']:6.1f} "
                          f"(Static : {annotated['is_static']})")
                    print("-" * 52)
                    print(f"  STATE     : [{annotated['stroke_state']}]")
                    print(f"  ACTIVE    : {annotated['stroke_active']}")
                    stroke_label = annotated['stroke_id'] if annotated['stroke_id'] > 0 else '---'
                    print(f"  STROKE ID : {stroke_label}")
                    print("====================================================")
                    last_print_time = now

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[STOP] Data collection halted.")
        streamer.close()

        if not event_log:
            print("No data collected. Exiting.")
            raise SystemExit(0)

        output = ModuleRunOutput('preprocess/contact')
        output.save_csv(
            CSV_FILENAME,
            [[row['ts_hw'], row['force'], row['jerk'],
              int(row['is_static']), int(row['contact']),
              row['stroke_state'], int(row['stroke_active']), row['stroke_id']]
             for row in event_log],
            header=['ts_hw', 'force', 'jerk', 'is_static', 'contact',
                    'stroke_state', 'stroke_active', 'stroke_id'],
        )

        print("[PLOT] Rendering State Timeline...")
        try:
            start_ts = event_log[0]['ts_hw']
            seconds = [(row['ts_hw'] - start_ts) / 1_000_000.0 for row in event_log]
            forces = [row['force'] for row in event_log]

            state_index = {IDLE: 0, AIR_MOVE: 1, CONTACT_STATIC: 2, CONTACT_DRAWING: 3}
            states = [state_index[row['stroke_state']] for row in event_log]
            active_flags = [row['stroke_active'] for row in event_log]

            figure, (axis_force, axis_state) = plt.subplots(
                2, 1, figsize=(14, 8), sharex=True
            )
            figure.suptitle('Module 6: Handwriting State Machine Timeline',
                            fontsize=16, fontweight='bold')

            threshold = cfg.imu.force_contact_threshold
            axis_force.plot(seconds, forces, label='Raw Force Value',
                            color='blue', linewidth=1.5)
            axis_force.axhline(y=threshold, color='red', linestyle='--',
                               label='Contact Threshold')
            axis_force.fill_between(
                seconds, forces, threshold,
                where=[value >= threshold for value in forces],
                color='red', alpha=0.2, label='Surface Contact Detected',
            )
            axis_force.set_ylabel('Force Output')
            axis_force.set_title('Physical Force Sensor')
            axis_force.grid(True, linestyle=':', alpha=0.7)
            axis_force.legend(loc='upper right')

            axis_state.step(seconds, states, where='post', color='purple', linewidth=2.5)
            axis_state.set_yticks([0, 1, 2, 3])
            axis_state.set_yticklabels(list(state_index))

            # Shade the whole session, not just CONTACT_DRAWING samples, so
            # micro-pauses inside a stroke are visibly part of it.
            for index in range(1, len(seconds)):
                if active_flags[index]:
                    axis_state.axvspan(seconds[index - 1], seconds[index],
                                       color='green', alpha=0.2)

            axis_state.set_ylabel('Logical State')
            axis_state.set_xlabel('Time (Seconds)')
            axis_state.set_title('Computed Stroke State (Green = Active Stroke Session)')
            axis_state.grid(True, axis='x', linestyle=':', alpha=0.7)

            plt.tight_layout()
            plt.savefig(output.path(PLOT_FILENAME), dpi=300)
            output.record(PLOT_FILENAME)
            output.finish()
            plt.show(block=True)

        except KeyboardInterrupt:
            print("\n[INFO] Matplotlib UI interrupted. Image was saved to disk.")
        except Exception as error:
            print(f"\n[ERROR] Plotting failed: {error}")
