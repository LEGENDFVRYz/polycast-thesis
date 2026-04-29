"""
contact.py
====================
Module 6 — Contact / Stroke State Detector

Transforms processed IMU events into one of four explicit stroke states,
then derives a logical stroke session from the physical pen-down lifecycle.

Input  (from imu.py — processed IMU events):
    {
        'sensor':     'IMU',
        'ts_hw':      int,            ← hardware timestamp, microseconds
        'acc_world':  (ax, ay, az),   ← world-frame linear acceleration, m/s²
        'jerk':       float,          ← |Δacc_world| / dt, m/s³
        'is_static':  bool,           ← ZUPT decision from preprocessor
        'contact':    bool,           ← force >= force_contact_threshold
        'force':      float,          ← raw force pass-through
        ...
    }

Output (same dict, three fields added):
    {
        ...everything from input...,
        'stroke_state':  str,    ← diagnostic kinematic substate:
                                    'IDLE' | 'AIR_MOVE' | 'CONTACT_STATIC' | 'CONTACT_DRAWING'
        'stroke_active': bool,   ← True for the entire confirmed pen-down session,
                                    including CONTACT_STATIC micro-pauses within a stroke.
                                    Drops to False only on confirmed pen-up.
        'stroke_id':     int     ← Stable within a session. Increments once per confirmed
                                    stroke (after min_draw_ms of cumulative drawing time).
                                    Emits last completed id between sessions; 0 before
                                    the first session ever opens.
    }

Three-layer design
------------------
Layer 1 — Physical contact latch:
    Symmetric hysteresis + time debounce on both pen-down and pen-up.
    Pen-down latches only after force >= force_enter for pen_down_debounce_ms → rejects bumps.
    Pen-up releases only after force <  force_exit  for pen_up_debounce_ms   → rejects tremors.

Layer 2 — Kinematic substate (diagnostic):
    Four-state map (IDLE/AIR_MOVE/CONTACT_STATIC/CONTACT_DRAWING) debounced over
    state_debounce_n consecutive samples. Emitted as stroke_state. Downstream logic
    does NOT gate on this — it is for logging and plots only.

Layer 3 — Logical stroke session:
    A session opens when cumulative CONTACT_DRAWING time within one contact latch cycle
    reaches min_draw_ms. stroke_active stays True across CONTACT_STATIC micro-pauses
    (e.g. peaks of the letter "M"). Session closes on pen-up. stroke_id is stable for
    the full session and persists (not zeroed) between sessions.
"""

from collections import deque
from background.pipelines.config import cfg

# ── State constants ────────────────────────────────────────────────────────────
IDLE             = 'IDLE'
AIR_MOVE         = 'AIR_MOVE'
CONTACT_STATIC   = 'CONTACT_STATIC'
CONTACT_DRAWING  = 'CONTACT_DRAWING'

_ALL_STATES = (IDLE, AIR_MOVE, CONTACT_STATIC, CONTACT_DRAWING)


# ── State detector ─────────────────────────────────────────────────────────────
class ContactStateDetector:
    def __init__(self):
        c   = cfg.contact
        imu = cfg.imu

        self._force_enter          = imu.force_contact_threshold
        self._force_exit           = imu.force_contact_threshold * c.force_exit_ratio
        self._pen_down_debounce_us = c.pen_down_debounce_ms * 1000.0
        self._pen_up_debounce_us   = c.pen_up_debounce_ms   * 1000.0
        self._min_draw_us          = c.min_draw_ms           * 1000.0
        self._debounce_n           = max(1, c.state_debounce_n)

        # Layer 1 — physical contact latch
        self._contact_active  = False
        self._pen_down_ts: int | None = None  # ts of first sample >= force_enter (while not active)
        self._pen_up_ts:   int | None = None  # ts of first sample <  force_exit  (while active)

        # Layer 2 — kinematic substate (diagnostic)
        self._state           = IDLE
        self._candidate       = IDLE
        self._candidate_count = 0

        # Layer 3 — logical stroke session
        self._stroke_live        = False
        self._stroke_id          = 0
        self._cumulative_draw_us = 0.0   # CONTACT_DRAWING time within current contact session
        self._last_ts: int | None = None

        # Diagnostics
        self._state_history: deque[str] = deque(maxlen=200)
        self._transitions: list[tuple]  = []

    # ──────────────────────────────────────────────────────────────────────────
    def feed(self, events: list[dict]) -> list[dict]:
        out = []
        for ev in events:
            if ev.get('sensor') != 'IMU':
                ev['stroke_state']  = None
                ev['stroke_active'] = False
                ev['stroke_id']     = self._stroke_id
                out.append(ev)
                continue
            out.append(self.process_one(ev))
        return out

    def process_one(self, ev: dict) -> dict:
        force     = ev.get('force', 0.0)
        is_static = ev.get('is_static', True)
        ts        = ev.get('ts_hw', 0)

        dt_us = (ts - self._last_ts) if (self._last_ts is not None and ts > self._last_ts) else 0.0
        self._last_ts = ts

        # ── Layer 1: Physical contact latch ───────────────────────────────────
        prev_contact = self._contact_active

        if not self._contact_active:
            if force >= self._force_enter:
                if self._pen_down_ts is None:
                    self._pen_down_ts = ts
                if (ts - self._pen_down_ts) >= self._pen_down_debounce_us:
                    self._contact_active = True
                    self._pen_up_ts = None
            else:
                self._pen_down_ts = None  # force dropped out, restart the pen-down timer
        else:
            if force < self._force_exit:
                if self._pen_up_ts is None:
                    self._pen_up_ts = ts
                if (ts - self._pen_up_ts) >= self._pen_up_debounce_us:
                    self._contact_active = False
                    self._pen_down_ts = None
            else:
                self._pen_up_ts = None  # force recovered — was just a tremor, cancel pen-up

        # ── Layer 2: Kinematic substate (debounced, diagnostic only) ──────────
        in_contact = self._contact_active
        moving     = not is_static

        if in_contact and moving:
            raw = CONTACT_DRAWING
        elif in_contact and not moving:
            raw = CONTACT_STATIC
        elif not in_contact and moving:
            raw = AIR_MOVE
        else:
            raw = IDLE

        if raw == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate       = raw
            self._candidate_count = 1

        if self._candidate_count >= self._debounce_n:
            if self._state != self._candidate:
                self._on_transition(self._state, self._candidate, ts)
            self._state = self._candidate

        # ── Layer 3: Logical stroke session ───────────────────────────────────
        # Pen just lifted off — close any open session
        if prev_contact and not self._contact_active:
            self._stroke_live        = False
            self._cumulative_draw_us = 0.0

        # Within a contact session: accumulate drawing time, then open once threshold met
        if self._contact_active:
            if self._state == CONTACT_DRAWING:
                self._cumulative_draw_us += dt_us

            if not self._stroke_live and self._cumulative_draw_us >= self._min_draw_us:
                self._stroke_live = True
                self._stroke_id  += 1

        # ── Annotate ──────────────────────────────────────────────────────────
        self._state_history.append(self._state)
        ev['stroke_state']  = self._state
        ev['stroke_active'] = self._stroke_live
        ev['stroke_id']     = self._stroke_id  # 0 until first session; last id between sessions
        return ev

    # ── Public API ────────────────────────────────────────────────────────────
    @property
    def state(self) -> str:
        return self._state

    def reset(self):
        self._contact_active     = False
        self._pen_down_ts        = None
        self._pen_up_ts          = None
        self._state              = IDLE
        self._candidate          = IDLE
        self._candidate_count    = 0
        self._stroke_live        = False
        self._stroke_id          = 0
        self._cumulative_draw_us = 0.0
        self._last_ts            = None
        self._state_history.clear()
        self._transitions.clear()

    def state_summary(self) -> dict:
        counts = {s: 0 for s in _ALL_STATES}
        for s in self._state_history:
            if s in counts:
                counts[s] += 1
        total = len(self._state_history)
        pcts  = {s: round(counts[s] / total * 100, 1) if total else 0.0 for s in _ALL_STATES}
        return {'counts': counts, 'pcts': pcts, 'total': total}

    def transition_log(self) -> list[tuple]:
        return list(self._transitions)

    def _on_transition(self, from_state: str, to_state: str, ts: int):
        self._transitions.append((from_state, to_state, ts))


# ==============================================================================
# HARDWARE DATA LOGGING & REPORTING (Module 6: State Machine Validator)
# ==============================================================================
if __name__ == '__main__':
    import time
    import os
    import csv
    import matplotlib.pyplot as plt

    os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.cleaner.normalizer import StreamNormalizer
    from background.pipelines.preprocess.imu import IMUPreprocessor
    from background.pipelines.config import cfg

    SERIAL_PORT  = getattr(cfg.serial, 'port', 'COM20')
    BAUD_RATE    = getattr(cfg.serial, 'baud', 115200)
    DISPLAY_RATE = 0.1

    streamer       = SerialStreamer(port=SERIAL_PORT, baud=BAUD_RATE)
    norm           = StreamNormalizer()
    imu_prep       = IMUPreprocessor()
    state_detector = ContactStateDetector()

    print("=" * 60)
    print(f"  [TEST] MODULE 6: State Machine Validator: {SERIAL_PORT}")
    print("  Draw a few strokes, hover, and hold still. Press Ctrl+C to stop.")
    print("=" * 60)

    event_log      = []
    last_print_time = 0

    try:
        while True:
            raw_packets = streamer.read_new_packets()
            if raw_packets:
                events = norm.normalize(raw_packets)
                for ev in events:
                    if ev['sensor'] == 'IMU':
                        prep_imu = imu_prep.process_one(ev)
                        if prep_imu:
                            state_event = state_detector.process_one(prep_imu)
                            if state_event:
                                event_log.append(state_event)
                                current_time = time.time()
                                if current_time - last_print_time >= DISPLAY_RATE:
                                    os.system('cls' if os.name == 'nt' else 'clear')
                                    print(f"========= STATE DETECTOR ({DISPLAY_RATE}s) =========")
                                    print(f"  Force     : {state_event['force']:6.1f} (Contact: {state_event['contact']})")
                                    print(f"  Jerk      : {state_event['jerk']:6.1f} (Static : {state_event['is_static']})")
                                    print("-" * 52)
                                    print(f"  STATE     : [{state_event['stroke_state']}]")
                                    print(f"  ACTIVE    : {state_event['stroke_active']}")
                                    print(f"  STROKE ID : {state_event['stroke_id'] if state_event['stroke_id'] > 0 else '---'}")
                                    print("====================================================")
                                    last_print_time = current_time
            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[STOP] Data collection halted.")
        streamer.close()

        if not event_log:
            print("No data collected. Exiting.")
            exit()

        csv_filename = "state_detector_report.csv"
        with open(csv_filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['ts_hw', 'force', 'jerk', 'is_static', 'contact',
                             'stroke_state', 'stroke_active', 'stroke_id'])
            for ev in event_log:
                writer.writerow([
                    ev['ts_hw'], ev['force'], ev['jerk'],
                    int(ev['is_static']), int(ev['contact']),
                    ev['stroke_state'], int(ev['stroke_active']), ev['stroke_id'],
                ])
        print(f"[EXPORT] Saved to {csv_filename}")

        print("[PLOT] Rendering State Timeline...")
        try:
            t_sec  = [(ev['ts_hw'] - event_log[0]['ts_hw']) / 1_000_000.0 for ev in event_log]
            forces = [ev['force'] for ev in event_log]

            state_map  = {IDLE: 0, AIR_MOVE: 1, CONTACT_STATIC: 2, CONTACT_DRAWING: 3}
            states_num = [state_map[ev['stroke_state']] for ev in event_log]
            active     = [ev['stroke_active'] for ev in event_log]

            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
            fig.suptitle('Module 6: Handwriting State Machine Timeline', fontsize=16, fontweight='bold')

            contact_threshold = cfg.imu.force_contact_threshold
            ax1.plot(t_sec, forces, label='Raw Force Value', color='blue', linewidth=1.5)
            ax1.axhline(y=contact_threshold, color='red', linestyle='--', label='Contact Threshold')
            ax1.fill_between(t_sec, forces, contact_threshold,
                             where=[f >= contact_threshold for f in forces],
                             color='red', alpha=0.2, label='Surface Contact Detected')
            ax1.set_ylabel('Force Output')
            ax1.set_title('Physical Force Sensor')
            ax1.grid(True, linestyle=':', alpha=0.7)
            ax1.legend(loc='upper right')

            ax2.step(t_sec, states_num, where='post', color='purple', linewidth=2.5)
            ax2.set_yticks([0, 1, 2, 3])
            ax2.set_yticklabels(['IDLE', 'AIR_MOVE', 'CONTACT_STATIC', 'CONTACT_DRAWING'])

            # Highlight the full logical stroke session (not just CONTACT_DRAWING moments)
            for i in range(1, len(t_sec)):
                if active[i]:
                    ax2.axvspan(t_sec[i - 1], t_sec[i], color='green', alpha=0.2)

            ax2.set_ylabel('Logical State')
            ax2.set_xlabel('Time (Seconds)')
            ax2.set_title('Computed Stroke State (Green = Active Stroke Session)')
            ax2.grid(True, axis='x', linestyle=':', alpha=0.7)

            plt.tight_layout()

            plot_filename = "state_detector_report.png"
            plt.savefig(plot_filename, dpi=300)
            print(f"[EXPORT] Plot saved successfully to {plot_filename}")
            plt.show(block=True)

        except KeyboardInterrupt:
            print("\n[INFO] Matplotlib UI interrupted. Image was saved to disk.")
        except Exception as e:
            print(f"\n[ERROR] Plotting failed: {e}")
