"""
tv6_contact_state.py
====================
Module 6 — Contact / Stroke State Detector

Transforms processed IMU events into one of four explicit stroke states.
This is the gating layer between sensor preprocessing and stroke integration.
Nothing downstream should ever write to a stroke buffer without consulting
this module first.

Input  (from tv4_imu_preprocess — processed IMU events):
    {
        'sensor':     'IMU',
        'ts_hw':      int,
        'acc_world':  (ax, ay, az),   ← world-frame linear acceleration, m/s²
        'jerk':       float,          ← |Δacc_world| / dt, m/s³
        'is_static':  bool,           ← ZUPT decision from preprocessor
        'contact':    bool,           ← force >= FORCE_CONTACT_THRESHOLD
        'force':      float,          ← raw force pass-through
        ...
    }

Output (state event — same dict, THREE fields added):
    {
        ...everything from input...,
        'stroke_state': str,     ← one of: 'IDLE', 'AIR_MOVE', 'CONTACT_STATIC', 'CONTACT_DRAWING'
        'stroke_active': bool,   ← True only when state == 'CONTACT_DRAWING'
        'stroke_id': int         ← Increments on every new confirmed stroke
    }
"""

import math
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
    def __init__(self,
                 force_enter:  float = None,
                 force_exit:   float = None,
                 debounce_n:   int   = 3,
                 min_draw_ms:  float = 30.0):
        
        base = getattr(cfg.imu, 'force_contact_threshold', 1000.0)
        self._force_enter = force_enter if force_enter is not None else base
        self._force_exit  = force_exit  if force_exit  is not None else base * 0.70
        self._debounce_n  = max(1, debounce_n)
        self._min_draw_ms = min_draw_ms

        # ── runtime state ────────────────────────────────────────────────────
        self._state          = IDLE        
        self._candidate      = IDLE        
        self._candidate_count = 0          
        self._contact_active = False       

        # Drawing duration guard & Stroke Tracking
        # A stroke is "live" from the first CONTACT_DRAWING that crosses
        # min_draw_ms within a contact session, until contact_active drops
        # (pen-up). Across that span the stroke_id stays stable, even when
        # the state dips to CONTACT_STATIC — those moments only pause point
        # logging (stroke_active=False) but do not close the stroke.
        self._draw_start_ts: int | None = None
        self._stroke_live = False
        self._stroke_id = 0

        # History for diagnostics
        self._state_history: deque[str] = deque(maxlen=200)
        self._transitions: list[tuple] = []

    def feed(self, events: list[dict]) -> list[dict]:
        out = []
        for ev in events:
            if ev.get('sensor') != 'IMU':
                ev['stroke_state']  = None
                ev['stroke_active'] = False
                ev['stroke_id']     = 0
                out.append(ev)
                continue
            out.append(self.process_one(ev))
        return out

    def process_one(self, ev: dict) -> dict:
        force     = ev.get('force', 0.0)
        is_static = ev.get('is_static', True)
        ts        = ev.get('ts_hw', 0)

        # ── 1. Update contact latch (hysteresis) ─────────────────────────────
        if not self._contact_active:
            if force >= self._force_enter:
                self._contact_active = True
        else:
            if force < self._force_exit:
                self._contact_active = False 

        # ── 2. Determine raw candidate state ─────────────────────────────────
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

        # ── 3. Debounce ───────────────────────────────────────────────────────
        if raw == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate       = raw
            self._candidate_count = 1

        if self._candidate_count >= self._debounce_n:
            if self._state != self._candidate:
                self._on_transition(self._state, self._candidate, ts)
            self._state = self._candidate

        # ── 4. Drawing duration guard & Stroke ID ─────────────────────────────
        if self._state == CONTACT_DRAWING:
            if self._draw_start_ts is None:
                self._draw_start_ts  = ts
                self._draw_confirmed = False

            elapsed_ms = ts - self._draw_start_ts
            if elapsed_ms >= self._min_draw_ms:
                self._draw_confirmed = True
        else:
            self._draw_start_ts  = None
            self._draw_confirmed = False

        stroke_active = (self._state == CONTACT_DRAWING and self._draw_confirmed)

        # Increment Stroke ID only when we officially transition into drawing
        if stroke_active and not self._was_drawing:
            self._stroke_id += 1
        self._was_drawing = stroke_active

        # ── 5. Record and annotate ────────────────────────────────────────────
        self._state_history.append(self._state)
        ev['stroke_state']  = self._state
        ev['stroke_active'] = stroke_active
        ev['stroke_id']     = self._stroke_id if stroke_active else 0
        
        return ev

    @property
    def state(self) -> str:
        return self._state

    def reset(self):
        self._state           = IDLE
        self._candidate       = IDLE
        self._candidate_count = 0
        self._contact_active  = False
        self._draw_start_ts   = None
        self._draw_confirmed  = False
        self._was_drawing     = False
        self._stroke_id       = 0
        self._state_history.clear()
        self._transitions.clear()

    def state_summary(self) -> dict:
        counts = {s: 0 for s in _ALL_STATES}
        for s in self._state_history:
            if s in counts:
                counts[s] += 1
        total = len(self._state_history)
        pcts  = {s: round(counts[s] / total * 100, 1) if total else 0.0
                 for s in _ALL_STATES}
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

    # --- WINDOWS SCIPY/QT CRASH FIX ---
    os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'
    # ----------------------------------

    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.cleaner.normalizer import StreamNormalizer
    from background.pipelines.preprocess.imu import IMUPreprocessor
    from background.pipelines.config import cfg

    SERIAL_PORT = getattr(cfg.serial, 'port', 'COM20')
    BAUD_RATE = getattr(cfg.serial, 'baud', 115200)
    DISPLAY_RATE = 0.1

    streamer = SerialStreamer(port=SERIAL_PORT, baud=BAUD_RATE)
    norm = StreamNormalizer()
    imu_prep = IMUPreprocessor()
    state_detector = ContactStateDetector()

    print("=" * 60)
    print(f"  [TEST] MODULE 6: State Machine Validator: {SERIAL_PORT}")
    print("  Draw a few strokes, hover, and hold still. Press Ctrl+C to stop.")
    print("=" * 60)

    event_log = []
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
                            # Safely using process_one!
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
                                    print(f"  STROKE ID : {state_event['stroke_id'] if state_event['stroke_active'] else '---'}")
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
            writer.writerow(['ts_hw', 'force', 'jerk', 'is_static', 'contact', 'stroke_state', 'stroke_id'])
            for ev in event_log:
                writer.writerow([
                    ev['ts_hw'], ev['force'], ev['jerk'], 
                    int(ev['is_static']), int(ev['contact']), 
                    ev['stroke_state'], ev['stroke_id']
                ])
        print(f"[EXPORT] Saved to {csv_filename}")

        print("[PLOT] Rendering State Timeline...")
        try:
            t_sec = [(ev['ts_hw'] - event_log[0]['ts_hw']) / 1_000_000.0 for ev in event_log]
            forces = [ev['force'] for ev in event_log]
            
            state_map = {'IDLE': 0, 'AIR_MOVE': 1, 'CONTACT_STATIC': 2, 'CONTACT_DRAWING': 3}
            states_num = [state_map[ev['stroke_state']] for ev in event_log]
            
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
            fig.suptitle('Module 6: Handwriting State Machine Timeline', fontsize=16, fontweight='bold')

            contact_threshold = getattr(cfg.imu, 'force_contact_threshold', 1000.0)
            ax1.plot(t_sec, forces, label='Raw Force Value', color='blue', linewidth=1.5)
            ax1.axhline(y=contact_threshold, color='red', linestyle='--', label='Contact Threshold')
            ax1.fill_between(t_sec, forces, contact_threshold, where=([f >= contact_threshold for f in forces]), color='red', alpha=0.2, label='Surface Contact Detected')
            ax1.set_ylabel('Force Output')
            ax1.set_title('Physical Force Sensor')
            ax1.grid(True, linestyle=':', alpha=0.7)
            ax1.legend(loc='upper right')

            ax2.step(t_sec, states_num, where='post', color='purple', linewidth=2.5)
            ax2.set_yticks([0, 1, 2, 3])
            ax2.set_yticklabels(['IDLE', 'AIR_MOVE', 'CONTACT_STATIC', 'CONTACT_DRAWING'])
            
            for i in range(1, len(t_sec)):
                if states_num[i] == 3:
                    ax2.axvspan(t_sec[i-1], t_sec[i], color='green', alpha=0.2)
                    
            ax2.set_ylabel('Logical State')
            ax2.set_xlabel('Time (Seconds)')
            ax2.set_title('Computed Stroke State (Green = Active Ink)')
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