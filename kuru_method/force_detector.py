"""
force_detector.py  —  PolyCast Spring-Mechanism Force Contact Detector
=======================================================================
Hardware design:
    • Force sensor is mounted at the REAR of the marker
    • A spring sits between the tip and the rear sensor
    • When tip touches the board → tip pushes back → spring compresses
      → sensor at rear receives force
    • When tip lifts → spring returns to natural length → sensor reads ~0

This design is fundamentally different from grip-based detection:
    IDLE state  →  force ≈ 0    (spring at rest, no compression)
    WRITING     →  force rises   (spring compressed by board contact)

Consequences for the pipeline:
    ✗ Adaptive baseline tracking is REMOVED — the resting state is always
      near zero by physics, not by user behaviour. Tracking a drifting
      baseline would only add lag and complexity we don't need.

    ✓ Instead: a fixed noise-floor dead-band eliminates sensor micro-noise
      at rest, giving us a clean zero reference.

    ✓ Hysteresis + debounce are kept — spring bounce on contact/release
      still causes short transient oscillations that debounce suppresses.

    ✓ EMA smoothing is kept to clean the sensor's analog noise before
      threshold comparison.

Force encoding from Arduino (analog sensor version):
    force = (analogRead(A0) / 4095.0) × 31.0    →  range [0, 31]
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from collections import deque
from data_stream import DataStream

# ── Configuration ─────────────────────────────────────────────────────
SERIAL_PORT      = 'COM5'
BAUD_RATE        = 115200
DATASET_FILENAME = ''           # '' = live;  'data.csv' = playback

MAX_DISPLAY   = 300             # rolling window width (samples)
FORCE_MAX     = 31.0            # maximum force value from Arduino encoding


# ─────────────────────────────────────────────────────────────────────
#  CONTACT DETECTION PIPELINE
# ─────────────────────────────────────────────────────────────────────
class ForceContactDetector:
    """
    Spring-mechanism rear-sensor pipeline:

        raw_force
            ↓  EMA smoothing          — reduce sensor analog noise
        smooth
            ↓  Noise-floor gate       — treat anything below NOISE_FLOOR as 0
        gated
            ↓  Hysteresis comparison  — enter on HIGH, exit on LOW
            ↓  Debounce               — N consecutive samples required
        contact_state  ∈ {0, 1}

    Thresholds are absolute (not relative to a baseline) because
    the sensor reads near 0 by design when the marker is lifted.
    """

    # ── Tuning — adjust once after first live test ─────────────────────
    EMA_ALPHA    = 0.30     # responsiveness vs. smoothness (0.1 = slow, 0.5 = fast)

    # Sensor noise floor: readings below this are treated as true zero.
    # Set this just above the maximum idle noise you observe in your sensor.
    NOISE_FLOOR  = 0.8      # force units (out of 31)

    # Hysteresis thresholds (absolute force values)
    # ENTER: spring must compress past this to register writing
    # EXIT:  spring must relax below this to register lifting
    THRESH_ENTER = 4.0      # ~13% of full scale — adjust to your spring stiffness
    THRESH_EXIT  = 2.0      # lower than ENTER — prevents chattering on release

    # Temporal debounce (samples at 100 Hz)
    # ON  — short: spring contact is fast and decisive
    # OFF — longer: spring bounce on lift takes a few ms to settle
    DEBOUNCE_ON  = 3
    DEBOUNCE_OFF = 8

    def __init__(self):
        self._ema          = None
        self._state        = 0
        self._consec_above = 0
        self._consec_below = 0

    # ── public API ────────────────────────────────────────────────────

    def process(self, raw_force: float):
        """
        Returns
        -------
        contact_state : int   0 (lifting) or 1 (writing)
        smooth        : float EMA-smoothed force value
        thresh_enter  : float entry threshold (constant)
        thresh_exit   : float exit  threshold (constant)
        """
        # Stage 1: EMA
        if self._ema is None:
            self._ema = raw_force
        else:
            self._ema = self.EMA_ALPHA * raw_force + (1.0 - self.EMA_ALPHA) * self._ema
        smooth = self._ema

        # Stage 2: Noise-floor gate — clamp genuine electrical noise to 0
        gated = smooth if smooth >= self.NOISE_FLOOR else 0.0

        # Stage 3: Hysteresis + debounce state machine
        if self._state == 0:                        # currently LIFTING
            if gated >= self.THRESH_ENTER:
                self._consec_above += 1
                self._consec_below  = 0
                if self._consec_above >= self.DEBOUNCE_ON:
                    self._state = 1                 # → WRITING
            else:
                self._consec_above = 0
        else:                                       # currently WRITING
            if gated < self.THRESH_EXIT:
                self._consec_below += 1
                self._consec_above  = 0
                if self._consec_below >= self.DEBOUNCE_OFF:
                    self._state = 0                 # → LIFTING
            else:
                self._consec_below = 0

        return self._state, smooth, self.THRESH_ENTER, self.THRESH_EXIT

    @property
    def is_writing(self) -> bool:
        return self._state == 1


# ─────────────────────────────────────────────────────────────────────
#  VISUALISATION DASHBOARD
# ─────────────────────────────────────────────────────────────────────
class ForceDetectorDashboard:
    def __init__(self, stream: DataStream):
        self.stream            = stream
        self.detector          = ForceContactDetector()
        self.playback_finished = False

        # Rolling display buffers
        self.buf_raw    = deque(maxlen=MAX_DISPLAY)
        self.buf_smooth = deque(maxlen=MAX_DISPLAY)
        self.buf_state  = deque(maxlen=MAX_DISPLAY)

        # Session statistics
        self.total_samples   = 0
        self.contact_samples = 0
        self.stroke_count    = 0
        self._prev_state     = 0

        self._build_figure()

    # ── figure layout ─────────────────────────────────────────────────

    def _build_figure(self):
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(14, 9))
        self.fig.canvas.manager.set_window_title(
            'PolyCast — Spring-Mechanism Force Contact Detector')

        self.ax_force = self.fig.add_subplot(2, 1, 1)
        self.ax_state = self.fig.add_subplot(2, 1, 2)

        # ── Panel 1: force signal ─────────────────────────────────────
        ax = self.ax_force
        self.line_raw,    = ax.plot([], [], color='#555555', lw=1.2,
                                    label='Raw force', alpha=0.75)
        self.line_smooth, = ax.plot([], [], color='white', lw=2.0,
                                    label='Smoothed (EMA)')

        # Fixed threshold reference lines — drawn once, always visible
        ax.axhline(ForceContactDetector.THRESH_ENTER, color='#FF6B6B',
                   linestyle='-.', lw=1.8,
                   label=f'Enter threshold  ({ForceContactDetector.THRESH_ENTER})')
        ax.axhline(ForceContactDetector.THRESH_EXIT,  color='#FFD93D',
                   linestyle='-.', lw=1.8,
                   label=f'Exit  threshold  ({ForceContactDetector.THRESH_EXIT})')
        ax.axhline(ForceContactDetector.NOISE_FLOOR,  color='#888888',
                   linestyle=':',  lw=1.2,
                   label=f'Noise floor  ({ForceContactDetector.NOISE_FLOOR})')

        ax.set_xlim(0, MAX_DISPLAY)
        ax.set_ylim(0, FORCE_MAX + 2)
        ax.set_title('Force Sensor Signal  (Spring-Mechanism, Rear-Mounted)',
                     fontsize=13, weight='bold')
        ax.set_ylabel('Force  (0 – 31 a.u.)')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.15)

        # ── Panel 2: binary state ─────────────────────────────────────
        ax = self.ax_state
        self.line_state, = ax.step([], [], where='post',
                                   color='#00FF99', lw=2.5,
                                   label='Contact state')

        ax.set_xlim(0, MAX_DISPLAY)
        ax.set_ylim(-0.25, 1.35)
        ax.set_yticks([0, 1])
        ax.set_yticklabels(['0  —  Lifting', '1  —  Writing'], fontsize=11)
        ax.set_title('Binary Contact State Output',
                     fontsize=13, weight='bold')
        ax.set_xlabel('Sample index  (100 Hz)')
        ax.grid(True, alpha=0.15)

        writing_p = mpatches.Patch(color='#00FF99', alpha=0.3, label='Writing')
        lifting_p = mpatches.Patch(color='#555555', alpha=0.3, label='Lifting')
        ax.legend(handles=[writing_p, lifting_p], loc='upper right', fontsize=9)

        # Filled areas — rebuilt each frame
        self._contact_fill_force = None
        self._state_fill         = None

        # Stats banner
        self.stats_text = self.fig.text(
            0.5, 0.97, 'Waiting for data…',
            ha='center', va='top', fontsize=10, color='white', weight='bold',
            bbox=dict(facecolor='#1a1a2e', alpha=0.85,
                      edgecolor='white', boxstyle='round,pad=0.5'),
        )

        plt.tight_layout(rect=[0, 0, 1, 0.94])

    # ── animation callback ────────────────────────────────────────────

    def update_plot(self, frame):
        if self.playback_finished:
            return self.line_raw, self.line_smooth, self.line_state

        packets_this_frame = 0
        if self.stream.mode == 'live':
            while self.stream.data_available():
                pkt = self.stream.get_packet()
                if pkt:
                    self._ingest_packet(pkt)
                    packets_this_frame += 1
        else:
            for _ in range(8):
                pkt = self.stream.get_packet()
                if pkt == 'EOF':
                    self.playback_finished = True
                    self.stream.close()
                    self._print_summary()
                    print('\n✅ End of dataset.')
                    break
                if pkt:
                    self._ingest_packet(pkt)
                    packets_this_frame += 1

        if packets_this_frame:
            self._redraw()
        return self.line_raw, self.line_smooth, self.line_state

    # ── data ingestion ────────────────────────────────────────────────

    def _ingest_packet(self, packet):
        for sample in packet['imu']:
            state, smooth, _, _ = self.detector.process(sample['force'])

            self.total_samples += 1
            if state:
                self.contact_samples += 1
            if state == 1 and self._prev_state == 0:
                self.stroke_count += 1
            self._prev_state = state

            self.buf_raw.append(sample['force'])
            self.buf_smooth.append(smooth)
            self.buf_state.append(state)

    # ── redraw ────────────────────────────────────────────────────────

    def _redraw(self):
        n          = len(self.buf_raw)
        x          = np.arange(n)
        raw_arr    = np.array(self.buf_raw)
        smooth_arr = np.array(self.buf_smooth)
        state_arr  = np.array(self.buf_state)

        # Panel 1
        self.line_raw.set_data(x, raw_arr)
        self.line_smooth.set_data(x, smooth_arr)

        if self._contact_fill_force:
            self._contact_fill_force.remove()
        self._contact_fill_force = self.ax_force.fill_between(
            x, 0, raw_arr,
            where=(state_arr == 1),
            color='#FF6B6B', alpha=0.18, step='post',
        )

        # Panel 2
        self.line_state.set_data(x, state_arr)

        if self._state_fill:
            for c in self._state_fill:
                c.remove()
        self._state_fill = [
            self.ax_state.fill_between(x, 0, state_arr,
                                       where=(state_arr == 1),
                                       color='#00FF99', alpha=0.28, step='post'),
            self.ax_state.fill_between(x, 0, state_arr,
                                       where=(state_arr == 0),
                                       color='#333333', alpha=0.15, step='post'),
        ]

        duty   = self.contact_samples / self.total_samples * 100 if self.total_samples else 0.0
        status = '✍  WRITING' if self.detector.is_writing else '⬆  LIFTING'
        self.stats_text.set_text(
            f'Status: {status}   |   Strokes: {self.stroke_count}   |   '
            f'Contact duty: {duty:.1f}%   |   Samples: {self.total_samples}'
        )

    # ── summary ───────────────────────────────────────────────────────

    def _print_summary(self):
        print('\n' + '='*55)
        print('  FORCE CONTACT DETECTOR — SESSION SUMMARY')
        print('='*55)
        duty = self.contact_samples / self.total_samples * 100 if self.total_samples else 0.0
        print(f'  Total samples    : {self.total_samples}')
        print(f'  Contact samples  : {self.contact_samples}')
        print(f'  Contact duty     : {duty:.1f}%')
        print(f'  Strokes detected : {self.stroke_count}')
        print(f'  Noise floor      : {ForceContactDetector.NOISE_FLOOR}')
        print(f'  Entry threshold  : {ForceContactDetector.THRESH_ENTER}')
        print(f'  Exit  threshold  : {ForceContactDetector.THRESH_EXIT}')
        print('='*55 + '\n')


# ─────────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────────────────
def main():
    stream = DataStream(SERIAL_PORT, BAUD_RATE, DATASET_FILENAME)
    if not stream.connect():
        sys.exit(1)

    dashboard = ForceDetectorDashboard(stream)
    mode_str  = 'CSV Playback' if stream.mode == 'csv' else 'Live Serial'
    print(f'🚀 Force Detector (Spring-Mechanism) running in {mode_str} mode')

    ani = FuncAnimation(dashboard.fig, dashboard.update_plot,
                        interval=40, blit=False, cache_frame_data=False)
    plt.show()
    stream.close()
    dashboard._print_summary()


if __name__ == '__main__':
    main()