"""
button_detector.py  —  PolyCast Tactile Switch Contact Detector (Async Stream)
===============================================================================
Hardware design:
    - One leg of a tactile push-button -> 3.3 V on the Arduino Nano ESP32
    - Other leg                        -> A0 pin (INPUT_PULLDOWN in firmware)
    - Marker tip touches board         -> button depresses -> 3.3 V on A0 -> HIGH
    - Marker tip lifts                 -> button releases  -> A0 pulled LOW

Firmware encoding (from imu_module.cpp):
    digitalRead(A0) == HIGH  ->  out_packet->force = 31.0f   (writing)
    digitalRead(A0) == LOW   ->  out_packet->force =  3.0f   (lifting)

What this detector does:
    1. Classifies each force sample as HIGH (>= MID_THRESHOLD) or LOW
    2. Applies temporal debounce — suppresses contact-bounce glitches
    3. Outputs a clean 0/1 binary state

Async adaptation:
    The dashboard now consumes individual IMU packets from AsyncDataParser
    instead of batched packets from DataStream.
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from collections import deque
from data_parser import AsyncDataParser

# -- Configuration ---------------------------------------------------------
SERIAL_PORT      = 'COM5'
BAUD_RATE        = 115200
DATASET_FILENAME = ''           # '' = live;  'data.csv' = playback

MAX_DISPLAY = 300               # rolling window width (samples)

# Firmware-defined signal levels
LEVEL_LOW  =  3.0               # force value sent when button is released
LEVEL_HIGH = 31.0               # force value sent when button is pressed

# -------------------------------------------------------------------------
#  CONTACT DETECTION PIPELINE
# -------------------------------------------------------------------------
class ButtonContactDetector:
    """
    Pipeline for the tactile button:

        raw_force  (31.0 or 3.0 from firmware)
            |  Level classification    — HIGH if >= MID_THRESHOLD, else LOW
        raw_state  in {0, 1}
            |  Debounce state machine  — N consecutive samples to confirm
        contact_state  in {0, 1}

    Why debounce matters:
        Mechanical button contacts bounce when first made or broken.
        During this bounce window (typically 1-5 ms = 1-5 samples at 100 Hz)
        the digitalRead can oscillate rapidly HIGH/LOW.  Without debounce
        you get false micro-strokes at the start and end of every real stroke.

    Why DEBOUNCE_OFF > DEBOUNCE_ON:
        Declaring WRITING requires only a short confirmation — the button
        bounces briefly then stays pressed.  Declaring LIFTING requires
        a longer confirmation — we don't want a momentary micro-lift during
        a continuous stroke to break the stroke prematurely.
    """

    # Midpoint between firmware levels — anything above is treated as HIGH
    MID_THRESHOLD = (LEVEL_LOW + LEVEL_HIGH) / 2.0     # 17.0

    # Debounce windows (samples at 100 Hz)
    DEBOUNCE_ON  = 2    # confirm button press  (2 samples = 20 ms)
    DEBOUNCE_OFF = 5    # confirm button release (5 samples = 50 ms)

    def __init__(self):
        self._state        = 0
        self._consec_high  = 0
        self._consec_low   = 0

    # -- public API --------------------------------------------------------

    def process(self, raw_force: float):
        """
        Parameters
        ----------
        raw_force : float   firmware-encoded force value (3.0 or 31.0)

        Returns
        -------
        contact_state : int   0 (lifting) or 1 (writing)
        raw_state     : int   0 or 1 — level classification before debounce
        """
        raw_state = 1 if raw_force >= self.MID_THRESHOLD else 0

        if self._state == 0:                    # currently LIFTING
            if raw_state == 1:
                self._consec_high += 1
                self._consec_low   = 0
                if self._consec_high >= self.DEBOUNCE_ON:
                    self._state = 1             # -> WRITING
            else:
                self._consec_high = 0
        else:                                   # currently WRITING
            if raw_state == 0:
                self._consec_low += 1
                self._consec_high  = 0
                if self._consec_low >= self.DEBOUNCE_OFF:
                    self._state = 0             # -> LIFTING
            else:
                self._consec_low = 0

        return self._state, raw_state

    @property
    def is_writing(self) -> bool:
        return self._state == 1


# -------------------------------------------------------------------------
#  VISUALISATION DASHBOARD (Async-adapted)
# -------------------------------------------------------------------------
class ButtonDetectorDashboard:
    def __init__(self, parser: AsyncDataParser):
        self.parser            = parser
        self.detector          = ButtonContactDetector()
        self.playback_finished = False

        # Rolling display buffers
        self.buf_raw       = deque(maxlen=MAX_DISPLAY)   # 3.0 / 31.0
        self.buf_raw_state = deque(maxlen=MAX_DISPLAY)   # 0 / 1 (pre-debounce)
        self.buf_state     = deque(maxlen=MAX_DISPLAY)   # 0 / 1 (post-debounce)

        # Session statistics
        self.total_samples   = 0
        self.contact_samples = 0
        self.stroke_count    = 0
        self.bounce_events   = 0        # raw transitions that get absorbed by debounce
        self._prev_state     = 0
        self._prev_raw_state = 0

        self._build_figure()

    # -- figure layout -----------------------------------------------------

    def _build_figure(self):
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(14, 10))
        self.fig.canvas.manager.set_window_title(
            'PolyCast — Tactile Button Contact Detector (Async)')

        # 3 panels: raw signal / pre-debounce state / debounced state
        self.ax_raw    = self.fig.add_subplot(3, 1, 1)
        self.ax_pre    = self.fig.add_subplot(3, 1, 2)
        self.ax_clean  = self.fig.add_subplot(3, 1, 3)

        self._build_raw_panel()
        self._build_pre_panel()
        self._build_clean_panel()

        # Filled areas — rebuilt each frame
        self._fill_raw   = None
        self._fill_pre   = None
        self._fill_clean = None

        self.stats_text = self.fig.text(
            0.5, 0.975, 'Waiting for data...',
            ha='center', va='top', fontsize=10, color='white', weight='bold',
            bbox=dict(facecolor='#1a1a2e', alpha=0.85,
                      edgecolor='white', boxstyle='round,pad=0.5'),
        )
        plt.tight_layout(rect=[0, 0, 1, 0.95])

    def _build_raw_panel(self):
        ax = self.ax_raw
        self.line_raw, = ax.plot([], [], color='#AAAAAA', lw=1.5,
                                 drawstyle='steps-post', label='Raw firmware value')
        ax.axhline(ButtonContactDetector.MID_THRESHOLD,
                   color='#FFD93D', linestyle='--', lw=1.5,
                   label=f'Mid threshold  ({ButtonContactDetector.MID_THRESHOLD:.0f})')
        ax.axhline(LEVEL_HIGH, color='#FF6B6B', linestyle=':',
                   lw=1.0, alpha=0.6, label=f'HIGH level  ({LEVEL_HIGH:.0f})')
        ax.axhline(LEVEL_LOW,  color='#6B8CFF', linestyle=':',
                   lw=1.0, alpha=0.6, label=f'LOW  level  ({LEVEL_LOW:.0f})')

        ax.set_xlim(0, MAX_DISPLAY)
        ax.set_ylim(-2, LEVEL_HIGH + 4)
        ax.set_title('Raw Firmware Signal  (3.0 = Lifting  /  31.0 = Writing)',
                     fontsize=12, weight='bold')
        ax.set_ylabel('Force value  (a.u.)')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.15)

    def _build_pre_panel(self):
        ax = self.ax_pre
        self.line_pre, = ax.step([], [], where='post',
                                 color='#FF9944', lw=2.0,
                                 label='Level-classified  (pre-debounce)')
        ax.set_xlim(0, MAX_DISPLAY)
        ax.set_ylim(-0.25, 1.35)
        ax.set_yticks([0, 1])
        ax.set_yticklabels(['0  --  LOW', '1  --  HIGH'], fontsize=10)
        ax.set_title('Pre-Debounce State  (may contain contact-bounce glitches)',
                     fontsize=12, weight='bold')
        ax.set_ylabel('Level')
        ax.grid(True, alpha=0.15)

        pre_p = mpatches.Patch(color='#FF9944', alpha=0.3, label='HIGH detected')
        ax.legend(handles=[pre_p], loc='upper right', fontsize=9)

    def _build_clean_panel(self):
        ax = self.ax_clean
        self.line_clean, = ax.step([], [], where='post',
                                   color='#00FF99', lw=2.5,
                                   label='Debounced contact state')
        ax.set_xlim(0, MAX_DISPLAY)
        ax.set_ylim(-0.25, 1.35)
        ax.set_yticks([0, 1])
        ax.set_yticklabels(['0  --  Lifting', '1  --  Writing'], fontsize=10)
        ax.set_title('Binary Contact State Output  (post-debounce)',
                     fontsize=12, weight='bold')
        ax.set_xlabel('Sample index  (100 Hz)')
        ax.grid(True, alpha=0.15)

        writing_p = mpatches.Patch(color='#00FF99', alpha=0.3, label='Writing')
        lifting_p = mpatches.Patch(color='#555555', alpha=0.3, label='Lifting')
        ax.legend(handles=[writing_p, lifting_p], loc='upper right', fontsize=9)

    # -- animation callback ------------------------------------------------

    def update_plot(self, frame):
        if self.playback_finished:
            return self.line_raw, self.line_pre, self.line_clean

        packets_this_frame = 0
        if self.parser.mode == 'live':
            while self.parser.data_available():
                pkt = self.parser.get_packet()
                if pkt and pkt != 'EOF' and isinstance(pkt, dict):
                    if pkt['type'] == 'imu':
                        self._ingest_imu(pkt)
                        packets_this_frame += 1
        else:
            for _ in range(10):
                pkt = self.parser.get_packet()
                if pkt == 'EOF':
                    self.playback_finished = True
                    self.parser.close()
                    self._print_summary()
                    print('\nEnd of dataset.')
                    break
                if pkt and isinstance(pkt, dict) and pkt['type'] == 'imu':
                    self._ingest_imu(pkt)
                    packets_this_frame += 1

        if packets_this_frame:
            self._redraw()
        return self.line_raw, self.line_pre, self.line_clean

    # -- data ingestion (async: single IMU packet) -------------------------

    def _ingest_imu(self, pkt):
        """Process a single IMU packet's force value."""
        state, raw_state = self.detector.process(pkt['force'])

        # Count contact-bounce events: raw flipped but debounced state didn't
        if raw_state != self._prev_raw_state and state == self._prev_state:
            self.bounce_events += 1

        self.total_samples += 1
        if state:
            self.contact_samples += 1
        if state == 1 and self._prev_state == 0:
            self.stroke_count += 1

        self._prev_state     = state
        self._prev_raw_state = raw_state

        self.buf_raw.append(pkt['force'])
        self.buf_raw_state.append(raw_state)
        self.buf_state.append(state)

    # -- redraw ------------------------------------------------------------

    def _redraw(self):
        n           = len(self.buf_raw)
        x           = np.arange(n)
        raw_arr     = np.array(self.buf_raw)
        raw_s_arr   = np.array(self.buf_raw_state)
        state_arr   = np.array(self.buf_state)

        # Panel 1 — raw signal
        self.line_raw.set_data(x, raw_arr)
        if self._fill_raw:
            self._fill_raw.remove()
        self._fill_raw = self.ax_raw.fill_between(
            x, LEVEL_LOW, raw_arr,
            where=(raw_s_arr == 1),
            color='#FF6B6B', alpha=0.18, step='post',
        )

        # Panel 2 — pre-debounce
        self.line_pre.set_data(x, raw_s_arr)
        if self._fill_pre:
            for c in self._fill_pre:
                c.remove()
        self._fill_pre = [
            self.ax_pre.fill_between(x, 0, raw_s_arr,
                                     where=(raw_s_arr == 1),
                                     color='#FF9944', alpha=0.25, step='post'),
        ]

        # Panel 3 — debounced output
        self.line_clean.set_data(x, state_arr)
        if self._fill_clean:
            for c in self._fill_clean:
                c.remove()
        self._fill_clean = [
            self.ax_clean.fill_between(x, 0, state_arr,
                                       where=(state_arr == 1),
                                       color='#00FF99', alpha=0.28, step='post'),
            self.ax_clean.fill_between(x, 0, state_arr,
                                       where=(state_arr == 0),
                                       color='#333333', alpha=0.15, step='post'),
        ]

        # Stats banner
        duty   = self.contact_samples / self.total_samples * 100 if self.total_samples else 0.0
        status = 'WRITING' if self.detector.is_writing else 'LIFTING'
        self.stats_text.set_text(
            f'Status: {status}   |   Strokes: {self.stroke_count}   |   '
            f'Contact duty: {duty:.1f}%   |   '
            f'Bounce events absorbed: {self.bounce_events}   |   '
            f'Samples: {self.total_samples}'
        )

    # -- summary -----------------------------------------------------------

    def _print_summary(self):
        print('\n' + '='*55)
        print('  BUTTON CONTACT DETECTOR — SESSION SUMMARY')
        print('='*55)
        duty = self.contact_samples / self.total_samples * 100 if self.total_samples else 0.0
        print(f'  Total samples          : {self.total_samples}')
        print(f'  Contact samples        : {self.contact_samples}')
        print(f'  Contact duty cycle     : {duty:.1f}%')
        print(f'  Writing strokes        : {self.stroke_count}')
        print(f'  Bounce events absorbed : {self.bounce_events}')
        print(f'  Debounce ON  window    : {ButtonContactDetector.DEBOUNCE_ON} samples')
        print(f'  Debounce OFF window    : {ButtonContactDetector.DEBOUNCE_OFF} samples')
        print('='*55 + '\n')


# -------------------------------------------------------------------------
#  ENTRY POINT
# -------------------------------------------------------------------------
def main():
    parser = AsyncDataParser(port=SERIAL_PORT, baud=BAUD_RATE, csv_path=DATASET_FILENAME)
    if not parser.connect():
        sys.exit(1)

    dashboard = ButtonDetectorDashboard(parser)
    mode_str  = 'CSV Playback' if parser.mode == 'csv' else 'Live Serial'
    print(f'Button Detector running in {mode_str} mode (Async Stream)')

    ani = FuncAnimation(dashboard.fig, dashboard.update_plot,
                        interval=40, blit=False, cache_frame_data=False)
    plt.show()
    parser.close()
    dashboard._print_summary()


if __name__ == '__main__':
    main()
