"""
force_detector.py  —  PolyCast FSR Contact Detector (Async Stream)
==================================================================
Hardware design:
    - FSR (Force Sensitive Resistor) mounted at marker tip
    - Voltage divider: 3.3V → FSR → A0 → 10kΩ → GND
    - Marker tip touches board → FSR compresses → resistance drops
      → A0 voltage rises → ADC value increases
    - Marker tip lifts → FSR relaxes → resistance rises
      → A0 voltage drops → ADC value decreases

Firmware encoding (from imu_module.cpp):
    Raw 12-bit ADC:  analogRead(A0)  →  0–4095  (sent as float)
    No force  →  ~0      (FSR resistance >> 10kΩ, voltage ≈ 0V)
    Light touch → ~200–700  (FSR ~30–100kΩ)
    Hard press  → ~2000–4000  (FSR ~1–10kΩ)

Detection pipeline:
    raw_force (0–4095 ADC)
      → Stage 1: EMA smoothing        — reduce analog noise
      → Stage 2: Noise-floor gate     — clamp idle noise to 0
      → Stage 3: Hysteresis (Schmitt) — enter on HIGH, exit on LOW
      → Stage 4: Temporal debounce    — N consecutive samples to confirm
      → contact_state ∈ {0, 1}

Async adaptation:
    Dashboard consumes individual IMU packets from AsyncDataParser.
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from collections import deque
from data_parser import AsyncDataParser

# -- Configuration ---------------------------------------------------------
from config import SERIAL_PORT, BAUD_RATE, FSR_DT_NOM_S
DATASET_FILENAME = ''           # '' = live;  'datasets/data.csv' = playback

MAX_DISPLAY = 300               # rolling window width (samples)
ADC_MAX     = 4095.0            # 12-bit ADC full scale


# -------------------------------------------------------------------------
#  CONTACT DETECTION PIPELINE
# -------------------------------------------------------------------------
class ForceContactDetector:
    """
    4-stage FSR analog pipeline:

        raw_force  (0–4095 ADC from firmware)
            |  EMA smoothing          — reduce ADC analog noise
        smooth
            |  Noise-floor gate       — treat anything below NOISE_FLOOR as 0
        gated
            |  Hysteresis comparison  — Schmitt trigger (ENTER / EXIT thresholds)
        raw_classify  ∈ {0, 1}       — pre-debounce classification
            |  Debounce state machine — N consecutive samples to confirm
        contact_state  ∈ {0, 1}

    All thresholds are in raw ADC units (0–4095).
    """

    # -- Tuning — adjust after first live test with your FSR ---------------

    # EMA smoothing coefficient (0.1 = very smooth/slow, 0.5 = fast/noisy)
    EMA_ALPHA    = 0.25

    # Sensor noise floor: ADC readings below this are treated as true zero.
    # Set just above the maximum idle noise observed when FSR is unloaded.
    NOISE_FLOOR  = 50.0

    # Hysteresis thresholds (raw ADC units)
    # ENTER: FSR must exceed this to register contact (writing)
    # EXIT:  FSR must drop below this to register release (lifting)
    # Gap between them prevents chattering near the transition point.
    #
    # Phase 8 Step 2 attempted to override these from calibration_profile.json
    # (Otsu on bimodal raw-FSR histogram). The Otsu split landed at ~970
    # ADC, which produced a *narrower* hysteresis band (970→152) than the
    # default and risked chatter on slow lifts. Layer-5 was already passing
    # cleanly with the legacy 250/120 thresholds, so the calibrated FSR
    # values are kept on disk for documentation but not applied. Use
    # calibration_get('fsr_thresh_enter') / get('fsr_thresh_exit') if you
    # want to experiment with them.
    THRESH_ENTER = 250.0    # ~100g light touch with 10kΩ divider
    THRESH_EXIT  = 120.0    # well below ENTER — hysteresis gap

    # Temporal debounce — time-based (Priority 1).
    # Stored as seconds, NOT sample counts. The detector accumulates elapsed
    # `dt` from each call's timestamp (or falls back to the rate_profile
    # nominal IMU/FSR dt) and trips when the cumulative confirming time
    # crosses the threshold. This is invariant to whether the underlying
    # stream is 50 / 100 / 200 Hz — what matters is wall-clock.
    DEBOUNCE_ON_S  = 0.030   # 30 ms confirmed contact rise
    DEBOUNCE_OFF_S = 0.080   # 80 ms confirmed contact fall

    def __init__(self):
        self._ema          = None
        self._state        = 0
        self._t_above      = 0.0    # accumulated seconds raw_classify == 1
        self._t_below      = 0.0    # accumulated seconds raw_classify == 0
        self._raw_classify = 0      # exposed for dashboard (pre-debounce)
        self._last_ts_us   = None   # for dt computation when ts is provided

    # -- public API --------------------------------------------------------

    def process(self, raw_force: float, ts: int | None = None,
                dt: float | None = None):
        """
        Parameters
        ----------
        raw_force : float   raw ADC value from FSR (0–4095)
        ts        : int     optional sender micros() timestamp; if supplied,
                            dt is derived from successive calls. Preferred.
        dt        : float   explicit time step in seconds; overrides ts.
                            Falls back to rate_profile nominal FSR dt if both
                            are None.

        Returns
        -------
        contact_state : int     0 (lifting) or 1 (writing)
        smooth_force  : float   EMA-smoothed ADC value
        """
        # Resolve dt for the debounce timer.
        if dt is None:
            if ts is not None and self._last_ts_us is not None:
                dt_us = ts - self._last_ts_us
                dt = dt_us / 1_000_000.0 if 0 < dt_us < 1_000_000 else FSR_DT_NOM_S
            else:
                dt = FSR_DT_NOM_S
        if ts is not None:
            self._last_ts_us = ts

        # Stage 1: EMA smoothing
        if self._ema is None:
            self._ema = raw_force
        else:
            self._ema = self.EMA_ALPHA * raw_force + (1.0 - self.EMA_ALPHA) * self._ema
        smooth = self._ema

        # Stage 2: Noise-floor gate — clamp idle electrical noise to 0
        gated = smooth if smooth >= self.NOISE_FLOOR else 0.0

        # Stage 3: Hysteresis classification (Schmitt trigger)
        if self._state == 0:
            self._raw_classify = 1 if gated >= self.THRESH_ENTER else 0
        else:
            self._raw_classify = 0 if gated < self.THRESH_EXIT else 1

        # Stage 4: Time-based debounce
        if self._state == 0:                        # currently LIFTING
            if self._raw_classify == 1:
                self._t_above += dt
                self._t_below  = 0.0
                if self._t_above >= self.DEBOUNCE_ON_S:
                    self._state = 1                 # → WRITING
                    self._t_above = 0.0
            else:
                self._t_above = 0.0
        else:                                       # currently WRITING
            if self._raw_classify == 0:
                self._t_below += dt
                self._t_above  = 0.0
                if self._t_below >= self.DEBOUNCE_OFF_S:
                    self._state = 0                 # → LIFTING
                    self._t_below = 0.0
            else:
                self._t_below = 0.0

        return self._state, smooth

    @property
    def is_writing(self) -> bool:
        return self._state == 1


# -------------------------------------------------------------------------
#  VISUALISATION DASHBOARD (Async-adapted)
# -------------------------------------------------------------------------
class ForceDetectorDashboard:
    def __init__(self, parser: AsyncDataParser):
        self.parser            = parser
        self.detector          = ForceContactDetector()
        self.playback_finished = False

        # Rolling display buffers
        self.buf_raw       = deque(maxlen=MAX_DISPLAY)   # raw ADC 0–4095
        self.buf_smooth    = deque(maxlen=MAX_DISPLAY)   # EMA-smoothed
        self.buf_pre_state = deque(maxlen=MAX_DISPLAY)   # 0/1 pre-debounce
        self.buf_state     = deque(maxlen=MAX_DISPLAY)   # 0/1 post-debounce

        # Session statistics
        self.total_samples   = 0
        self.contact_samples = 0
        self.stroke_count    = 0
        self.bounce_events   = 0
        self._prev_state     = 0
        self._prev_pre_state = 0

        self._build_figure()

    # -- figure layout -----------------------------------------------------

    def _build_figure(self):
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(14, 10))
        self.fig.canvas.manager.set_window_title(
            'PolyCast — FSR Force Contact Detector (Async)')

        # 3 panels: force signal / pre-debounce state / debounced state
        self.ax_force  = self.fig.add_subplot(3, 1, 1)
        self.ax_pre    = self.fig.add_subplot(3, 1, 2)
        self.ax_clean  = self.fig.add_subplot(3, 1, 3)

        self._build_force_panel()
        self._build_pre_panel()
        self._build_clean_panel()

        # Filled areas — rebuilt each frame
        self._fill_force = None
        self._fill_pre   = None
        self._fill_clean = None

        self.stats_text = self.fig.text(
            0.5, 0.975, 'Waiting for data...',
            ha='center', va='top', fontsize=10, color='white', weight='bold',
            bbox=dict(facecolor='#1a1a2e', alpha=0.85,
                      edgecolor='white', boxstyle='round,pad=0.5'),
        )
        plt.tight_layout(rect=[0, 0, 1, 0.95])

    def _build_force_panel(self):
        ax = self.ax_force
        self.line_raw,    = ax.plot([], [], color='#555555', lw=1.2,
                                    label='Raw ADC')
        self.line_smooth, = ax.plot([], [], color='white', lw=2.0,
                                    label='Smoothed (EMA)')

        # Fixed threshold reference lines
        ax.axhline(ForceContactDetector.THRESH_ENTER, color='#FF6B6B',
                   linestyle='-.', lw=1.8,
                   label=f'Enter threshold  ({ForceContactDetector.THRESH_ENTER:.0f})')
        ax.axhline(ForceContactDetector.THRESH_EXIT,  color='#FFD93D',
                   linestyle='-.', lw=1.8,
                   label=f'Exit  threshold  ({ForceContactDetector.THRESH_EXIT:.0f})')
        ax.axhline(ForceContactDetector.NOISE_FLOOR,  color='#888888',
                   linestyle=':',  lw=1.2,
                   label=f'Noise floor  ({ForceContactDetector.NOISE_FLOOR:.0f})')

        ax.set_xlim(0, MAX_DISPLAY)
        ax.set_ylim(0, ADC_MAX + 200)
        ax.set_title('FSR Force Signal  (Raw ADC 0–4095)',
                     fontsize=12, weight='bold')
        ax.set_ylabel('ADC value')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.15)

    def _build_pre_panel(self):
        ax = self.ax_pre
        self.line_pre, = ax.step([], [], where='post',
                                 color='#FF9944', lw=2.0,
                                 label='Hysteresis output (pre-debounce)')
        ax.set_xlim(0, MAX_DISPLAY)
        ax.set_ylim(-0.25, 1.35)
        ax.set_yticks([0, 1])
        ax.set_yticklabels(['0  --  Below exit', '1  --  Above enter'],
                           fontsize=10)
        ax.set_title('Pre-Debounce State  (Schmitt trigger hysteresis)',
                     fontsize=12, weight='bold')
        ax.set_ylabel('Classification')
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
        ax.set_xlabel('Sample index')
        ax.grid(True, alpha=0.15)

        writing_p = mpatches.Patch(color='#00FF99', alpha=0.3, label='Writing')
        lifting_p = mpatches.Patch(color='#555555', alpha=0.3, label='Lifting')
        ax.legend(handles=[writing_p, lifting_p], loc='upper right', fontsize=9)

    # -- animation callback ------------------------------------------------

    def update_plot(self, frame):
        if self.playback_finished:
            return self.line_raw, self.line_smooth, self.line_pre, self.line_clean

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
        return self.line_raw, self.line_smooth, self.line_pre, self.line_clean

    # -- data ingestion (async: single IMU packet) -------------------------

    def _ingest_imu(self, pkt):
        """Process a single IMU packet's force value."""
        state, smooth = self.detector.process(pkt['force'], ts=pkt.get('ts'))
        pre_state = self.detector._raw_classify

        # Count bounce events: pre-debounce flipped but final state didn't
        if pre_state != self._prev_pre_state and state == self._prev_state:
            self.bounce_events += 1

        self.total_samples += 1
        if state:
            self.contact_samples += 1
        if state == 1 and self._prev_state == 0:
            self.stroke_count += 1

        self._prev_state     = state
        self._prev_pre_state = pre_state

        self.buf_raw.append(pkt['force'])
        self.buf_smooth.append(smooth)
        self.buf_pre_state.append(pre_state)
        self.buf_state.append(state)

    # -- redraw ------------------------------------------------------------

    def _redraw(self):
        n          = len(self.buf_raw)
        x          = np.arange(n)
        raw_arr    = np.array(self.buf_raw)
        smooth_arr = np.array(self.buf_smooth)
        pre_arr    = np.array(self.buf_pre_state)
        state_arr  = np.array(self.buf_state)

        # Panel 1 — force signal
        self.line_raw.set_data(x, raw_arr)
        self.line_smooth.set_data(x, smooth_arr)

        # Dynamic Y-axis: zoom to data range for better visibility
        if n > 0:
            peak = max(float(np.max(raw_arr)), float(np.max(smooth_arr)), 500.0)
            self.ax_force.set_ylim(0, peak * 1.15)

        if self._fill_force:
            self._fill_force.remove()
        self._fill_force = self.ax_force.fill_between(
            x, 0, raw_arr,
            where=(state_arr == 1),
            color='#FF6B6B', alpha=0.18, step='post',
        )

        # Panel 2 — pre-debounce
        self.line_pre.set_data(x, pre_arr)
        if self._fill_pre:
            for c in self._fill_pre:
                c.remove()
        self._fill_pre = [
            self.ax_pre.fill_between(x, 0, pre_arr,
                                     where=(pre_arr == 1),
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
        latest_smooth = f'{self.buf_smooth[-1]:.0f}' if self.buf_smooth else '--'
        self.stats_text.set_text(
            f'Status: {status}   |   Strokes: {self.stroke_count}   |   '
            f'Contact duty: {duty:.1f}%   |   '
            f'Bounce events absorbed: {self.bounce_events}   |   '
            f'Smooth: {latest_smooth}   |   '
            f'Samples: {self.total_samples}'
        )

    # -- summary -----------------------------------------------------------

    def _print_summary(self):
        print('\n' + '='*60)
        print('  FSR FORCE CONTACT DETECTOR — SESSION SUMMARY')
        print('='*60)
        duty = self.contact_samples / self.total_samples * 100 if self.total_samples else 0.0
        print(f'  Total samples          : {self.total_samples}')
        print(f'  Contact samples        : {self.contact_samples}')
        print(f'  Contact duty cycle     : {duty:.1f}%')
        print(f'  Writing strokes        : {self.stroke_count}')
        print(f'  Bounce events absorbed : {self.bounce_events}')
        print(f'  ---')
        print(f'  EMA alpha              : {ForceContactDetector.EMA_ALPHA}')
        print(f'  Noise floor            : {ForceContactDetector.NOISE_FLOOR:.0f} ADC')
        print(f'  Enter threshold        : {ForceContactDetector.THRESH_ENTER:.0f} ADC')
        print(f'  Exit  threshold        : {ForceContactDetector.THRESH_EXIT:.0f} ADC')
        print(f'  Debounce ON  window    : {ForceContactDetector.DEBOUNCE_ON_S*1000:.0f} ms')
        print(f'  Debounce OFF window    : {ForceContactDetector.DEBOUNCE_OFF_S*1000:.0f} ms')
        print('='*60 + '\n')


# -------------------------------------------------------------------------
#  ENTRY POINT
# -------------------------------------------------------------------------
def main():
    parser = AsyncDataParser(port=SERIAL_PORT, baud=BAUD_RATE,
                             csv_path=DATASET_FILENAME)
    if not parser.connect():
        sys.exit(1)

    dashboard = ForceDetectorDashboard(parser)
    mode_str  = 'CSV Playback' if parser.mode == 'csv' else 'Live Serial'
    print(f'Force Detector (FSR) running in {mode_str} mode (Async Stream)')

    ani = FuncAnimation(dashboard.fig, dashboard.update_plot,
                        interval=40, blit=False, cache_frame_data=False)
    plt.show()
    parser.close()
    dashboard._print_summary()


if __name__ == '__main__':
    main()
