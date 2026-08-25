"""
contact.py  —  PolyCast FSR Contact Detector (pipeline_ekf)
===========================================================
`ForceContactDetector`, split out of `kuru_method/asynchronous_stream/
force_detector.py` so the background thread can import it without dragging
in that module's live dashboard.

The original file also holds `ForceDetectorDashboard` and a `main()` that
open a serial port and a matplotlib window. Importing it therefore pulled
`data_parser` -> `pyserial`, plus `matplotlib`, into every consumer -- even
though the EKF only ever uses the detector class. Only the detector is
reproduced here; the dashboard stays in kuru_method as a standalone tool.

Hardware design:
    - FSR (Force Sensitive Resistor) mounted at marker tip
    - Voltage divider: 3.3V -> FSR -> A0 -> 10kohm -> GND
    - Marker tip touches board -> FSR compresses -> resistance drops
      -> A0 voltage rises -> ADC value increases
    - Marker tip lifts -> FSR relaxes -> resistance rises
      -> A0 voltage drops -> ADC value decreases

Firmware encoding (from imu_module.cpp):
    Raw 12-bit ADC:  analogRead(A0)  ->  0-4095  (sent as float)
    No force  ->  ~0      (FSR resistance >> 10kohm, voltage ~ 0V)
    Light touch -> ~200-700  (FSR ~30-100kohm)
    Hard press  -> ~2000-4000  (FSR ~1-10kohm)

Detection pipeline:
    raw_force (0-4095 ADC)
      -> Stage 1: EMA smoothing        -- reduce analog noise
      -> Stage 2: Noise-floor gate     -- clamp idle noise to 0
      -> Stage 3: Hysteresis (Schmitt) -- enter on HIGH, exit on LOW
      -> Stage 4: Temporal debounce    -- N consecutive samples to confirm
      -> contact_state in {0, 1}
"""

from background.pipeline_ekf.config import FSR_DT_NOM_S

ADC_MAX = 4095.0            # 12-bit ADC full scale


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


