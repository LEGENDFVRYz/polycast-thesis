"""
Module 5 — UWB Range Preprocessor

Cleans raw anchor distances before they reach trilateration.

Input  (from tv2_normalizer — UWB events, updated schema):
    {
        'sensor':     'UWB',
        'packet_id':  int,
        'sample_idx': 0,
        'ts_hw':      int,
        'dists':      (d0, d1, d2, d3)    - raw metres from 4 anchors
    }

Output (cleaned range event):
    {
        'sensor':       'UWB',
        'ts_hw':        int,
        'packet_id':    int,
        'raw_dists':    (d0, d1, d2, d3),           - original values, unchanged
        'clean_dists':  (d0, d1, d2, d3),           - after offset + median + EMA filtering
        'valid_mask':   (True, True, True, True),   - flags for validity
        'outlier_flags':(False,False,False,False),  - flags for outliers
    }

Processing pipeline per anchor:
    1. Offset Calibration
    2. Sanity check and Jump detection
    3. EMA Median filter: window=5 over per-anchor history buffer (alpha=0.25)

    - All four anchors are processed independently
    - An anchor remains "valid" unless it fails the sanity check AND the jump test
"""

import math
import statistics
from collections import deque
from background.pipelines.config import cfg
from background.pipelines.preprocess.uwb.range_denoiser import (
    RangeDenoiserBuffer, RangeDenoiserInferer,
)


class _AnchorKalman:
    """
    1D Kalman filter for a single UWB anchor range.

    State: [distance, velocity]  (metres, metres/s)
    Models pen as constant-velocity with process noise sigma_q (m/s²).
    Measurement noise sigma_r (m) tuned to BU03 static noise floor.

    Rejects measurements implying speed > max_speed_ms by holding the
    predicted state instead — this kills multipath oscillations which
    would require physically impossible accelerations to be real.
    """

    def __init__(self, sigma_r: float, sigma_q: float, max_speed_ms: float):
        self._sigma_r     = sigma_r
        self._sigma_q     = sigma_q
        self._max_speed   = max_speed_ms
        self._x: "list[float] | None" = None  # [dist, vel]
        self._P = [[1.0, 0.0], [0.0, 1.0]]    # 2×2 covariance

    @property
    def ready(self) -> bool:
        return self._x is not None

    def seed(self, dist: float) -> None:
        self._x = [dist, 0.0]
        self._P = [[self._sigma_r ** 2, 0.0], [0.0, 0.25]]

    def update(self, z: float, dt: float) -> tuple[float, bool]:
        """
        Predict + update for one measurement.
        Returns (filtered_distance, was_rejected).
        rejected=True means z implied impossible speed; predicted state held.
        """
        if self._x is None:
            self.seed(z)
            return z, False

        # ── Predict ──────────────────────────────────────────────────
        x0, x1 = self._x
        x0_p = x0 + x1 * dt
        x1_p = x1

        q = self._sigma_q ** 2
        P00 = self._P[0][0] + dt * (self._P[1][0] + self._P[0][1]) + dt * dt * self._P[1][1] + q * dt**4 / 4
        P01 = self._P[0][1] + dt * self._P[1][1] + q * dt**3 / 2
        P10 = self._P[1][0] + dt * self._P[1][1] + q * dt**3 / 2
        P11 = self._P[1][1] + q * dt**2

        # ── Speed gate — reject if measurement implies impossible velocity ──
        implied_speed = abs(z - x0_p) / max(dt, 1e-6)
        if implied_speed > self._max_speed:
            # Hold predicted state, inflate covariance slightly
            self._x = [x0_p, x1_p]
            self._P = [[P00, P01], [P10, P11]]
            return x0_p, True

        # ── Update ───────────────────────────────────────────────────
        S  = P00 + self._sigma_r ** 2
        K0 = P00 / S
        K1 = P10 / S
        y  = z - x0_p

        self._x = [x0_p + K0 * y, x1_p + K1 * y]
        self._P = [
            [(1 - K0) * P00,       (1 - K0) * P01],
            [P10 - K1 * P00,       P11 - K1 * P01],
        ]
        return self._x[0], False

    def reset(self) -> None:
        self._x = None
        self._P = [[1.0, 0.0], [0.0, 1.0]]


class UWBRangePreprocessor:
    """
    Stateful per-anchor range cleaner.

    Pipeline per anchor:
        1. Hardware blind-spot / zero check
        2. Offset calibration
        3. Sanity check (physical range bounds)
        4. Cross-anchor simultaneous dropout detection (holds last good on burst)
        5. Median pre-filter (window=5) — removes single-sample spikes
        6. Per-anchor 1D Kalman filter (position + velocity state)
           — rejects multipath oscillations via speed gate
    """

    def __init__(self, offsets: tuple = (0.0, 0.0, 0.0, 0.0)):
        n = cfg.uwb.num_anchors
        self.offsets = offsets

        # Median pre-filter buffers
        self._history: list[deque] = [
            deque(maxlen=cfg.uwb.median_window) for _ in range(n)]

        # Per-anchor 1D Kalman filters — each anchor gets its own sigma_r
        sigma_r_list = cfg.uwb.kalman_sigma_r_per_anchor
        self._kf: list[_AnchorKalman] = [
            _AnchorKalman(
                sigma_r     = sigma_r_list[i],
                sigma_q     = cfg.uwb.kalman_sigma_q,
                max_speed_ms= cfg.uwb.kalman_max_speed_ms,
            ) for i in range(n)
        ]

        # Last accepted clean value per anchor (for dropout hold)
        self._last_good: list[float | None] = [None] * n

        # Anti-lockout: consecutive speed-gate rejections before hard reset
        self._reject_count: list[int] = [0] * n

        self._prev_ts: int | None = None

        # Per-anchor CNN denoiser (active when model assets exist)
        rd = cfg.range_denoiser
        if rd.enabled:
            self._dn_buf: list[RangeDenoiserBuffer] = [
                RangeDenoiserBuffer(rd.window_size) for _ in range(n)]
            self._dn_inf: list[RangeDenoiserInferer] = [
                RangeDenoiserInferer(i, rd.window_size) for i in range(n)]
        else:
            self._dn_buf = None
            self._dn_inf = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def feed(self, events: list[dict]) -> list[dict]:
        out = []
        for ev in events:
            if ev.get('sensor') != 'UWB':
                continue
            result = self.process_one(ev)
            if result:
                out.append(result)
        return out

    def process_one(self, ev: dict) -> dict | None:
        ts    = ev.get('ts_hw')
        dists = ev.get('dists')
        if ts is None or dists is None:
            return None
        if len(dists) != cfg.uwb.num_anchors:
            return None

        dt_s = (
            max((ts - self._prev_ts) / 1_000_000.0, 1e-6)
            if self._prev_ts is not None
            else 1.0 / cfg.uwb.rate_hz
        )
        dt_s = min(dt_s, 0.5)   # clamp absurd gaps (reconnect, pause)
        self._prev_ts = ts

        raw_dists     = tuple(dists)
        n             = cfg.uwb.num_anchors

        # ── Stage 1: offset + sanity per anchor ──────────────────────────
        offset_dists: list[float | None] = []
        for i, d in enumerate(raw_dists):
            if d is None or (isinstance(d, float) and math.isnan(d)) or d <= cfg.uwb.dropout_zero_thresh:
                offset_dists.append(None)   # hard zero / NaN
            else:
                od = d + self.offsets[i]
                sane = cfg.pipeline.uwb_min_range_m <= od <= cfg.pipeline.uwb_max_range_m
                offset_dists.append(od if sane else None)

        # ── Stage 2: dropout detection — both simultaneous and single-anchor ──
        # Any anchor reporting None (zero/NaN/insane) is substituted with its
        # last good value regardless of how many dropped. The cross-anchor check
        # was previously required to avoid false-positives from the old EMA, but
        # with the Kalman speed gate handling noise, a single-anchor hold is safe.
        offset_dists = [
            (self._last_good[i] if od is None else od)
            for i, od in enumerate(offset_dists)
        ]

        # ── Stages 3–4: median + Kalman per anchor ───────────────────────
        clean_dists:   list[float] = []
        valid_mask:    list[bool]  = []
        outlier_flags: list[bool]  = []

        for i, od in enumerate(offset_dists):
            if od is None:
                # No good value and no last_good — hold whatever Kalman predicts
                fallback = self._last_good[i] if self._last_good[i] is not None else -1.0
                clean_dists.append(fallback)
                valid_mask.append(False)
                outlier_flags.append(True)
                continue

            # Median pre-filter
            self._history[i].append(od)
            median_val = statistics.median(self._history[i])

            # 1D Kalman
            filtered, rejected = self._kf[i].update(median_val, dt_s)

            if rejected:
                self._reject_count[i] += 1
                if self._reject_count[i] >= cfg.uwb.max_jumps_n:
                    # Locked out too long — snap Kalman to current reality
                    self._kf[i].seed(median_val)
                    self._reject_count[i] = 0
                    filtered = median_val
                    rejected = False
            else:
                self._reject_count[i] = 0

            self._last_good[i] = filtered

            # ── CNN denoiser (runs on top of Kalman output) ───────────────
            if self._dn_buf is not None and not rejected:
                window = self._dn_buf[i].push(filtered)
                if window is not None:
                    filtered = self._dn_inf[i].predict(window)
                    self._last_good[i] = filtered

            clean_dists.append(filtered)
            valid_mask.append(not rejected)
            outlier_flags.append(rejected)

        return {
            'sensor':        'UWB',
            'ts_hw':         ts,
            'packet_id':     ev.get('packet_id'),
            'raw_dists':     raw_dists,
            'clean_dists':   tuple(clean_dists),
            'valid_mask':    tuple(valid_mask),
            'outlier_flags': tuple(outlier_flags),
        }

    def reset(self):
        n = cfg.uwb.num_anchors
        sigma_r_list = cfg.uwb.kalman_sigma_r_per_anchor
        self._history     = [deque(maxlen=cfg.uwb.median_window) for _ in range(n)]
        self._kf          = [
            _AnchorKalman(sigma_r_list[i], cfg.uwb.kalman_sigma_q,
                          cfg.uwb.kalman_max_speed_ms) for i in range(n)]
        self._last_good   = [None] * n
        self._reject_count= [0] * n
        self._prev_ts     = None
        if self._dn_buf is not None:
            for buf in self._dn_buf:
                buf.reset()



# ==============================================================================
# HARDWARE DATA LOGGING & REPORTING (UWB Range Preprocessor)
#   - Collects processed UWB events in the background
#   - Exports to a structured CSV on exit (Ctrl+C)
#   - Generates and displays a 2x2 Matplotlib timeline report on exit
# ==============================================================================
if __name__ == '__main__':
    import time
    import os
    import csv
    import matplotlib.pyplot as plt
    
    # Adjust imports based on your exact file structure
    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.cleaner.normalizer import StreamNormalizer

    # CONFIGURATION
    SERIAL_PORT = cfg.serial.port
    BAUD_RATE = cfg.serial.baud
    DISPLAY_RATE = 0.2          # Console update rate (seconds)

    # Initialize pipeline modules
    streamer = SerialStreamer(port=SERIAL_PORT, baud=BAUD_RATE)
    norm = StreamNormalizer()
    
    # ── Inject Offsets from Global Config ──
    # Defaults to zeros if not yet added to the config file
    uwb_offsets = cfg.uwb.range_offsets_m
    prep = UWBRangePreprocessor(offsets=uwb_offsets)

    print("=" * 60)
    print(f"  UWB Data Logger & Reporter: {SERIAL_PORT}")
    print(f"  Active Calibration Offsets: {uwb_offsets}")
    print("  Collecting data silently... Press Ctrl+C to stop and generate reports.")
    print("=" * 60)

    # Buffer to store all processed events
    event_log = []
    last_print_time = 0

    try:
        while True:
            # 1. Fetch raw packets
            raw_packets = streamer.read_new_packets()
            
            if raw_packets:
                # 2. Normalize to flat events
                events = norm.normalize(raw_packets)
                
                # 3. Preprocess UWB events
                processed_uwb = prep.feed(events)
                
                # 4. Store in memory for post-run reporting
                if processed_uwb:
                    event_log.extend(processed_uwb)
                    
                    # 5. Live Dashboard (Throttled)
                    current_time = time.time()
                    if current_time - last_print_time >= DISPLAY_RATE:
                        latest = processed_uwb[-1]
                        
                        os.system('cls' if os.name == 'nt' else 'clear')
                        print(f"========= LIVE UWB PREPROCESSOR ({DISPLAY_RATE}s update) =========")
                        print(f"  Pkt ID     : {latest['packet_id']}")
                        print(f"  TS (hw)    : {latest['ts_hw']}")
                        print("-" * 57)
                        
                        for i in range(4):
                            raw = latest['raw_dists'][i]
                            clean = latest['clean_dists'][i]
                            
                            if not latest['valid_mask'][i] or latest['outlier_flags'][i]:
                                status = "OUTLIER/SPIKE"
                            else:
                                status = "VALID"
                                
                            print(f"  Anchor {i}: Raw: {raw:6.2f} m  |  Clean: {clean:6.2f} m  | [{status}]")
                            
                        print("=========================================================")
                        last_print_time = current_time

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[STOP] Data collection halted.")
        streamer.close()

        if not event_log:
            print("No data collected to report. Exiting.")
            exit()

        print(f"Captured {len(event_log)} UWB events. Generating reports...")

        # --- 1. CSV EXPORT ---
        csv_filename = "uwb_range_report.csv"
        with open(csv_filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'ts_hw', 'packet_id',
                'raw_0', 'raw_1', 'raw_2', 'raw_3',
                'clean_0', 'clean_1', 'clean_2', 'clean_3',
                'outlier_0', 'outlier_1', 'outlier_2', 'outlier_3'
            ])
            for ev in event_log:
                r = ev['raw_dists']
                c = ev['clean_dists']
                o = ev['outlier_flags']
                writer.writerow([
                    ev['ts_hw'], ev['packet_id'],
                    r[0], r[1], r[2], r[3],
                    c[0], c[1], c[2], c[3],
                    int(o[0]), int(o[1]), int(o[2]), int(o[3])
                ])
        print(f"[EXPORT] Data saved to {csv_filename}")

        # --- 2. PLOT REPORT ---
        print("[PLOT] Rendering visual report...")
        
        start_ts = event_log[0]['ts_hw']
        # Convert microsecond timestamps to relative seconds
        t_sec = [(ev['ts_hw'] - start_ts) / 1_000_000.0 for ev in event_log]

        # Create a 2x2 grid for the 4 anchors
        fig, axs = plt.subplots(2, 2, figsize=(14, 10), sharex=True)
        fig.suptitle('UWB Range Preprocessor: Raw vs. Clean (Spike Rejection)', fontsize=16, fontweight='bold')

        # Flatten axes array for easy iteration
        axs_flat = axs.flatten()

        for i in range(4):
            ax = axs_flat[i]
            
            raw_vals = [ev['raw_dists'][i] for ev in event_log]
            clean_vals = [ev['clean_dists'][i] for ev in event_log]
            outliers = [ev['outlier_flags'][i] for ev in event_log]

            # Plot raw with high transparency
            ax.plot(t_sec, raw_vals, label='Raw Distance', color='red', alpha=0.3, linewidth=1.5)
            # Plot clean with thick solid line
            ax.plot(t_sec, clean_vals, label='Clean Distance (Median+EMA)', color='blue', linewidth=2)
            
            # Highlight outlier moments with red vertical markers
            outlier_times = [t_sec[j] for j, is_out in enumerate(outliers) if is_out]
            outlier_y = [raw_vals[j] for j, is_out in enumerate(outliers) if is_out]
            if outlier_times:
                ax.scatter(outlier_times, outlier_y, color='black', marker='x', s=50, label='Rejected Spike')

            ax.set_title(f'Anchor A{i}')
            ax.set_ylabel('Distance (Metres)')
            ax.grid(True, linestyle='--', alpha=0.6)
            if i == 1: # Only put legend on one chart to save space
                ax.legend(loc='upper right')

        axs[1, 0].set_xlabel('Time (Seconds)')
        axs[1, 1].set_xlabel('Time (Seconds)')

        plt.tight_layout()
        plt.subplots_adjust(top=0.92)
        
        plot_filename = "uwb_range_report.png"
        plt.savefig(plot_filename, dpi=300)
        print(f"[PLOT] Report saved as {plot_filename}")
        
        plt.show()