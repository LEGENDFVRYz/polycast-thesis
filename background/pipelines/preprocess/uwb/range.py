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


class UWBRangePreprocessor:
    """
    Stateful per-anchor range cleaner

    Maintains a rolling buffer and EMA state for each of the 4 anchors.
    """

    def __init__(self, offsets: tuple = (0.0, 0.0, 0.0, 0.0)):
        n = cfg.uwb.num_anchors
        
        # Hardware calibration offsets (antenna delay)
        self.offsets = offsets
        
        # Rolling raw-value buffer for median filter
        self._history:     list[deque] = [
            deque(maxlen=cfg.uwb.median_window) for _ in range(n)]
        # Last EMA output per anchor (None = not seeded yet)
        self._ema:         list[float | None] = [None] * n
        # Last filtered value used for jump detection
        self._prev_clean:  list[float | None] = [None] * n
        
        # --- Anti-Lockout Counters ---
        self._jump_count = [0] * n
        self.max_jumps = 5

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def feed(self, events: list[dict]) -> list[dict]:
        """
        Process a batch of normalized UWB events.
        """
        out = []
        for ev in events:
            if ev.get('sensor') != 'UWB':
                continue
            result = self.process_one(ev)
            if result:
                out.append(result)
        return out

    def process_one(self, ev: dict) -> dict | None:
        """
        Process one normalized UWB event through the full per-anchor pipeline.

        Returns None if the event is missing required fields.
        """
        ts    = ev.get('ts_hw')
        dists = ev.get('dists')

        if ts is None or dists is None:
            return None
        if len(dists) != cfg.uwb.num_anchors:
            return None    # unexpected anchor count

        raw_dists    = tuple(dists)
        clean_dists  = []
        valid_mask   = []
        outlier_flags = []

        for i, d_raw in enumerate(raw_dists):
            valid, outlier, d_clean = self._process_anchor(i, d_raw)
            clean_dists.append(d_clean)
            valid_mask.append(valid)
            outlier_flags.append(outlier)

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
        """
        Reset all per-anchor state.
        """
        n = cfg.uwb.num_anchors
        self._history    = [deque(maxlen=cfg.uwb.median_window) for _ in range(n)]
        self._ema        = [None] * n
        self._prev_clean = [None] * n

    # ------------------------------------------------------------------
    # Per-anchor pipeline
    # ------------------------------------------------------------------

    def _process_anchor(self, idx: int, d_raw: float) -> tuple[bool, bool, float]:
        """
        Run the multi-stage pipeline for one anchor.

        Returns (valid, outlier, clean_value):
            
            - valid         : anchor reading is considered trustworthy
            - outlier       : this specific sample was suspicious (flagged but corrected)
            - clean_value   : best filtered value to use (falls back to prev if bad)
        """
        
        # --- 1. Hardware Blind Spot Check (The Fix) ---
        # If the hand blocks the anchor, it might report 0.0, <= 0.05, or NaN.
        # We reject this immediately to prevent median buffer corruption.
        if d_raw is None or math.isnan(d_raw) or d_raw <= 0.05:
            # Hold the last known good state. If there is no history yet, flag as -1.0
            clean_val = self._ema[idx] if self._ema[idx] is not None else -1.0
            return False, True, clean_val

        # --- 2. Offset Application ---
        raw_with_offset = d_raw + self.offsets[idx]

        # --- 3. Sanity check ---
        sane = (cfg.pipeline.uwb_min_range_m <= raw_with_offset <= cfg.pipeline.uwb_max_range_m)

        # --- 4. Jump detection ---
        jump = False
        if sane and self._prev_clean[idx] is not None:
            delta = abs(raw_with_offset - self._prev_clean[idx])
            if delta > cfg.uwb.max_range_jump_m:
                jump = True

        # --- 5. Anti-Lockout Recovery Logic ---
        if jump:
            self._jump_count[idx] += 1
            if self._jump_count[idx] >= self.max_jumps:
                # We have been locked out for too long. Force a hard reset to reality.
                jump = False
                self._jump_count[idx] = 0
                self._ema[idx] = raw_with_offset  # Instantly snap EMA to current location
                self._history[idx].clear()
        else:
            self._jump_count[idx] = 0  # Reset counter if normal movement
        
        outlier = (not sane) or jump
        valid   = sane and not jump
        
        if outlier:
            # Use previous clean value if available, else skip seeding
            if self._ema[idx] is not None:
                clean_val = self._ema[idx] 
            elif sane:
                # First sample ever but jumped — still seed with raw offset (no history)
                clean_val = raw_with_offset
                valid = True
                outlier = False
            else:
                # Insane and no history → return raw offset as a placeholder
                clean_val = raw_with_offset
            return valid, outlier, clean_val

        # --- 6. Median filter --- 
        self._history[idx].append(raw_with_offset)
        median_val = statistics.median(self._history[idx])

        # --- 7. Stage 3b: ADAPTIVE EMA (The Upgrade) ---
        base_alpha = cfg.uwb.ema_alpha
        
        if self._ema[idx] is None:
            self._ema[idx] = median_val
        else:
            # Calculate the physical distance between current state and new reading
            delta = abs(median_val - self._ema[idx])
            
            # Dynamic adjustment:
            # If moving fast (> 10cm jump), triple the alpha to catch up instantly.
            # If resting/slow, use base alpha to aggressively smooth out the noise.
            if delta > 0.10:
                dynamic_alpha = min(base_alpha * 3.0, 1.0) 
            else:
                dynamic_alpha = base_alpha
                
            self._ema[idx] = dynamic_alpha * median_val + (1.0 - dynamic_alpha) * self._ema[idx]

        clean_val = self._ema[idx]
        self._prev_clean[idx] = clean_val

        return True, False, clean_val



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