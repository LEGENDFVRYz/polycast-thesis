"""
imu_validator.py  —  PolyCast IMU Integration Validator (Async Stream)
======================================================================
"""

import os
import sys
from pathlib import Path
import numpy as np
import matplotlib
# If SAVE_PNG is set (or no DISPLAY), use a headless backend so this script
# can be driven non-interactively for per-CSV IMU diagnostic PNGs.
if os.environ.get('SAVE_PNG') or os.environ.get('IMU_VALIDATOR_HEADLESS'):
    matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from collections import deque

from data_parser     import AsyncDataParser
from preprocessor    import UWBPreprocessor, IMUPreprocessor
from imu_integrator  import IMUIntegrator, quat_to_rotmat
from force_detector import ForceContactDetector
from fusion_engine   import IRLSTrilateration

# ── Configuration ──────────────────────────────────────────────────────
from config import SERIAL_PORT, BAUD_RATE, ANCHORS, MARKER_LENGTH, UWB_OFFSETS
DATASET_FILENAME = 'datasets_str_50hz/abc_s1.csv'   # '' = live; 'path/to/data.csv' = playback

MAX_SAMPLES      = 500  # rolling window width for time-series plots
RESET_INTERVAL_S = 2.0  # seconds between dead-reckoning resets to UWB


class IMUValidatorDashboard:
    def __init__(self, parser: AsyncDataParser):
        self.parser   = parser
        self._imu     = IMUIntegrator()
        self._button  = ForceContactDetector()
        
        # Preprocessors
        self.uwb_cleaner = UWBPreprocessor(offsets=UWB_OFFSETS)
        self.imu_cleaner = IMUPreprocessor()
        
        # IRLS Solver for UWB ground-truth
        bmin = [-0.30, -0.30, -0.50]
        bmax = [ 1.55,  1.55,  1.00]
        self._irls = IRLSTrilateration(ANCHORS, bmin, bmax, tag_z=MARKER_LENGTH)
        
        self.finished = False

        # Dead-reckoning state
        self._dr_pos    = None   # [px, py]
        self._dr_vel    = np.zeros(2)
        self._irls_pos  = None   # latest IRLS position
        self._reset_accumulator = 0.0
        self._last_imu_ts = None  # for per-packet dt computation

        # Rolling buffers
        self._buf_ax   = deque(maxlen=MAX_SAMPLES)
        self._buf_ay   = deque(maxlen=MAX_SAMPLES)
        self._buf_tilt = deque(maxlen=MAX_SAMPLES)
        self._buf_spd  = deque(maxlen=MAX_SAMPLES)
        self._buf_btn  = deque(maxlen=MAX_SAMPLES)

        self._dr_trail_x  = deque(maxlen=1000)
        self._dr_trail_y  = deque(maxlen=1000)
        self._uwb_scatter_x = deque(maxlen=100)
        self._uwb_scatter_y = deque(maxlen=100)

        self._build_figure()

    # ── layout ────────────────────────────────────────────────────────

    def _build_figure(self):
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(15, 9))
        self.fig.canvas.manager.set_window_title('PolyCast — IMU Integration Validator (Async)')

        # Change grid to 3 rows, 2 columns.
        gs = self.fig.add_gridspec(3, 2, hspace=0.5, wspace=0.25)
        
        # Left half spanning all 3 rows
        self.ax_2d   = self.fig.add_subplot(gs[:, 0]) 
        # Right half stacked on 3 rows
        self.ax_acc  = self.fig.add_subplot(gs[0, 1])
        self.ax_tilt = self.fig.add_subplot(gs[1, 1])
        self.ax_btn  = self.fig.add_subplot(gs[2, 1])

        # Panel 1 — 2D whiteboard
        ax = self.ax_2d
        ax.set_xlim(-0.2, 1.4)
        ax.set_ylim(-0.2, 1.4)
        ax.set_aspect('equal')
        ax.set_title('Dead-reckoning vs UWB (2D whiteboard)', fontsize=11)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.grid(True, alpha=0.2)
        
        ax.scatter(ANCHORS[:, 0], ANCHORS[:, 1], c='red', marker='s', s=80, zorder=5)
        for i, a in enumerate(ANCHORS):
            ax.text(a[0], a[1]+0.06, f'A{i}', color='red', fontsize=8, ha='center')
            
        self._line_dr,    = ax.plot([], [], '-', color='#00FF99', lw=1.5, label='IMU dead-reckoning', zorder=4)
        self._dot_dr,     = ax.plot([], [], 'o', color='#00FF99', ms=8, zorder=5)
        self._scat_uwb,   = ax.plot([], [], '.', color='lightgray', ms=4, alpha=0.6, label='UWB (IRLS)', zorder=3)
        self._reset_line, = ax.plot([], [], '|', color='yellow', ms=10, label='DR reset to UWB', zorder=6)
        ax.legend(fontsize=8, loc='upper right')
        self._reset_marks_x, self._reset_marks_y = [], []

        # Panel 2 — acceleration channels
        ax = self.ax_acc
        ax.set_xlim(0, MAX_SAMPLES)
        ax.set_ylim(-10, 10)
        ax.set_title('Whiteboard-frame acceleration', fontsize=11)
        ax.set_ylabel('m/s²')
        ax.axhline(0, color='white', lw=0.4, alpha=0.3)
        ax.grid(True, alpha=0.15)
        self._line_ax, = ax.plot([], [], color='#00BFFF', lw=1.5, label='a_wb_x (horiz)')
        self._line_ay, = ax.plot([], [], color='#FF6B6B', lw=1.5, label='a_wb_y (vert)')
        ax.legend(fontsize=8, loc='upper right')

        # Panel 3 — marker tilt
        ax = self.ax_tilt
        ax.set_xlim(0, MAX_SAMPLES)
        ax.set_ylim(0, 90)
        ax.set_title('Marker tilt from board normal', fontsize=11)
        ax.set_ylabel('Tilt angle (°)')
        ax.axhline(45, color='yellow', lw=0.8, linestyle='--', alpha=0.5, label='45° typical writing angle')
        ax.grid(True, alpha=0.15)
        self._line_tilt, = ax.plot([], [], color='#FFD93D', lw=1.5, label='Tilt angle')
        ax.legend(fontsize=8, loc='upper right')

        # Panel 4 — contact + speed
        ax = self.ax_btn
        ax.set_xlim(0, MAX_SAMPLES)
        ax.set_title('Contact state + IMU speed', fontsize=11)
        ax.grid(True, alpha=0.15)
        ax_spd = ax.twinx()
        ax_spd.set_ylim(0, 1.5)
        ax_spd.set_ylabel('Speed (m/s)', color='#AAAAFF')
        ax.set_ylim(-0.25, 1.35)
        ax.set_yticks([0, 1])
        ax.set_yticklabels(['Lifting', 'Writing'], fontsize=9)
        self._line_btn,  = ax.plot([], [], color='#00FF99', lw=2, drawstyle='steps-post')
        self._line_spd,  = ax_spd.plot([], [], color='#AAAAFF', lw=1.2, alpha=0.8)
        ax.legend(handles=[
            mpatches.Patch(color='#00FF99', alpha=0.4, label='Writing'),
            mpatches.Patch(color='#AAAAFF', alpha=0.4, label='Speed'),
        ], fontsize=8, loc='upper right')
        self._fill_btn = None

        self._stats_text = self.fig.text(
            0.5, 0.96, 'Waiting for data...', ha='center', va='top', fontsize=10, color='white',
            bbox=dict(facecolor='#111111', alpha=0.8, edgecolor='white', boxstyle='round,pad=0.4'),
        )
        
        # Swapped tight_layout for subplots_adjust to clear the warning
        self.fig.subplots_adjust(top=0.90, bottom=0.08, left=0.08, right=0.92)

    # ── animation callback ─────────────────────────────────────────────

    def animate(self, frame):
        if self.finished:
            return

        packets = []
        if self.parser.mode == 'live':
            while self.parser.data_available():
                pkt = self.parser.get_packet()
                if pkt and pkt != 'EOF':
                    packets.append(pkt)
        else:
            for _ in range(15):
                pkt = self.parser.get_packet()
                if pkt == 'EOF':
                    self.finished = True
                    self.parser.close()
                    self._print_summary()
                    break
                if pkt:
                    packets.append(pkt)

        for pkt in packets:
            self._ingest(pkt)

        if packets:
            self._redraw()

    # ── data ingestion ─────────────────────────────────────────────────

    def _ingest(self, pkt):
        if pkt['type'] == 'uwb':
            # Clean UWB and solve IRLS
            filt, _, _ = self.uwb_cleaner.process(*pkt['dists'])
            pos_xyz, _ = self._irls.solve(filt)
            self._irls_pos = pos_xyz[:2].copy()
            self._uwb_scatter_x.append(float(pos_xyz[0]))
            self._uwb_scatter_y.append(float(pos_xyz[1]))

        elif pkt['type'] == 'imu':
            # Compute dt from per-packet microsecond timestamps
            dt = 0.010  # default 10ms
            ts = pkt.get('ts')
            if ts is not None and self._last_imu_ts is not None:
                dt_us = ts - self._last_imu_ts
                if 0 < dt_us < 2_000_000:
                    dt = dt_us / 1_000_000.0
            if ts is not None:
                self._last_imu_ts = ts

            # Clean IMU and integrate
            clean = self.imu_cleaner.process_sample(*pkt['quat'], *pkt['acc'])
            q, acc = clean[0:4], clean[4:7]

            a_wb = self._imu.get_wb_acceleration(q, acc)

            # Dead-reckoning integration
            if self._dr_pos is None and self._irls_pos is not None:
                self._dr_pos = self._irls_pos.copy()
                self._dr_vel = np.zeros(2)

            if self._dr_pos is not None:
                self._dr_vel += a_wb * dt
                self._dr_pos += self._dr_vel * dt

                self._reset_accumulator += dt
                if self._reset_accumulator >= RESET_INTERVAL_S and self._irls_pos is not None:
                    self._reset_marks_x.append(self._dr_pos[0])
                    self._reset_marks_y.append(self._dr_pos[1])
                    self._dr_pos = self._irls_pos.copy()
                    self._dr_vel = np.zeros(2)
                    self._reset_accumulator = 0.0

                self._dr_trail_x.append(float(self._dr_pos[0]))
                self._dr_trail_y.append(float(self._dr_pos[1]))

            # Tilt angle
            R = quat_to_rotmat(*q)
            body_x_world = R[:, 0]
            cos_z = abs(float(np.clip(body_x_world[2], -1, 1)))
            tilt_deg = np.degrees(np.arccos(cos_z))
            tilt_deg = max(0.0, min(90.0, tilt_deg))

            # Speed
            speed = float(np.linalg.norm(self._dr_vel)) if self._dr_pos is not None else 0.0

            # Contact
            state, _ = self._button.process(pkt['force'])

            self._buf_ax.append(float(a_wb[0]))
            self._buf_ay.append(float(a_wb[1]))
            self._buf_tilt.append(tilt_deg)
            self._buf_spd.append(speed)
            self._buf_btn.append(float(state))

    # ── redraw ────────────────────────────────────────────────────────

    def _redraw(self):
        x_ts = np.arange(len(self._buf_ax))

        if self._dr_trail_x:
            self._line_dr.set_data(list(self._dr_trail_x), list(self._dr_trail_y))
            self._dot_dr.set_data([self._dr_trail_x[-1]], [self._dr_trail_y[-1]])
            
        self._scat_uwb.set_data(list(self._uwb_scatter_x), list(self._uwb_scatter_y))
        
        if self._reset_marks_x:
            self._reset_line.set_data(self._reset_marks_x, self._reset_marks_y)

        self._line_ax.set_data(x_ts, list(self._buf_ax))
        self._line_ay.set_data(x_ts, list(self._buf_ay))
        if len(self._buf_ax) > 0:
            mx = max(max(abs(v) for v in self._buf_ax), max(abs(v) for v in self._buf_ay), 1.0)
            self.ax_acc.set_ylim(-mx * 1.1, mx * 1.1)

        self._line_tilt.set_data(x_ts, list(self._buf_tilt))
        
        btn_arr, spd_arr = list(self._buf_btn), list(self._buf_spd)
        self._line_btn.set_data(x_ts, btn_arr)
        self._line_spd.set_data(x_ts, spd_arr)
        
        if self._fill_btn: self._fill_btn.remove()
        if len(x_ts) > 0:
            self._fill_btn = self.ax_btn.fill_between(
                x_ts, 0, btn_arr, where=[b > 0.5 for b in btn_arr], color='#00FF99', alpha=0.25, step='post')

        # Stats
        hl = 'YES' if self._imu.heading_locked else 'accumulating...'
        ax_last = f"{self._buf_ax[-1]:.2f}" if self._buf_ax else '—'
        ay_last = f"{self._buf_ay[-1]:.2f}" if self._buf_ay else '—'
        tilt_last = f"{self._buf_tilt[-1]:.1f}°" if self._buf_tilt else '—'
        writing = 'WRITING' if self._button.is_writing else 'LIFTING'
        self._stats_text.set_text(
            f"Heading locked: {hl}   |   a_wb_x: {ax_last} m/s²   "
            f"a_wb_y: {ay_last} m/s²   |   Tilt: {tilt_last}   |   {writing}"
        )

    # ── summary ───────────────────────────────────────────────────────

    def _print_summary(self):
        print('\n' + '='*55)
        print('  IMU VALIDATOR — SESSION SUMMARY')
        print('='*55)
        print(f'  Heading locked : {self._imu.heading_locked}')
        if self._imu.heading_locked:
            print(f'  Heading vector : {self._imu.heading_vec.round(4)}')
        if self._buf_ax:
            print(f'  a_wb_x  mean/std : {np.mean(self._buf_ax):.3f} / {np.std(self._buf_ax):.3f} m/s²')
            print(f'  a_wb_y  mean/std : {np.mean(self._buf_ay):.3f} / {np.std(self._buf_ay):.3f} m/s²')
            print(f'  Tilt    mean/std : {np.mean(self._buf_tilt):.1f}° / {np.std(self._buf_tilt):.1f}°')
        print('='*55 + '\n')


def _run_headless(dash, parser, save_dir: str, stem: str):
    """Drain all CSV packets synchronously, redraw once, save PNG."""
    while True:
        pkt = parser.get_packet()
        if pkt == 'EOF':
            break
        if pkt:
            dash._ingest(pkt)
    dash._redraw()
    # Overwrite the banner with the dataset stem so each PNG is self-identifying
    hl = 'YES' if dash._imu.heading_locked else 'accumulating...'
    hvec = dash._imu.heading_vec.round(3) if dash._imu.heading_locked else None
    if dash._buf_ax:
        ax_stats = f"a_wb_x {np.mean(dash._buf_ax):+.2f}±{np.std(dash._buf_ax):.2f}"
        ay_stats = f"a_wb_y {np.mean(dash._buf_ay):+.2f}±{np.std(dash._buf_ay):.2f}"
        tilt_stats = f"tilt {np.mean(dash._buf_tilt):.0f}°±{np.std(dash._buf_tilt):.0f}°"
    else:
        ax_stats = ay_stats = tilt_stats = '—'
    banner = (f"[{stem}]   heading={hl}"
              + (f" {hvec}" if hvec is not None else '')
              + f"   |   {ax_stats}   {ay_stats}   |   {tilt_stats}")
    dash._stats_text.set_text(banner)

    out_dir = Path(save_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f'imu_validator_{stem}.png'
    dash.fig.savefig(out_path, dpi=110, facecolor=dash.fig.get_facecolor())
    print(f'saved {out_path}')
    dash._print_summary()


def main():
    dataset = sys.argv[1] if len(sys.argv) > 1 else DATASET_FILENAME
    parser  = AsyncDataParser(port=SERIAL_PORT, baud=BAUD_RATE, csv_path=dataset)
    if not parser.connect():
        sys.exit(1)

    dash = IMUValidatorDashboard(parser)
    mode_str = 'CSV Playback' if parser.mode == 'csv' else 'Live Serial'

    save_dir = os.environ.get('SAVE_PNG')
    if save_dir and parser.mode == 'csv':
        stem = Path(dataset).stem if dataset else 'live'
        print(f'IMU Validator running HEADLESS on {stem}.csv — saving to {save_dir}/')
        _run_headless(dash, parser, save_dir, stem)
        parser.close()
        return

    print(f'IMU Validator running in {mode_str} mode  (close window to stop)')
    ani = FuncAnimation(dash.fig, dash.animate, interval=40, blit=False, cache_frame_data=False)
    plt.show()
    parser.close()
    dash._print_summary()


if __name__ == '__main__':
    main()