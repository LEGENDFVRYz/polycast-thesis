"""
imu_validator.py  —  PolyCast IMU Integration Validator
========================================================
Standalone dashboard that lets you verify IMU behaviour BEFORE running
the full EKF.  Use this to check:

    1.  Axis mapping is correct
        Dead-reckoning path should move in the same direction as UWB
        positions when you move the marker left/right/up/down.

    2.  Accelerometer magnitude is plausible
        At rest: ~0 m/s².  Fast stroke: 2–6 m/s².  Any values >15 m/s²
        are glitches and should be rare.

    3.  Heading estimation converges
        The console prints "Heading locked" with a direction vector after
        the first ~30 samples.  Subsequent wb_X and wb_Y channels should
        look symmetric when you move the marker in those directions.

    4.  Contact state is clean
        The bottom panel shows the button output at 100 Hz.
        You should see clean 0→1 transitions with no bounce-glitches.

    5.  Dead-reckoning drift rate
        The dead-reckoning position is reset to the IRLS UWB position
        every RESET_INTERVAL_S seconds (default 2).  How far the two
        diverge in that window tells you the IMU noise level and how
        much the EKF bias estimator will need to correct.

Dashboard layout
----------------
    Panel 1 (top-left):   Dead-reckoning vs UWB — 2D whiteboard view
    Panel 2 (top-right):  Whiteboard-frame acceleration channels (ax, ay)
    Panel 3 (bottom-left): Marker tilt angle from quaternion
    Panel 4 (bottom-right): Contact state (button) and IMU speed estimate

Usage
-----
    python imu_validator.py                          # live serial
    python imu_validator.py datasets_v2/data.csv     # CSV playback
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from collections import deque
from data_stream    import DataStream
from imu_integrator import IMUIntegrator, quat_to_rotmat
from button_detector import ButtonContactDetector

# ── Configuration ──────────────────────────────────────────────────────
SERIAL_PORT      = 'COM5'
BAUD_RATE        = 115200
DATASET_FILENAME = '../datasets_v2/big_rectangle.csv'   # '' = live; 'path/to/data.csv' = playback

MAX_SAMPLES      = 500 # rolling window width for time-series plots
RESET_INTERVAL_S = 2.0  # seconds between dead-reckoning resets to UWB

# Anchor positions (must match data_stream.py / ekf_fusion.py)
ANCHORS = np.array([
    [0.00, 0.00, 0.07],
    [1.23, 0.00, 0.07],
    [1.23, 1.23, 0.07],
    [0.00, 1.23, 0.07],
], dtype=float)
MARKER_Z = 0.21  # m — UWB tag height above board


class IMUValidatorDashboard:
    def __init__(self, stream: DataStream):
        self.stream   = stream
        self._imu     = IMUIntegrator()
        self._button  = ButtonContactDetector()
        self._irls    = self._make_irls()
        self.finished = False

        # Dead-reckoning state
        self._dr_pos    = None   # [px, py]
        self._dr_vel    = np.zeros(2)
        self._irls_pos  = None   # latest IRLS position
        self._reset_accumulator = 0.0  # time since last reset
        self._dt_imu    = 0.010  # 10 ms nominal

        # Rolling buffers
        self._buf_ax   = deque(maxlen=MAX_SAMPLES)
        self._buf_ay   = deque(maxlen=MAX_SAMPLES)
        self._buf_tilt = deque(maxlen=MAX_SAMPLES)
        self._buf_spd  = deque(maxlen=MAX_SAMPLES)
        self._buf_btn  = deque(maxlen=MAX_SAMPLES)

        self._dr_trail_x  = deque(maxlen=200)
        self._dr_trail_y  = deque(maxlen=200)
        self._uwb_scatter_x = deque(maxlen=100)
        self._uwb_scatter_y = deque(maxlen=100)

        self._build_figure()

    # ── layout ────────────────────────────────────────────────────────

    def _make_irls(self):
        from kuru_method.batched_stream.fusion_engine import IRLSTrilateration
        bmin = [-0.30, -0.30, -0.50]
        bmax = [ 1.55,  1.55,  1.00]
        return IRLSTrilateration(ANCHORS, bmin, bmax, tag_z=MARKER_Z)

    def _build_figure(self):
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(15, 9))
        self.fig.canvas.manager.set_window_title('PolyCast — IMU Integration Validator')

        gs = self.fig.add_gridspec(2, 2, hspace=0.38, wspace=0.35)
        self.ax_2d   = self.fig.add_subplot(gs[0, 0])
        self.ax_acc  = self.fig.add_subplot(gs[0, 1])
        self.ax_tilt = self.fig.add_subplot(gs[1, 0])
        self.ax_btn  = self.fig.add_subplot(gs[1, 1])

        # Panel 1 — 2D whiteboard
        ax = self.ax_2d
        ax.set_xlim(-0.2, 1.4)
        ax.set_ylim(-0.2, 1.4)
        ax.set_aspect('equal')
        ax.set_title('Dead-reckoning vs UWB (2D whiteboard)', fontsize=11)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.grid(True, alpha=0.2)
        # Anchors
        ax.scatter(ANCHORS[:, 0], ANCHORS[:, 1],
                   c='red', marker='s', s=80, zorder=5)
        for i, a in enumerate(ANCHORS):
            ax.text(a[0], a[1]+0.06, f'A{i}', color='red',
                    fontsize=8, ha='center')
        # Artists
        self._line_dr,     = ax.plot([], [], '-', color='#00FF99', lw=1.5,
                                     label='IMU dead-reckoning', zorder=4)
        self._dot_dr,      = ax.plot([], [], 'o', color='#00FF99', ms=8, zorder=5)
        self._scat_uwb,    = ax.plot([], [], '.', color='lightgray', ms=4,
                                     alpha=0.6, label='UWB (IRLS)', zorder=3)
        self._reset_line,  = ax.plot([], [], '|', color='yellow', ms=10,
                                     label='DR reset to UWB', zorder=6)
        ax.legend(fontsize=8, loc='upper right')
        self._reset_marks_x = []
        self._reset_marks_y = []

        # Panel 2 — acceleration channels
        ax = self.ax_acc
        ax.set_xlim(0, MAX_SAMPLES)
        ax.set_ylim(-10, 10)
        ax.set_title('Whiteboard-frame acceleration', fontsize=11)
        ax.set_ylabel('m/s²')
        ax.set_xlabel('Sample index (100 Hz)')
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
        ax.set_xlabel('Sample index (100 Hz)')
        ax.axhline(45, color='yellow', lw=0.8, linestyle='--', alpha=0.5,
                   label='45° typical writing angle')
        ax.grid(True, alpha=0.15)
        self._line_tilt, = ax.plot([], [], color='#FFD93D', lw=1.5, label='Tilt angle')
        ax.legend(fontsize=8, loc='upper right')

        # Panel 4 — contact + speed
        ax = self.ax_btn
        ax.set_xlim(0, MAX_SAMPLES)
        ax.set_title('Contact state + IMU speed', fontsize=11)
        ax.set_xlabel('Sample index (100 Hz)')
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
        self._ax_spd = ax_spd
        self._fill_btn = None

        # Stats text
        self._stats_text = self.fig.text(
            0.5, 0.98, 'Waiting for data...',
            ha='center', va='top', fontsize=10, color='white',
            bbox=dict(facecolor='#111111', alpha=0.8, edgecolor='white',
                      boxstyle='round,pad=0.4'),
        )
        plt.tight_layout(rect=[0, 0, 1, 0.95])

    # ── animation callback ─────────────────────────────────────────────

    def animate(self, frame):
        if self.finished:
            return

        n_processed = 0
        if self.stream.mode == 'live':
            while self.stream.data_available():
                pkt = self.stream.get_packet()
                if pkt:
                    self._ingest(pkt)
                    n_processed += 1
        else:
            for _ in range(6):
                pkt = self.stream.get_packet()
                if pkt == 'EOF':
                    self.finished = True
                    self.stream.close()
                    self._print_summary()
                    break
                if pkt:
                    self._ingest(pkt)
                    n_processed += 1

        if n_processed:
            self._redraw()

    # ── data ingestion ─────────────────────────────────────────────────

    def _ingest(self, packet):
        # UWB IRLS position
        filt = packet['uwb']
        pos_xyz, _ = self._irls.solve(filt)
        self._irls_pos = pos_xyz[:2].copy()
        self._uwb_scatter_x.append(float(pos_xyz[0]))
        self._uwb_scatter_y.append(float(pos_xyz[1]))

        for sample in packet['imu']:
            q   = sample['quat']
            acc = sample['acc']

            a_wb = self._imu.get_wb_acceleration(q, acc)

            # Dead-reckoning integration
            if self._dr_pos is None and self._irls_pos is not None:
                self._dr_pos = self._irls_pos.copy()
                self._dr_vel = np.zeros(2)

            if self._dr_pos is not None:
                self._dr_vel += a_wb * self._dt_imu
                self._dr_pos += self._dr_vel * self._dt_imu

                self._reset_accumulator += self._dt_imu
                if self._reset_accumulator >= RESET_INTERVAL_S and self._irls_pos is not None:
                    self._reset_marks_x.append(self._dr_pos[0])
                    self._reset_marks_y.append(self._dr_pos[1])
                    self._dr_pos = self._irls_pos.copy()
                    self._dr_vel = np.zeros(2)
                    self._reset_accumulator = 0.0

                self._dr_trail_x.append(float(self._dr_pos[0]))
                self._dr_trail_y.append(float(self._dr_pos[1]))

            # Tilt angle: angle between body X (board normal) and board surface
            R = quat_to_rotmat(*q)
            body_x_world = R[:, 0]   # body X direction in world frame
            # Angle with horizontal plane = 90° - angle with world Z
            cos_z = abs(float(np.clip(body_x_world[2], -1, 1)))
            tilt_deg = np.degrees(np.arccos(cos_z))  # 0° = perpendicular to board
            tilt_deg = max(0.0, min(90.0, tilt_deg))

            # Speed from DR velocity
            speed = float(np.linalg.norm(self._dr_vel)) if self._dr_pos is not None else 0.0

            # Contact state
            state, _ = self._button.process(sample['force'])

            self._buf_ax.append(float(a_wb[0]))
            self._buf_ay.append(float(a_wb[1]))
            self._buf_tilt.append(tilt_deg)
            self._buf_spd.append(speed)
            self._buf_btn.append(float(state))

    # ── redraw ────────────────────────────────────────────────────────

    def _redraw(self):
        x_ts = np.arange(len(self._buf_ax))

        # Panel 1
        if self._dr_trail_x:
            self._line_dr.set_data(list(self._dr_trail_x), list(self._dr_trail_y))
            if self._dr_trail_x:
                self._dot_dr.set_data([self._dr_trail_x[-1]], [self._dr_trail_y[-1]])
        self._scat_uwb.set_data(list(self._uwb_scatter_x), list(self._uwb_scatter_y))
        if self._reset_marks_x:
            self._reset_line.set_data(self._reset_marks_x, self._reset_marks_y)

        # Panel 2
        self._line_ax.set_data(x_ts, list(self._buf_ax))
        self._line_ay.set_data(x_ts, list(self._buf_ay))
        if len(self._buf_ax) > 0:
            mx = max(max(abs(v) for v in self._buf_ax),
                     max(abs(v) for v in self._buf_ay), 1.0)
            self.ax_acc.set_ylim(-mx * 1.1, mx * 1.1)

        # Panel 3
        self._line_tilt.set_data(x_ts, list(self._buf_tilt))

        # Panel 4
        btn_arr = list(self._buf_btn)
        spd_arr = list(self._buf_spd)
        self._line_btn.set_data(x_ts, btn_arr)
        self._line_spd.set_data(x_ts, spd_arr)
        if self._fill_btn:
            self._fill_btn.remove()
        if len(x_ts) > 0:
            self._fill_btn = self.ax_btn.fill_between(
                x_ts, 0, btn_arr,
                where=[b > 0.5 for b in btn_arr],
                color='#00FF99', alpha=0.25, step='post',
            )

        # Stats banner
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
            print(f'  a_wb_x  mean/std : '
                  f'{np.mean(self._buf_ax):.3f} / {np.std(self._buf_ax):.3f} m/s²')
            print(f'  a_wb_y  mean/std : '
                  f'{np.mean(self._buf_ay):.3f} / {np.std(self._buf_ay):.3f} m/s²')
            print(f'  Tilt    mean/std : '
                  f'{np.mean(self._buf_tilt):.1f}° / {np.std(self._buf_tilt):.1f}°')
        print('='*55 + '\n')


def main():
    dataset = sys.argv[1] if len(sys.argv) > 1 else DATASET_FILENAME
    stream  = DataStream(SERIAL_PORT, BAUD_RATE, dataset)
    if not stream.connect():
        sys.exit(1)

    dash = IMUValidatorDashboard(stream)
    mode_str = 'CSV Playback' if stream.mode == 'csv' else 'Live Serial'
    print(f'IMU Validator running in {mode_str} mode  (close window to stop)')

    ani = FuncAnimation(dash.fig, dash.animate, interval=40,
                        blit=False, cache_frame_data=False)
    plt.show()
    stream.close()
    dash._print_summary()


if __name__ == '__main__':
    main()