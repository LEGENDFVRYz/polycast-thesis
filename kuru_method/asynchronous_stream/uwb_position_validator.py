"""
uwb_position_validator.py  --  PolyCast UWB-Only Position Validator
===================================================================
Shows IRLS trilateration scatter on a 2D whiteboard view WITHOUT any
EKF or IMU fusion.  Use this to judge UWB position quality in isolation.

Display
-------
    Blue dots   -- IRLS trilateration positions (one per UWB packet)
    Red squares -- anchor positions
    Green +     -- ground-truth position (if --truth x,y supplied)
    Text overlay -- running stats (mean, scatter radius, count)

Usage
-----
    python uwb_position_validator.py                          # live serial
    python uwb_position_validator.py --csv datasets/abc.csv   # CSV playback
    python uwb_position_validator.py --truth 0.64,0.63        # with ground truth

Close the window to print session summary.
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from kuru_method.asynchronous_stream.config          import SERIAL_PORT, BAUD_RATE, ANCHORS, MARKER_LENGTH, UWB_OFFSETS
from kuru_method.asynchronous_stream.data_parser     import AsyncDataParser
from kuru_method.asynchronous_stream.preprocessor    import UWBPreprocessor
from kuru_method.asynchronous_stream.fusion_engine   import IRLSTrilateration

# -- Configuration ----------------------------------------------------------
DATASET_FILENAME = ''   # '' = live serial;  'datasets/data.csv' = playback
TRUTH_XY         = None                 # Ground-truth position or None  e.g. (0.64, 0.63)


class UWBPositionDashboard:
    def __init__(self, parser, uwb_cleaner, irls, TRUTH_XY=None):
        self.parser      = parser
        self.uwb_cleaner = uwb_cleaner
        self.irls        = irls
        self.TRUTH_XY    = TRUTH_XY
        self.finished    = False

        # Collected positions
        self._xs = []
        self._ys = []
        self._residuals = []

        # -- Figure setup --
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.canvas.manager.set_window_title(
            'PolyCast — UWB Position Validator')

        ax = self.ax
        ax.set_xlim(-0.25, float(np.max(ANCHORS[:, 0])) + 0.25)
        ax.set_ylim(-0.25, float(np.max(ANCHORS[:, 1])) + 0.25)
        ax.set_aspect('equal')
        ax.set_xlabel('X  (metres)')
        ax.set_ylabel('Y  (metres)')
        ax.set_title('UWB-Only Position (IRLS Trilateration)')
        ax.grid(True, alpha=0.3)

        # Anchor markers
        ax.scatter(ANCHORS[:, 0], ANCHORS[:, 1],
                   s=200, c='red', marker='s', zorder=10)
        for i, a in enumerate(ANCHORS):
            ax.annotate(f'A{i}', (a[0], a[1]),
                        textcoords='offset points', xytext=(0, 12),
                        ha='center', fontsize=11, fontweight='bold',
                        color='red')

        # Ground truth marker
        if self.TRUTH_XY is not None:
            ax.plot(self.TRUTH_XY[0], self.TRUTH_XY[1], '+',
                    color='limegreen', markersize=20, markeredgewidth=3,
                    zorder=9, label='Ground truth')
            ax.legend(loc='upper right')

        # Scatter for IRLS positions
        self._scatter = ax.scatter([], [], s=12, c='royalblue', alpha=0.5,
                                   zorder=5)

        # Stats text
        self._stats_text = ax.text(
            0.02, 0.02, '', transform=ax.transAxes,
            fontsize=9, fontfamily='monospace', verticalalignment='bottom',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    def update_plot(self, _frame):
        if self.finished:
            return

        while True:
            pkt = self.parser.get_packet()
            
            if pkt == 'EOF':
                self.finished = True
                break
            if pkt is None: # No more packets in buffer for now
                break
            if not isinstance(pkt, dict) or pkt['type'] != 'uwb':
                continue

            # Process UWB distances
            filtered, weights, despiked = self.uwb_cleaner.process(*pkt['dists'])

            # Solve IRLS trilateration
            pos_xyz, residuals = self.irls.solve(despiked, weights)
            px, py = float(pos_xyz[0]), float(pos_xyz[1])

            # Basic bounds check
            board_max = float(np.max(ANCHORS[:, :2])) + 0.30
            if -0.30 <= px <= board_max and -0.30 <= py <= board_max:
                self._xs.append(px)
                self._ys.append(py)
                self._residuals.append(residuals)
                
        # 5. Keep the scatter plot fast by limiting the number of dots drawn
        # Only keep the last 100 points for the 'live' view
        if len(self._xs) > 100:
            self._xs = self._xs[-100:]
            self._ys = self._ys[-100:]

        # Update scatter
        if self._xs:
            self._scatter.set_offsets(np.column_stack([self._xs, self._ys]))

        # Update stats
        n = len(self._xs)
        if n > 0:
            mean_x = np.mean(self._xs)
            mean_y = np.mean(self._ys)
            dists_from_mean = np.sqrt(
                (np.array(self._xs) - mean_x)**2 +
                (np.array(self._ys) - mean_y)**2)
            r95 = np.percentile(dists_from_mean, 95) if n > 5 else 0.0
            std_x = np.std(self._xs)
            std_y = np.std(self._ys)

            lines = [
                f'Points: {n}',
                f'Mean:   ({mean_x:.3f}, {mean_y:.3f})',
                f'Std:    ({std_x:.3f}, {std_y:.3f})',
                f'R95:    {r95*100:.1f} cm',
            ]

            if self.TRUTH_XY is not None:
                bias = np.sqrt((mean_x - self.TRUTH_XY[0])**2 +
                               (mean_y - self.TRUTH_XY[1])**2)
                lines.append(f'Bias:   {bias*100:.1f} cm')

            self._stats_text.set_text('\n'.join(lines))

    def print_final_summary(self):
        n = len(self._xs)
        print()
        print('=' * 55)
        print('  UWB POSITION VALIDATOR — SESSION SUMMARY')
        print('=' * 55)
        if n == 0:
            print('  No valid IRLS solutions collected.')
            print('=' * 55)
            return

        mean_x = np.mean(self._xs)
        mean_y = np.mean(self._ys)
        std_x  = np.std(self._xs)
        std_y  = np.std(self._ys)
        dists_from_mean = np.sqrt(
            (np.array(self._xs) - mean_x)**2 +
            (np.array(self._ys) - mean_y)**2)
        r95 = np.percentile(dists_from_mean, 95)

        print(f'  Points collected: {n}')
        print(f'  Mean position:    ({mean_x:.4f}, {mean_y:.4f})')
        print(f'  Std (x, y):       ({std_x:.4f}, {std_y:.4f})')
        print(f'  95th-pctl radius: {r95*100:.1f} cm')

        if self.TRUTH_XY is not None:
            bias = np.sqrt((mean_x - self.TRUTH_XY[0])**2 +
                           (mean_y - self.TRUTH_XY[1])**2)
            print(f'  Ground truth:     ({self.TRUTH_XY[0]:.3f}, {self.TRUTH_XY[1]:.3f})')
            print(f'  Bias from truth:  {bias*100:.1f} cm')

        # Per-anchor mean absolute residual
        if self._residuals:
            res_arr = np.array(self._residuals)
            mar = np.mean(np.abs(res_arr), axis=0)
            print(f'  Mean |residual|:  '
                  f'[{", ".join(f"{r*100:.1f} cm" for r in mar)}]')

        print('=' * 55)
        print()


def main():
    parser = AsyncDataParser(port=SERIAL_PORT, baud=BAUD_RATE,
                             csv_path=DATASET_FILENAME)
    if not parser.connect():
        sys.exit(1)

    uwb_cleaner = UWBPreprocessor(offsets=UWB_OFFSETS)

    bmin = [-0.30, -0.30, -0.50]
    bmax = [float(np.max(ANCHORS[:, 0])) + 0.30,
            float(np.max(ANCHORS[:, 1])) + 0.30,
            1.00]
    irls = IRLSTrilateration(ANCHORS, bmin, bmax, tag_z=MARKER_LENGTH)

    dashboard = UWBPositionDashboard(parser, uwb_cleaner, irls, TRUTH_XY)

    mode_str = 'CSV Playback' if parser.mode == 'csv' else 'Live Serial'
    print(f'[UWB Position Validator] Running in {mode_str} mode')
    if TRUTH_XY:
        print(f'[UWB Position Validator] Ground truth: ({TRUTH_XY[0]:.3f}, {TRUTH_XY[1]:.3f})')
    print('Close window to generate summary.')

    _ani = FuncAnimation(dashboard.fig, dashboard.update_plot,
                         interval=30, blit=False, cache_frame_data=False)
    plt.show()

    parser.close()
    dashboard.print_final_summary()


if __name__ == '__main__':
    main()
