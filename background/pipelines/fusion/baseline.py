"""
Module 7 - Baseline complementary-filter fusion.

Simpler alternative to the ESKF, kept for A/B comparison against the production
filter. Same process_event() interface, so the two are interchangeable.

Approach:
    IMU      integrates acceleration to velocity to position for fast local detail
    UWB      pulls position toward the absolute fix by a fixed alpha, killing drift
    Contact  gates which samples count as drawn ink, supplied upstream

Unlike the ESKF there is no covariance, no measurement gating and no lever-arm
correction: UWB is trusted at a constant weight regardless of solve quality.

Run directly for a live hardware dashboard, useful for A/B comparison against
`python -m background.pipelines.fusion.eskf` on the same motion:
    python -m background.pipelines.fusion.baseline
"""

from background.pipelines.config import cfg

# AnchorConfig exposes board_size_x, not board_size_m, so these lookups always
# fall through to the literal defaults below. Preserved deliberately: the tuned
# baseline results this engine is compared against were produced with these
# values, and "fixing" the attribute name would silently change the board size.
_DEFAULT_BOARD_WIDTH_M = 1.25
_DEFAULT_BOARD_HEIGHT_M = 1.24

# Nominal step used before the first timestamp pair and whenever the measured
# delta is non-monotonic or implausibly long.
_FALLBACK_DT_S = 0.02
_MAX_PLAUSIBLE_DT_S = 0.5

STATE_UWB_CORRECTION = 'UWB_CORRECTION'


class FusionEngine:
    """Complementary filter fusing IMU integration with absolute UWB position."""

    def __init__(self, fusion_alpha: float = 0.15):
        self.board_width = getattr(cfg.anchors, 'board_size_m', _DEFAULT_BOARD_WIDTH_M)
        self.board_height = getattr(cfg.anchors, 'board_size_y', _DEFAULT_BOARD_HEIGHT_M)

        self.pos_x = self.board_width / 2.0
        self.pos_y = self.board_height / 2.0
        self.vel_x = 0.0
        self.vel_y = 0.0

        # Share of each UWB fix blended into position. Higher tracks UWB more
        # closely at the cost of importing its jitter into the stroke.
        self.fusion_alpha = fusion_alpha

        self.last_ts = None
        self.last_uwb_x = self.pos_x
        self.last_uwb_y = self.pos_y

    def process_event(self, event: dict) -> dict | None:
        """
        Fuse one preprocessed IMU or UWB event and return the fused position.

        Returns None for events without a timestamp or from an unknown sensor.
        """

        ts = event.get('ts_hw')
        if ts is None:
            return None

        dt_s = self._elapsed_seconds(ts)
        self.last_ts = ts

        sensor = event.get('sensor')
        if sensor == 'IMU':
            state_label, stroke_id, stroke_active = self._integrate_imu(event, dt_s)
        elif sensor == 'POSITION':
            state_label, stroke_id, stroke_active = self._correct_with_uwb(event)
        else:
            return None

        self._clamp_to_board()

        return {
            'ts_hw': ts,
            'source': sensor,
            'fused_x': self.pos_x,
            'fused_y': self.pos_y,
            'uwb_x': self.last_uwb_x,
            'uwb_y': self.last_uwb_y,
            'state': state_label,
            'stroke_id': stroke_id,
            'stroke_active': stroke_active,
        }

    def _elapsed_seconds(self, ts: int) -> float:
        """Seconds since the previous event, falling back on implausible deltas."""

        if self.last_ts is None:
            return _FALLBACK_DT_S

        dt_s = (ts - self.last_ts) / 1_000_000.0
        if dt_s <= 0 or dt_s > _MAX_PLAUSIBLE_DT_S:
            return _FALLBACK_DT_S
        return dt_s

    def _integrate_imu(self, event: dict, dt_s: float) -> tuple[str, int, bool]:
        """Integrate acceleration into velocity and position, honouring ZUPT."""

        if event.get('is_static', False):
            # Hard stop rather than decay: without a covariance to reflect
            # growing uncertainty, this is the only brake on integration drift.
            self.vel_x = 0.0
            self.vel_y = 0.0
        else:
            acc_x, acc_y = event.get('acc_board', (0.0, 0.0))
            self.vel_x += acc_x * dt_s
            self.vel_y += acc_y * dt_s

        self.pos_x += self.vel_x * dt_s
        self.pos_y += self.vel_y * dt_s

        return (
            event.get('stroke_state', 'UNKNOWN'),
            event.get('stroke_id', 0),
            event.get('stroke_active', False),
        )

    def _correct_with_uwb(self, event: dict) -> tuple[str, int, bool]:
        """Blend the absolute UWB fix into position at the fixed alpha."""

        uwb_x, uwb_y = event.get('pos_clean', (self.pos_x, self.pos_y))
        self.last_uwb_x, self.last_uwb_y = uwb_x, uwb_y

        self.pos_x += self.fusion_alpha * (uwb_x - self.pos_x)
        self.pos_y += self.fusion_alpha * (uwb_y - self.pos_y)

        # UWB carries no contact information, so stroke fields stay neutral and
        # reconstruct.py lays no ink on these events.
        return STATE_UWB_CORRECTION, 0, False

    def _clamp_to_board(self) -> None:
        self.pos_x = max(0.0, min(self.board_width, self.pos_x))
        self.pos_y = max(0.0, min(self.board_height, self.pos_y))


if __name__ == '__main__':
    import os
    import time

    os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

    import matplotlib.pyplot as plt

    from background.pipelines.cleaner.normalizer import StreamNormalizer
    from background.pipelines.cleaner.time_alignment import TimeAlignLayer
    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.module_output import ModuleRunOutput
    from background.pipelines.preprocess.contact import ContactStateDetector
    from background.pipelines.preprocess.imu import IMUPreprocessor
    from background.pipelines.preprocess.uwb.position import UWBPositionFilter
    from background.pipelines.preprocess.uwb.range import UWBRangePreprocessor
    from background.pipelines.preprocess.uwb.trilateration import UWBSolver

    DISPLAY_RATE_S = 0.1
    REPORT_NAME = 'fusion_baseline_report'

    def export_plot(output: 'ModuleRunOutput', event_log: list[dict]) -> None:
        """Plot the fused ink path against the raw UWB reference."""

        ink_events = [fused for fused in event_log if fused['stroke_active']]
        if not ink_events:
            print("\n[WARNING] No active drawing strokes detected. Showing all movement instead.")
            ink_events = event_log

        fused_x = [fused['fused_x'] for fused in ink_events]
        fused_y = [fused['fused_y'] for fused in ink_events]

        # IMU rows only copy the last UWB fix forward, so plotting every row
        # would draw the same point repeatedly instead of the actual UWB path.
        uwb_events = [fused for fused in ink_events if fused['source'] == 'POSITION']
        uwb_x = [fused['uwb_x'] for fused in uwb_events]
        uwb_y = [fused['uwb_y'] for fused in uwb_events]

        board_width = getattr(cfg.anchors, 'board_size_m', _DEFAULT_BOARD_WIDTH_M)
        board_height = getattr(cfg.anchors, 'board_size_y', _DEFAULT_BOARD_HEIGHT_M)

        figure = plt.figure(figsize=(12, 12))
        figure.suptitle('Module 7: IMU + UWB Sensor Fusion (Baseline v1)', fontsize=16, fontweight='bold')

        axes = plt.subplot(1, 1, 1)
        axes.add_patch(plt.Rectangle(
            (0, 0), board_width, board_height,
            fill=False, edgecolor='black', linestyle='--', lw=2,
        ))
        axes.plot(uwb_x, uwb_y, label='UWB Reference (Slower, Jagged)',
                  color='blue', alpha=0.4, marker='.', linestyle='dashed')
        axes.plot(fused_x, fused_y, label='Fused Kinematic Output (Smooth, Corrected)',
                  color='orange', linewidth=2.5)

        axes.set_xlim(-0.1, board_width + 0.1)
        axes.set_ylim(-0.1, board_height + 0.1)
        axes.set_aspect('equal')
        axes.set_title('Spatial Trajectory Validation')
        axes.grid(True, linestyle=':', alpha=0.7)
        axes.legend(loc='upper right')

        plt.tight_layout()
        plot_filename = f"{REPORT_NAME}.png"
        figure.savefig(output.path(plot_filename), dpi=300)
        output.record(plot_filename)
        print(f"[EXPORT] Plot saved -> {plot_filename}")

    port = getattr(cfg.serial, 'port', 'COM20')
    baud = getattr(cfg.serial, 'baud', 115200)

    streamer = SerialStreamer(port=port, baud=baud)
    normalizer = StreamNormalizer()
    aligner = TimeAlignLayer(buffer_size=500)

    imu_prep = IMUPreprocessor()
    contact = ContactStateDetector()

    range_prep = UWBRangePreprocessor(
        offsets=getattr(cfg.uwb, 'range_offsets_m', (0.0, 0.0, 0.0, 0.0))
    )
    trilateration = UWBSolver()
    position_filter = UWBPositionFilter()

    fusion = FusionEngine(fusion_alpha=0.15)

    print("=" * 60)
    print(f"  [TEST] MODULE 7: MASTER FUSION ENGINE: {port}")
    print("  Ready for Tests F1, F2, F3, and F4. Press Ctrl+C to stop.")
    print("=" * 60)

    event_log: list[dict] = []
    last_render = 0.0

    try:
        while True:
            raw_packets = streamer.read_new_packets()
            if raw_packets:
                aligner.add_events(normalizer.normalize(raw_packets))
                sorted_events = aligner.get_all_sorted()
                aligner.clear()

                for event in sorted_events:
                    if event['sensor'] == 'IMU':
                        preprocessed = imu_prep.process_one(event)
                        if not preprocessed:
                            continue
                        with_contact = contact.process_one(preprocessed)
                        fused = fusion.process_event(with_contact)
                        if fused:
                            event_log.append(fused)

                    elif event['sensor'] == 'UWB':
                        for ranged in range_prep.feed([event]):
                            solved = trilateration.process_one(ranged)
                            if not solved:
                                continue
                            positioned = position_filter.process_one(solved)
                            if not positioned:
                                continue
                            fused = fusion.process_event(positioned)
                            if fused:
                                event_log.append(fused)

            now = time.time()
            if event_log and (now - last_render) >= DISPLAY_RATE_S:
                latest = event_log[-1]
                os.system('cls' if os.name == 'nt' else 'clear')
                print(f"========= LIVE FUSION ({DISPLAY_RATE_S}s) =========")
                print(f"  State      : {latest['state']}")
                print(f"  Stroke ID  : {latest['stroke_id']} (Active: {latest['stroke_active']})")
                print("-" * 50)
                print(f"  Fused Pos  : X: {latest['fused_x']:6.3f} m | Y: {latest['fused_y']:6.3f} m")
                print(f"  UWB Anchor : X: {latest['uwb_x']:6.3f} m | Y: {latest['uwb_y']:6.3f} m")
                print("=================================================")
                last_render = now

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[STOP] Data collection halted.")
        streamer.close()

        if not event_log:
            print("No data collected. Exiting.")
            raise SystemExit(0)

        output = ModuleRunOutput('fusion/baseline')
        output.save_csv(
            f"{REPORT_NAME}.csv",
            [[fused['ts_hw'], fused['source'], fused['fused_x'], fused['fused_y'],
              fused['uwb_x'], fused['uwb_y'], fused['state'], fused['stroke_id'],
              int(fused['stroke_active'])]
             for fused in event_log],
            header=['ts_hw', 'source', 'fused_x', 'fused_y',
                    'uwb_x', 'uwb_y', 'state', 'stroke_id', 'stroke_active'],
        )

        print("[PLOT] Rendering Fusion Performance Report...")
        try:
            export_plot(output, event_log)
        except Exception as error:
            print(f"\n[ERROR] Plotting failed: {error}")

        output.finish()
