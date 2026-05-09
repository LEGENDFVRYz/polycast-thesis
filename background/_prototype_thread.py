import math
import threading, time, json, serial
from queue import Queue, Empty, Full

# --- CONFIG & LOGGING ---
from config import prototype_config, CONN_LOG, ERROR_LOG, CANVAS_WIDTH, CANVAS_HEIGHT, IS_PROD
from app.utils.utils import log_message

# --- GRAPHICS ENGINE ---
from background.image_generator import draw_segment, image_lock, logical_to_pixel

# --- LOGIC ENGINE: EKF blue-trace stroke-coordinate provider ---
# Drop tracker.py into background/pipelines/tracker.py.
from background.pipelines.tracker import StrokeTracker

# --- ONLINE TRAIL SMOOTHER (mirrors main_ekf.py's causal 5-point WMA) ---
from kuru_method.asynchronous_stream.trail_smoother import TrailSmoother

# --- BENCHMARK (opt-in via BENCHMARK=1 env var) ---
from benchmark import get_logger as _get_bm_logger

# Relocation break guard: matches AUTO_BREAK_RELOCATION / RELOC_SEGMENT_M in main_ekf.py.
# Inserts a stroke break when the smoothed tip jumps > 4.5 cm in one IMU step
# while contact is active (FSR stuck during a pen reposition).
_RELOC_SEGMENT_M = 0.045

# Bounded queue between the serial reader thread and the EKF consumer thread.
# Sized to absorb a short EKF stall (~1 s of IMU traffic at 100 Hz) without
# unbounded memory growth. When the consumer falls behind we drop the OLDEST
# packet so the worker stays close to live data instead of replaying a stale
# backlog — essential on the Raspberry Pi where the EKF can't always keep up.
_PACKET_QUEUE_MAX = 256

# Cap WebSocket broadcast frequency on the consumer hot path. The browser
# debug overlay only needs ~60 Hz; sending one message per IMU sample wastes
# CPU and starves the EKF on the Pi.
_WS_BROADCAST_HZ = 60.0
_WS_BROADCAST_INTERVAL = 1.0 / _WS_BROADCAST_HZ

# Tracker result keys that should be treated as the black
# Recognition/extraction overlay coordinates shown by main_ekf.py's line_norm.
# The thread falls back to pen-tip x/y if none of these are present.
_BLACK_POINT_KEYS = (
    "line_norm", "normalized_preview", "recognition_extraction_overlay",
    "recognition_overlay", "extraction_overlay", "extraction", "recognized",
    "recognition", "stroke", "render", "draw", "output", "black",
)
_BLACK_XY_KEY_PAIRS = (
    ("line_norm_x", "line_norm_y"),
    ("norm_x", "norm_y"),
    ("normalized_x", "normalized_y"),
    ("extract_x", "extract_y"),
    ("extraction_x", "extraction_y"),
    ("recognition_x", "recognition_y"),
    ("recog_x", "recog_y"),
    ("stroke_x", "stroke_y"),
    ("render_x", "render_y"),
    ("draw_x", "draw_y"),
    ("output_x", "output_y"),
    ("black_x", "black_y"),
)


class PrototypeSerialThread(threading.Thread):
    def __init__(self, stop_event, ws_server=None, prefer_extraction=True):
        super().__init__(daemon=True)
        self.stop_event = stop_event
        self.ws_server = ws_server
        self.prefer_extraction = prefer_extraction
        self.serial_conn = None
        self.tracker = StrokeTracker()  # EKF fusion + calibrated pen-tip XY
        self._bm = _get_bm_logger()     # Benchmark logger (no-op when BENCHMARK != 1)
        self.smoother = TrailSmoother() # 5-point causal WMA, same kernel as main_ekf.py
        self.last_point = None
        self._last_draw_m = None        # last rendered output coordinate in metres
        self._last_coord_source = None
        self.xpressure = 8              # temporary

        # Hardware Config
        self.port = prototype_config.SERIAL_PORT
        self.baud = prototype_config.BAUD_RATE

        # Physical board bounds supplied by the EKF tracker.
        # Supports both the old square {b_min,b_max} shape and the new
        # independent {x_min,x_max,y_min,y_max} shape.
        bounds = self.tracker.get_bbox()
        b_min = bounds.get('b_min', 0.0)
        b_max = bounds.get('b_max', 1.0)
        self.p_min_x = float(bounds.get('x_min', b_min))
        self.p_max_x = float(bounds.get('x_max', b_max))
        self.p_min_y = float(bounds.get('y_min', b_min))
        self.p_max_y = float(bounds.get('y_max', b_max))

        self.p_width = max(1e-9, self.p_max_x - self.p_min_x)
        self.p_height = max(1e-9, self.p_max_y - self.p_min_y)

        # Reader/consumer split: the reader thread does nothing but drain the
        # serial port into this queue; the main thread (run()) consumes it.
        self._packet_queue: Queue = Queue(maxsize=_PACKET_QUEUE_MAX)
        self._reader_thread = None
        self._reader_stop = threading.Event()
        self._reader_failed = False
        self._dropped_packets = 0
        self._last_ws_send_ts = 0.0

    def _map_meters_to_pixels(self, mx, my):
        """
        Translates Physical World (Meters) -> Digital World (Master Canvas Pixels)
        """
        # 1. Normalize to 0.0 - 1.0 based on Tracker Config
        norm_x = (float(mx) - self.p_min_x) / self.p_width
        norm_y = (float(my) - self.p_min_y) / self.p_height

        # 2. Scale to Master Canvas Resolution (e.g., 4K)
        px = int(norm_x * CANVAS_WIDTH)
        py = int((1.0 - norm_y) * CANVAS_HEIGHT)     # inverse, since image origin is at top-left

        # 3. Clamp to screen edges
        px = max(0, min(CANVAS_WIDTH - 1, px))
        py = max(0, min(CANVAS_HEIGHT - 1, py))

        return px, py

    @staticmethod
    def _point_from_value(value):
        """Extract a numeric (x, y) point from a tuple/list/dict value."""
        if value is None:
            return None
        if isinstance(value, dict):
            x = PrototypeSerialThread._first_number(value, (
                'x', 'meter_x', 'line_norm_x', 'norm_x', 'extract_x',
                'extraction_x', 'recognition_x', 'stroke_x', 'render_x',
                'draw_x', 'output_x', 'black_x'
            ))
            y = PrototypeSerialThread._first_number(value, (
                'y', 'meter_y', 'line_norm_y', 'norm_y', 'extract_y',
                'extraction_y', 'recognition_y', 'stroke_y', 'render_y',
                'draw_y', 'output_y', 'black_y'
            ))
            if x is not None and y is not None:
                return x, y
            return None
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            try:
                return float(value[0]), float(value[1])
            except (TypeError, ValueError):
                return None
        return None

    @staticmethod
    def _first_number(mapping, keys):
        for key in keys:
            if key in mapping and mapping[key] is not None:
                try:
                    return float(mapping[key])
                except (TypeError, ValueError):
                    pass
        return None

    @staticmethod
    def _pick_black_output_point(result, fallback_x, fallback_y, prefer_extraction=True):
        """Pick the coordinate that should be rendered as the black output line."""
        if not isinstance(result, dict) or not prefer_extraction:
            return fallback_x, fallback_y, 'pen_tip'

        # Nested point style: {'line_norm': {'x': ..., 'y': ...}} or {'line_norm': (x, y)}.
        for key in _BLACK_POINT_KEYS:
            if key in result:
                point = PrototypeSerialThread._point_from_value(result.get(key))
                if point is not None:
                    return point[0], point[1], key

        # Flat key style: {'norm_x': ..., 'norm_y': ...}.
        for x_key, y_key in _BLACK_XY_KEY_PAIRS:
            x = PrototypeSerialThread._first_number(result, (x_key,))
            y = PrototypeSerialThread._first_number(result, (y_key,))
            if x is not None and y is not None:
                return x, y, x_key.rsplit('_', 1)[0]

        return fallback_x, fallback_y, 'pen_tip'

    @staticmethod
    def _unpack_tracker_result(result, prefer_extraction=True):
        """Accept the legacy tuple or a dict with black-line output coordinates.

        Returns:
            raw_meter_x, raw_meter_y, output_meter_x, output_meter_y,
            is_drawing, stroke_state, coord_source
        """
        if isinstance(result, dict):
            raw_x = PrototypeSerialThread._first_number(result, (
                'x', 'meter_x', 'tip_x', 'pen_x', 'raw_x'
            ))
            raw_y = PrototypeSerialThread._first_number(result, (
                'y', 'meter_y', 'tip_y', 'pen_y', 'raw_y'
            ))
            out_x, out_y, coord_source = PrototypeSerialThread._pick_black_output_point(
                result, raw_x, raw_y, prefer_extraction=prefer_extraction
            )

            # Some trackers may only emit the extracted/black coordinate. Use it
            # as the raw coordinate too so bounds mapping still works.
            if raw_x is None:
                raw_x = out_x
            if raw_y is None:
                raw_y = out_y
            if raw_x is None or raw_y is None or out_x is None or out_y is None:
                raise ValueError(f"Tracker result missing coordinates: {result!r}")

            is_drawing = bool(result.get('is_drawing', result.get('drawing', False)))
            stroke_state = result.get('stroke_state', result.get('state', 'AIR_MOVE'))
            return (
                float(raw_x), float(raw_y), float(out_x), float(out_y),
                is_drawing, str(stroke_state), str(coord_source)
            )

        meter_x, meter_y, is_drawing, stroke_state = result
        return (
            float(meter_x), float(meter_y), float(meter_x), float(meter_y),
            bool(is_drawing), str(stroke_state), 'pen_tip'
        )

    def _reset_stroke_state(self):
        self.smoother.reset()
        self._last_draw_m = None
        self.last_point = None
        self._last_coord_source = None

    def run(self):
        while not self.stop_event.is_set():
            try:
                print(f"[SERIAL] Connecting to {self.port} @ {self.baud}...")
                self.serial_conn = serial.Serial(self.port, self.baud, timeout=1)
                self.serial_conn.flushInput()
                log_message(CONN_LOG, "[SERIAL] Connected")
                self._bm.on_connection_event("CONNECTED")

                self._drain_queue()
                self._reader_stop.clear()
                self._reader_failed = False
                self._reader_thread = threading.Thread(
                    target=self._reader_loop, daemon=True, name="serial-reader",
                )
                self._reader_thread.start()

                self._consume_loop()

                self._reader_stop.set()
                if self._reader_thread is not None and self._reader_thread.is_alive():
                    self._reader_thread.join(timeout=1.0)
                self._reader_thread = None

                # Reader exited because of an I/O fault — reconnect.
                if self._reader_failed and not self.stop_event.is_set():
                    if self.serial_conn and self.serial_conn.is_open:
                        self.serial_conn.close()
                    self._bm.on_connection_event("DISCONNECTED: serial read failed")
                    time.sleep(2)
                    continue

            except Exception as e:
                print(f"[SERIAL] Connection Error: {e}")
                log_message(ERROR_LOG, f"[SERIAL] Connection Error: {e}")
                self._bm.on_connection_event(f"DISCONNECTED: {e}")
                self._reader_stop.set()
                if self._reader_thread is not None and self._reader_thread.is_alive():
                    self._reader_thread.join(timeout=1.0)
                self._reader_thread = None
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.close()
                time.sleep(2)

        if self.serial_conn:
            self.serial_conn.close()
        print("[SERIAL] Thread exited.")

    # ------------------------------------------------------------------
    # Reader thread — keep this loop tiny so the OS serial buffer drains
    # ------------------------------------------------------------------
    def _reader_loop(self):
        try:
            while not self._reader_stop.is_set() and not self.stop_event.is_set():
                conn = self.serial_conn
                if conn is None or not conn.is_open:
                    break
                try:
                    if conn.in_waiting:
                        line = conn.readline()
                        if not line:
                            continue
                        try:
                            self._packet_queue.put_nowait(line)
                        except Full:
                            # Drop oldest, keep newest. Stale packets are worse
                            # than missing ones for live stroke rendering.
                            try:
                                self._packet_queue.get_nowait()
                                self._dropped_packets += 1
                            except Empty:
                                pass
                            try:
                                self._packet_queue.put_nowait(line)
                            except Full:
                                self._dropped_packets += 1
                    else:
                        time.sleep(0.001)
                except (OSError, serial.SerialException) as e:
                    print(f"[SERIAL] Reader error: {e}")
                    self._reader_failed = True
                    break
        finally:
            try:
                self._packet_queue.put_nowait(None)
            except Full:
                pass

    # ------------------------------------------------------------------
    # Consumer loop — runs on the main thread, blocks on the queue
    # ------------------------------------------------------------------
    def _consume_loop(self):
        while not self.stop_event.is_set():
            conn = self.serial_conn
            if conn is None or not conn.is_open:
                return
            try:
                line = self._packet_queue.get(timeout=0.5)
            except Empty:
                continue
            if line is None:
                return
            try:
                self._process_packet(line)
            except Exception as e:
                print(f"[SERIAL] Data processing error: {e}")
                self._bm.on_error(str(e))

    def _drain_queue(self):
        try:
            while True:
                self._packet_queue.get_nowait()
        except Empty:
            pass

    # ------------------------------------------------------------------
    # Per-packet processing (was inline in run())
    # ------------------------------------------------------------------
    def _process_packet(self, line):
        _t_recv = time.perf_counter()
        self._bm.on_packet_recv(line, _t_recv)

        # --- STEP 1: EKF PIPELINE (Meters) ---
        # UWB packets update the EKF and usually return None.
        # IMU packets emit either the calibrated pen-tip XY or, when available,
        # the extraction/line_norm XY that matches the black overlay in main_ekf.py.
        result = self.tracker.process_packet(line)
        self._bm.on_fusion_done(line, time.perf_counter(), result)

        if result is None:
            return

        (raw_meter_x, raw_meter_y,
         output_meter_x, output_meter_y,
         is_drawing, stroke_state,
         coord_source) = self._unpack_tracker_result(
            result, prefer_extraction=self.prefer_extraction
        )

        # Gate on physical contact, not a debounced stroke-active flag.
        in_contact = stroke_state in ('CONTACT_DRAWING', 'CONTACT_STATIC')
        px = py = None
        sx_m = sy_m = None

        # --- STEP 2: COMPUTE DRAW COMMAND (thread-local — no lock needed) ---
        draw_cmd = None
        if not in_contact:
            self._reset_stroke_state()
        else:
            if self._last_coord_source is not None and self._last_coord_source != coord_source:
                self.last_point = None
                self._last_draw_m = None
                self.smoother.reset()

            self._last_coord_source = coord_source

            if coord_source == 'pen_tip':
                # Legacy fallback: use the same online trail smoother as main_ekf.py.
                if self.last_point is None:
                    self.smoother.reset()
                self.smoother.push(raw_meter_x, raw_meter_y)
                sx_m, sy_m = self.smoother.get()
            else:
                # Black recognition/extraction coordinates are already output coordinates.
                # Do not smooth them again or the web render will no longer match line_norm.
                self.smoother.reset()
                sx_m, sy_m = output_meter_x, output_meter_y

            px, py = self._map_meters_to_pixels(sx_m, sy_m)

            # Relocation break guard — mirrors AUTO_BREAK_RELOCATION.
            if self._last_draw_m is not None and is_drawing:
                dx = sx_m - self._last_draw_m[0]
                dy = sy_m - self._last_draw_m[1]
                if math.sqrt(dx * dx + dy * dy) > _RELOC_SEGMENT_M:
                    self.last_point = None
                    self._last_draw_m = None
                    if coord_source == 'pen_tip':
                        self.smoother.reset()

            if self.last_point and is_drawing:
                meter_segment = (self._last_draw_m, (sx_m, sy_m)) if self._last_draw_m else None
                draw_cmd = (
                    self.last_point[0], self.last_point[1], px, py,
                    self.xpressure, meter_segment, coord_source, stroke_state,
                    self._last_draw_m, sx_m, sy_m,
                )

            self.last_point = (px, py)
            self._last_draw_m = (sx_m, sy_m)

        # --- STEP 3: DRAW (image_lock held only for the canvas mutation) ---
        if draw_cmd is not None:
            x0, y0, x1, y1, pressure, meter_seg, src, state, prev_m, cx_m, cy_m = draw_cmd
            with image_lock:
                draw_segment(x0, y0, x1, y1, pressure,
                             meter_segment=meter_seg,
                             source=src, state=state)
            if not IS_PROD:
                print(
                    f"[DRAW:{src}] "
                    f"m({prev_m[0]:.4f},{prev_m[1]:.4f}) "
                    f"→ m({cx_m:.4f},{cy_m:.4f}) | "
                    f"px({x0},{y0}) → ({x1},{y1}) "
                    f"[{state}]"
                )

        # --- STEP 4: BROADCAST (Web) — rate-limited so the EKF never starves ---
        if self.ws_server and in_contact and px is not None and py is not None:
            now = time.monotonic()
            if (now - self._last_ws_send_ts) >= _WS_BROADCAST_INTERVAL:
                self._last_ws_send_ts = now
                mjpeg_x, mjpeg_y = logical_to_pixel(px, py)
                payload = {
                    "x": mjpeg_x,
                    "y": mjpeg_y,
                    "canvas_x": px,
                    "canvas_y": py,
                    "meter_x": sx_m,
                    "meter_y": sy_m,
                    "raw_meter_x": raw_meter_x,
                    "raw_meter_y": raw_meter_y,
                    "p": self.xpressure,
                    "state": stroke_state,
                    "source": coord_source,
                }
                self.ws_server.send_message_to_all(json.dumps(payload))

    def stop(self):
        self.stop_event.set()
        self._reader_stop.set()
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
