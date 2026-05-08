import threading, time, json, serial

# --- CONFIG & LOGGING ---
from config import prototype_config, CONN_LOG, ERROR_LOG, CANVAS_WIDTH, CANVAS_HEIGHT
from app.utils.utils import log_message

# --- GRAPHICS ENGINE ---
from background.image_generator import draw_segment, image_lock, logical_to_pixel

# --- LOGIC ENGINE: EKF blue-trace stroke-coordinate provider ---
# Drop tracker.py into background/pipelines/tracker.py.
from background.pipelines.tracker import StrokeTracker

# --- BENCHMARK (opt-in via BENCHMARK=1 env var) ---
from benchmark import get_logger as _get_bm_logger


class PrototypeSerialThread(threading.Thread):
    def __init__(self, stop_event, ws_server=None):
        super().__init__(daemon=True)
        self.stop_event = stop_event
        self.ws_server = ws_server
        self.serial_conn = None
        self.tracker = StrokeTracker()  # EKF fusion + calibrated pen-tip XY
        self._bm = _get_bm_logger()     # Benchmark logger (no-op when BENCHMARK != 1)
        self.last_point = None
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
    def _unpack_tracker_result(result):
        """Accept the legacy tuple or a future dict from the tracker."""
        if isinstance(result, dict):
            meter_x = result.get('x', result.get('meter_x'))
            meter_y = result.get('y', result.get('meter_y'))
            is_drawing = bool(result.get('is_drawing', result.get('drawing', False)))
            stroke_state = result.get('stroke_state', result.get('state', 'AIR_MOVE'))
            return float(meter_x), float(meter_y), is_drawing, str(stroke_state)
        meter_x, meter_y, is_drawing, stroke_state = result
        return float(meter_x), float(meter_y), bool(is_drawing), str(stroke_state)

    def run(self):
        while not self.stop_event.is_set():
            try:
                print(f"[SERIAL] Connecting to {self.port} @ {self.baud}...")
                self.serial_conn = serial.Serial(self.port, self.baud, timeout=1)
                self.serial_conn.flushInput()
                log_message(CONN_LOG, "[SERIAL] Connected")
                self._bm.on_connection_event("CONNECTED")

                while not self.stop_event.is_set() and self.serial_conn.is_open:
                    if self.serial_conn.in_waiting:
                        try:
                            line = self.serial_conn.readline()
                            if not line:
                                continue

                            _t_recv = time.perf_counter()
                            self._bm.on_packet_recv(line, _t_recv)

                            # --- STEP 1: EKF PIPELINE (Meters) ---
                            # UWB packets update the EKF and usually return None.
                            # IMU packets emit the calibrated pen-tip XY used by
                            # the blue trace in main_ekf.py.
                            result = self.tracker.process_packet(line)
                            self._bm.on_fusion_done(line, time.perf_counter(), result)

                            if result is None:
                                continue

                            meter_x, meter_y, is_drawing, stroke_state = self._unpack_tracker_result(result)

                            # --- STEP 2: GRAPHICS (Pixels) ---
                            px, py = self._map_meters_to_pixels(meter_x, meter_y)

                            # Gate on physical contact, not a debounced stroke-active flag.
                            # Pen-up samples reset last_point so the next real contact does
                            # not connect through hover/air movement.
                            in_contact = stroke_state in ('CONTACT_DRAWING', 'CONTACT_STATIC')

                            with image_lock:
                                if not in_contact:
                                    self.last_point = None
                                else:
                                    if self.last_point and is_drawing:
                                        draw_segment(self.last_point[0], self.last_point[1], px, py, self.xpressure)
                                        print(f"[DRAW] ({self.last_point[0]},{self.last_point[1]}) → ({px},{py}) [{stroke_state}]")
                                    self.last_point = (px, py)

                            # --- STEP 3: BROADCAST (Web) ---
                            if self.ws_server and in_contact:
                                mjpeg_x, mjpeg_y = logical_to_pixel(px, py)
                                payload = {"x": mjpeg_x, "y": mjpeg_y, "p": self.xpressure, "state": stroke_state}
                                self.ws_server.send_message_to_all(json.dumps(payload))

                        except Exception as e:
                            print(f"[SERIAL] Data processing error: {e}")
                            self._bm.on_error(str(e))

                    else:
                        time.sleep(0.001)  # Sleep if buffer empty

            except Exception as e:
                print(f"[SERIAL] Connection Error: {e}")
                log_message(ERROR_LOG, f"[SERIAL] Connection Error: {e}")
                self._bm.on_connection_event(f"DISCONNECTED: {e}")
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.close()
                time.sleep(2)  # Reconnect delay

        if self.serial_conn:
            self.serial_conn.close()
        print("[SERIAL] Thread exited.")

    def stop(self):
        self.stop_event.set()
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
