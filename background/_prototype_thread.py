import threading, time, json, serial

# --- CONFIG & LOGGING ---
from config import prototype_config, CONN_LOG, ERROR_LOG, CANVAS_WIDTH, CANVAS_HEIGHT
from app.utils.utils import log_message

# --- GRAPHICS ENGINE ---
from background.image_generator import draw_segment, image_lock

# --- LOGIC ENGINE (Imported from the new file) ---
# from background.stroke_processor import StrokeTracker     # IF FINALIZED
from background.pipelines.tracker import StrokeTracker

# --- BENCHMARK (opt-in via BENCHMARK=1 env var) ---
from benchmark import get_logger as _get_bm_logger


class PrototypeSerialThread(threading.Thread):
    def __init__(self, stop_event, ws_server=None):
        super().__init__(daemon=True)
        self.stop_event = stop_event
        self.ws_server = ws_server
        self.serial_conn = None
        self.tracker = StrokeTracker()  # Initialize the Logic Engine
        self._bm = _get_bm_logger()     # Benchmark logger (no-op when BENCHMARK != 1)
        self.last_point = None
        self.xpressure = 8              # temporary 
        
        # Hardware Config
        self.port = prototype_config.SERIAL_PORT
        self.baud = prototype_config.BAUD_RATE
        
        # Get boundery box
        bounds = self.tracker.get_bbox()
        self.p_min_x = self.p_min_y = bounds['b_min']
        self.p_max_x = self.p_max_y = bounds['b_max']
        
        self.p_width = self.p_max_x - self.p_min_x
        self.p_height = self.p_max_y - self.p_min_y

    def _map_meters_to_pixels(self, mx, my):
        """
        Translates Physical World (Meters) -> Digital World (Master Canvas Pixels)
        """
        # 1. Normalize to 0.0 - 1.0 based on Tracker Config
        norm_x = (mx - self.p_min_x) / self.p_width
        norm_y = (my - self.p_min_y) / self.p_height
        
        # 2. Scale to Master Canvas Resolution (e.g., 4K)
        px = int(norm_x * CANVAS_WIDTH)
        py = int((1.0 - norm_y) * CANVAS_HEIGHT)     # inverse, since image origin is at top-left
        
        # 3. Clamp to screen edges
        px = max(0, min(CANVAS_WIDTH - 1, px))
        py = max(0, min(CANVAS_HEIGHT - 1, py))
        
        return px, py

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
                            if not line: continue

                            _t_recv = time.perf_counter()
                            self._bm.on_packet_recv(line, _t_recv)

                            # --- STEP 1: MATH (Meters) ---
                            # Hand off raw bytes to the processor
                            result = self.tracker.process_packet(line)
                            self._bm.on_fusion_done(line, time.perf_counter(), result)
                            
                            if result:
                                meter_x, meter_y, is_drawing = result
                                
                                # --- STEP 2: GRAPHICS (Pixels) ---
                                # Convert meters to pixels
                                px, py = self._map_meters_to_pixels(meter_x, meter_y)
                                
                                with image_lock:
                                    if not is_drawing:
                                        self.last_point = None
                                    else:
                                        if self.last_point:
                                            # Draw stroke (p=3 is "Pen Down")
                                            draw_segment(self.last_point[0], self.last_point[1], px, py, self.xpressure) 
                                        self.last_point = (px, py)

                                # --- STEP 3: BROADCAST (Web) ---
                                # if self.ws_server:
                                #     payload = {"x": px, "y": py, "p": self.xpressure}
                                #     self.ws_server.send_message_to_all(json.dumps(payload))
                                print(f"[IMAGE RECIEVED] ({px}, {py})")
                                
                        except Exception as e:
                            print(f"[SERIAL] Data processing error: {e}")
                            self._bm.on_error(str(e))
                            
                    else:
                        time.sleep(0.001) # Sleep if buffer empty

            except Exception as e:
                print(f"[SERIAL] Connection Error: {e}")
                log_message(ERROR_LOG, f"[SERIAL] Connection Error: {e}")
                self._bm.on_connection_event(f"DISCONNECTED: {e}")
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.close()
                time.sleep(2) # Reconnect delay

        if self.serial_conn:
            self.serial_conn.close()
        print("[SERIAL] Thread exited.")

    def stop(self):
        self.stop_event.set()
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()






