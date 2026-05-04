import threading, time, os, io, shutil
from datetime import datetime
from PIL import Image, ImageDraw
from config import (
    CANVAS_WIDTH, CANVAS_HEIGHT, MJPEG_WIDTH, MJPEG_HEIGHT,
    ARCHIVE_DIR,
    MJPEG_FPS, JPEG_QUALITY,
    MAX_STREAM_CLIENTS,
    AUTOSAVE_INTERVAL_SEC, AUTOSAVE_ONLY_WHEN_DIRTY, MIN_FREE_DISK_MB,
)

# --- Shared Resources ---
scale_x = MJPEG_WIDTH / CANVAS_WIDTH
scale_y = MJPEG_HEIGHT / CANVAS_HEIGHT
AVG_SCALE = (scale_x + scale_y) / 2.0
STROKE_FACTOR = 0.5

canvas = Image.new("L", (MJPEG_WIDTH, MJPEG_HEIGHT), "white")
draw = ImageDraw.Draw(canvas)
image_lock = threading.Lock()

# --- Frame cache (shared across all MJPEG clients) ---
stroke_seq = 0
last_encoded_seq = -1
latest_jpeg_bytes = b""
frame_lock = threading.Lock()

# --- Stream client cap ---
stream_client_count = 0
_stream_count_lock = threading.Lock()

# --- Encoder lifecycle ---
_encoder_thread = None
_encoder_stop = None

# --- Stats (for /api/status) ---
_last_encode_ts = 0.0
_recent_encode_intervals = []
_stats_lock = threading.Lock()


def _encode_canvas_jpeg(snap):
    buf = io.BytesIO()
    snap.convert("RGB").save(buf, format="JPEG", quality=JPEG_QUALITY)
    return buf.getvalue()


def _encoder_loop(stop_event):
    global last_encoded_seq, latest_jpeg_bytes, _last_encode_ts
    interval = 1.0 / max(1, MJPEG_FPS)

    try:
        from benchmark import get_logger as _get_bm_logger
        _bm = _get_bm_logger()
    except Exception:
        _bm = None

    try:
        with image_lock:
            snap = canvas.copy()
            seq = stroke_seq
        initial = _encode_canvas_jpeg(snap)
        with frame_lock:
            latest_jpeg_bytes = initial
            last_encoded_seq = seq
        print(f"[ENCODER] primed initial frame ({len(initial)} bytes)")
    except Exception as e:
        print(f"[ENCODER] prime failed: {e!r}")

    while not stop_event.is_set():
        try:
            tick_start = time.time()
            encode_ms = 0.0

            with image_lock:
                current_seq = stroke_seq
                need_encode = current_seq != last_encoded_seq
                snap = canvas.copy() if need_encode else None

            if need_encode and snap is not None:
                t0 = time.perf_counter()
                jpeg_bytes = _encode_canvas_jpeg(snap)
                encode_ms = (time.perf_counter() - t0) * 1000.0
                with frame_lock:
                    latest_jpeg_bytes = jpeg_bytes
                    last_encoded_seq = current_seq

                now = time.time()
                with _stats_lock:
                    if _last_encode_ts:
                        _recent_encode_intervals.append(now - _last_encode_ts)
                        if len(_recent_encode_intervals) > 30:
                            _recent_encode_intervals.pop(0)
                    _last_encode_ts = now

            elapsed = time.time() - tick_start
            sleep_for = max(0.0, interval - elapsed)
            if _bm is not None and need_encode:
                try:
                    _bm.on_software_perf(encode_ms=encode_ms, sleep_ms=sleep_for * 1000.0)
                except Exception:
                    pass

            if stop_event.wait(timeout=sleep_for):
                break
        except Exception as e:
            print(f"[ENCODER] loop error: {e!r}")
            if stop_event.wait(timeout=interval):
                break


def start_encoder_thread():
    """Idempotent: start the single encoder thread for this process."""
    global _encoder_thread, _encoder_stop
    if _encoder_thread is not None and _encoder_thread.is_alive():
        return
    _encoder_stop = threading.Event()
    _encoder_thread = threading.Thread(
        target=_encoder_loop, args=(_encoder_stop,), daemon=True, name="mjpeg-encoder"
    )
    _encoder_thread.start()
    print(f"[ENCODER] started @ {MJPEG_FPS} fps, JPEG quality={JPEG_QUALITY}")


def stop_encoder_thread():
    global _encoder_thread, _encoder_stop
    if _encoder_stop is not None:
        _encoder_stop.set()
    if _encoder_thread is not None:
        _encoder_thread.join(timeout=2.0)
    _encoder_thread = None
    _encoder_stop = None


def try_register_stream_client():
    """Atomically reserve a stream slot. Returns True on success, False if cap reached."""
    global stream_client_count
    with _stream_count_lock:
        if stream_client_count >= MAX_STREAM_CLIENTS:
            return False
        stream_client_count += 1
        return True


def unregister_stream_client():
    global stream_client_count
    with _stream_count_lock:
        if stream_client_count > 0:
            stream_client_count -= 1


def get_stream_client_count():
    with _stream_count_lock:
        return stream_client_count


def get_renderer_fps():
    with _stats_lock:
        if not _recent_encode_intervals:
            return 0.0
        avg = sum(_recent_encode_intervals) / len(_recent_encode_intervals)
        return 0.0 if avg <= 0 else 1.0 / avg


def get_last_encoded_ts():
    with _stats_lock:
        return _last_encode_ts


# --- Archiver Class ---
class Archiver:
    """
    Manages the archiving thread.
    The thread starts archiving immediately and stops when told.
    """
    def __init__(self, initial_archive_path):
        self.archive_path = initial_archive_path
        self.thread = None
        self.stop_event = None
        self.last_saved_seq = -1
        self.last_saved_ts = 0.0

    def _disk_free_mb(self):
        try:
            return shutil.disk_usage(self.archive_path).free // (1024 * 1024)
        except Exception:
            return None

    def _save_snapshot(self, snap):
        ts = datetime.now().strftime("%Y%m%d-%H%M%S")
        final_path = os.path.join(self.archive_path, f"note_{ts}.jpg")
        tmp_path = final_path + ".tmp"
        try:
            snap.convert("RGB").save(tmp_path, format="JPEG", quality=JPEG_QUALITY)
            os.replace(tmp_path, final_path)
            self.last_saved_ts = time.time()
            print(f"[ARCHIVER] saved {final_path}")
            return True
        except Exception as e:
            print(f"[ARCHIVER] save error: {e}")
            try:
                if os.path.exists(tmp_path):
                    os.remove(tmp_path)
            except OSError:
                pass
            return False

    def _run_archiver(self):
        print(f"✅ THREAD: Started. Archiving to: {self.archive_path}")

        try:
            os.makedirs(self.archive_path, exist_ok=True)
        except Exception as e:
            print(f"❌ THREAD: Could not create directory {self.archive_path}. Error: {e}")
            print("🛑 THREAD: Exiting.")
            return

        while not self.stop_event.is_set():
            try:
                with image_lock:
                    current_seq = stroke_seq
                    is_dirty = current_seq != self.last_saved_seq
                    snap = canvas.copy() if (is_dirty or not AUTOSAVE_ONLY_WHEN_DIRTY) else None

                if snap is not None:
                    free_mb = self._disk_free_mb()
                    if free_mb is not None and free_mb < MIN_FREE_DISK_MB:
                        print(f"[ARCHIVER] free disk {free_mb} MB < {MIN_FREE_DISK_MB} MB — skipping save")
                    else:
                        if self._save_snapshot(snap):
                            self.last_saved_seq = current_seq

                if self.stop_event.wait(timeout=AUTOSAVE_INTERVAL_SEC):
                    break

            except Exception as e:
                print(f"❌ THREAD: Error in loop: {e}")
                if self.stop_event.wait(timeout=AUTOSAVE_INTERVAL_SEC):
                    break

        print("🛑 THREAD: Stopped gracefully.")

    def start(self, archive_path=None):
        if self.thread and self.thread.is_alive():
            print("🖥️ MAIN: Thread is already running. Please stop it first.")
            return

        if archive_path:
            self.archive_path = archive_path

        print(f"🖥️ MAIN: Starting new archiver thread for folder: {self.archive_path}")

        self.stop_event = threading.Event()
        self.last_saved_seq = -1

        self.thread = threading.Thread(target=self._run_archiver)
        self.thread.start()

    def stop(self):
        if not self.thread or not self.thread.is_alive():
            print("🖥️ MAIN: No thread is currently running.")
            return

        print("🖥️ MAIN: Signaling thread to stop...")
        self.stop_event.set()
        self.thread.join()

        print("🖥️ MAIN: Thread has successfully stopped.")
        self.thread = None
        self.stop_event = None

    def is_running(self):
        return self.thread and self.thread.is_alive()


# --- Drawing & Streaming Functions ---
def logical_to_pixel(x, y):
    """Convert logical coordinates to pixel space."""
    px = int((x / CANVAS_WIDTH) * MJPEG_WIDTH)
    py = int((y / CANVAS_HEIGHT) * MJPEG_HEIGHT)
    px = max(0, min(MJPEG_WIDTH - 1, px))
    py = max(0, min(MJPEG_HEIGHT - 1, py))
    return px, py


def draw_segment(x0, y0, x1, y1, p):
    """Draw a line between two points and bump the dirty counter.
    Callers MUST already hold image_lock (non-reentrant)."""
    global stroke_seq
    px0, py0 = logical_to_pixel(x0, y0)
    px1, py1 = logical_to_pixel(x1, y1)
    width_px = max(1, int(p * AVG_SCALE * STROKE_FACTOR))
    draw.line([(px0, py0), (px1, py1)], fill=0, width=width_px)
    stroke_seq += 1


def generate_frames():
    """Yield the cached MJPEG frame paced at MJPEG_FPS. No per-client encode."""
    try:
        from benchmark import get_logger as _get_bm_logger
        _bm = _get_bm_logger()
    except Exception:
        _bm = None

    interval = 1.0 / max(1, MJPEG_FPS)

    while True:
        start = time.time()
        with frame_lock:
            payload = latest_jpeg_bytes

        if payload:
            if _bm is not None:
                try:
                    _bm.on_broadcast(time.perf_counter())
                except Exception:
                    pass
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + payload + b"\r\n")

        elapsed = time.time() - start
        time.sleep(max(0.0, interval - elapsed))
