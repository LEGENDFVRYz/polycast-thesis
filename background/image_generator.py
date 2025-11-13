import threading, time, os
from datetime import datetime
from PIL import Image, ImageDraw
from config import (
    CANVAS_WIDTH, CANVAS_HEIGHT, MJPEG_WIDTH, MJPEG_HEIGHT, 
    ARCHIVE_DIR, # We will use this as the default
    MJPEG_FPS
)

# --- Shared Resources ---
scale_x = MJPEG_WIDTH / CANVAS_WIDTH
scale_y = MJPEG_HEIGHT / CANVAS_HEIGHT
AVG_SCALE = (scale_x + scale_y) / 2.0
STROKE_FACTOR = 0.5

canvas = Image.new("L", (MJPEG_WIDTH, MJPEG_HEIGHT), "white")
draw = ImageDraw.Draw(canvas)
image_lock = threading.Lock()


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

    def _run_archiver(self):
        """
        This is the private function that runs in the thread.
        """
        print(f"✅ THREAD: Started. Archiving to: {self.archive_path}")
        
        # Create the directory if it doesn't exist
        try:
            os.makedirs(self.archive_path, exist_ok=True)
        except Exception as e:
            print(f"❌ THREAD: Could not create directory {self.archive_path}. Error: {e}")
            print("🛑 THREAD: Exiting.")
            return

        # Loop until the stop_event is set
        while not self.stop_event.is_set():
            try:
                # 1. Do the work (no 'wait' needed)
                with image_lock:
                    snap = canvas.copy()
                
                ts = datetime.now().strftime("%Y%m%d-%H%M%S")
                fname = os.path.join(self.archive_path, f"note_{ts}.jpg")
                
                try:
                    snap.convert("RGB").save(fname, quality=85)
                    print(f"[ARCHIVER] saved {fname}")
                except Exception as e:
                    print(f"[ARCHIVER] save error: {e}")

                # 2. Wait for 1 second OR until the stop_event is set
                # This acts as our 1-second timer and our stop mechanism
                interrupted = self.stop_event.wait(timeout=1.0)
                if interrupted:
                    break # Stop event was set, exit loop

            except Exception as e:
                print(f"❌ THREAD: Error in loop: {e}")
                # Prevent rapid-fire error loops
                if self.stop_event.wait(timeout=1.0):
                    break

        print("🛑 THREAD: Stopped gracefully.")

    def start(self, archive_path=None):
        """
        Starts the archiver thread.
        Optionally updates the archive path before starting.
        """
        if self.thread and self.thread.is_alive():
            print("🖥️ MAIN: Thread is already running. Please stop it first.")
            return
        
        # If a new path is provided, update it
        if archive_path:
            self.archive_path = archive_path
            
        print(f"🖥️ MAIN: Starting new archiver thread for folder: {self.archive_path}")
        
        self.stop_event = threading.Event()
        
        self.thread = threading.Thread(target=self._run_archiver)
        self.thread.start()

    def stop(self):
        """
        Signals the worker thread to stop and waits for it to finish.
        """
        if not self.thread or not self.thread.is_alive():
            print("🖥️ MAIN: No thread is currently running.")
            return

        print("🖥️ MAIN: Signaling thread to stop...")
        
        # 1. Set the stop signal. This will wake up stop_event.wait()
        self.stop_event.set()
        
        # 2. Wait for the thread to finish
        self.thread.join()
        
        print("🖥️ MAIN: Thread has successfully stopped.")
        self.thread = None
        self.stop_event = None

    def is_running(self):
        """Helper to check if the thread is alive."""
        return self.thread and self.thread.is_alive()

# --- Drawing & Streaming Functions (Unchanged) ---
def logical_to_pixel(x, y):
    """Convert logical coordinates to pixel space."""
    px = int((x / CANVAS_WIDTH) * MJPEG_WIDTH)
    py = int((y / CANVAS_HEIGHT) * MJPEG_HEIGHT)
    px = max(0, min(MJPEG_WIDTH - 1, px))
    py = max(0, min(MJPEG_HEIGHT - 1, py))
    return px, py

def draw_segment(x0, y0, x1, y1, p):
    """Draw a line between two points."""
    px0, py0 = logical_to_pixel(x0, y0)
    px1, py1 = logical_to_pixel(x1, y1)
    width_px = max(1, int(p * AVG_SCALE * STROKE_FACTOR))
    draw.line([(px0, py0), (px1, py1)], fill=0, width=width_px)

def generate_frames():
    """Generate MJPEG frames for streaming."""
    import io, time
    interval = 1.0 / MJPEG_FPS
    while True:
        start = time.time()
        with image_lock:
            frame = canvas.copy()
        buf = io.BytesIO()
        frame.convert("RGB").save(buf, format="JPEG")
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.getvalue() + b"\r\n")
        elapsed = time.time() - start
        time.sleep(max(0, interval - elapsed))

