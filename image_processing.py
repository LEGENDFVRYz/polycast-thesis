import threading, time, os
from datetime import datetime
from PIL import Image, ImageDraw
from config import CANVAS_WIDTH, CANVAS_HEIGHT, MJPEG_WIDTH, MJPEG_HEIGHT, ARCHIVE_DIR, MJPEG_FPS

# Scaling factors
scale_x = MJPEG_WIDTH / CANVAS_WIDTH
scale_y = MJPEG_HEIGHT / CANVAS_HEIGHT
AVG_SCALE = (scale_x + scale_y) / 2.0
STROKE_FACTOR = 0.5

# Canvas and lock
canvas = Image.new("L", (MJPEG_WIDTH, MJPEG_HEIGHT), "white")
draw = ImageDraw.Draw(canvas)
image_lock = threading.Lock()


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


def archiver_thread():
    """Automatically save the latest canvas every second."""
    while True:
        time.sleep(1)
        with image_lock:
            snap = canvas.copy()
        ts = datetime.now().strftime("%Y%m%d-%H%M%S")
        fname = os.path.join(ARCHIVE_DIR, f"note_{ts}.jpg")
        try:
            snap.convert("RGB").save(fname, quality=85)
            print(f"[ARCHIVER] saved {fname}")
        except Exception as e:
            print("[ARCHIVER] error:", e)


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
