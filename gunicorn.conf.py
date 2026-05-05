import threading

# workers=1 REQUIRED: PIL canvas, image_lock, and AdminStatusManager are
# in-process singletons — forking to N workers splits memory spaces.
workers = 1
worker_class = "gthread"
threads = 16          # 16-thread pool; covers 30-40 concurrent SSE/MJPEG clients

# --- Network ---
bind = "127.0.0.1:5050"

# --- Timeouts ---
# Match nginx proxy_read_timeout (3600s) so gunicorn never kills MJPEG/SSE
# connections before nginx does.
timeout = 3600
graceful_timeout = 30
keepalive = 5

# --- Performance (Raspberry Pi) ---
# gthread writes heartbeat temp files; /dev/shm is RAM-backed, avoiding
# SD card wear and I/O latency on the Pi.
worker_tmp_dir = "/dev/shm"

# --- Logging ---
accesslog = "logs/prod_access.log"
errorlog  = "logs/prod_error.log"
loglevel  = "info"


def on_starting(server):
    """Launch the browser WebSocket broadcaster before the worker forks."""
    try:
        from prototype import start_browser_ws_server
        from config import BROWSER_WS_PORT
        threading.Thread(
            target=start_browser_ws_server,
            args=(BROWSER_WS_PORT,),
            daemon=True,
        ).start()
    except Exception as exc:
        import logging
        logging.getLogger("gunicorn.error").warning(
            "Browser WS server did not start: %s", exc
        )
