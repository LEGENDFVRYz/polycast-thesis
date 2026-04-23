import threading

# --- Worker model ---
# workers=1 is REQUIRED: AdminStatusManager, PIL canvas, and image_lock
# are in-process singletons. Multiple workers fork separate memory spaces.
# (threading.Event never fires cross-process) and MJPEG (shared canvas)
workers = 1
worker_class = "gthread"
threads = 16        # (baseline) 16-thread pool handles 30 clients

# --- Network ---
bind = "127.0.0.1:5050"

# --- Timeouts ---
timeout = 120       # Long-running SSE/MJPEG connections; keep-alive baseline
keepalive = 5

# --- Logging ---
accesslog = "logs/prod_access.log"
errorlog = "logs/prod_error.log"
loglevel = "info"


def on_starting(server):
    """Start the browser WebSocket server before the worker is forked."""
    from prototype import start_browser_ws_server
    from config import BROWSER_WS_PORT
    threading.Thread(
        target=start_browser_ws_server,
        args=(BROWSER_WS_PORT,),
        daemon=True
    ).start()
