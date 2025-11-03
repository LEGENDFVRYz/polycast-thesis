import os
import threading
import socket

# --- 2. Define the shared event here ---
config_done_event = threading.Event()

# Background Task: Thread Reference
prototype_reader_thread_stop_event = threading.Event()
prototype_reader_thread = None


# Canvas dimensions
CANVAS_WIDTH, CANVAS_HEIGHT = 35560, 22219
MJPEG_WIDTH, MJPEG_HEIGHT = 600, 400
MJPEG_FPS = 20

# Ports
BROWSER_WS_PORT = 5001
FLASK_PORT = 5050

# Paths
ARCHIVE_DIR = "archive"
LOG_DIR = "logs"

# Flags
IS_AUTO_ARCHIVING = False

# Create directories
os.makedirs(ARCHIVE_DIR, exist_ok=True)
os.makedirs(LOG_DIR, exist_ok=True)

# Log paths
CONN_LOG = os.path.join(LOG_DIR, "conn.logs")
ERROR_LOG = os.path.join(LOG_DIR, "error.logs")


# Prototype endpoint
class _Config:
    UDP_PORT = 12345
    PROTOTYPE_IP = None     # will be set by the listener in admin/configure

    @property
    def ESP32_WS_URL(self):
        if self.PROTOTYPE_IP is None:
            return None
        return f"ws://{self.PROTOTYPE_IP}/ws"

prototype_config = _Config()