import os
import threading
import sys
import glob
import serial

try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass


def _env_str(key, default):
    val = os.environ.get(key)
    return val if val is not None and val != "" else default


def _env_int(key, default):
    val = os.environ.get(key)
    if val is None or val == "":
        return default
    try:
        return int(val)
    except ValueError:
        return default


def _env_float(key, default):
    val = os.environ.get(key)
    if val is None or val == "":
        return default
    try:
        return float(val)
    except ValueError:
        return default


def _env_bool(key, default):
    val = os.environ.get(key)
    if val is None or val == "":
        return default
    return val.strip().lower() in ("1", "true", "yes", "on")


APP_ENV = _env_str("APP_ENV", "dev").lower()
IS_PROD = APP_ENV in ("prod", "production")


# --- 2. Define the shared event here ---
config_done_event = threading.Event()

# Background Task: Thread Reference
prototype_reader_thread_stop_event = threading.Event()
prototype_reader_thread = None


# Canvas dimensions
CANVAS_WIDTH  = _env_int("CANVAS_WIDTH",  1980)
CANVAS_HEIGHT = _env_int("CANVAS_HEIGHT", 1980)
MJPEG_WIDTH   = _env_int("MJPEG_WIDTH",   600)
MJPEG_HEIGHT  = _env_int("MJPEG_HEIGHT",  600)
MJPEG_FPS     = _env_int("MJPEG_FPS",     6 if IS_PROD else 20)
JPEG_QUALITY  = _env_int("JPEG_QUALITY",  65 if IS_PROD else 85)

# Stream client cap
MAX_STREAM_CLIENTS = _env_int("MAX_STREAM_CLIENTS", 40)

# Autosave / archiver
AUTOSAVE_INTERVAL_SEC    = _env_float("AUTOSAVE_INTERVAL_SEC",    1.0)
AUTOSAVE_ONLY_WHEN_DIRTY = _env_bool ("AUTOSAVE_ONLY_WHEN_DIRTY", True)
MIN_FREE_DISK_MB         = _env_int  ("MIN_FREE_DISK_MB",         1024)

# SQLite
SQLITE_BUSY_TIMEOUT_MS = _env_int("SQLITE_BUSY_TIMEOUT_MS", 5000)

# Flask secret
SECRET_KEY = _env_str("SECRET_KEY", "polycast-creator_BatsiKuruSyaniOmit")

# Ports
BROWSER_WS_PORT = _env_int("BROWSER_WS_PORT", 5001)
FLASK_PORT      = _env_int("FLASK_PORT",      5050)

# Paths
ARCHIVE_DIR = _env_str("ARCHIVE_DIR", "archive")
LOG_DIR     = _env_str("LOG_DIR",     "logs")

# Flags
IS_AUTO_ARCHIVING = False

# Create directories
os.makedirs(ARCHIVE_DIR, exist_ok=True)
os.makedirs(LOG_DIR, exist_ok=True)

# Log paths
CONN_LOG  = os.path.join(LOG_DIR, "conn.logs")
ERROR_LOG = os.path.join(LOG_DIR, "error.logs")


class _Config:
    SERIAL_PORT = _env_str("SERIAL_PORT", "COM3")
    BAUD_RATE   = _env_int("BAUD_RATE",   115200)

    def list_serial_ports(self):
        """Helper to print available ports if you don't know which one to use."""
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result


prototype_config = _Config()
