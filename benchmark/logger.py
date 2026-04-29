"""
benchmark/logger.py — Unified Benchmark Logger
Applies docs/bg_rules.md + docs/web_rules.md

Opt-in via environment variable:
    BENCHMARK=1 python main.py

When BENCHMARK is not set (or "0"), get_logger() returns _NullLogger whose
every method is a no-op — zero files opened, zero threads started.

Session CSVs are written to:
    benchmark/sessions/<session_id>/
        latency_log.csv
        packet_log.csv
        resource_log.csv
        client_log.csv
        error_log.csv
        software_perf_log.csv
"""

import csv
import os
import threading
import time
import uuid
from collections import deque

BENCHMARK_ENABLED = os.environ.get("BENCHMARK", "0") == "1"

# ── Singleton holder ──────────────────────────────────────────────────────────
_instance = None
_instance_lock = threading.Lock()


def get_logger(extra_tags: dict = None):
    """Return the process-wide BenchmarkLogger or _NullLogger singleton."""
    global _instance
    if _instance is None:
        with _instance_lock:
            if _instance is None:
                if BENCHMARK_ENABLED:
                    _instance = BenchmarkLogger(extra_tags=extra_tags)
                else:
                    _instance = _NullLogger()
    return _instance


# ── Inline packet parser (hot path — imports nothing from the pipeline) ───────
def _parse_seq_and_ts(raw_line: bytes) -> tuple:
    """Returns (seq_id, ts_hw, sensor_type, field_count). Never raises."""
    try:
        line = raw_line.decode("utf-8", errors="replace").strip()
        parts = line.split(",")
        if not parts:
            return None, None, None, 0
        t = parts[0]
        fc = len(parts)
        seq = int(parts[1]) if fc > 1 else None
        last = parts[-1] if parts[-1] else ""
        ts = int(last) if last.lstrip("-").isdigit() else None
        sensor = "IMU" if t == "I" else ("UWB" if t == "U" else None)
        return seq, ts, sensor, fc
    except Exception:
        return None, None, None, 0


# ── CSV column definitions ────────────────────────────────────────────────────
_LATENCY_FIELDS = [
    "session_id", "seq_id", "sensor_type", "ts_hw",
    "server_recv_time", "fusion_done_time", "pi_proc_latency_ms",
]
_PACKET_FIELDS = [
    "session_id", "seq_id", "sensor_type", "ts_hw",
    "server_recv_time", "field_count", "crc_valid", "condition",
]
_RESOURCE_FIELDS = [
    "session_id", "sample_time", "cpu_percent", "ram_mb", "client_count",
]
_CLIENT_FIELDS = [
    "session_id", "event_time", "client_count", "event_type",
]
_ERROR_FIELDS = [
    "session_id", "event_time", "error_event", "connection_event",
]
_SW_PERF_FIELDS = [
    "session_id", "frame_time", "encode_ms", "sleep_ms", "target_fps",
]

# Expected field counts for integrity check
_EXPECTED_FIELDS = {"IMU": 11, "UWB": 7}


# ── Live logger ───────────────────────────────────────────────────────────────
class BenchmarkLogger:
    def __init__(self, extra_tags: dict = None):
        self.session_id = str(uuid.uuid4())[:8]
        self.session_dir = os.path.join("benchmark", "sessions", self.session_id)
        os.makedirs(self.session_dir, exist_ok=True)

        self._extra_tags = extra_tags or {}
        self._condition = ""

        # Deque buffers — append() is GIL-atomic; no explicit lock needed
        self._latency_buf  = deque()
        self._packet_buf   = deque()
        self._resource_buf = deque()
        self._client_buf   = deque()
        self._error_buf    = deque()
        self._sw_perf_buf  = deque()

        # Pending packet correlation: seq_id -> partial row
        # Accessed only from the serial thread — no contention
        self._pending: dict = {}

        # Open CSV files once; keep handles for the session lifetime
        self._files = {}
        self._writers = {}
        for name, fields in [
            ("latency_log",      _LATENCY_FIELDS),
            ("packet_log",       _PACKET_FIELDS),
            ("resource_log",     _RESOURCE_FIELDS),
            ("client_log",       _CLIENT_FIELDS),
            ("error_log",        _ERROR_FIELDS),
            ("software_perf_log", _SW_PERF_FIELDS),
        ]:
            path = os.path.join(self.session_dir, f"{name}.csv")
            fh = open(path, "w", newline="", encoding="utf-8")
            writer = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            fh.flush()
            self._files[name] = fh
            self._writers[name] = writer

        # Background writer thread
        self._flush_event = threading.Event()
        self._stop_flag = False
        self._writer_thread = threading.Thread(
            target=self._flush_loop, daemon=True, name="bm-writer"
        )
        self._writer_thread.start()

        print(f"[BM] Session {self.session_id} — writing to {self.session_dir}")

    # ── Public API ────────────────────────────────────────────────────────────

    def on_packet_recv(self, raw_line: bytes, server_recv_time: float) -> None:
        seq, ts_hw, sensor, fc = _parse_seq_and_ts(raw_line)
        now = server_recv_time

        # Integrity: field count matches protocol expectation
        crc_valid = (sensor is not None) and (fc == _EXPECTED_FIELDS.get(sensor, -1))

        # Store partial row for correlation with on_fusion_done
        if seq is not None:
            self._pending[seq] = {
                "seq_id": seq,
                "sensor_type": sensor,
                "ts_hw": ts_hw,
                "server_recv_time": now,
                "field_count": fc,
                "crc_valid": crc_valid,
            }

        # Packet log row is complete at recv time
        self._packet_buf.append({
            "session_id": self.session_id,
            "seq_id": seq,
            "sensor_type": sensor,
            "ts_hw": ts_hw,
            "server_recv_time": now,
            "field_count": fc,
            "crc_valid": crc_valid,
            "condition": self._condition,
        })

    def on_fusion_done(self, raw_line: bytes, fusion_done_time: float,
                       result: tuple | None) -> None:
        seq, _, _, _ = _parse_seq_and_ts(raw_line)
        pending = self._pending.pop(seq, None) if seq is not None else None

        if pending is None:
            return

        srv = pending["server_recv_time"]
        proc_ms = (fusion_done_time - srv) * 1000.0

        self._latency_buf.append({
            "session_id": self.session_id,
            "seq_id": seq,
            "sensor_type": pending["sensor_type"],
            "ts_hw": pending["ts_hw"],
            "server_recv_time": round(srv, 6),
            "fusion_done_time": round(fusion_done_time, 6),
            "pi_proc_latency_ms": round(proc_ms, 4),
        })

    def on_broadcast(self, broadcast_time: float) -> None:
        # Recorded as a separate event row; not packet-aligned
        self._latency_buf.append({
            "session_id": self.session_id,
            "seq_id": None,
            "sensor_type": "MJPEG",
            "ts_hw": None,
            "server_recv_time": None,
            "fusion_done_time": None,
            "pi_proc_latency_ms": None,
        })

    def on_software_perf(self, encode_ms: float, sleep_ms: float) -> None:
        self._sw_perf_buf.append({
            "session_id": self.session_id,
            "frame_time": round(time.perf_counter(), 6),
            "encode_ms": round(encode_ms, 4),
            "sleep_ms": round(sleep_ms, 4),
            "target_fps": 20,
        })

    def on_connection_event(self, event: str) -> None:
        self._error_buf.append({
            "session_id": self.session_id,
            "event_time": round(time.perf_counter(), 6),
            "error_event": "",
            "connection_event": event,
        })

    def on_error(self, error_msg: str) -> None:
        self._error_buf.append({
            "session_id": self.session_id,
            "event_time": round(time.perf_counter(), 6),
            "error_event": error_msg,
            "connection_event": "",
        })

    def on_resource_sample(self, cpu_percent: float, ram_mb: float,
                           client_count: int = 0) -> None:
        self._resource_buf.append({
            "session_id": self.session_id,
            "sample_time": round(time.perf_counter(), 6),
            "cpu_percent": round(cpu_percent, 2),
            "ram_mb": round(ram_mb, 2),
            "client_count": client_count,
        })

    def on_client_event(self, client_count: int, event_type: str) -> None:
        self._client_buf.append({
            "session_id": self.session_id,
            "event_time": round(time.perf_counter(), 6),
            "client_count": client_count,
            "event_type": event_type,
        })

    def set_condition(self, condition_label: str) -> None:
        self._condition = condition_label

    def stop(self) -> None:
        self._stop_flag = True
        self._flush_event.set()
        self._writer_thread.join(timeout=10)
        self._flush_all()
        for fh in self._files.values():
            try:
                fh.flush()
                fh.close()
            except Exception:
                pass
        print(f"[BM] Session {self.session_id} closed.")

    # ── Internal ──────────────────────────────────────────────────────────────

    def _flush_loop(self) -> None:
        while not self._stop_flag:
            self._flush_event.wait(timeout=2.0)
            self._flush_event.clear()
            self._flush_all()

    def _flush_all(self) -> None:
        self._drain("latency_log",      self._latency_buf)
        self._drain("packet_log",       self._packet_buf)
        self._drain("resource_log",     self._resource_buf)
        self._drain("client_log",       self._client_buf)
        self._drain("error_log",        self._error_buf)
        self._drain("software_perf_log", self._sw_perf_buf)
        for fh in self._files.values():
            try:
                fh.flush()
            except Exception:
                pass

    def _drain(self, name: str, buf: deque) -> None:
        if not buf:
            return
        writer = self._writers[name]
        while buf:
            try:
                writer.writerow(buf.popleft())
            except Exception:
                pass


# ── Null logger (zero overhead when BENCHMARK=0) ─────────────────────────────
class _NullLogger:
    session_id = ""
    session_dir = ""

    def on_packet_recv(self, raw_line, server_recv_time): pass
    def on_fusion_done(self, raw_line, fusion_done_time, result): pass
    def on_broadcast(self, broadcast_time): pass
    def on_software_perf(self, encode_ms, sleep_ms): pass
    def on_connection_event(self, event): pass
    def on_error(self, error_msg): pass
    def on_resource_sample(self, cpu_percent, ram_mb, client_count=0): pass
    def on_client_event(self, client_count, event_type): pass
    def set_condition(self, condition_label): pass
    def stop(self): pass
