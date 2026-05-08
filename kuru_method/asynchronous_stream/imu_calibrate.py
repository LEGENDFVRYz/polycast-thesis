"""
imu_calibrate.py  —  Trigger BNO085 DCD save on the marker (Item D)
====================================================================
Sends `CAL\\n` to the receiver over USB serial.  The receiver forwards
the request to the sender via ESP-NOW; the sender calls
`sh2_saveDcdNow()` and replies with `CAL:OK` (or `CAL:ERR <code>`),
which the receiver echoes back over USB.

After a successful save the BNO085 will restore the calibration on
every subsequent power-up.

Usage
-----
    python imu_calibrate.py            # uses SERIAL_PORT from config.py
    python imu_calibrate.py --port COM7
"""

from __future__ import annotations

import argparse
import sys
import time
import serial

from config import SERIAL_PORT, BAUD_RATE


CAL_REPLY_TIMEOUT_S = 3.0   # wall-clock budget for the round-trip


def trigger_dcd_save(port: str = SERIAL_PORT,
                     baud: int = BAUD_RATE,
                     timeout_s: float = CAL_REPLY_TIMEOUT_S) -> int:
    """
    Send the CAL request and block until the reply (or timeout).

    Returns
    -------
    0 on success, non-zero status code on sender-reported error,
    -1 on timeout, -2 on serial open failure.
    """
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"[CAL] open {port} failed: {e}")
        return -2

    try:
        # Drop bootloader/data noise, then send the command.
        ser.reset_input_buffer()
        ser.write(b"CAL\n")
        ser.flush()
        print(f"[CAL] sent on {port} @ {baud} — waiting for reply...")

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            if line.startswith("CAL:"):
                print(f"[CAL] reply: {line}")
                if line == "CAL:OK":
                    return 0
                if line == "CAL:SENT":
                    # forwarded to sender — keep waiting for terminal reply
                    continue
                # CAL:ERR <code>
                parts = line.split()
                try:
                    return int(parts[-1])
                except ValueError:
                    return -3

        print("[CAL] timeout — no reply received")
        return -1
    finally:
        ser.close()


def _cli():
    p = argparse.ArgumentParser(description="Trigger BNO085 DCD save")
    p.add_argument("--port", default=SERIAL_PORT,
                   help=f"serial port (default: {SERIAL_PORT})")
    p.add_argument("--baud", default=BAUD_RATE, type=int,
                   help=f"baud rate (default: {BAUD_RATE})")
    p.add_argument("--timeout", default=CAL_REPLY_TIMEOUT_S, type=float,
                   help="reply timeout seconds")
    args = p.parse_args()
    rc = trigger_dcd_save(args.port, args.baud, args.timeout)
    sys.exit(0 if rc == 0 else 1)


if __name__ == "__main__":
    _cli()
