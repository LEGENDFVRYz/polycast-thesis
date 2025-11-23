import csv
import time
import serial

CSV_FILE = "square.csv"
TIMESTAMP_COL = "packet_ts"

SERIAL_PORT = 'COM1'
BAUD_RATE = 115200

# Open serial port
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

with open(CSV_FILE, newline="") as f:
    reader = csv.DictReader(f)
    rows = list(reader)

prev_ts = None

# Send header first (optional)
header = ",".join(reader.fieldnames) + "\n"
ser.write(header.encode())

for row in rows:
    current_ts = int(row[TIMESTAMP_COL])

    # calculate real-time delay
    if prev_ts is not None:
        delta_us = current_ts - prev_ts
        delta_s = delta_us / 1_000_000.0
        time.sleep(delta_s)

    # format row as CSV line
    line = ",".join(row[field] for field in reader.fieldnames) + "\n"

    # send over serial
    ser.write(line.encode())

    prev_ts = current_ts

ser.close()
