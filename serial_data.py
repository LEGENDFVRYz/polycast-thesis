import serial

ser = serial.Serial("COM5", 115200)  # change COM port if needed

while True:
    line = ser.readline().decode(errors="ignore").strip()
    print(line)