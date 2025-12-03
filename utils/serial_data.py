import serial

ser = serial.Serial("COM3", 115200)  # change COM port if needed

while True:
    line = ser.readline().decode(errors="ignore").strip()
    print(line)