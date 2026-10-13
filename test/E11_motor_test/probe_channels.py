"""Probe individual motor channels L and R to test channel-to-encoder correspondence."""

import time
import serial

ser = serial.Serial(port=None, baudrate=115200, timeout=0.1)
ser.dtr = True
ser.rts = False
ser.port = "COM17"
ser.open()
time.sleep(0.3)
ser.write(b"STOP\n")
time.sleep(0.5)
ser.reset_input_buffer()

print("--- Testing L100 (Commanding Left motor only, Right should be 0) ---")
ser.write(b"L100\n")
ser.flush()
t_end = time.time() + 2.0
while time.time() < t_end:
    line = ser.readline().decode("ascii", errors="replace").strip()
    if line and "L=" in line:
        print(line)

ser.write(b"STOP\n")
time.sleep(0.5)
ser.reset_input_buffer()

print("\n--- Testing R100 (Commanding Right motor only, Left should be 0) ---")
ser.write(b"R100\n")
ser.flush()
t_end = time.time() + 2.0
while time.time() < t_end:
    line = ser.readline().decode("ascii", errors="replace").strip()
    if line and "L=" in line:
        print(line)

ser.write(b"STOP\n")
time.sleep(0.2)
ser.close()
