#!/usr/bin/python3
# ================================================================
# LYNX_CHECK_Encoders.py
# Spin each wheel by hand and watch its reading, in wheel turns.
# Ctrl+C to stop.
# ================================================================

N = 12000                    # ticks per wheel turn

import serial
import time
from lynx_port import get_port

ser = serial.Serial(port=get_port(), baudrate=115200, timeout=0.05)
time.sleep(2)
ser.readline()

def read_pair(sel):
    ser.reset_input_buffer()
    ser.write(bytes([sel]))
    try:
        a, b = ser.readline().decode().strip().split()
        return int(a), int(b)
    except (ValueError, UnicodeDecodeError):
        return None, None

print(f"\n  {'FL':>8}  {'RL':>8}  {'FR':>8}  {'RR':>8}   (turns)")
print("  " + "-" * 40)

try:
    while True:
        fl, rl = read_pair(5)
        fr, rr = read_pair(6)
        if fl is not None and fr is not None:
            print(f"  {fl/N:>+8.3f}  {rl/N:>+8.3f}  {fr/N:>+8.3f}  {rr/N:>+8.3f}",
                  end='\r')
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n\nDone.")

finally:
    ser.write(bytes([7]))
    ser.close()
    print("=^..^=")
