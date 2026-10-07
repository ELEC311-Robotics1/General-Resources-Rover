#!/usr/bin/python3
# ================================================================
# LYNX_CHECK_Encoders.py
# Live encoder readout — spin wheels by hand and watch the counts.
#
# Prints all four encoder values (FL, RL, FR, RR) at ~10 Hz.
# Ctrl+C to stop.
#
# What to check:
#   - Counts increase when wheel spins forward
#   - Counts decrease when wheel spins backward
#   - No stuck readings (dead encoder)
#   - No jumps (loose connector)
# ================================================================

import serial
import time
import sys

from lynx_port import get_port
ser = serial.Serial(port=get_port(), baudrate=115200, timeout=0.05)

time.sleep(2)
ser.readline()

def read_pair(sel):
    ser.reset_input_buffer()
    ser.write(bytes([sel]))
    try:
        vals = ser.readline().decode().strip().split()
        if len(vals) == 2:
            return int(vals[0]), int(vals[1])
        return None, None
    except:
        return None, None

print("=" * 55)
print("  ENCODER CHECK — Spin wheels by hand")
print("  Ctrl+C to stop")
print("=" * 55)
print(f"\n  {'FL':>8}  {'RL':>8}  {'FR':>8}  {'RR':>8}")
print("  " + "-" * 40)

try:
    while True:
        fl, rl = read_pair(5)
        fr, rr = read_pair(6)

        if fl is not None and fr is not None:
            print(f"  {fl:>8}  {rl:>8}  {fr:>8}  {rr:>8}", end='\r')

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n\nDone.")

finally:
    ser.write(bytes([7]))
    ser.close()
    print("=^..^=")
