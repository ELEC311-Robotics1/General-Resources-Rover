#!/usr/bin/python3
# ================================================================
# LYNX_CHECK_Motors.py
# Pulse each motor in each direction. Watch and confirm.
#
# Sequence:
#   1. Left forward   (1 second)
#   2. Left backward  (1 second)
#   3. Right forward  (1 second)
#   4. Right backward (1 second)
#
# Press Enter between each test to confirm it moved correctly.
# Records a pass/fail truth table at the end.
# ================================================================

import serial
import time
import sys

from lynx_port import get_port
ser = serial.Serial(port=get_port(), baudrate=115200, timeout=0.05)

time.sleep(2)
ser.readline()

PWM_TEST = 50
PULSE    = 1.0

def send_motor(channel, u):
    pwm = min(int(abs(u)), 127)
    sel = (1 if u >= 0 else 2) if channel == 1 else (3 if u >= 0 else 4)
    ser.write(bytes([sel, pwm]))

def stop_all():
    ser.write(bytes([1, 0]))
    ser.write(bytes([3, 0]))

tests = [
    ("Left Forward",   1,  PWM_TEST),
    ("Left Backward",  1, -PWM_TEST),
    ("Right Forward",  2,  PWM_TEST),
    ("Right Backward", 2, -PWM_TEST),
]

results = []

print("=" * 55)
print("  MOTOR CHECK — Watch each motor pulse")
print(f"  PWM: {PWM_TEST}  Pulse: {PULSE}s")
print("=" * 55)

try:
    for name, ch, pwm in tests:
        input(f"\n  Ready to test: {name}. Press Enter...")
        print(f"  Pulsing {name}...", end="", flush=True)

        send_motor(ch, pwm)
        time.sleep(PULSE)
        stop_all()
        time.sleep(0.3)

        ok = input("  Correct? (y/n): ").strip().lower()
        results.append((name, "PASS" if ok == 'y' else "FAIL"))

except KeyboardInterrupt:
    print("\n\nInterrupted.")
    stop_all()

finally:
    stop_all()
    time.sleep(0.1)
    ser.write(bytes([7]))
    ser.close()

# ── Truth table ──────────────────────────────────────────────
print("\n" + "=" * 55)
print("  MOTOR CHECK RESULTS")
print("  " + "-" * 35)
for name, status in results:
    mark = "OK" if status == "PASS" else "XX"
    print(f"  [{mark}]  {name}")
print("=" * 55)

all_pass = all(s == "PASS" for _, s in results)
if all_pass:
    print("  All motors verified.")
else:
    print("  Some motors FAILED — check wiring or Sabertooth config.")
print("=^..^=")
