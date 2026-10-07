#!/usr/bin/python3
# ================================================================
# LYNX_CHECK_Encoder_Coherence.py
# Check if front and rear encoders on the same side agree.
#
# Runs four tests per side:
#   1. Slow forward  (PWM 50)
#   2. Fast forward  (PWM 80)
#   3. Slow backward (PWM 50)
#   4. Fast backward (PWM 80)
#
# Thresholds:
#   < 10% mismatch  →  OK
#   10-20%          →  WARNING (check belt tension)
#   > 20%           →  FAIL (encoder or wiring fault)
# ================================================================

import serial
import time
import sys

from lynx_port import get_port
ser = serial.Serial(port=get_port(), baudrate=115200, timeout=0.05)
time.sleep(2)
ser.readline()

DURATION  = 2.0
THRESHOLD_WARN = 0.10
THRESHOLD_FAIL = 0.20

TESTS = [
    ("Slow Fwd",  50),
    ("Fast Fwd",  80),
    ("Slow Bwd", -50),
    ("Fast Bwd", -80),
]

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

def send_motor(channel, u):
    pwm = min(int(abs(u)), 127)
    sel = (1 if u >= 0 else 2) if channel == 1 else (3 if u >= 0 else 4)
    ser.write(bytes([sel, pwm]))

def stop_all():
    ser.write(bytes([1, 0]))
    ser.write(bytes([3, 0]))

def test_side(side, motor_ch, enc_sel):
    print(f"\n  === {side.upper()} SIDE ===")

    for name, pwm in TESTS:
        # Read initial
        a0, b0 = read_pair(enc_sel)
        if a0 is None:
            print(f"    {name:10s}  FAIL: cannot read encoders")
            continue

        # Spin
        send_motor(motor_ch, pwm)
        time.sleep(DURATION)
        stop_all()
        time.sleep(0.5)

        # Read final
        a1, b1 = read_pair(enc_sel)
        if a1 is None:
            print(f"    {name:10s}  FAIL: cannot read encoders after spin")
            continue

        delta_front = abs(a1 - a0)
        delta_rear  = abs(b1 - b0)

        if delta_front == 0 and delta_rear == 0:
            print(f"    {name:10s}  FAIL: no movement")
            continue

        avg = (delta_front + delta_rear) / 2.0
        if avg == 0:
            avg = 1

        mismatch = abs(delta_front - delta_rear) / avg

        if mismatch < THRESHOLD_WARN:
            status = "OK"
        elif mismatch < THRESHOLD_FAIL:
            status = "WARNING"
        else:
            status = "FAIL"

        print(f"    {name:10s}  F={delta_front:6d}  R={delta_rear:6d}  "
              f"diff={mismatch*100:5.1f}%  {status}")

        time.sleep(0.3)

print("=" * 65)
print("  ENCODER COHERENCE CHECK — Four Speed Modes")
print(f"  Duration: {DURATION}s per test")
print("=" * 65)

try:
    test_side("LEFT",  1, 5)
    test_side("RIGHT", 2, 6)

except KeyboardInterrupt:
    print("\n\nInterrupted.")

finally:
    stop_all()
    time.sleep(0.1)
    ser.write(bytes([7]))
    ser.close()
    print("\n" + "=" * 65)
    print("  Done.")
    print("=" * 65)
    print("=^..^=")