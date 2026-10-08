#!/usr/bin/python3
# ================================================================
# LYNX_CHECK_Encoder_Coherence.py
# Do front and rear encoders on the same side agree? Rover on blocks.
#
# F, R: signed change in wheel turns.   diff: |F - R| / mean(|F|, |R|)
# rate: side average, rad/s, measured while the motor runs.
#   < 10 %  OK     10-20 %  WARNING     > 20 %  FAIL
# ================================================================

PIT_PWM_MAX = 80             # pit-mode cap
N           = 12000          # ticks per wheel turn
SPINUP      = 0.5            # s before the rate is measured
WINDOW      = 1.5            # s over which the rate is measured

TESTS = [
    ("Slow Fwd",  50),
    ("Fast Fwd",  80),
    ("Slow Bwd", -50),
    ("Fast Bwd", -80),
]

import math
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

def send_motor(channel, u):
    pwm = min(int(abs(u)), PIT_PWM_MAX)
    sel = (1 if u >= 0 else 2) if channel == 1 else (3 if u >= 0 else 4)
    ser.write(bytes([sel, pwm]))

def stop_all():
    ser.write(bytes([1, 0]))
    ser.write(bytes([3, 0]))

def test_side(side, ch, sel):
    print(f"\n  === {side} ===")
    print(f"    {'':9s} {'F':>7s} {'R':>7s} {'diff':>6s} {'rate':>7s}")
    for name, pwm in TESTS:
        a0, b0 = read_pair(sel)
        send_motor(ch, pwm)
        time.sleep(SPINUP)
        aA, bA = read_pair(sel)
        time.sleep(WINDOW)
        aB, bB = read_pair(sel)
        stop_all()
        time.sleep(0.5)
        a1, b1 = read_pair(sel)
        if None in (a0, aA, aB, a1):
            print(f"    {name:9s} cannot read encoders")
            continue

        F, R = (a1 - a0) / N, (b1 - b0) / N
        rate = math.pi * ((aB - aA) + (bB - bA)) / N / WINDOW
        mean = (abs(F) + abs(R)) / 2
        diff = abs(F - R) / mean if mean > 0 else float('inf')
        status = "OK" if diff < 0.10 else ("WARNING" if diff < 0.20 else "FAIL")
        print(f"    {name:9s} {F:+7.3f} {R:+7.3f} {diff*100:5.1f}% {rate:+7.2f}  {status}")
        time.sleep(0.3)

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
    print("=^..^=")
