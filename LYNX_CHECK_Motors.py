#!/usr/bin/python3
# ================================================================
# LYNX_CHECK_Motors.py
# Pulse each channel in each direction. Rover on blocks.
# Watch all four wheels, write down what you saw, then press Enter
# to see what the encoders saw.
# ================================================================

PIT_PWM_MAX = 80             # pit-mode cap
PWM_TEST    = 50
PULSE       = 1.0            # s
N           = 12000          # ticks per wheel turn

import serial
import time
from lynx_port import get_port

ser = serial.Serial(port=get_port(), baudrate=115200, timeout=0.05)
time.sleep(2)
ser.readline()

def send_motor(channel, u):
    pwm = min(int(abs(u)), PIT_PWM_MAX)
    sel = (1 if u >= 0 else 2) if channel == 1 else (3 if u >= 0 else 4)
    ser.write(bytes([sel, pwm]))

def stop_all():
    ser.write(bytes([1, 0]))
    ser.write(bytes([3, 0]))

def read_pair(sel):
    ser.reset_input_buffer()
    ser.write(bytes([sel]))
    try:
        a, b = ser.readline().decode().strip().split()
        return int(a), int(b)
    except (ValueError, UnicodeDecodeError):
        return None, None

def read_all():
    fl, rl = read_pair(5)
    fr, rr = read_pair(6)
    return None if fl is None or fr is None else (fl, rl, fr, rr)

tests = [
    ("Left Forward",   1,  PWM_TEST),
    ("Left Backward",  1, -PWM_TEST),
    ("Right Forward",  2,  PWM_TEST),
    ("Right Backward", 2, -PWM_TEST),
]

try:
    for name, ch, pwm in tests:
        input(f"\n  {name}. Hands clear, press Enter...")
        c0 = read_all()
        send_motor(ch, pwm)
        time.sleep(PULSE)
        stop_all()
        time.sleep(0.4)
        c1 = read_all()

        input("  Write down what each wheel did, then press Enter...")
        if c0 is None or c1 is None:
            print("  encoders: no reply")
        else:
            d = [(b - a) / N for a, b in zip(c0, c1)]
            print(f"  turns:  FL {d[0]:+.3f}  RL {d[1]:+.3f}  "
                  f"FR {d[2]:+.3f}  RR {d[3]:+.3f}")

except (KeyboardInterrupt, EOFError):
    print("\n\nInterrupted.")

finally:
    stop_all()
    time.sleep(0.1)
    ser.write(bytes([7]))
    ser.close()
    print("=^..^=")
