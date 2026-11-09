#!/usr/bin/python3
# ================================================================
# LYNX_HO_W10_Calibration.py
# Measures the motor line: wheel speed = k (PWM - PWM0), where k is the
# slope (rad/s per PWM) and PWM0 the deadband (below it, no motion).
# ON THE FLOOR, about 1.5 m clear ahead and behind.
# Run on the rover's Pi, from your team folder:
#     python3 General-Resources-Rover/LYNX_HO_W10_Calibration.py
#
# 1. PWM0: the PWM rises by 1 every RAMP_T s, forward then backward,
#    until the encoders see the wheels move (RAMP_MOVE rad).
# 2. k: forward, then back, at PWM 40, 60, 80 (MOVE_T s each), so the
#    rover ends near where it started. The encoders give the wheel
#    speed of each move; with PWM0 fixed, k is the least-squares slope.
# It depends on the floor: calibrate where you will drive.
#
# Every encoder read is logged to calib_<time>.csv.
# The motors stop at the end, on Ctrl+C, a closed SSH session,
# or a lost USB link.
# ================================================================

PWMS   = [40, 60, 80]        # PWMs to try, forward and backward
RAMP_T = 0.15                # s at each PWM step of the ramp
RAMP_MOVE = 0.05             # rad: the wheels have moved
RAMP_MAX  = 70               # PWM: give up the ramp here
MOVE_T = 1.5                 # s at each PWM
SKIP   = 0.5                 # s at the start of each move not used
REST   = 0.5                 # s stopped between moves
N      = 12000               # ticks per wheel turn

import math
import signal
import sys
import time

import serial
from lynx_port import get_port

# ── Serial (as in LYNX_HO_Pit.py) ───────────────────────────────
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

def read_angles():
    """Wheel angles (phiL, phiR) in rad, or None if a read failed."""
    fl, rl = read_pair(5)
    fr, rr = read_pair(6)
    if fl is None or fr is None:
        return None
    return math.pi * (fl + rl) / N, math.pi * (fr + rr) / N

def send_motor(channel, u):
    pwm = min(int(abs(u)), 127)
    sel = (1 if u >= 0 else 2) if channel == 1 else (3 if u >= 0 else 4)
    ser.write(bytes([sel, pwm]))

def stop_all():
    ser.write(bytes([1, 0]))
    ser.write(bytes([3, 0]))

# ── The ramp: first PWM at which both sides move ───────────────
def ramp(sign, log, t0):
    a0 = None
    for p in range(0, RAMP_MAX + 1):
        end = time.monotonic() + RAMP_T
        while time.monotonic() < end:
            send_motor(1, sign * p)
            send_motor(2, sign * p)
            a = read_angles()
            if a:
                log.write(f"{time.monotonic() - t0:.3f},{sign * p},"
                          f"{sign * p},{a[0]:.5f},{a[1]:.5f}\n")
                a0 = a0 or a
                if (abs(a[0] - a0[0]) > RAMP_MOVE
                        and abs(a[1] - a0[1]) > RAMP_MOVE):
                    stop_all()
                    time.sleep(REST)
                    return p
            time.sleep(0.03)
    stop_all()
    raise RuntimeError(f"NO MOTION up to PWM {RAMP_MAX}")

# ── One move: drive at u for MOVE_T s, return the wheel speeds ─
def move(u, log, t0):
    start = time.monotonic()
    first = last = None
    while time.monotonic() - start < MOVE_T:
        send_motor(1, u)
        send_motor(2, u)
        a = read_angles()
        t = time.monotonic()
        if a:
            log.write(f"{t - t0:.3f},{u},{u},{a[0]:.5f},{a[1]:.5f}\n")
            if t - start >= SKIP:
                first = first or (t, a[0], a[1])
                last = (t, a[0], a[1])
        time.sleep(0.05)
    stop_all()
    time.sleep(REST)
    if not first or last[0] - first[0] < 0.3:
        raise RuntimeError("NO ENCODER DATA")
    T = last[0] - first[0]
    return (last[1] - first[1]) / T, (last[2] - first[2]) / T

# ── Run, and always stop the motors on the way out ─────────────
def shutdown(*_):
    raise KeyboardInterrupt

signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)
signal.signal(signal.SIGHUP, shutdown)            # SSH session closed

logf = open(time.strftime("calib_%Y%m%d_%H%M%S.csv"), "w", buffering=1)
logf.write("t_s,uL_pwm,uR_pwm,phiL_rad,phiR_rad\n")
reason = "Done."
rows = []                                         # (PWM, wL, wR)
try:
    input("  Clear 1.5 m ahead and behind. Enter to go, Ctrl+C to stop.")
    print("  Ramp: the PWM rises until the wheels move...")
    t0 = time.monotonic()
    p_fwd = ramp(+1, logf, t0)
    p_bwd = ramp(-1, logf, t0)
    pwm0 = (p_fwd + p_bwd) / 2
    print(f"  wheels start: PWM {p_fwd} forward, {p_bwd} backward")
    print(f"  PWM0 = {pwm0:.1f}\n")
    for p in PWMS:
        for u in (p, -p):                         # forward, then back
            wL, wR = move(u, logf, t0)
            rows.append((u, wL, wR))
            print(f"  PWM {u:+4d}:  left {wL:+7.3f}  right {wR:+7.3f} rad/s")
    # least squares through (PWM0, 0): |w| = k (|PWM| - PWM0)
    xs = [abs(u) - pwm0 for u, wL, wR in rows]
    ys = [(abs(wL) + abs(wR)) / 2 for u, wL, wR in rows]
    k = sum(x * y for x, y in zip(xs, ys)) / sum(x * x for x in xs)
    print(f"\n  k    = {k:.4f} rad/s per PWM   (slope)")
    print(f"  PWM0 = {pwm0:.1f}               (deadband, from the ramp)")
    print(f"  Run with:  ... LYNX_HO_W10_Aut.py {k:.4f} {pwm0:.1f}")
    if k <= 0:
        print("  k is not positive: check the motor or encoder signs.")
except (KeyboardInterrupt, EOFError):
    reason = "Interrupted."
except Exception as e:                            # link lost...
    reason = str(e) or e.__class__.__name__
finally:
    try:                                          # motors first
        stop_all()
        time.sleep(0.1)
        ser.write(bytes([7]))
        ser.close()
    except Exception:
        pass
    logf.close()
    try:                                          # the terminal may be gone
        print(f"\n  {reason} Motors stopped. Log: {logf.name}")
        print("=^..^=")
    except Exception:
        pass
