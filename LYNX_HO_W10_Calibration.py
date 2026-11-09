#!/usr/bin/python3
# ================================================================
# LYNX_HO_W10_Calibration.py
# Short runs that measure the numbers the math needs.
# Run on the rover's Pi, from your team folder:
#     python3 General-Resources-Rover/LYNX_HO_W10_Calibration.py area
#     python3 General-Resources-Rover/LYNX_HO_W10_Calibration.py left
#     python3 General-Resources-Rover/LYNX_HO_W10_Calibration.py right
#     python3 General-Resources-Rover/LYNX_HO_W10_Calibration.py straight
#
# area:     AT THE DESK, no motors. Prints the area of the orange target
#           the rover's camera sees. Hold it Z_GOAL mm ahead: that is A*.
# left:     ON THE FLOOR. A circle to the left: left side at PWM_SLOW, right at PWM_FAST,
#           for CIRCLE_T seconds. Prints the wheel speeds. You measure R.
# right:    the mirror image: left at PWM_FAST, right at PWM_SLOW.
# straight: both sides at the same PWM until the wheels turn
#           STRAIGHT_PHI rad. Prints the angles. You measure the distance d.
#
# Every encoder read is logged to calib_<run>_<time>.csv.
# The motors stop at the end, on Ctrl+C, a closed SSH session,
# a lost USB link, or a run that takes too long.
# ================================================================

PWM_SLOW     = 40            # inner side of a circle
PWM_FAST     = 70            # outer side of a circle
CIRCLE_T     = 12.0          # s of circle
SKIP         = 1.5           # s at the start not used (speeding up)
STRAIGHT_PWM = 50            # PWM, both sides, in the straight run
STRAIGHT_PHI = 15.0          # rad of wheel turn in the straight run
TIMEOUT      = 30.0          # s, longest any run may take
LO           = (10, 120, 80) # HSV limits: orange (as Lecture 16)
HI           = (25, 255, 255)
MIN_AREA     = 300           # px^2; smaller blobs are ignored
Z_GOAL       = 500           # mm: where the rover should stop
N            = 12000         # ticks per wheel turn

import math
import signal
import sys
import time

import serial
from lynx_port import get_port

# ── The area run: camera only, no motors ──────────────────────
def area():
    import cv2                             # only this run needs it
    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    print(f"  Hold the target {Z_GOAL} mm in front of the camera,")
    print("  centered. Read A* when it is steady. Ctrl+C to stop.")
    try:
        while True:
            ok, img = cam.read()
            if not ok:
                print("\r  no camera frame          ", end="", flush=True)
                time.sleep(0.5)
                continue
            hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0),
                               cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, LO, HI)
            cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                     cv2.CHAIN_APPROX_SIMPLE)
            cs = [c for c in cs if cv2.contourArea(c) > MIN_AREA]
            if cs:
                m = cv2.moments(max(cs, key=cv2.contourArea))
                cx = m["m10"] / m["m00"]
                print(f"\r  area {m['m00']:7.0f} px^2   column {cx:4.0f}",
                      end="", flush=True)
            else:
                print("\r  no orange target seen     ", end="", flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print()
    finally:
        cam.release()

if len(sys.argv) > 1 and sys.argv[1] == "area":
    area()
    sys.exit(0)

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

# ── The two runs ────────────────────────────────────────────────
def circle(log, pL, pR):
    t0 = time.monotonic()
    first = None                           # (t, phiL, phiR) after SKIP
    last = None
    while time.monotonic() - t0 < CIRCLE_T:
        send_motor(1, pL)
        send_motor(2, pR)
        a = read_angles()
        t = time.monotonic() - t0
        if a:
            log.write(f"{t:.3f},{pL},{pR},"
                      f"{a[0]:.5f},{a[1]:.5f}\n")
            if t >= SKIP:
                first = first or (t, a[0], a[1])
                last = (t, a[0], a[1])
        time.sleep(0.05)
    stop_all()
    if not first or last[0] - first[0] < 1.0:
        raise RuntimeError("NO ENCODER DATA")
    T = last[0] - first[0]
    wL = (last[1] - first[1]) / T
    wR = (last[2] - first[2]) / T
    print(f"\n  left : PWM {pL:3d}   wheel speed {wL:6.3f} rad/s")
    print(f"  right: PWM {pR:3d}   wheel speed {wR:6.3f} rad/s")
    print("  Now measure the radius R of the circle, center to the")
    print("  middle of the rover's path, in mm.")

def straight(log):
    t0 = time.monotonic()
    a0 = None
    while True:
        if time.monotonic() - t0 > TIMEOUT:
            raise TimeoutError("NO PROGRESS: PWM below the deadband?")
        send_motor(1, STRAIGHT_PWM)
        send_motor(2, STRAIGHT_PWM)
        a = read_angles()
        if a:
            t = time.monotonic() - t0
            log.write(f"{t:.3f},{STRAIGHT_PWM},{STRAIGHT_PWM},"
                      f"{a[0]:.5f},{a[1]:.5f}\n")
            a0 = a0 or a
            if ((a[0] - a0[0]) + (a[1] - a0[1])) / 2 >= STRAIGHT_PHI:
                break
        time.sleep(0.05)
    stop_all()
    time.sleep(0.8)                        # let it coast, then read again
    a = read_angles() or a
    dL, dR = a[0] - a0[0], a[1] - a0[1]
    print(f"\n  left  wheel turned {dL:6.2f} rad")
    print(f"  right wheel turned {dR:6.2f} rad")
    print(f"  average           {(dL + dR) / 2:6.2f} rad")
    print("  Now measure the distance d the rover moved, in mm.")

# ── Run, and always stop the motors on the way out ─────────────
def shutdown(*_):
    raise KeyboardInterrupt

signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)
signal.signal(signal.SIGHUP, shutdown)            # SSH session closed

RUNS = {"left":     lambda log: circle(log, PWM_SLOW, PWM_FAST),
        "right":    lambda log: circle(log, PWM_FAST, PWM_SLOW),
        "straight": straight}
run = sys.argv[1] if len(sys.argv) > 1 else ""
if run not in RUNS:
    print("  usage: LYNX_HO_W10_Calibration.py area | left | right "
          "| straight")
    ser.close()
    sys.exit(1)

logf = open(time.strftime(f"calib_{run}_%Y%m%d_%H%M%S.csv"), "w", buffering=1)
logf.write("t_s,uL_pwm,uR_pwm,phiL_rad,phiR_rad\n")
reason = "Done."
try:
    input(f"  {run}: clear the floor, mark the start. "
          "Enter to go, Ctrl+C to stop.")
    RUNS[run](logf)
except (KeyboardInterrupt, EOFError):
    reason = "Interrupted."
except Exception as e:                            # link lost, timeout...
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
