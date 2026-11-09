#!/usr/bin/python3
# ================================================================
# LYNX_HO_W10_Aut.py
# The rover drives itself: it chases an orange target with the camera
# (visual servoing), keeps its pose from the encoders (dead reckoning),
# and drives back to where it started (home).  ON THE FLOOR, clear space.
# Run on the rover's Pi, from your team folder, with YOUR calibration:
#     python3 General-Resources-Rover/LYNX_HO_W10_Aut.py r L k PWM0 A
#         r     wheel radius (mm)           from the straight run
#         L     track width (mm)            from the circles
#         k     motor slope (rad/s per PWM) from the circles
#         PWM0  deadband (PWM)              from the circles
#         A     goal area A* (px^2)         from the area run
# then open  http://<rover address>:5000  on a phone or laptop.
#
# Keys, in this terminal:
#     c       chase the target (camera)
#     h       go home to (0, 0) (encoders only)
#     z       pose = (0, 0, 0) here
#     + / -   turn gain KW up / down
#     space   stop (no mode)              q   quit
# The motors stop on q, space, Ctrl+C, a closed SSH session, a lost USB
# link, and (in chase) when the camera sends no frame for STALE seconds.
# Log: aut_log_<time>.csv in the folder you ran it from.
# ================================================================

LO       = (10, 120, 80)     # HSV limits: orange
HI       = (25, 255, 255)
MIN_AREA = 300               # px^2; smaller blobs are ignored
DEAD_A   = 0.05              # dead zone: this share of A* counts as 0
KV       = 0.05              # chase: mm/s per px^2 of area error
KW       = 0.004             # chase: rad/s per pixel of error
V_MAX    = 250.0             # mm/s, fastest forward or back
KD       = 1.0               # home: mm/s per mm of distance
KTH      = 1.5               # home: rad/s per rad of heading error
HOME_TOL = 50.0              # mm: closer than this is home
PWM_MAX  = 110               # cap on every command (Sabertooth max 127)
N        = 12000             # ticks per wheel turn
CAM      = 0                 # camera number
CAM_W    = 640               # camera image size asked for, px
CAM_H    = 480
JPEG_Q   = 60                # stream quality, 0-100 (smaller: faster)
STALE    = 0.5               # s without a frame -> motors stop (chase)
STREAM_FPS = 10              # pictures per second sent to each phone

import logging
import math
import os
import select
import signal
import sys
import termios
import threading
import time
import tty

import cv2
import numpy as np
import serial
from flask import Flask, Response
from werkzeug.serving import make_server

from lynx_port import get_port

# ── Your calibration, from the command line ────────────────────
try:
    R_EFF, L_EFF, K_MOT, PWM0, A_GOAL = map(float, sys.argv[1:6])
except ValueError:
    print("  usage: LYNX_HO_W10_Aut.py r L k PWM0 A   (your numbers)")
    sys.exit(1)
J = np.array([[R_EFF / 2, R_EFF / 2],       # (v, w) = J (wL, wR)
              [-R_EFF / L_EFF, R_EFF / L_EFF]])

# ── YOUR FUNCTION: inverse kinematics, then the motor line ─────
def wheels_pwm(v, w):
    """PWM (uL, uR) for v (mm/s) and w (rad/s)."""
    # wL, wR = ...  # rad/s: inverse kinematics, solve with J
    # uL, uR = ...  # PWM0 + |wheel speed| / K_MOT, with the
    #               # sign of the wheel speed; 0 if (almost) 0
    uL, uR = 0.0, 0.0   # replace this line
    return uL, uR

# ── Desk check: before any motor turns ─────────────────────────
def desk_check():
    tests = [((0, 0),   lambda L, R: L == 0 and R == 0, "stop"),
             ((100, 0), lambda L, R: abs(L - R) < 1e-6 and L > 0,
              "straight"),
             ((0, 0.5), lambda L, R: R > 0 > L,         "turn left")]
    ok = True
    for (v, w), good, name in tests:
        uL, uR = wheels_pwm(v, w)
        mark = "ok " if good(uL, uR) else "WRONG"
        ok = ok and mark == "ok "
        print(f"  {mark} {name:9s} v={v:3d}, w={w}:  "
              f"uL={uL:7.1f}  uR={uR:7.1f}")
    return ok

# ── Chase: Lecture 18, from the image to v and w ───────────────
def chase(u, A, kw):
    ex = CAM_W / 2 - u                      # pixel error, + left
    e = A_GOAL - A                          # area error, + too far
    if abs(e) < DEAD_A * A_GOAL:
        e = 0.0                             # dead zone: no chatter
    v = max(-V_MAX, min(V_MAX, KV * e))     # mm/s
    w = kw * ex                             # rad/s
    return v, w

# ── Home: from the pose to v and w ─────────────────────────────
def home(x, y, th):
    d = math.hypot(x, y)                    # distance to (0, 0)
    err = math.atan2(-y, -x) - th           # heading error
    err = math.atan2(math.sin(err), math.cos(err))   # to -pi..pi
    w = KTH * err
    v = min(V_MAX, KD * d) if abs(err) < 0.5 else 0.0   # turn first
    return v, w, d

# ── Serial ──────────────────────────────────────────────────────
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
    pwm = min(int(abs(u)), 127)
    sel = (1 if u >= 0 else 2) if channel == 1 else (3 if u >= 0 else 4)
    ser.write(bytes([sel, pwm]))

def stop_all():
    ser.write(bytes([1, 0]))
    ser.write(bytes([3, 0]))

# ── Vision (as in Lecture 16) ───────────────────────────────────
def find(img):
    """Largest orange blob: (cx, area), or None."""
    blur = cv2.GaussianBlur(img, (5, 5), 0)
    hsv  = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LO, HI)
    cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                             cv2.CHAIN_APPROX_SIMPLE)
    cs = [c for c in cs if cv2.contourArea(c) > MIN_AREA]
    if not cs:
        return None
    m = cv2.moments(max(cs, key=cv2.contourArea))
    return m["m10"] / m["m00"], m["m00"]

# ── Shared state ────────────────────────────────────────────────
lock    = threading.Lock()
latest  = {"img": None, "n": 0}
seen    = {"cx": None, "area": 0.0, "t": 0.0, "jpg": None, "n": 0,
           "fps": 0.0}
cmd     = {"mode": "", "uL": 0, "uR": 0, "kw": KW,
           "zero": False}
pose    = {"x": 0.0, "y": 0.0, "th": 0.0, "d": 0.0}
fault   = {"msg": ""}
running = True
t0      = time.monotonic()
logf    = open(time.strftime("aut_log_%Y%m%d_%H%M%S.csv"), "w", buffering=1)
logf.write("t_s,mode,uL_pwm,uR_pwm,phiL_rad,phiR_rad,x_mm,y_mm,th_rad\n")

# ── Camera threads (as in Lecture 16) ──────────────────────────
def open_camera():
    cam = cv2.VideoCapture(CAM)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cam

def grabber():
    cam = open_camera()
    while running:
        ok, img = cam.read()
        if not ok:                         # camera unplugged or busy
            cam.release()
            time.sleep(1.0)
            cam = open_camera()
            continue
        with lock:
            latest.update(img=img, n=latest["n"] + 1)
    cam.release()

def camera():
    last, t_fps, count, fps = 0, time.monotonic(), 0, 0.0
    while running:
        with lock:
            img, n = latest["img"], latest["n"]
        if img is None or n == last:
            time.sleep(0.005)
            continue
        last = n
        h, w = img.shape[:2]
        found = find(img)
        cv2.line(img, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
        if found:
            cv2.circle(img, (int(found[0]), h // 2), 8, (0, 0, 255), -1)
        count += 1
        if time.monotonic() - t_fps >= 1.0:
            fps = count / (time.monotonic() - t_fps)
            count, t_fps = 0, time.monotonic()
        with lock:
            m, uL, uR = cmd["mode"], cmd["uL"], cmd["uR"]
            x, y, th = pose["x"], pose["y"], pose["th"]
        text = fault["msg"] or (f"{m or 'stop':5s}  L {uL:+4d} R {uR:+4d}"
                                f"  x {x:5.0f} y {y:5.0f}"
                                f"  th {math.degrees(th):4.0f}")
        cv2.putText(img, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2)
        cv2.putText(img, f"{w}x{h}  {fps:.0f} fps", (10, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        ok, jpg = cv2.imencode(".jpg", img,
                               [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
        with lock:
            seen.update(cx=found[0] if found else None,
                        area=found[1] if found else 0.0, n=n, fps=fps,
                        t=time.monotonic(), jpg=jpg.tobytes())

# ── Control loop, 20 Hz: the only code that talks to the Mega ──
def run_loop():
    prev = None                            # last wheel angles
    while running:
        fl, rl = read_pair(5)
        fr, rr = read_pair(6)
        if fl is None or fr is None:
            raise IOError("no encoder reply")
        phiL = math.pi * (fl + rl) / N     # rad, average of the pair
        phiR = math.pi * (fr + rr) / N
        with lock:
            if cmd["zero"]:
                pose.update(x=0.0, y=0.0, th=0.0)
                cmd["zero"] = False
            if prev:                       # dead reckoning, Lecture 17
                dL, dR = phiL - prev[0], phiR - prev[1]
                ds = R_EFF * (dL + dR) / 2
                pose["x"] += ds * math.cos(pose["th"])
                pose["y"] += ds * math.sin(pose["th"])
                pose["th"] += R_EFF * (dR - dL) / L_EFF
            prev = (phiL, phiR)
            x, y, th = pose["x"], pose["y"], pose["th"]
            mode, kw = cmd["mode"], cmd["kw"]
            cx, area = seen["cx"], seen["area"]
            fresh = time.monotonic() - seen["t"] < STALE

        v = w = 0.0
        if mode == "chase" and fresh and cx is not None:
            v, w = chase(cx, area, kw)
        elif mode == "home":
            v, w, d = home(x, y, th)
            if d < HOME_TOL:
                v = w = 0.0
                with lock:
                    cmd["mode"] = ""
        uL, uR = wheels_pwm(v, w)
        s = max(1.0, abs(uL) / PWM_MAX, abs(uR) / PWM_MAX)
        uL, uR = round(uL / s), round(uR / s)   # scale both: keep the turn
        send_motor(1, uL)
        send_motor(2, uR)
        with lock:
            cmd.update(uL=uL, uR=uR)
        t = time.monotonic() - t0
        logf.write(f"{t:.3f},{mode or 'stop'},{uL},{uR},"
                   f"{phiL:.5f},{phiR:.5f},{x:.1f},{y:.1f},{th:.4f}\n")
        time.sleep(0.05)

def loop():
    try:
        run_loop()
    except Exception as e:                 # USB unplugged, Mega reset...
        fault["msg"] = f"LINK LOST: {e.__class__.__name__}"
        with lock:
            cmd.update(mode="", uL=0, uR=0)
        try:
            stop_all()
        except Exception:
            pass

# ── Web page: the camera only ───────────────────────────────────
app = Flask(__name__)
logging.getLogger("werkzeug").setLevel(logging.ERROR)

PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>LYNX autonomous</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style> body { background:#08080f; color:#d8dce8; font-family:sans-serif; margin:16px; }
        img { max-width:100%; } </style></head><body>
<p>Keys in the SSH terminal: c chase, h home, z zero pose, + / - turn gain, space stop, q quit.</p>
<img src="/video">
</body></html>"""

@app.route("/")
def index():
    return PAGE

def frames():
    last = 0
    while running:
        with lock:
            jpg, n = seen["jpg"], seen["n"]
        if jpg and n != last:              # send each new picture once
            last = n
            yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
            time.sleep(1 / STREAM_FPS)
        else:
            time.sleep(0.02)

@app.route("/video")
def video():
    return Response(frames(), mimetype="multipart/x-mixed-replace; boundary=f")

# ── Keys ────────────────────────────────────────────────────────
def handle_key(ch):
    """Apply one key press. Returns False to quit."""
    ch = ch.lower()
    if ch == "q":
        return False
    with lock:
        if ch == " ":
            cmd["mode"] = ""
        elif ch == "c" and not fault["msg"]:
            cmd["mode"] = "chase"
        elif ch == "h" and not fault["msg"]:
            cmd["mode"] = "home"
        elif ch == "z":
            cmd["zero"] = True
        elif ch in "+=":
            cmd["kw"] = cmd["kw"] * 1.25
        elif ch == "-":
            cmd["kw"] = cmd["kw"] / 1.25
    return True

def status():
    with lock:
        m, uL, uR, kw = cmd["mode"], cmd["uL"], cmd["uR"], cmd["kw"]
        x, y, th = pose["x"], pose["y"], pose["th"]
        cx, fps = seen["cx"], seen["fps"]
    where = "-" if cx is None else f"{cx:4.0f}"
    msg = fault["msg"]
    print(f"\r  {m or 'stop':5s}  L {uL:+4d} R {uR:+4d}  "
          f"x {x:6.0f} y {y:6.0f} th {math.degrees(th):5.0f}  "
          f"cx {where:>4s}  KW {kw:.4f}  {fps:3.0f} fps  {msg:12s}",
          end="", flush=True)

# ── Run, and always stop the motors on the way out ─────────────
def shutdown(*_):
    raise KeyboardInterrupt

signal.signal(signal.SIGTERM, shutdown)
signal.signal(signal.SIGHUP, shutdown)            # SSH session closed

if __name__ == "__main__":
    print(f"  Calibration: r {R_EFF} mm, L {L_EFF} mm, "
          f"k {K_MOT} rad/s per PWM, PWM0 {PWM0}, A* {A_GOAL} px^2")
    if not desk_check():
        print("  wheels_pwm is not right yet: fix it, then run again.")
        stop_all()
        ser.close()
        sys.exit(1)
    server = make_server("0.0.0.0", 5000, app, threaded=True)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    threading.Thread(target=grabber, daemon=True).start()
    threading.Thread(target=camera, daemon=True).start()
    worker = threading.Thread(target=loop, daemon=True)
    worker.start()

    print("  Camera: http://<rover address>:5000")
    print("  c chase  h home  z zero  + / - KW  space stop  q quit\n")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)                         # one key at a time
        status()
        while True:
            r, _, _ = select.select([fd], [], [], 0.2)
            if r:
                keys = os.read(fd, 64).decode(errors="ignore")
                if not all(handle_key(ch) for ch in keys):
                    break
            status()
    except KeyboardInterrupt:
        pass
    finally:
        running = False                           # motors first, always
        worker.join(timeout=1.0)
        try:
            stop_all()
            time.sleep(0.1)
            ser.write(bytes([7]))
            ser.close()
        except Exception:
            pass
        logf.close()
        try:                                      # the terminal may be gone
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            print(f"\n\n  Motors stopped. Log: {logf.name}")
            print("=^..^=")
        except Exception:
            pass
        server.shutdown()
