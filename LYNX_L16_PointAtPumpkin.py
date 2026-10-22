#!/usr/bin/python3
# ================================================================
# LYNX_L16_PointAtPumpkin.py
# The rover turns toward an orange object.  ROVER ON BLOCKS.
# Run on the rover's Pi, from your team folder:
#     python3 General-Resources-Rover/LYNX_L16_PointAtPumpkin.py
# then open  http://<rover address>:5000  on a phone or laptop.
#
# The image is split into three bands. Object in the left band: turn
# left. Right band: turn right. Middle band, or nothing seen: stop.
#
# Keys, in this terminal:
#     m       motors on / off (they start off)
#     space   motors off                  q   quit
# The motors stop on q, Ctrl+C, a closed SSH session, a lost USB link,
# and when the camera sends no frame for STALE seconds.
# ================================================================

LO       = (10, 120, 80)     # HSV limits: orange
HI       = (25, 255, 255)
MIN_AREA = 300               # px^2; smaller blobs are ignored
PWM      = 40                # turning PWM, both sides
CAM      = 0                 # camera number
STALE    = 0.5               # s without a frame -> motors stop

import logging
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

# ── Serial ──────────────────────────────────────────────────────
ser = serial.Serial(port=get_port(), baudrate=115200, timeout=0.05)
time.sleep(2)
ser.readline()

def send_motor(channel, u):
    pwm = min(int(abs(u)), 127)
    sel = (1 if u >= 0 else 2) if channel == 1 else (3 if u >= 0 else 4)
    ser.write(bytes([sel, pwm]))

def stop_all():
    ser.write(bytes([1, 0]))
    ser.write(bytes([3, 0]))

# ── Vision ──────────────────────────────────────────────────────
def find(img):
    """Largest orange blob: (cx, cy, theta) in px, px, rad, or None."""
    blur = cv2.GaussianBlur(img, (5, 5), 0)
    hsv  = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LO, HI)
    cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                             cv2.CHAIN_APPROX_SIMPLE)
    cs = [c for c in cs if cv2.contourArea(c) > MIN_AREA]
    if not cs:
        return None
    m = cv2.moments(max(cs, key=cv2.contourArea))
    cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
    th = 0.5 * np.arctan2(2 * m["mu11"], m["mu20"] - m["mu02"])
    return cx, cy, th

def steer(cx, w):
    """Bands: (uL, uR) from the centroid column cx and image width w."""
    if cx is None:
        return 0, 0
    if cx < w / 3:
        return -PWM, +PWM          # left band: turn left
    if cx > 2 * w / 3:
        return +PWM, -PWM          # right band: turn right
    return 0, 0                    # middle band: stop

# ── Shared state ────────────────────────────────────────────────
lock    = threading.Lock()
seen    = {"cx": None, "w": 640, "t": 0.0, "jpg": None}
cmd     = {"on": False, "uL": 0, "uR": 0, "fresh": False}
fault   = {"msg": ""}
running = True

# ── Camera thread: grab, find, draw, encode ─────────────────────
def camera():
    cam = cv2.VideoCapture(CAM)
    while running:
        ok, img = cam.read()
        if not ok:                         # camera unplugged or busy
            cam.release()
            time.sleep(1.0)
            cam = cv2.VideoCapture(CAM)
            continue
        h, w = img.shape[:2]
        found = find(img)
        for x in (w // 3, 2 * w // 3):     # band edges
            cv2.line(img, (x, 0), (x, h), (255, 255, 0), 1)
        if found:
            cx, cy, th = found
            dx, dy = 60 * np.cos(th), 60 * np.sin(th)
            cv2.line(img, (int(cx - dx), int(cy - dy)),
                     (int(cx + dx), int(cy + dy)), (255, 0, 255), 2)
            cv2.circle(img, (int(cx), int(cy)), 6, (0, 0, 255), -1)
        with lock:
            on, uL, uR = cmd["on"], cmd["uL"], cmd["uR"]
        text = fault["msg"] or (f"motors {'ON ' if on else 'OFF'}"
                                f"  L {uL:+d}  R {uR:+d}")
        cv2.putText(img, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2)
        ok, jpg = cv2.imencode(".jpg", img)
        with lock:
            seen.update(cx=found[0] if found else None, w=w,
                        t=time.monotonic(), jpg=jpg.tobytes())
    cam.release()

# ── Control loop, 20 Hz: the only code that talks to the Mega ──
def run_loop():
    while running:
        with lock:
            fresh = time.monotonic() - seen["t"] < STALE
            cx, w, on = seen["cx"], seen["w"], cmd["on"]
        uL, uR = steer(cx, w) if (on and fresh) else (0, 0)
        send_motor(1, uL)
        send_motor(2, uR)
        with lock:
            cmd.update(uL=uL, uR=uR, fresh=fresh)
        time.sleep(0.05)

def loop():
    try:
        run_loop()
    except Exception as e:                 # USB unplugged, Mega reset...
        fault["msg"] = f"LINK LOST: {e.__class__.__name__}"
        with lock:
            cmd.update(on=False, uL=0, uR=0)
        try:
            stop_all()
        except Exception:
            pass

# ── Web page: the camera only ───────────────────────────────────
app = Flask(__name__)
logging.getLogger("werkzeug").setLevel(logging.ERROR)

PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>LYNX eyes</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style> body { background:#08080f; color:#d8dce8; font-family:sans-serif; margin:16px; }
        img { max-width:100%; } </style></head><body>
<p>Keys in the SSH terminal: m motors on/off, space off, q quit.</p>
<img src="/video">
</body></html>"""

@app.route("/")
def index():
    return PAGE

def frames():
    while running:
        with lock:
            jpg = seen["jpg"]
        if jpg:
            yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        time.sleep(0.1)

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
            cmd["on"] = False
        elif ch == "m" and not fault["msg"]:
            cmd["on"] = not cmd["on"]
    return True

def status():
    with lock:
        on, uL, uR, fresh = cmd["on"], cmd["uL"], cmd["uR"], cmd["fresh"]
        cx = seen["cx"]
    where = "-" if cx is None else f"{cx:4.0f}"
    cam = "" if fresh else "NO CAMERA"
    msg = fault["msg"] or cam
    print(f"\r  motors {'ON ' if on else 'OFF'}   L {uL:+4d}   R {uR:+4d}"
          f"   cx {where:>4s}   {msg:30s}", end="", flush=True)

# ── Run, and always stop the motors on the way out ─────────────
def shutdown(*_):
    raise KeyboardInterrupt

signal.signal(signal.SIGTERM, shutdown)
signal.signal(signal.SIGHUP, shutdown)            # SSH session closed

if __name__ == "__main__":
    server = make_server("0.0.0.0", 5000, app, threaded=True)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    threading.Thread(target=camera, daemon=True).start()
    worker = threading.Thread(target=loop, daemon=True)
    worker.start()

    print("  Camera: http://<rover address>:5000")
    print("  m motors on/off   space off   q quit\n")
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
        try:                                      # the terminal may be gone
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            print("\n\n  Motors stopped.")
            print("=^..^=")
        except Exception:
            pass
        server.shutdown()
