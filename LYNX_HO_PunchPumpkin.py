#!/usr/bin/python3
# ================================================================
# LYNX_HO_PunchPumpkin.py
# Drive the rover from a phone, watching only its camera.
# Run on the rover's Pi, from your team folder:
#     python3 General-Resources-Rover/LYNX_HO_PunchPumpkin.py
# or, with your own HSV limits (H S V low, then H S V high):
#     python3 General-Resources-Rover/LYNX_HO_PunchPumpkin.py 8 140 90 22 255 255
#
# The terminal prints a 4-digit code. On your phone open
#     http://<rover address>:5000/?k=<code>     camera + joystick
#     http://<rover address>:5000               camera only, for anyone
#
# Keys, in this terminal:
#     space   brake on / off (and keyboard PWM to zero)     q   quit
#     w / s   left PWM up / down      o / k   right PWM up / down
# Backup driving: the keys latch, as in Week 8. The last input used
# drives: a key takes over from the joystick, a touch takes it back.
# The motors stop on q, Ctrl+C, a closed SSH session, a lost USB link,
# a camera that stops, and a joystick that goes quiet (finger lifted,
# phone locked, Wi-Fi lost). One driver per rover.
# ================================================================

PWM_MAX = 60                 # full joystick, straight ahead (cap 127)
STEP    = 10                 # keyboard PWM change per press
TURN    = 0.6                # share of PWM_MAX used for turning
QUIET   = 0.3                # s without a joystick message -> stop
STALE   = 0.5                # s without a camera frame -> stop
MIN_AREA = 300               # px^2; smaller orange blobs are ignored
LO = (10, 120, 80)           # HSV limits: orange
HI = (25, 255, 255)
CAM = 0
CAM_W, CAM_H = 640, 480       # camera image size asked for, px

import fcntl
import logging
import os
import random
import select
import signal
import sys
import termios
import threading
import time
import tty

# ── One driver per rover ────────────────────────────────────────
lockf = open("/tmp/lynx_drive.lock", "w")
try:
    fcntl.flock(lockf, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    print("  Rover busy: someone is already driving it.")
    sys.exit(1)

if len(sys.argv) > 1:                        # your own HSV limits
    try:
        v = [int(a) for a in sys.argv[1:]]
        assert len(v) == 6
    except (ValueError, AssertionError):
        print("  Give six whole numbers: H S V low, then H S V high.")
        sys.exit(1)
    LO, HI = tuple(v[:3]), tuple(v[3:])

import cv2
import numpy as np
import serial
from flask import Flask, Response, request
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

# ── Joystick to wheels ──────────────────────────────────────────
def wheels(x, y):
    """(x right, y forward, each -1..1) -> (uL, uR) PWM."""
    x = TURN * max(-1.0, min(1.0, x))
    y = max(-1.0, min(1.0, y))
    uL, uR = y + x, y - x           # right turn: left faster
    s = max(1.0, abs(uL), abs(uR))  # keep both in -1..1
    return round(PWM_MAX * uL / s), round(PWM_MAX * uR / s)

# ── Vision ──────────────────────────────────────────────────────
def find(img):
    """Largest orange blob: (cx, cy, area, theta), or None."""
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
    return cx, cy, m["m00"], th

# ── Shared state ────────────────────────────────────────────────
lock    = threading.Lock()
CODE    = f"{random.randint(0, 9999):04d}"
joy     = {"x": 0.0, "y": 0.0, "t": 0.0}
keyb    = {"on": False, "uL": 0, "uR": 0}    # on: the keys drive
latest  = {"img": None, "n": 0}    # newest camera frame
seen    = {"t": 0.0, "jpg": None, "n": 0}
cmd     = {"uL": 0, "uR": 0, "brake": False, "why": ""}
fault   = {"msg": ""}
running = True

# ── Grabber thread: read the camera nonstop, keep only the newest ──
# The camera driver queues frames; reading them as fast as they arrive
# keeps the picture and the overlay current instead of lagging behind.
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

# ── Camera thread: newest frame -> find, draw, encode ───────────
def camera():
    last = 0
    while running:
        with lock:
            img, n = latest["img"], latest["n"]
        if img is None or n == last:       # nothing new yet
            time.sleep(0.005)
            continue
        last = n
        h, w = img.shape[:2]
        cv2.line(img, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
        found = find(img)
        if found:
            cx, cy, area, th = found
            dx, dy = 60 * np.cos(th), 60 * np.sin(th)
            cv2.line(img, (int(cx - dx), int(cy - dy)),
                     (int(cx + dx), int(cy + dy)), (255, 0, 255), 2)
            cv2.circle(img, (int(cx), int(cy)), 6, (0, 0, 255), -1)
            cv2.putText(img, f"area {area:.0f}", (10, h - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        with lock:
            uL, uR, why = cmd["uL"], cmd["uR"], cmd["why"]
        text = fault["msg"] or f"L {uL:+d}  R {uR:+d}  {why}"
        cv2.putText(img, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2)
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 60])
        with lock:
            seen.update(t=time.monotonic(), jpg=jpg.tobytes(), n=n)

# ── Control loop, 20 Hz: the only code that talks to the Mega ──
def decide(brake, cam_age, keys, joy_age, x, y):
    """First true test wins. Returns (uL, uR, why)."""
    if brake:
        return 0, 0, "BRAKE"
    if cam_age > STALE:     # camera stopped: blind
        return 0, 0, "NO CAMERA"
    if keys is not None:    # last input was a key
        return keys[0], keys[1], "KEYS"
    if joy_age > QUIET:     # thumb lifted, phone gone
        return 0, 0, ""
    uL, uR = wheels(x, y)
    return uL, uR, ""

def run_loop():
    while running:
        now = time.monotonic()
        with lock:
            x, y, tj = joy["x"], joy["y"], joy["t"]
            brake, t_cam = cmd["brake"], seen["t"]
            keys = (keyb["uL"], keyb["uR"]) if keyb["on"] else None
        uL, uR, why = decide(brake, now - t_cam, keys, now - tj, x, y)
        send_motor(1, uL)
        send_motor(2, uR)
        with lock:
            cmd.update(uL=uL, uR=uR, why=why)
        time.sleep(0.05)

def loop():
    try:
        run_loop()
    except Exception as e:                 # USB unplugged, Mega reset...
        fault["msg"] = f"LINK LOST: {e.__class__.__name__}"
        with lock:
            cmd.update(uL=0, uR=0)
        try:
            stop_all()
        except Exception:
            pass

# ── Web pages ───────────────────────────────────────────────────
app = Flask(__name__)
logging.getLogger("werkzeug").setLevel(logging.ERROR)

STYLE = """<meta name="viewport" content="width=device-width, initial-scale=1">
<style> body { background:#08080f; color:#d8dce8; font-family:sans-serif;
               margin:0; text-align:center; }
        img { width:100%; max-width:640px; display:block; margin:0 auto; }
        #pad { width:220px; height:220px; margin:14px auto; border-radius:50%;
               background:#12131f; border:2px solid #22d3ee; position:relative;
               touch-action:none; user-select:none; -webkit-user-select:none; }
        #knob { width:80px; height:80px; border-radius:50%; background:#dc1496;
                position:absolute; left:70px; top:70px; pointer-events:none; }
</style>"""

WATCH = """<!doctype html><html><head><meta charset="utf-8">
<title>LYNX camera</title>""" + STYLE + """</head><body>
<img src="/video"><p>Camera only. To drive, use the code from the terminal.</p>
</body></html>"""

DRIVE = """<!doctype html><html><head><meta charset="utf-8">
<title>LYNX drive</title>""" + STYLE + """</head><body>
<img src="/video">
<div id="pad"><div id="knob"></div></div>
<p>Hold and drag. Let go to stop.</p>
<script>
const K = "@CODE@", R = 110, pad = document.getElementById("pad"),
      knob = document.getElementById("knob");
let x = 0, y = 0, held = false;
function send() {
  fetch("/cmd?k=" + K + "&x=" + x.toFixed(2) + "&y=" + y.toFixed(2),
        {cache: "no-store"}).catch(() => {});
}
function move(e) {
  const b = pad.getBoundingClientRect();
  let dx = e.clientX - b.left - R, dy = e.clientY - b.top - R;
  const d = Math.hypot(dx, dy), lim = R - 40;
  if (d > lim) { dx *= lim / d; dy *= lim / d; }
  knob.style.left = (R - 40 + dx) + "px";
  knob.style.top  = (R - 40 + dy) + "px";
  x = dx / lim; y = -dy / lim;                 // y up = forward
}
function release() {
  held = false; x = 0; y = 0;
  knob.style.left = "70px"; knob.style.top = "70px"; send();
}
pad.addEventListener("pointerdown", e => {
  held = true; pad.setPointerCapture(e.pointerId); move(e); send(); });
pad.addEventListener("pointermove", e => { if (held) move(e); });
pad.addEventListener("pointerup", release);
pad.addEventListener("pointercancel", release);
document.addEventListener("visibilitychange", release);
setInterval(() => { if (held) send(); }, 100);  // keep-alive while held
</script></body></html>"""

@app.route("/")
def index():
    if request.args.get("k") == CODE:
        return DRIVE.replace("@CODE@", CODE)
    return WATCH

@app.route("/cmd")
def joystick():
    if request.args.get("k") != CODE:
        return "", 403
    try:
        x = float(request.args.get("x", 0))
        y = float(request.args.get("y", 0))
    except ValueError:
        return "", 400
    with lock:
        joy.update(x=x, y=y, t=time.monotonic())
        keyb.update(on=False, uL=0, uR=0)    # a touch takes over
    return "", 204

def frames():
    last = 0
    while running:
        with lock:
            jpg, n = seen["jpg"], seen["n"]
        if jpg and n != last:              # send each new picture once
            last = n
            yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        time.sleep(0.02)

@app.route("/video")
def video():
    return Response(frames(), mimetype="multipart/x-mixed-replace; boundary=f")

# ── Keys ────────────────────────────────────────────────────────
KEYS = {"w": ("uL", +STEP), "s": ("uL", -STEP),
        "o": ("uR", +STEP), "k": ("uR", -STEP)}

def handle_key(ch):
    """Apply one key press. Returns False to quit."""
    ch = ch.lower()
    if ch == "q":
        return False
    with lock:
        if ch == " ":
            cmd["brake"] = not cmd["brake"]
            keyb.update(uL=0, uR=0)
        elif ch in KEYS:
            side, d = KEYS[ch]
            keyb["on"] = True                 # a key takes over
            keyb[side] = max(-PWM_MAX, min(PWM_MAX, keyb[side] + d))
    return True

def status():
    with lock:
        uL, uR, why = cmd["uL"], cmd["uR"], cmd["why"]
    msg = fault["msg"] or why
    print(f"\r  code {CODE}   L {uL:+4d}   R {uR:+4d}   {msg:30s}",
          end="", flush=True)

# ── Run, and always stop the motors on the way out ─────────────
def shutdown(*_):
    raise KeyboardInterrupt

signal.signal(signal.SIGTERM, shutdown)
signal.signal(signal.SIGHUP, shutdown)            # SSH session closed

if __name__ == "__main__":
    server = make_server("0.0.0.0", 5000, app, threaded=True)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    threading.Thread(target=grabber, daemon=True).start()
    threading.Thread(target=camera, daemon=True).start()
    worker = threading.Thread(target=loop, daemon=True)
    worker.start()

    print(f"  Drive: http://<rover address>:5000/?k={CODE}")
    print("  Watch: http://<rover address>:5000")
    print(f"  HSV limits: LO {LO}  HI {HI}")
    print("  space brake on/off   q quit")
    print("  backup: w/s left, o/k right (keys latch)\n")
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
