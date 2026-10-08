#!/usr/bin/python3
# ================================================================
# LYNX_Joy.py
# Drive the rover from a phone: a touch joystick in the browser.
# Run on the rover's Pi:
#     python3 LYNX_Joy.py
# then open  http://<rover address>:5000  on the phone.
#
# Stick up/down: forward/backward.  Stick left/right: turn.
# Lift your finger and the rover stops. If the phone stops sending
# (screen locked, Wi-Fi lost) the rover stops after DEADMAN seconds.
# Data is logged to joy_log_<time>.csv in the folder you ran it from.
# ================================================================

PWM_MAX  = 120               # cap on each side (Sabertooth max 127)
TURN_MAX = 60                # PWM added/subtracted for a full sideways stick
DEADMAN  = 0.5               # s without an update -> motors stop
N        = 12000             # ticks per wheel turn

import logging
import math
import signal
import threading
import time

import serial
from flask import Flask, request

from lynx_port import get_port

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
    pwm = min(int(abs(u)), PWM_MAX)
    sel = (1 if u >= 0 else 2) if channel == 1 else (3 if u >= 0 else 4)
    ser.write(bytes([sel, pwm]))

def stop_all():
    ser.write(bytes([1, 0]))
    ser.write(bytes([3, 0]))

# ── Shared state ────────────────────────────────────────────────
lock    = threading.Lock()
cmd     = {"uL": 0, "uR": 0, "t": 0.0}      # t: time of the last update
running = True
t0      = time.monotonic()
logf    = open(time.strftime("joy_log_%Y%m%d_%H%M%S.csv"), "w", buffering=1)
logf.write("t_s,uL_pwm,uR_pwm,phiL_rad,phiR_rad\n")

def stick_to_pwm(x, y):
    """Stick x (right +) and y (up +), each in [-1, 1], to side PWMs."""
    x = max(-1.0, min(1.0, x))
    y = max(-1.0, min(1.0, y))
    v, w = y * PWM_MAX, x * TURN_MAX            # forward part, turning part
    uL = max(-PWM_MAX, min(PWM_MAX, v + w))     # stick right: left side faster
    uR = max(-PWM_MAX, min(PWM_MAX, v - w))
    return int(uL), int(uR)

# ── Control loop, 20 Hz: the only code that talks to the Mega ──
def loop():
    try:
        while running:
            with lock:
                fresh = time.monotonic() - cmd["t"] < DEADMAN
                uL, uR = (cmd["uL"], cmd["uR"]) if fresh else (0, 0)
            send_motor(1, uL)
            send_motor(2, uR)
            fl, rl = read_pair(5)
            fr, rr = read_pair(6)
            if fl is not None and fr is not None:
                phiL = math.pi * (fl + rl) / N
                phiR = math.pi * (fr + rr) / N
                logf.write(f"{time.monotonic() - t0:.3f},{uL},{uR},"
                           f"{phiL:.5f},{phiR:.5f}\n")
            time.sleep(0.05)
    except Exception:                         # USB unplugged, Mega reset...
        try:
            stop_all()
        except Exception:
            pass

# ── Web page ────────────────────────────────────────────────────
app = Flask(__name__)
logging.getLogger("werkzeug").setLevel(logging.ERROR)

PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>LYNX joystick</title>
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<style>
 html, body { margin:0; height:100%; background:#08080f; color:#d8dce8;
              font-family:sans-serif; touch-action:none; overflow:hidden; }
 #pad { display:block; margin:24px auto 8px; }
 #txt { text-align:center; font-size:1.2em; }
</style></head><body>
<canvas id="pad" width="300" height="300"></canvas>
<div id="txt">touch and drag</div>
<script>
const c = document.getElementById("pad"), g = c.getContext("2d"),
      txt = document.getElementById("txt"), R = 120;
let x = 0, y = 0, held = false;

function draw() {
  g.clearRect(0, 0, 300, 300);
  g.strokeStyle = "#6e7590"; g.lineWidth = 2;
  g.beginPath(); g.arc(150, 150, R, 0, 2 * Math.PI); g.stroke();
  g.fillStyle = held ? "#22d3ee" : "#dc1496";
  g.beginPath(); g.arc(150 + x * R, 150 - y * R, 28, 0, 2 * Math.PI); g.fill();
}
function move(e) {
  const r = c.getBoundingClientRect(), p = e.touches ? e.touches[0] : e;
  let dx = (p.clientX - r.left - 150) / R, dy = -(p.clientY - r.top - 150) / R;
  const m = Math.hypot(dx, dy);
  if (m > 1) { dx /= m; dy /= m; }
  x = dx; y = dy; draw();
}
function send() {
  fetch("/cmd", {method: "POST", headers: {"Content-Type": "application/json"},
                 body: JSON.stringify({x: held ? x : 0, y: held ? y : 0})})
    .then(r => r.json()).then(d => { txt.textContent = "L " + d.uL + "   R " + d.uR; })
    .catch(() => { txt.textContent = "no connection"; });
}
function down(e) { held = true; move(e); e.preventDefault(); }
function up(e)   { held = false; x = 0; y = 0; draw(); send(); }

c.addEventListener("touchstart", down); c.addEventListener("touchmove", move);
c.addEventListener("touchend", up);     c.addEventListener("mousedown", down);
c.addEventListener("mousemove", e => { if (held) move(e); });
window.addEventListener("mouseup", up);
setInterval(() => { if (held) send(); }, 100);       // 10 updates per second
draw();
</script></body></html>"""

@app.route("/")
def index():
    return PAGE

@app.route("/cmd", methods=["POST"])
def command():
    data = request.get_json(silent=True) or {}
    try:
        x, y = float(data.get("x", 0)), float(data.get("y", 0))
    except (TypeError, ValueError):
        x, y = 0.0, 0.0
    if not (math.isfinite(x) and math.isfinite(y)):
        x, y = 0.0, 0.0
    uL, uR = stick_to_pwm(x, y)
    with lock:
        cmd.update(uL=uL, uR=uR, t=time.monotonic())
    return {"uL": uL, "uR": uR}

# ── Run, and always stop the motors on the way out ─────────────
def shutdown(*_):
    raise KeyboardInterrupt

signal.signal(signal.SIGTERM, shutdown)
signal.signal(signal.SIGHUP, shutdown)        # SSH session closed

if __name__ == "__main__":
    worker = threading.Thread(target=loop, daemon=True)
    worker.start()
    print("  Joystick: http://<rover address>:5000   Ctrl+C to quit")
    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        worker.join(timeout=1.0)
        stop_all()
        time.sleep(0.1)
        ser.write(bytes([7]))
        ser.close()
        logf.close()
        print("\n  Motors stopped. Log saved.")
        print("=^..^=")
