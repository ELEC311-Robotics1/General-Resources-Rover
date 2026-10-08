#!/usr/bin/python3
# ================================================================
# LYNX_HO_Pit.py
# Drive the rover from the keyboard; watch the wheels in a browser.
# Run on the rover's Pi:
#     python3 LYNX_HO_Pit.py
# then open  http://<rover address>:5000  on your laptop.
#
# Keys, in this terminal:
#     w / s   left  PWM up / down        o / k   right PWM up / down
#     space   both to zero               q       quit
#
# Data is logged to pit_log_<time>.csv in the folder you ran it from.
# ================================================================

PWM_MAX = 120                # cap on every command (Sabertooth max 127)
STEP    = 10                 # PWM change per key press
N       = 12000              # ticks per wheel turn
WINDOW  = 15.0               # s of history on the plot

import io
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
from collections import deque

import serial
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from flask import Flask, Response
from werkzeug.serving import make_server

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
cmd     = {"uL": 0, "uR": 0}
hist    = deque()                     # (t, phiL, phiR)
fault   = {"msg": ""}
running = True
t0      = time.monotonic()
logf    = open(time.strftime("pit_log_%Y%m%d_%H%M%S.csv"), "w", buffering=1)
logf.write("t_s,uL_pwm,uR_pwm,phiL_rad,phiR_rad\n")

# ── Control loop, 20 Hz: the only code that talks to the Mega ──
def run_loop():
    while running:
        t = time.monotonic() - t0
        with lock:
            uL, uR = cmd["uL"], cmd["uR"]
        send_motor(1, uL)
        send_motor(2, uR)

        fl, rl = read_pair(5)
        fr, rr = read_pair(6)
        if fl is not None and fr is not None:
            phiL = math.pi * (fl + rl) / N    # average of the pair, rad
            phiR = math.pi * (fr + rr) / N
            with lock:
                hist.append((t, phiL, phiR))
                while hist and hist[0][0] < t - WINDOW:
                    hist.popleft()
            logf.write(f"{t:.3f},{uL},{uR},{phiL:.5f},{phiR:.5f}\n")
        time.sleep(0.05)

def loop():
    try:
        run_loop()
    except Exception as e:                 # USB unplugged, Mega reset...
        fault["msg"] = f"LINK LOST: {e.__class__.__name__}"
        try:
            stop_all()
        except Exception:
            pass

# ── Plot ────────────────────────────────────────────────────────
plt.style.use("dark_background")
fig, ax = plt.subplots(figsize=(7, 3.6), dpi=90)
lineL, = ax.plot([], [], color="#22d3ee", label=r"$\varphi_L$")
lineR, = ax.plot([], [], color="#dc1496", label=r"$\varphi_R$")
ax.set_xlabel("t (s)")
ax.set_ylabel("wheel angle (rad)")
ax.grid(alpha=0.25)
ax.legend(loc="upper left")
ax.set_title("L +0  R +0", fontsize=10)
fig.tight_layout()
plot_lock = threading.Lock()

def frame():
    with lock:
        data = list(hist)
        uL, uR = cmd["uL"], cmd["uR"]
    with plot_lock:
        if data:
            t, pL, pR = zip(*data)
            lineL.set_data(t, pL)
            lineR.set_data(t, pR)
            ax.set_xlim(max(0, t[-1] - WINDOW), max(WINDOW, t[-1]))
            lo, hi = min(pL + pR), max(pL + pR)
            pad = max(1.0, 0.1 * (hi - lo))
            ax.set_ylim(lo - pad, hi + pad)
        title = fault["msg"] or f"L {uL:+d}  R {uR:+d}"
        ax.set_title(title, fontsize=10)
        buf = io.BytesIO()
        fig.savefig(buf, format="jpg")
    return buf.getvalue()

def stream():
    while running:
        jpg = frame()
        yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        time.sleep(0.2)

# ── Web page: the plot only ─────────────────────────────────────
app = Flask(__name__)
logging.getLogger("werkzeug").setLevel(logging.ERROR)   # keep the terminal clean

PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>LYNX pit</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style> body { background:#08080f; color:#d8dce8; font-family:sans-serif; margin:16px; }
        img { max-width:100%; } </style></head><body>
<p>Keys in the SSH terminal: w/s left, o/k right, space stop, q quit.</p>
<img src="/plot">
</body></html>"""

@app.route("/")
def index():
    return PAGE

@app.route("/plot")
def plot():
    return Response(stream(), mimetype="multipart/x-mixed-replace; boundary=f")

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
            cmd.update(uL=0, uR=0)
        elif ch in KEYS:
            side, d = KEYS[ch]
            cmd[side] = max(-PWM_MAX, min(PWM_MAX, cmd[side] + d))
    return True

def status():
    with lock:
        uL, uR = cmd["uL"], cmd["uR"]
    msg = fault["msg"] or ""
    print(f"\r  L {uL:+4d}   R {uR:+4d}   {msg:30s}", end="", flush=True)

# ── Run, and always stop the motors on the way out ─────────────
def shutdown(*_):
    raise KeyboardInterrupt

signal.signal(signal.SIGTERM, shutdown)
signal.signal(signal.SIGHUP, shutdown)            # SSH session closed

if __name__ == "__main__":
    server = make_server("0.0.0.0", 5000, app, threaded=True)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    worker = threading.Thread(target=loop, daemon=True)
    worker.start()

    print("  Plot: http://<rover address>:5000")
    print("  w/s left   o/k right   space stop   q quit\n")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)                         # one key at a time, no Enter
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
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        running = False
        worker.join(timeout=1.0)
        server.shutdown()
        stop_all()
        time.sleep(0.1)
        ser.write(bytes([7]))
        ser.close()
        logf.close()
        print("\n\n  Motors stopped. Log saved.")
        print("=^..^=")