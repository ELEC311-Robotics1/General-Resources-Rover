#!/usr/bin/python3
# ================================================================
# LYNX_HO_Pit.py
# Drive the rover from a browser. Run on the rover's Pi:
#     python3 LYNX_HO_Pit.py
# then open  http://<rover address>:5000  on your laptop.
#
# Every command runs for its hold time, then the motors stop.
# Data is logged to pit_log_<time>.csv in the folder you ran it from.
# Ctrl+C to stop.
# ================================================================

PIT_PWM_MAX = 80             # cap on every command
HOLD_MAX    = 5.0            # s, longest hold allowed
N           = 12000          # ticks per wheel turn
WINDOW      = 15.0           # s of history on the plot

import io
import math
import signal
import sys
import threading
import time
from collections import deque

import serial
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from flask import Flask, Response, redirect, request

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
    pwm = min(int(abs(u)), PIT_PWM_MAX)
    sel = (1 if u >= 0 else 2) if channel == 1 else (3 if u >= 0 else 4)
    ser.write(bytes([sel, pwm]))

def stop_all():
    ser.write(bytes([1, 0]))
    ser.write(bytes([3, 0]))

# ── Shared state ────────────────────────────────────────────────
lock    = threading.Lock()
cmd     = {"uL": 0, "uR": 0, "until": 0.0}
hist    = deque()                     # (t, phiL, phiR)
running = True
t0      = time.monotonic()
logf    = open(time.strftime("pit_log_%Y%m%d_%H%M%S.csv"), "w", buffering=1)
logf.write("t,uL,uR,phiL,phiR\n")

# ── Control loop, 20 Hz: the only code that talks to the Mega ──
def loop():
    try:
        run_loop()
    except Exception as e:                 # USB unplugged, Mega reset...
        fault["msg"] = f"LINK LOST: {e.__class__.__name__}"
        try:
            stop_all()
        except Exception:
            pass

fault = {"msg": ""}

def run_loop():
    while running:
        t = time.monotonic() - t0
        with lock:
            live = time.monotonic() < cmd["until"]
            uL, uR = (cmd["uL"], cmd["uR"]) if live else (0, 0)
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

# ── Plot ────────────────────────────────────────────────────────
plt.style.use("dark_background")
fig, ax = plt.subplots(figsize=(7, 3.6), dpi=90)
lineL, = ax.plot([], [], color="#22d3ee", label=r"$\varphi_L$")
lineR, = ax.plot([], [], color="#dc1496", label=r"$\varphi_R$")
ax.set_xlabel("t (s)")
ax.set_ylabel("wheel angle (rad)")
ax.grid(alpha=0.25)
ax.legend(loc="upper left")
ax.set_title("stopped", fontsize=10)
fig.tight_layout()
plot_lock = threading.Lock()

def frame():
    with lock:
        data = list(hist)
        uL, uR, left = cmd["uL"], cmd["uR"], cmd["until"] - time.monotonic()
    with plot_lock:
        if data:
            t, pL, pR = zip(*data)
            lineL.set_data(t, pL)
            lineR.set_data(t, pR)
            ax.set_xlim(max(0, t[-1] - WINDOW), max(WINDOW, t[-1]))
            lo, hi = min(pL + pR), max(pL + pR)
            pad = max(1.0, 0.1 * (hi - lo))
            ax.set_ylim(lo - pad, hi + pad)
        state = f"L {uL:+d}  R {uR:+d}   {left:.1f} s left" if left > 0 else "stopped"
        if fault["msg"]:
            state = fault["msg"] + "  (restart the program)"
        ax.set_title(state, fontsize=10)
        buf = io.BytesIO()
        fig.savefig(buf, format="jpg")
    return buf.getvalue()

def stream():
    while running:
        jpg = frame()
        yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        time.sleep(0.2)

# ── Web page ────────────────────────────────────────────────────
app = Flask(__name__)

PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>LYNX pit</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
 body {{ background:#08080f; color:#d8dce8; font-family:sans-serif; margin:16px; }}
 input {{ width:5em; font-size:1.1em; }}
 button {{ font-size:1.1em; padding:6px 18px; }}
 .stop {{ background:#dc1496; color:white; border:none; }}
 img {{ max-width:100%; }}
</style></head><body>
<form method="post" action="/go">
 Left <input name="uL" type="number" min="-{cap}" max="{cap}" value="{uL}">
 Right <input name="uR" type="number" min="-{cap}" max="{cap}" value="{uR}">
 Hold <input name="hold" type="number" min="0.5" max="{hmax}" step="0.5" value="{hold}"> s
 <button>Go</button>
</form>
<form method="post" action="/stop"><button class="stop">STOP</button></form>
<p><img src="/plot"></p>
</body></html>"""

last = {"uL": 0, "uR": 0, "hold": 3.0}

def number(name, lo, hi, default):
    try:
        return min(max(float(request.form.get(name, default)), lo), hi)
    except ValueError:
        return default

@app.route("/")
def index():
    return PAGE.format(cap=PIT_PWM_MAX, hmax=HOLD_MAX, **last)

@app.route("/go", methods=["POST"])
def go():
    uL = int(number("uL", -PIT_PWM_MAX, PIT_PWM_MAX, 0))
    uR = int(number("uR", -PIT_PWM_MAX, PIT_PWM_MAX, 0))
    hold = number("hold", 0.5, HOLD_MAX, 3.0)
    last.update(uL=uL, uR=uR, hold=hold)
    with lock:
        cmd.update(uL=uL, uR=uR, until=time.monotonic() + hold)
    return redirect("/")

@app.route("/stop", methods=["POST"])
def stop():
    with lock:
        cmd.update(uL=0, uR=0, until=0.0)
    return redirect("/")

@app.route("/plot")
def plot():
    return Response(stream(), mimetype="multipart/x-mixed-replace; boundary=f")

# ── Run, and always stop the motors on the way out ─────────────
def shutdown(*_):
    raise KeyboardInterrupt

signal.signal(signal.SIGTERM, shutdown)
if hasattr(signal, "SIGHUP"):
    signal.signal(signal.SIGHUP, shutdown)       # SSH session closed

if __name__ == "__main__":
    worker = threading.Thread(target=loop, daemon=True)
    worker.start()
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
        print("\nMotors stopped. Log saved.")
        print("=^..^=")
