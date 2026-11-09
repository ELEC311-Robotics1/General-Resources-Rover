#!/usr/bin/python3
# ================================================================
# LYNX_HSV_Tuner.py
# Find the HSV limits of your target with the ROVER's camera.
# No motors. Run on the rover's Pi, from your team folder:
#     python3 General-Resources-Rover/LYNX_HSV_Tuner.py
# then open  http://<rover address>:5000  on a phone or laptop.
#
# Left: the camera. Right: the mask (white = inside the limits).
# Six sliders set LO and HI. Click the camera image to read the
# H, S, V of that pixel. The page and this terminal show the six
# numbers, ready for LYNX_HO_W10_Aut.py. Ctrl+C here to quit.
# ================================================================

LO    = (10, 120, 80)        # starting limits: orange
HI    = (25, 255, 255)
CAM   = 0                    # camera number
CAM_W = 640                  # camera image size asked for, px
CAM_H = 480
JPEG_Q = 60                  # stream quality, 0-100
STREAM_FPS = 10              # pictures per second sent to the page

import logging
import threading
import time

import cv2
import numpy as np
from flask import Flask, Response, request, jsonify

# ── Shared state ────────────────────────────────────────────────
lock    = threading.Lock()
latest  = {"img": None, "n": 0}            # newest camera frame
shown   = {"jpg": None, "n": 0, "hsv": None}
limits  = {"lo": list(LO), "hi": list(HI)}
running = True

# ── Grabber thread: read the camera nonstop, keep only the newest ──
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

# ── Vision thread: newest frame -> mask, side by side, encode ──
def vision():
    last = 0
    while running:
        with lock:
            img, n = latest["img"], latest["n"]
            lo, hi = tuple(limits["lo"]), tuple(limits["hi"])
        if img is None or n == last:
            time.sleep(0.01)
            continue
        last = n
        blur = cv2.GaussianBlur(img, (5, 5), 0)
        hsv  = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lo, hi)
        both = np.hstack([img, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
        ok, jpg = cv2.imencode(".jpg", both,
                               [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
        with lock:
            shown.update(jpg=jpg.tobytes(), n=n, hsv=hsv)

# ── Web page ────────────────────────────────────────────────────
app = Flask(__name__)
logging.getLogger("werkzeug").setLevel(logging.ERROR)

PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>LYNX HSV tuner</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
 body { background:#08080f; color:#d8dce8; font-family:sans-serif; margin:12px; }
 img { width:100%; max-width:1280px; cursor:crosshair; }
 .row { display:flex; align-items:center; gap:8px; margin:4px 0; }
 .row label { width:52px; } .row input { flex:1; } .row span { width:36px; }
 #out { font-family:monospace; font-size:18px; color:#9ee84b; margin:8px 0; }
 #pix { font-family:monospace; color:#22d3ee; }
</style></head><body>
<img id="v" src="/video">
<div id="pix">Click the camera image (left half) to read H, S, V.</div>
<div id="sl"></div>
<div id="out"></div>
<script>
const names = ["H lo","S lo","V lo","H hi","S hi","V hi"];
const tops  = [179,255,255,179,255,255];
const start = %START%;
const sl = document.getElementById("sl");
names.forEach((n, i) => {
  sl.insertAdjacentHTML("beforeend",
    `<div class="row"><label>${n}</label>` +
    `<input type="range" id="s${i}" min="0" max="${tops[i]}" value="${start[i]}">` +
    `<span id="t${i}">${start[i]}</span></div>`);
});
function send() {
  const v = names.map((n, i) => document.getElementById("s" + i).value);
  v.forEach((x, i) => document.getElementById("t" + i).textContent = x);
  fetch("/set?v=" + v.join(","));
  document.getElementById("out").textContent =
    "LO HI for LYNX_HO_W10_Aut.py:  " + v.join(" ");
}
names.forEach((n, i) => document.getElementById("s" + i).oninput = send);
document.getElementById("v").onclick = (e) => {
  const r = e.target.getBoundingClientRect();
  const x = (e.clientX - r.left) / r.width * 2;     // 0..2 (two halves)
  const y = (e.clientY - r.top) / r.height;
  if (x >= 1) return;                               // right half: the mask
  fetch(`/pick?x=${x}&y=${y}`).then(r => r.json()).then(d => {
    document.getElementById("pix").textContent =
      d.ok ? `row ${d.row}, column ${d.col}:  H ${d.h}  S ${d.s}  V ${d.v}`
           : "no frame yet";
  });
};
send();
</script></body></html>"""

@app.route("/")
def index():
    return PAGE.replace("%START%", str(limits["lo"] + limits["hi"]))

@app.route("/set")
def set_limits():
    try:
        v = [int(a) for a in request.args.get("v", "").split(",")]
        assert len(v) == 6
    except (ValueError, AssertionError):
        return "bad", 400
    with lock:
        limits.update(lo=v[:3], hi=v[3:])
    print(f"\r  LO {tuple(v[:3])}  HI {tuple(v[3:])}   ->  "
          f"{' '.join(map(str, v))}      ", end="", flush=True)
    return "ok"

@app.route("/pick")
def pick():
    with lock:
        hsv = shown["hsv"]
    if hsv is None:
        return jsonify(ok=False)
    rows, cols = hsv.shape[:2]
    x, y = float(request.args.get("x", 0)), float(request.args.get("y", 0))
    c = min(cols - 1, max(0, int(x * cols)))
    r = min(rows - 1, max(0, int(y * rows)))
    h, s, v = (int(a) for a in hsv[r, c])
    print(f"\n  row {r}, column {c}:  H {h}  S {s}  V {v}")
    return jsonify(ok=True, row=r, col=c, h=h, s=s, v=v)

def frames():
    last = 0
    while running:
        with lock:
            jpg, n = shown["jpg"], shown["n"]
        if jpg and n != last:              # send each new picture once
            last = n
            yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
            time.sleep(1 / STREAM_FPS)     # spare the Wi-Fi
        else:
            time.sleep(0.02)

@app.route("/video")
def video():
    return Response(frames(), mimetype="multipart/x-mixed-replace; boundary=f")

# ── Run ─────────────────────────────────────────────────────────
if __name__ == "__main__":
    threading.Thread(target=grabber, daemon=True).start()
    threading.Thread(target=vision, daemon=True).start()
    print("  Open http://<rover address>:5000   (Ctrl+C to quit)")
    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        lo, hi = limits["lo"], limits["hi"]
        print(f"\n\n  LO = {tuple(lo)}")
        print(f"  HI = {tuple(hi)}")
        print(f"  For LYNX_HO_W10_Aut.py:  k PWM0 {' '.join(map(str, lo + hi))}")
