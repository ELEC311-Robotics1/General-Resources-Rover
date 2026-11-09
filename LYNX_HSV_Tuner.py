#!/usr/bin/python3
# ================================================================
# LYNX_HSV_Tuner.py
# Find the HSV limits of your target with the ROVER's camera.
# No motors. Run on the rover's Pi, from your team folder:
#     python3 General-Resources-Rover/LYNX_HSV_Tuner.py
# then open  http://<rover address>:5000  on a phone or laptop.
#
# Left: the camera. Right: the mask (white = inside the limits).
# Six sliders, by channel: H lo, H hi, S lo, S hi, V lo, V hi.
# Click the target (either half) for an estimate of its colour from a
# small patch: median and spread of H, S, V, and suggested limits
# ("use these" puts them on the sliders). The page and this terminal
# show LO and HI, ready for LYNX_HO_W10_Aut.py. Ctrl+C here to quit.
# ================================================================

LO    = (10, 120, 80)        # starting limits: orange
HI    = (25, 255, 255)
CAM   = 0                    # camera number
CAM_W = 640                  # camera image size asked for, px
CAM_H = 480
JPEG_Q = 60                  # stream quality, 0-100
STREAM_FPS = 10              # pictures per second made and sent
SHRINK = 0.5                 # size of the streamed picture (half)
PATCH  = 7                   # px: a click reads a (2*PATCH+1)^2 patch
MARGIN_H = 5                 # suggested limits: H spread + this
MARGIN_SV = 40               # suggested limits: S, V low end - this

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

# ── Vision thread: STREAM_FPS times a second, newest frame only ─
def vision():
    last = 0
    while running:
        t_next = time.monotonic() + 1 / STREAM_FPS
        with lock:
            img, n = latest["img"], latest["n"]
            lo, hi = tuple(limits["lo"]), tuple(limits["hi"])
        if img is not None and n != last:
            last = n
            work(img, n, lo, hi)
        time.sleep(max(0.0, t_next - time.monotonic()))

def work(img, n, lo, hi):
    """Mask, picture pair at half size, JPEG; keep full-size HSV."""
    blur = cv2.GaussianBlur(img, (5, 5), 0)
    hsv  = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)   # full size: clicks
    mask = cv2.inRange(hsv, lo, hi)
    small = cv2.resize(img, None, fx=SHRINK, fy=SHRINK)
    smask = cv2.resize(mask, None, fx=SHRINK, fy=SHRINK,
                       interpolation=cv2.INTER_NEAREST)
    both = np.hstack([small, cv2.cvtColor(smask, cv2.COLOR_GRAY2BGR)])
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
 img { width:100%; max-width:960px; cursor:crosshair; }
 .row { display:flex; align-items:center; gap:8px; margin:4px 0; }
 .row label { width:52px; } .row input { flex:1; } .row span { width:36px; }
 #out { font-family:monospace; font-size:18px; color:#9ee84b; margin:8px 0; }
 #pix { font-family:monospace; color:#22d3ee; line-height:1.5; }
 button { font-size:14px; margin-left:8px; }
</style></head><body>
<img id="v" src="/video">
<div id="pix">Click the target for an estimate of its H, S, V.</div>
<div id="sl"></div>
<div id="out"></div>
<script>
// values are kept in LO HI order: Hl Sl Vl Hh Sh Vh (index 0..5)
const names = ["H lo","S lo","V lo","H hi","S hi","V hi"];
const tops  = [179,255,255,179,255,255];
const order = [0, 3, 1, 4, 2, 5];       // shown: H lo, H hi, S lo, ...
const start = %START%;
const sl = document.getElementById("sl");
order.forEach((i) => {
  sl.insertAdjacentHTML("beforeend",
    `<div class="row"><label>${names[i]}</label>` +
    `<input type="range" id="s${i}" min="0" max="${tops[i]}" value="${start[i]}">` +
    `<span id="t${i}">${start[i]}</span></div>`);
});
let suggested = null;
let pending = false;                 // at most one request per 0.2 s
function values() {
  return names.map((n, i) => document.getElementById("s" + i).value);
}
function show() {
  const v = values();
  v.forEach((x, i) => document.getElementById("t" + i).textContent = x);
  document.getElementById("out").textContent =
    "LO HI for LYNX_HO_W10_Aut.py:  " + v.join(" ");
  if (!pending) {
    pending = true;
    setTimeout(() => { fetch("/set?v=" + values().join(",")); pending = false; }, 200);
  }
}
names.forEach((n, i) => document.getElementById("s" + i).oninput = show);
document.getElementById("v").onclick = (e) => {
  const r = e.target.getBoundingClientRect();
  const x = ((e.clientX - r.left) / r.width * 2) % 1;   // either half
  const y = (e.clientY - r.top) / r.height;
  fetch(`/pick?x=${x}&y=${y}`).then(r => r.json()).then(d => {
    const pix = document.getElementById("pix");
    if (!d.ok) { pix.textContent = "no frame yet"; return; }
    suggested = d.lo.concat(d.hi);
    pix.innerHTML =
      `row ${d.row}, column ${d.col}, patch ${d.size}x${d.size}:<br>` +
      `median  H ${d.med[0]}  S ${d.med[1]}  V ${d.med[2]}<br>` +
      `spread  H ${d.p5[0]}-${d.p95[0]}  S ${d.p5[1]}-${d.p95[1]}` +
      `  V ${d.p5[2]}-${d.p95[2]}<br>` +
      `suggested  LO ${d.lo.join(" ")}  HI ${d.hi.join(" ")} ` +
      `<button id="use">use these</button>`;
    document.getElementById("use").onclick = () => {
      suggested.forEach((x, i) => document.getElementById("s" + i).value = x);
      show();
    };
  });
};
show();
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
    patch = hsv[max(0, r - PATCH):r + PATCH + 1,
                max(0, c - PATCH):c + PATCH + 1].reshape(-1, 3)
    med = np.median(patch, axis=0).astype(int)
    p5 = np.percentile(patch, 5, axis=0).astype(int)
    p95 = np.percentile(patch, 95, axis=0).astype(int)
    lo = [max(0, int(p5[0]) - MARGIN_H),
          max(0, int(p5[1]) - MARGIN_SV), max(0, int(p5[2]) - MARGIN_SV)]
    hi = [min(179, int(p95[0]) + MARGIN_H), 255, 255]
    print(f"\n  row {r}, column {c}: median H {med[0]} S {med[1]} V {med[2]}"
          f"   suggested LO {tuple(lo)} HI {tuple(hi)}")
    return jsonify(ok=True, row=r, col=c, size=2 * PATCH + 1,
                   med=[int(a) for a in med], p5=[int(a) for a in p5],
                   p95=[int(a) for a in p95], lo=lo, hi=hi)

def frames():
    last = 0
    while running:
        with lock:
            jpg, n = shown["jpg"], shown["n"]
        if jpg and n != last:              # send each new picture once
            last = n
            yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        time.sleep(1 / STREAM_FPS)         # spare the Wi-Fi and the Pi

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
