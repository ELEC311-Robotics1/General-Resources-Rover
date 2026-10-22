# ================================================================
# W9_hsv_tuner.py
# Find the HSV limits of your object with the laptop webcam.
#     py W9_hsv_tuner.py
#
# Left: the camera. Right: the mask. Six sliders set LO and HI.
# Click the camera image to print the H, S, V of that pixel.
# Keys, with the window selected:
#     s   save a screenshot        q   quit, print LO and HI
# ================================================================

CAM   = 0                  # webcam number
WIDTH = 480                # width of each half on screen, px
LO    = (10, 120, 80)      # starting limits: orange
HI    = (25, 255, 255)

import time
import cv2
import numpy as np

WIN   = "W9_hsv_tuner"
NAMES = ["H lo", "S lo", "V lo", "H hi", "S hi", "V hi"]
TOPS  = [179, 255, 255, 179, 255, 255]

cam = cv2.VideoCapture(CAM)
cv2.namedWindow(WIN)
for name, top, start in zip(NAMES, TOPS, LO + HI):
    cv2.createTrackbar(name, WIN, start, top, lambda v: None)

view = {"hsv": None, "scale": 1.0}

def on_click(event, x, y, flags, param):
    if event != cv2.EVENT_LBUTTONDOWN or view["hsv"] is None or x >= WIDTH:
        return
    r, c = int(y / view["scale"]), int(x / view["scale"])
    h, s, v = view["hsv"][r, c]
    print(f"  row {r}, column {c}:  H {h}  S {s}  V {v}")

cv2.setMouseCallback(WIN, on_click)

lo, hi = LO, HI
while True:
    ok, img = cam.read()
    if not ok:
        print("  No frame from the camera.")
        break
    vals = [cv2.getTrackbarPos(n, WIN) for n in NAMES]
    lo, hi = tuple(vals[:3]), tuple(vals[3:])

    blur = cv2.GaussianBlur(img, (5, 5), 0)
    hsv  = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lo, hi)

    scale = WIDTH / img.shape[1]
    view.update(hsv=hsv, scale=scale)
    left  = cv2.resize(img, None, fx=scale, fy=scale)
    right = cv2.resize(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), None,
                       fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    shown = np.hstack([left, right])
    cv2.putText(shown, f"LO = {lo}   HI = {hi}", (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.imshow(WIN, shown)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    if key == ord("s"):
        name = time.strftime("hsv_tuner_%H%M%S.png")
        cv2.imwrite(name, shown)
        print(f"  Saved {name}")

cam.release()
cv2.destroyAllWindows()
print(f"  LO = {lo}")
print(f"  HI = {hi}")
