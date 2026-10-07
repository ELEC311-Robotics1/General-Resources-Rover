#!/usr/bin/python3
# ================================================================
# LYNX_COM_CONTROL.py
# Position Control — PID and SMC (tanh)
#
# Both wheels track independent reference trajectories.
# No haptic feedback. No data-driven. Just clean control.
#
# Switch controller at the top: CONTROLLER = 'PID' or 'SMC'
#
# Arduino protocol:
#   sel=1,pwm  -> left forward
#   sel=2,pwm  -> left backward
#   sel=3,pwm  -> right forward
#   sel=4,pwm  -> right backward
#   sel=5      -> read left encoders  (FL, RL)
#   sel=6      -> read right encoders (FR, RR)
#   sel=7      -> watchdog reset
# ================================================================

import serial
import time
import numpy as np
import sys
import matplotlib.pyplot as plt

if sys.platform == 'win32':
    import ctypes
    ctypes.windll.winmm.timeBeginPeriod(1)

# ── Controller Selection ─────────────────────────────────────
print("Select controller:")
print("  1 — PID")
print("  2 — SMC (tanh)")
choice = input("> ").strip()
CONTROLLER = 'SMC' if choice == '2' else 'PID'

# ── Parameters ───────────────────────────────────────────────
T             = 0.01
PI            = np.pi
TICKS_PER_REV = 6000
T_sim         = 10.0
PERIOD_MS     = 6000.0
MAX_PWM       = 120

# ── PID Gains ────────────────────────────────────────────────
Kp = 80.0
Ki = 5.0
Kd = 2.0

# ── SMC Gains ────────────────────────────────────────────────
K_smc      = 100.0
LAMBDA_smc = 100.0
PHI        = 0.02

# ── Serial ───────────────────────────────────────────────────
from lynx_port import get_port
ser = serial.Serial(port=get_port(), baudrate=115200, timeout=0.05)
time.sleep(2)
ser.readline()

# ── State ────────────────────────────────────────────────────
eL_prev  = 0.0;  eR_prev  = 0.0
eL_sum   = 0.0;  eR_sum   = 0.0
xL_prev  = 0.0;  xR_prev  = 0.0
xLd_prev = 0.0;  xRd_prev = 0.0

ISE = 0.0;  ITAE = 0.0;  ISC = 0.0
log = []
t = 0.0;  dt = T

# ── Helpers ──────────────────────────────────────────────────
def read_encoders(sel):
    ser.reset_input_buffer()
    ser.write(bytes([sel]))
    try:
        a, b = ser.readline().decode().strip().split()
        a, b = int(a), int(b)
        return (a + b) / 2.0
    except:
        return None

def send_motor(channel, u):
    pwm = min(int(abs(u)), 127)
    sel = (1 if u >= 0 else 2) if channel == 1 else (3 if u >= 0 else 4)
    ser.write(bytes([sel, pwm]))

# ── Reference trajectories ───────────────────────────────────
def ref_left(t_ms):
    return PI * np.sin(2 * PI / PERIOD_MS * t_ms)

def ref_right(t_ms):
    return PI * np.cos(2 * PI / PERIOD_MS * t_ms) * np.tanh(t_ms / 2000.0)

# ── Controllers ──────────────────────────────────────────────
def control_pid(e, e_prev, e_sum, dt):
    """PID: u = Kp*e + Ki*integral(e) + Kd*de/dt"""
    de = (e - e_prev) / dt if dt > 0 else 0.0
    e_sum += e * dt
    e_sum = np.clip(e_sum, -10.0, 10.0)     # anti-windup
    u = Kp * e + Ki * e_sum + Kd * de
    return np.clip(u, -MAX_PWM, MAX_PWM), e_sum

def control_smc(e, e_prev, x, x_prev, xd, xd_prev, dt):
    """SMC: s = de/dt + lambda * e,  u = K * tanh(phi * s)"""
    eDot = ((xd - xd_prev) - (x - x_prev)) / dt if dt > 0 else 0.0
    s = eDot + LAMBDA_smc * e
    u = K_smc * np.tanh(PHI * s)
    return np.clip(u, -MAX_PWM, MAX_PWM)

# ── Main loop ────────────────────────────────────────────────
print("=" * 50)
print(f"  POSITION CONTROL — {CONTROLLER}")
print(f"  Duration: {T_sim}s  |  Ts: {T*1000:.0f}ms")
print("=" * 50)
print("\nExperiment starts\n")

try:
    while t < T_sim:
        t1 = time.perf_counter()
        t += dt

        rawL = read_encoders(5)
        rawR = read_encoders(6)
        if rawL is None or rawR is None:
            dt = time.perf_counter() - t1
            continue

        xL = (PI / TICKS_PER_REV) * rawL
        xR = (PI / TICKS_PER_REV) * rawR

        t_ms = t * 1000.0
        xLd = ref_left(t_ms)
        xRd = ref_right(t_ms)

        eL = xLd - xL
        eR = xRd - xR

        # ── Compute control ──────────────────────────────
        if CONTROLLER == 'PID':
            uL, eL_sum = control_pid(eL, eL_prev, eL_sum, dt)
            uR, eR_sum = control_pid(eR, eR_prev, eR_sum, dt)
        else:
            uL = control_smc(eL, eL_prev, xL, xL_prev, xLd, xLd_prev, dt)
            uR = control_smc(eR, eR_prev, xR, xR_prev, xRd, xRd_prev, dt)

        # ── Send ─────────────────────────────────────────
        send_motor(1, uL)
        send_motor(2, uR)

        # ── Store ────────────────────────────────────────
        eL_prev  = eL;     eR_prev  = eR
        xL_prev  = xL;     xR_prev  = xR
        xLd_prev = xLd;    xRd_prev = xRd

        ISE  += dt * (eL**2  + eR**2)
        ITAE += dt * t * (abs(eL) + abs(eR))
        ISC  += dt * ((uL/MAX_PWM)**2 + (uR/MAX_PWM)**2)

        log.append((t, xLd, xL, eL, uL, xRd, xR, eR, uR, dt))

        elapsed = time.perf_counter() - t1
        if elapsed < T:
            time.sleep(T - elapsed)
        dt = time.perf_counter() - t1

except KeyboardInterrupt:
    pass

finally:
    if sys.platform == 'win32':
        ctypes.windll.winmm.timeEndPeriod(1)
    for _ in range(5):
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.write(bytes([1, 0]))
            ser.write(bytes([3, 0]))
            time.sleep(0.05)
        except:
            pass
    try:
        ser.write(bytes([7]))
        time.sleep(0.1)
        ser.close()
    except:
        pass
    print("\nMotors stopped.")

# ── Save ─────────────────────────────────────────────────────
log = np.array(log)
fname = f"data_{CONTROLLER.lower()}_rover.txt"
np.savetxt(fname, log, delimiter='\t',
           header="t xLd xL eL uL xRd xR eR uR dt")

print(f"\nISE  = {ISE:.4f}")
print(f"ITAE = {ITAE:.4f}")
print(f"ISC  = {ISC:.4f}")
print(f"\nData saved to {fname}")
print("Experiment ends")

# ── Plot ─────────────────────────────────────────────────────
t_log = log[:, 0]

fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
fig.suptitle(f"Position Control — {CONTROLLER}")

axes[0, 0].plot(t_log, log[:, 1], 'k--', label='ref')
axes[0, 0].plot(t_log, log[:, 2], label='act')
axes[0, 0].set_ylabel('pos (rad)');  axes[0, 0].set_title('Left')
axes[0, 0].legend()

axes[0, 1].plot(t_log, log[:, 5], 'k--', label='ref')
axes[0, 1].plot(t_log, log[:, 6], label='act')
axes[0, 1].set_ylabel('pos (rad)');  axes[0, 1].set_title('Right')
axes[0, 1].legend()

axes[1, 0].plot(t_log, log[:, 3], 'r')
axes[1, 0].set_ylabel('error (rad)')

axes[1, 1].plot(t_log, log[:, 7], 'r')
axes[1, 1].set_ylabel('error (rad)')

axes[2, 0].plot(t_log, log[:, 4])
axes[2, 0].set_ylabel('u (PWM)');  axes[2, 0].set_xlabel('t (s)')

axes[2, 1].plot(t_log, log[:, 8])
axes[2, 1].set_ylabel('u (PWM)');  axes[2, 1].set_xlabel('t (s)')

plt.tight_layout()
plt.savefig(f"{CONTROLLER.lower()}_rover.png", dpi=150)
plt.show()
print("=^..^=")