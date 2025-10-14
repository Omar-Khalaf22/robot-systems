"""
Lab 2 — Task 1 (Physical robot, 0.60 m corridor)
Drive straight and stop at 1.00 m from the front wall using a PID in RPM space.
Adds side safety for the 0.60 m corridor so you don't scrape rails.

Run on robot (repo root):  PYTHONPATH=src python -m robot_systems.okhalaf_Lab2_Task1
"""
import time
from statistics import median

# --- HamBot import (package or same-folder fallback) ---
try:
    from robot_systems.robot import HamBot
except Exception:
    from robot import HamBot  # fallback if you run directly beside robot.py

# ========== PHYSICAL MAZE SETTINGS ==========
PHYSICAL = True
SQUARE_M = 0.60 if PHYSICAL else 1.00
CORRIDOR_W = SQUARE_M

DT = 0.032
TARGET_M = 1.00        # rubric target
RPM_MAX = 60.0         # keep under hardware clamp (±75)
FORWARD_SIGN = -1

# Approach/safety tuned for 0.60 m corridor
FRONT_SLOW_M = 1.20 if PHYSICAL else 1.80
SAFETY_STOP_M = 0.20 if PHYSICAL else 0.25

# Side safety zones (meters) for the narrow corridor
SIDE_SLOW_M = 0.12
SIDE_MIN_M  = 0.09

# PID gains (RPM per meter of error); start conservative
KP = 120.0
KI = 0.0
KD = 20.0
I_MAX = 150.0
D_ALPHA = 0.25  # derivative low-pass (0=no filter, 1=all filter)

# ---------- helpers ----------
def clamp(x, lo, hi): 
    return lo if x < lo else hi if x > hi else x

def _to_meters(x):
    # Accept -1 / 0 / None as invalid
    if x is None or x <= 0:
        return None
    # Heuristic: raw RPLidar wrappers often give mm; indoors meters rarely > 10
    return x / 1000.0 if x > 10.0 else x

def _median_meters(vals):
    vals_m = [_to_meters(v) for v in vals]
    vals_m = [v for v in vals_m if v is not None]
    if not vals_m:
        return None
    return median(vals_m)

def _window(vals, center_idx, half=3):
    n = len(vals)
    idxs = [(center_idx + i) % n for i in range(-half, half + 1)]
    return [vals[k] for k in idxs]

def lidar_front_m(ranges):  return _median_meters(_window(ranges, 180, 3))
def lidar_left_m(ranges):   return _median_meters(_window(ranges,  90, 3))
def lidar_right_m(ranges):  return _median_meters(_window(ranges, 270, 3))

class PID:
    def __init__(self, kp, ki, kd, dt, i_max=I_MAX, d_alpha=D_ALPHA):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.i = 0.0
        self.prev_e = 0.0
        self.d = 0.0
        self.i_max = abs(i_max)
        self.d_alpha = d_alpha
    def reset(self):
        self.i = 0.0; self.prev_e = 0.0; self.d = 0.0
    def step(self, e):
        # integral (anti-windup clamp)
        self.i += e * self.dt
        self.i = clamp(self.i, -self.i_max, self.i_max)
        # derivative (low-pass filtered)
        d_raw = (e - self.prev_e) / self.dt if self.dt > 0 else 0.0
        self.d = (1 - self.d_alpha) * d_raw + self.d_alpha * self.d
        self.prev_e = e
        return self.kp * e + self.ki * self.i + self.kd * self.d

# ---------- controller ----------
def controller_step(bot, pid):
    ranges = bot.get_range_image()          # 360 values, typically in mm
    front_m = lidar_front_m(ranges)
    left_m  = lidar_left_m(ranges)
    right_m = lidar_right_m(ranges)

    # Safety: invalid or dangerously close -> stop
    if front_m is None or (front_m is not None and front_m < SAFETY_STOP_M):
        bot.set_left_motor_speed(0.0)
        bot.set_right_motor_speed(0.0)
        print(f"front={front_m} -> SAFETY STOP")
        return

    # PID on front error (meters) -> RPM
    e = TARGET_M - front_m
    u_rpm = pid.step(e)

    # Slow down as we approach (front slow zone)
    if front_m < FRONT_SLOW_M:
        u_rpm = clamp(u_rpm, -25.0, 25.0)

    # Side safety in the narrow corridor
    if (left_m  is not None and left_m  < SIDE_MIN_M) or \
       (right_m is not None and right_m < SIDE_MIN_M):
        u_rpm = 0.0
    elif (left_m  is not None and left_m  < SIDE_SLOW_M) or \
         (right_m is not None and right_m < SIDE_SLOW_M):
        u_rpm = clamp(u_rpm, -25.0, 25.0)

    # Final clamp and command
    u_rpm = clamp(u_rpm, -RPM_MAX, RPM_MAX)
    cmd = FORWARD_SIGN * u_rpm
    bot.set_left_motor_speed(cmd)
    bot.set_right_motor_speed(cmd)

    print(f"front={front_m:.3f}m  e={e:+.3f}  L/R={u_rpm:+.1f} rpm  "
          f"sideL={left_m} sideR={right_m}")

def run():
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    pid = PID(KP, KI, KD, DT)
    try:
        while True:
            controller_step(bot, pid)
            time.sleep(DT)
    except KeyboardInterrupt:
        pass
    finally:
        bot.stop_motors()
        try: bot.disconnect_robot()
        except Exception: pass

if __name__ == "__main__":
    run()

#----------------FIRST TEST---------------------

#------------------------BREAK CODE------------

"""# okhalaf_Lab2_Task1.py  — HamBot direct, physical maze defaults (0.60 m tiles)
import time, argparse
from statistics import median

# --- HamBot import (package or flat layout) ---
try:
    from robot_systems.robot import HamBot
except Exception:
    try:
        from robot import HamBot
    except Exception as e:
        raise ImportError("Could not import HamBot from robot_systems/robot.py or robot.py") from e


# ----------------- helpers -----------------
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def _to_meters(x):
    # Accept -1/0/None as invalid
    if x is None or x <= 0:
        return None
    # Heuristic: your LiDAR driver returns mm; meters would be <= ~10 in lab
    return x / 1000.0 if x > 10.0 else x

def _median_meters(vals):
    vals_m = [_to_meters(v) for v in vals]
    vals_m = [v for v in vals_m if v is not None]
    if not vals_m:
        return None
    return median(vals_m)

def _window(vals, center_idx, half=3):
    n = len(vals)
    idxs = [(center_idx + i) % n for i in range(-half, half + 1)]
    return [vals[k] for k in idxs]

def lidar_front_m(ranges):  # 180° front
    return _median_meters(_window(ranges, 180, 3))

def lidar_left_m(ranges):   # 90° left
    return _median_meters(_window(ranges,  90, 3))

def lidar_right_m(ranges):  # 270° right
    return _median_meters(_window(ranges, 270, 3))


class PID:
    def __init__(self, kp, ki, kd, dt, i_max=150.0, d_alpha=0.25):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.i = 0.0
        self.prev_e = 0.0
        self.d = 0.0
        self.i_max = abs(i_max)
        self.d_alpha = d_alpha  # derivative low-pass blend
    def reset(self):
        self.i = 0.0; self.prev_e = 0.0; self.d = 0.0
    def step(self, e):
        # Integral with anti-windup clamp
        self.i += e * self.dt
        self.i = clamp(self.i, -self.i_max, self.i_max)
        # Derivative on error with low-pass smoothing
        d_raw = (e - self.prev_e) / self.dt if self.dt > 0 else 0.0
        self.d = (1 - self.d_alpha) * d_raw + self.d_alpha * self.d
        self.prev_e = e
        return self.kp * e + self.ki * self.i + self.kd * self.d


# ----------------- defaults (PHYSICAL) -----------------
DT = 0.032
PHYSICAL_DEFAULT = True   # physical maze by default (0.60 m)

def physical_params():
    return dict(
        TARGET_M=1.00,          # rubric target
        RPM_MAX=60.0,           # stay under ±75 clamp
        KP=120.0, KI=0.0, KD=20.0,
        FRONT_SLOW_M=1.20,      # start being gentle when close
        SAFETY_STOP_M=0.20,     # hard stop if too close
        SIDE_SLOW_M=0.12,       # corridor side slow zone
        SIDE_MIN_M=0.09         # corridor side hard stop
    )

def sim_params():
    return dict(
        TARGET_M=1.00,
        RPM_MAX=60.0,
        KP=120.0, KI=0.0, KD=20.0,
        FRONT_SLOW_M=1.80,
        SAFETY_STOP_M=0.25,
        SIDE_SLOW_M=0.18,
        SIDE_MIN_M=0.12
    )


def controller_step(bot, pid, P):
    ranges = bot.get_range_image()
    front = lidar_front_m(ranges)
    left  = lidar_left_m(ranges)
    right = lidar_right_m(ranges)

    if front is None:
        bot.stop_motors()
        print("front=None -> stop")
        return

    e = P["TARGET_M"] - front                     # meters
    u_rpm = pid.step(e)
    u_rpm = clamp(u_rpm, -P["RPM_MAX"], P["RPM_MAX"])

    # side corridor safety (physical 0.60 m)
    if (left  is not None and left  < P["SIDE_MIN_M"]) or \
       (right is not None and right < P["SIDE_MIN_M"]):
        u_rpm = 0.0
    elif (left  is not None and left  < P["SIDE_SLOW_M"]) or \
         (right is not None and right < P["SIDE_SLOW_M"]):
        u_rpm = clamp(u_rpm, -25.0, 25.0)

    # gentle stop near front wall
    if front < P["SAFETY_STOP_M"]:
        u_rpm = 0.0
    elif front < P["FRONT_SLOW_M"]:
        u_rpm = clamp(u_rpm, -35.0, 35.0)

    bot.set_left_motor_speed(u_rpm)
    bot.set_right_motor_speed(u_rpm)
    print(f"front={front:.3f}m err={e:+.3f} -> rpm={u_rpm:+.1f}  | left={left} right={right}")


def main():
    ap = argparse.ArgumentParser(description="Lab2 Task1: PID stop @1.0m (HamBot)")
    ap.add_argument("--sim", action="store_true", help="use sim thresholds instead of physical")
    args = ap.parse_args()

    P = sim_params() if args.sim else physical_params()

    pid = PID(P["KP"], P["KI"], P["KD"], DT, i_max=150.0, d_alpha=0.25)
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    try:
        while True:
            controller_step(bot, pid, P)
            time.sleep(DT)
    except KeyboardInterrupt:
        pass
    finally:
        bot.stop_motors()
        try: bot.disconnect_robot()
        except Exception: pass


if __name__ == "__main__":
    main()
"""

#-----------OG CODE-------------------

"""
USFID_Lab2_Task1.py — Task 1: PID Forward Wall Stop (HamBot direct)

- Uses HamBot.get_range_image() (0° back, 90° left, 180° front, 270° right)
- PID computes a motor RPM command directly (no geometry needed).
- Commands both wheels to same RPM for straight approach.

Safety:
  * RPM clamped to HamBot's ±75 RPM envelope.
  * Stops if front distance is None/invalid or < safety threshold.
"""


"""
import time
from statistics import median

# --- Imports for HamBot (support both package and flat file layouts). Remove this when sure that an issue won't happen---
try:
    from robot_systems.robot import HamBot
except Exception:
    try:
        from robot import HamBot  # same folder fallback
    except Exception as e:
        raise ImportError("Could not import HamBot. Ensure robot.py or robot_systems/robot.py is in the same folder.") from e

# Units helper: RPLidar typically returns millimeters; convert to meters when needed.
def _to_meters(x):
    # Accept -1 (invalid), return None for invalid/zero-ish
    if x is None or x <= 0:
        return None
    # Heuristic: if value is > 10, assume mm -> m
    return x / 1000.0 if x > 9.0 else x

def _median_meters(vals):
    vals_m = [ _to_meters(v) for v in vals ]
    vals_m = [ v for v in vals_m if v is not None ]
    if not vals_m:
        return None
    return median(vals_m)

def _window(vals, center_idx, half=3): #AI is here fam 
    n = len(vals)
    idxs = [ (center_idx + i) % n for i in range(-half, half+1) ]
    return [ vals[k] for k in idxs ]

def lidar_front_m(ranges):  # 180° is front per your Lidar wrapper
    return _median_meters(_window(ranges, 180, 3))

def lidar_left_m(ranges):   # 90° left
    return _median_meters(_window(ranges, 90, 3))

def lidar_right_m(ranges):  # 270° right
    return _median_meters(_window(ranges, 270, 3))

def lidar_diag_left_m(ranges):   # 135°
    return _median_meters(_window(ranges, 135, 2))

def lidar_diag_right_m(ranges):  # 225°
    return _median_meters(_window(ranges, 225, 2))

# Simple PID that outputs in "RPM units" (we drive motors directly by RPM)
class PID:
    def __init__(self, kp, ki, kd, dt, i_max=100.0, d_alpha=0.2):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.i = 0.0
        self.prev_e = 0.0
        self.d = 0.0
        self.i_max = abs(i_max)
        self.d_alpha = d_alpha
    def reset(self):
        self.i = 0.0; self.prev_e = 0.0; self.d = 0.0
    def step(self, e):
        self.i += e * self.dt
        if self.i > self.i_max: self.i = self.i_max
        if self.i < -self.i_max: self.i = -self.i_max
        d_raw = (e - self.prev_e) / self.dt if self.dt > 0 else 0.0
        self.d = (1 - self.d_alpha) * d_raw + self.d_alpha * self.d
        self.prev_e = e
        return self.kp*e + self.ki*self.i + self.kd*self.d

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

DT = 0.032
TARGET_M = 1.0
SAFETY_STOP_M = 0.25

# Tune in RPM domain. Start with P-only, then add D, then tiny I.
KP = 120.0     # rpm per meter error
KI = 0.0
KD = 20.0

RPM_MAX = 60.0  # keep margin under HamBot's ±75 clamp

def controller_step(bot, pid):
    ranges = bot.get_range_image()
    y = lidar_front_m(ranges)  # meters
    if y is None:
        bot.stop_motors()
        print("front=None -> stop")
        return

    e = TARGET_M - y  # meters
    u_rpm = clamp(pid.step(e), -RPM_MAX, RPM_MAX)

    # Hard stop near wall, gentle
    if y < SAFETY_STOP_M:
        u_rpm = 0.0

    # Drive both wheels forward/back equally
    bot.set_left_motor_speed(u_rpm)
    bot.set_right_motor_speed(u_rpm)

    print(f"front={y:.3f} m  err={e:+.3f} -> rpm={u_rpm:+.1f}")

def run():
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    pid = PID(KP, KI, KD, DT, i_max=150.0, d_alpha=0.25)
    try:
        while True:
            controller_step(bot, pid)
            time.sleep(DT)
    except KeyboardInterrupt:
        pass
    finally:
        bot.stop_motors()
        try:
            bot.disconnect_robot()
        except Exception:
            pass

if __name__ == "__main__":
    run()

    """