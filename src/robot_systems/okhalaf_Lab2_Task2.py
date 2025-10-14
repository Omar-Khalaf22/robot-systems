"""
Lab 2 — Task 2 (Physical robot, 0.60 m corridor)
Wall-following with a tiny FSM and a side-distance PID that outputs RPM delta.

Run on robot (repo root):  PYTHONPATH=src python -m robot_systems.okhalaf_Lab2_Task2
Toggle LEFT_WALL = True/False below to choose side.
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
LEFT_WALL = True  # set False for right-wall

# Lateral target ~1/3 of a 0.60 m lane keeps room for turns
SIDE_TARGET_M = 0.20

# Forward speed profile (RPM)
BASE_CRUISE_RPM = 40.0
BASE_SLOW_RPM   = 25.0
TURN_RPM        = 35.0
RPM_MAX         = 60.0

# Front gating tuned for short lanes
FRONT_SLOW_M  = 0.65   # start slowing
FRONT_TURN_M  = 0.35   # commit to 90° turn
FRONT_CLEAR_M = 0.55   # resume FOLLOW after turn

# Edge-wrap (convex corner) heuristics
EDGE_RISE_M        = 0.18  # sudden increase in side distance
EDGE_DIAG_CLEAR_M  = 0.45  # diagonal must be open to wrap

# Side PID (error in meters -> delta RPM)
KP_A = 220.0
KI_A = 0.0
KD_A = 40.0
I_A_MAX = 200.0
D_A_ALPHA = 0.25

# ---------- helpers ----------
def clamp(x, lo, hi): 
    return lo if x < lo else hi if x > hi else x

def _to_meters(x):
    if x is None or x <= 0:
        return None
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

def lidar_front_m(ranges):      return _median_meters(_window(ranges, 180, 3))
def lidar_left_m(ranges):       return _median_meters(_window(ranges,  90, 3))
def lidar_right_m(ranges):      return _median_meters(_window(ranges, 270, 3))
def lidar_diag_left_m(ranges):  return _median_meters(_window(ranges, 135, 2))
def lidar_diag_right_m(ranges): return _median_meters(_window(ranges, 225, 2))

class PID:
    def __init__(self, kp, ki, kd, dt, i_max=I_A_MAX, d_alpha=D_A_ALPHA):
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
        self.i = clamp(self.i, -self.i_max, self.i_max)
        d_raw = (e - self.prev_e) / self.dt if self.dt > 0 else 0.0
        self.d = (1 - self.d_alpha) * d_raw + self.d_alpha * self.d
        self.prev_e = e
        return self.kp * e + self.ki * self.i + self.kd * self.d

# ---------- FSM ----------
class FSM:
    FOLLOW, FRONT_TURN, EDGE_WRAP = range(3)

def controller_step(bot, pid):
    ranges = bot.get_range_image()
    d_front = lidar_front_m(ranges)
    d_side  = lidar_left_m(ranges) if LEFT_WALL else lidar_right_m(ranges)
    d_diag  = lidar_diag_left_m(ranges) if LEFT_WALL else lidar_diag_right_m(ranges)

    # init persistent state
    if not hasattr(controller_step, "state"):
        controller_step.state = FSM.FOLLOW
        controller_step.prev_side = d_side if d_side is not None else SIDE_TARGET_M

    state = controller_step.state
    prev_side = controller_step.prev_side

    # ----- transitions -----
    if state == FSM.FOLLOW:
        if d_front is not None and d_front < FRONT_TURN_M:
            state = FSM.FRONT_TURN
        elif (d_side is not None and prev_side is not None and
              (d_side - prev_side) > EDGE_RISE_M and
              (d_diag is not None and d_diag > EDGE_DIAG_CLEAR_M)):
            state = FSM.EDGE_WRAP

    elif state == FSM.FRONT_TURN:
        if (d_front is not None and d_front > FRONT_CLEAR_M and
            d_side is not None and abs(SIDE_TARGET_M - d_side) < 0.12):
            state = FSM.FOLLOW

    elif state == FSM.EDGE_WRAP:
        if d_side is not None and abs(SIDE_TARGET_M - d_side) < 0.12:
            state = FSM.FOLLOW

    controller_step.state = state

    # ----- control per state -----
    if state == FSM.FOLLOW:
        # base forward rpm from front clearance
        base = BASE_CRUISE_RPM if (d_front is not None and d_front > FRONT_SLOW_M) else BASE_SLOW_RPM
        # side error -> rpm delta (flip sign for right-wall)
        if d_side is None:
            delta = 0.0
        else:
            e_side = (SIDE_TARGET_M - d_side) if LEFT_WALL else (d_side - SIDE_TARGET_M)
            delta = pid.step(e_side) * (1.0 if LEFT_WALL else -1.0)

        l_rpm = clamp(base + delta, -RPM_MAX, RPM_MAX)
        r_rpm = clamp(base - delta, -RPM_MAX, RPM_MAX)

    elif state == FSM.FRONT_TURN:
        # in-place rotation toward the tracked wall
        delta = TURN_RPM if LEFT_WALL else -TURN_RPM
        l_rpm = clamp(-delta, -RPM_MAX, RPM_MAX)
        r_rpm = clamp( delta, -RPM_MAX, RPM_MAX)

    else:  # EDGE_WRAP: arc forward while turning toward the wall
        base = BASE_SLOW_RPM
        delta = 0.75 * (TURN_RPM if LEFT_WALL else -TURN_RPM)
        l_rpm = clamp(base - delta, -RPM_MAX, RPM_MAX)
        r_rpm = clamp(base + delta, -RPM_MAX, RPM_MAX)

    bot.set_left_motor_speed(l_rpm)
    bot.set_right_motor_speed(r_rpm)
    controller_step.prev_side = d_side

    side = 'L' if LEFT_WALL else 'R'
    print(f"st={state}  front={d_front}  {side}={d_side}  diag={d_diag}  -> L={l_rpm:+.1f} R={r_rpm:+.1f}")

def run():
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    pid = PID(KP_A, KI_A, KD_A, DT)
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







"""
USFID_Lab2_Task2.py — Task 2: Left/Right Wall Following (HamBot direct)

State machine + PID, all in motor-RPM space.
  FOLLOW     : side PID -> angular correction (rpm delta); base rpm from front clearance
  FRONT_TURN : obstacle ahead -> in-place 90° style turn via opposite wheel RPMs
  EDGE_WRAP  : when side distance jumps and diagonal open -> arc while turning to re-acquire wall

LIDAR orientation from your wrapper: 0° back, 90° left, 180° front, 270° right.
"""

"""
import time
from statistics import median

# --- Imports for HamBot (support both package and flat file layouts) ---
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
    return x / 1000.0 if x > 10.0 else x

def _median_meters(vals):
    vals_m = [ _to_meters(v) for v in vals ]
    vals_m = [ v for v in vals_m if v is not None ]
    if not vals_m:
        return None
    return median(vals_m)

def _window(vals, center_idx, half=3):
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

class FSM:
    FOLLOW, FRONT_TURN, EDGE_WRAP = range(3)

DT = 0.032
LEFT_WALL = True      # set False for right-wall following
SIDE_TARGET_M = 0.35

# Speeds in RPM (HamBot clamps to ±75 RPM)
BASE_CRUISE_RPM = 40.0
BASE_SLOW_RPM   = 18.0
TURN_RPM        = 35.0

# Side PID -> rpm delta
KP_A = 220.0   # rpm per meter side error
KI_A = 0.0
KD_A = 40.0

# Front thresholds (meters)
FRONT_SLOW_M  = 1.2
FRONT_TURN_M  = 0.6
FRONT_CLEAR_M = 1.6

# Edge detection
EDGE_RISE_M        = 0.30
EDGE_DIAG_CLEAR_M  = 1.0

def controller_step(bot, pid):
    ranges = bot.get_range_image()
    d_front = lidar_front_m(ranges)
    d_side  = lidar_left_m(ranges) if LEFT_WALL else lidar_right_m(ranges)
    d_diag  = lidar_diag_left_m(ranges) if LEFT_WALL else lidar_diag_right_m(ranges)

    # initialize persistent state
    if not hasattr(controller_step, "state"):
        controller_step.state = FSM.FOLLOW
        controller_step.prev_side = d_side if d_side is not None else SIDE_TARGET_M
    state = controller_step.state
    prev_side = controller_step.prev_side if controller_step.prev_side is not None else d_side

    # --- Transition logic ---
    if state == FSM.FOLLOW:
        if d_front is not None and d_front < FRONT_TURN_M:
            state = FSM.FRONT_TURN
        elif (d_side is not None and prev_side is not None and 
              (d_side - prev_side) > EDGE_RISE_M and 
              (d_diag is not None and d_diag > EDGE_DIAG_CLEAR_M)):
            state = FSM.EDGE_WRAP

    elif state == FSM.FRONT_TURN:
        # Reacquire side and front clear
        if (d_front is not None and d_front > FRONT_CLEAR_M and 
            d_side is not None and abs(SIDE_TARGET_M - d_side) < 0.12):
            state = FSM.FOLLOW

    elif state == FSM.EDGE_WRAP:
        if d_side is not None and abs(SIDE_TARGET_M - d_side) < 0.12:
            state = FSM.FOLLOW

    controller_step.state = state

    # --- Control per state ---
    if state == FSM.FOLLOW:
        # base forward rpm from front clearance
        base = BASE_CRUISE_RPM if (d_front is None or d_front > FRONT_SLOW_M) else BASE_SLOW_RPM
        # side error -> rpm delta (sign differs per wall side)
        if d_side is None:
            delta = 0.0
        else:
            e_side = (SIDE_TARGET_M - d_side) if LEFT_WALL else (d_side - SIDE_TARGET_M)
            delta = pid.step(e_side) * (1.0 if LEFT_WALL else -1.0)
        l_rpm = clamp(base + delta, -70.0, 70.0)
        r_rpm = clamp(base - delta, -70.0, 70.0)

    elif state == FSM.FRONT_TURN:
        # in-place rotation toward tracked wall
        base = 0.0
        delta = TURN_RPM if LEFT_WALL else -TURN_RPM
        l_rpm = clamp(-delta, -70.0, 70.0)
        r_rpm = clamp( delta, -70.0, 70.0)

    else:  # EDGE_WRAP
        base = BASE_SLOW_RPM
        delta = 0.75 * (TURN_RPM if LEFT_WALL else -TURN_RPM)
        l_rpm = clamp(base - delta, -70.0, 70.0)
        r_rpm = clamp(base + delta, -70.0, 70.0)

    bot.set_left_motor_speed(l_rpm)
    bot.set_right_motor_speed(r_rpm)
    controller_step.prev_side = d_side

    side = 'L' if LEFT_WALL else 'R'
    print(f"st={state} front={d_front} {side}={d_side} diag={d_diag} -> L={l_rpm:+.1f} R={r_rpm:+.1f}")

def run():
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    pid = PID(KP_A, KI_A, KD_A, DT, i_max=200.0, d_alpha=0.25)
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