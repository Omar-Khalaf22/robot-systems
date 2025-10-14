import time
from statistics import median

# --- HamBot import (package or same-folder fallback) ---
try:
    from robot_systems.robot import HamBot
except Exception:
    from robot import HamBot  # fallback if you run directly beside robot.py

# ========== SETTINGS ==========
PHYSICAL = True
DT = 0.032  # control period (s)

# Task target / stop logic
TARGET_M = 1.00      # goal distance to front wall
STOP_ENTER = 0.98    # trigger stop once below this (hold stop for Task 1)
SAFETY_STOP_M = 0.20 if PHYSICAL else 0.25  # hard stop if too close

# Speed & limits (RPM)
RPM_MAX = 60.0
BASE_RPM_MIN = 12.0
BASE_RPM_MAX = 35.0      # conservative cruise
SPEED_K = 35.0           # rpm per meter beyond TARGET_M for base speed
FRONT_SLOW_M = 1.20 if PHYSICAL else 1.80  # extra slow near wall

# Side safety (meters) for narrow corridor
SIDE_SLOW_M = 0.12
SIDE_MIN_M  = 0.09

# LiDAR orientation (raw index where "forward" points)
# Your earlier code used ranges[180] as "front".
RAW_FRONT_DEG = 180  # change to 0/90/270 if needed after a quick board test

# Centering PID (error = left_m - right_m, in meters; output = RPM correction)
KP_C = 120.0
KI_C = 0.0
KD_C = 30.0
I_MAX = 150.0
D_ALPHA = 0.25         # derivative low-pass (0=no filter, 1=all filter)
CORR_RPM_MAX = 15.0    # cap steering correction

# Motor polarity (fix wiring here only)
LEFT_POLARITY  = +1    # flip to -1 if left motor is reversed
RIGHT_POLARITY = +1    # flip to -1 if right motor is reversed

# Front distance filtering / outlier guard
FRONT_FAN_HALF_DEG = 6     # ±degrees around forward
FRONT_FAN_STEP_DEG = 1
FRONT_MAX_JUMP_M = 0.30    # max allowed jump per loop before clamping
FRONT_LPF_ALPHA = 0.25     # 0..1 (higher = more responsive)

# Command slew limit (to avoid jerk when readings spike)
SLEW_MAX_DRPM = 10.0       # max RPM change per loop (per wheel)


# ========== Helpers ==========
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def _to_meters(x):
    # Accept -1 / 0 / None as invalid
    if x is None or x <= 0:
        return None
    # Many RPLidar wrappers return mm; indoors meters rarely > 10
    return x / 1000.0 if x > 10.0 else x

def _fan_median_meters(ranges, center_deg, half_deg=FRONT_FAN_HALF_DEG, step_deg=FRONT_FAN_STEP_DEG):
    """Median of a small degree fan after converting to meters."""
    n = len(ranges)
    vals = []
    for d in range(center_deg - half_deg, center_deg + half_deg + 1, step_deg):
        raw_deg = (d + RAW_FRONT_DEG) % 360
        idx = int(raw_deg * n / 360)
        m = _to_meters(ranges[idx])
        if m is not None:
            vals.append(m)
    return median(vals) if vals else None

def lidar_front_m(ranges):  return _fan_median_meters(ranges,   0)
def lidar_left_m(ranges):   return _fan_median_meters(ranges, +90)
def lidar_right_m(ranges):  return _fan_median_meters(ranges, -90)


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


class Controller:
    """Holds filter state, PID, and previous wheel commands for slew limiting."""
    def __init__(self):
        self.center_pid = PID(KP_C, KI_C, KD_C, DT)
        self.front_filt = None   # filtered front distance (m)
        self.hold_stop = False
        self.prev_cmdL = 0.0
        self.prev_cmdR = 0.0

    def _update_front_filtered(self, front_raw):
        """Outlier clamp + low-pass filter for front distance."""
        if front_raw is None:
            return None
        if self.front_filt is None:
            self.front_filt = front_raw
            return self.front_filt
        # clamp large jumps
        delta = front_raw - self.front_filt
        if abs(delta) > FRONT_MAX_JUMP_M:
            front_raw = self.front_filt + (FRONT_MAX_JUMP_M if delta > 0 else -FRONT_MAX_JUMP_M)
        # low-pass
        self.front_filt = FRONT_LPF_ALPHA * front_raw + (1.0 - FRONT_LPF_ALPHA) * self.front_filt
        return self.front_filt

    def _slew(self, target, prev):
        """Limit per-step change in RPM."""
        return prev + clamp(target - prev, -SLEW_MAX_DRPM, SLEW_MAX_DRPM)

    def set_wheels(self, bot, rpm_left, rpm_right):
        # Apply polarity and clamp, with slew limiting
        rpm_left = clamp(rpm_left, -RPM_MAX, RPM_MAX)
        rpm_right = clamp(rpm_right, -RPM_MAX, RPM_MAX)
        rpm_left = self._slew(rpm_left, self.prev_cmdL)
        rpm_right = self._slew(rpm_right, self.prev_cmdR)
        self.prev_cmdL, self.prev_cmdR = rpm_left, rpm_right
        bot.set_left_motor_speed(LEFT_POLARITY * rpm_left)
        bot.set_right_motor_speed(RIGHT_POLARITY * rpm_right)

    def stop_and_hold(self, bot, reason):
        self.prev_cmdL = self.prev_cmdR = 0.0
        bot.set_left_motor_speed(0.0)
        bot.set_right_motor_speed(0.0)
        print(reason)


def controller_step(bot, ctrl: Controller):
    ranges = bot.get_range_image()  # 360 values, typically in mm
    front_raw = lidar_front_m(ranges)
    front_m = ctrl._update_front_filtered(front_raw)
    left_m  = lidar_left_m(ranges)
    right_m = lidar_right_m(ranges)

    # ---------- Safety: invalid or dangerously close ----------
    if front_m is None or (front_m is not None and front_m < SAFETY_STOP_M):
        ctrl.stop_and_hold(bot, f"[SAFETY STOP] front={front_m}m  L={left_m} R={right_m}")
        return

    # ---------- Task: stop at ~1.0 m (single-shot; hold stop) ----------
    if not ctrl.hold_stop and front_m < STOP_ENTER:
        ctrl.hold_stop = True
        ctrl.stop_and_hold(bot, f"[TASK STOP @~1m] front={front_m:.3f}m (raw={front_raw:.3f}m)")
        return

    if ctrl.hold_stop:
        ctrl.stop_and_hold(bot, f"[HOLD] front={front_m:.3f}m  L={left_m} R={right_m}")
        return

    # ---------- Base forward speed (slow down approaching target) ----------
    base_rpm = BASE_RPM_MIN + SPEED_K * max(front_m - TARGET_M, 0.0)
    base_rpm = clamp(base_rpm, BASE_RPM_MIN, BASE_RPM_MAX)
    if front_m < FRONT_SLOW_M:
        base_rpm = min(base_rpm, 25.0)

    # ---------- Side safety ----------
    if (left_m  is not None and left_m  < SIDE_MIN_M) or \
       (right_m is not None and right_m < SIDE_MIN_M):
        ctrl.stop_and_hold(bot, f"[SIDE STOP] L={left_m} R={right_m} front={front_m:.3f}m")
        return
    elif (left_m  is not None and left_m  < SIDE_SLOW_M) or \
         (right_m is not None and right_m < SIDE_SLOW_M):
        base_rpm = min(base_rpm, 25.0)

    # ---------- Centering PID (left-right) -> steering correction RPM ----------
    if left_m is None or right_m is None:
        corr = 0.0
        e_center = 0.0
    else:
        e_center = (left_m - right_m)   # + => closer to right wall; yaw left
        u = ctrl.center_pid.step(e_center)
        corr = clamp(u, -CORR_RPM_MAX, CORR_RPM_MAX)

    left_cmd  = base_rpm - corr
    right_cmd = base_rpm + corr
    ctrl.set_wheels(bot, left_cmd, right_cmd)

    # ---------- Debug ----------
    def f3(x): 
        return "None" if x is None else f"{x:.3f}"
    print(f"front_raw={f3(front_raw)}m  front={f3(front_m)}m  L={f3(left_m)} R={f3(right_m)}  "
          f"e_center={e_center:+.3f}  base={base_rpm:.1f}rpm  corr={corr:+.1f}rpm  "
          f"cmdL={left_cmd:+.1f} cmdR={right_cmd:+.1f}")


def run():
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    ctrl = Controller()
    try:
        while True:
            controller_step(bot, ctrl)
            time.sleep(DT)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.stop_and_hold(bot, "[EXIT] Motors stopped")
        try:
            bot.disconnect_robot()
        except Exception:
            pass

if __name__ == "__main__":
    run()


"""import time
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

# ---------- Timing ----------
DT = 0.032

# ---------- Target / stop behavior ----------
TARGET_M = 1.00          # stop 1.00 m from front wall
STOP_ENTER = 0.98        # trigger stop once < 0.98 m
# (We HOLD stop; no re-arm in Task 1)

# ---------- Speed limits (RPM) ----------
RPM_MAX = 60.0           # keep under hardware clamp (±75 recommended)
BASE_RPM_MIN = 12.0      # creep speed near target
BASE_RPM_MAX = 45.0      # cruise speed in corridor
SPEED_K = 35.0           # rpm per meter (scales base speed with distance)

# ---------- Side safety (meters) ----------
SIDE_SLOW_M = 0.12
SIDE_MIN_M  = 0.09

# ---------- Approach safety zone ----------
FRONT_SLOW_M = 1.20 if PHYSICAL else 1.80
SAFETY_STOP_M = 0.20 if PHYSICAL else 0.25

# ---------- LiDAR orientation ----------
# Your previous code used index 180 as "front".
# We'll express angles in world coords (0° = forward, +90° = left) and map
# them to raw indices with this offset:
RAW_FRONT_DEG = 180      # if ranges[180] ~= forward; adjust if needed

# ---------- PID gains ----------
# Centering PID: input is (left_m - right_m) in meters
KP_C = 160.0
KI_C = 0.0
KD_C = 40.0
I_MAX = 150.0
D_ALPHA = 0.25           # derivative low-pass (0=no filter, 1=all filter)
CORR_RPM_MAX = 20.0      # cap steering correction RPM

# ---------- Motor polarity (fix wiring here only) ----------
LEFT_POLARITY  = +1      # flip to -1 if your left motor is reversed
RIGHT_POLARITY = +1      # flip to -1 if your right motor is reversed


# ========== helpers ==========
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def _to_meters(x):
    # Accept -1 / 0 / None as invalid
    if x is None or x <= 0:
        return None
    # Many RPLidar wrappers return mm; indoors meters rarely > 10
    return x / 1000.0 if x > 10.0 else x

def _fan_median_meters(ranges, center_deg, half_deg=6, step_deg=1):
    #Median of a small degree fan after converting to meters.
    n = len(ranges)
    vals = []
    for d in range(center_deg - half_deg, center_deg + half_deg + 1, step_deg):
        raw_deg = (d + RAW_FRONT_DEG) % 360
        idx = int(raw_deg * n / 360)
        m = _to_meters(ranges[idx])
        if m is not None:
            vals.append(m)
    return median(vals) if vals else None

def lidar_front_m(ranges):  return _fan_median_meters(ranges,   0, half_deg=6)
def lidar_left_m(ranges):   return _fan_median_meters(ranges, +90, half_deg=6)
def lidar_right_m(ranges):  return _fan_median_meters(ranges, -90, half_deg=6)


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


# ========== controller ==========
def set_wheels(bot, rpm_left, rpm_right):
    # Apply polarity and clamp
    rpm_left  = clamp(rpm_left,  -RPM_MAX, RPM_MAX)  * LEFT_POLARITY
    rpm_right = clamp(rpm_right, -RPM_MAX, RPM_MAX)  * RIGHT_POLARITY
    bot.set_left_motor_speed(rpm_left)
    bot.set_right_motor_speed(rpm_right)

def controller_step(bot, center_pid, hold_stop):
    ranges = bot.get_range_image()  # 360 values, typically in mm
    front_m = lidar_front_m(ranges)
    left_m  = lidar_left_m(ranges)
    right_m = lidar_right_m(ranges)

    # ---------- Safety: sensor invalid or dangerously close ----------
    if front_m is None or (front_m is not None and front_m < SAFETY_STOP_M):
        set_wheels(bot, 0.0, 0.0)
        print(f"[SAFETY STOP] front={front_m}m  L={left_m} R={right_m}")
        return True  # hold stop

    # ---------- Task: stop at ~1.0 m (single-shot; hold stop) ----------
    if not hold_stop and front_m < STOP_ENTER:
        set_wheels(bot, 0.0, 0.0)
        print(f"[TASK STOP @~1m] front={front_m:.3f}m")
        return True  # now hold stop

    if hold_stop:
        # We already stopped for Task 1; keep holding
        set_wheels(bot, 0.0, 0.0)
        print(f"[HOLD] front={front_m:.3f}m  L={left_m} R={right_m}")
        return True

    # ---------- Base forward speed (slow down approaching target) ----------
    # Scale with distance to target, but keep within bounds
    base_rpm = BASE_RPM_MIN + SPEED_K * max(front_m - TARGET_M, 0.0)
    base_rpm = clamp(base_rpm, BASE_RPM_MIN, BASE_RPM_MAX)

    # Extra slow-down in front slow zone
    if front_m < FRONT_SLOW_M:
        base_rpm = min(base_rpm, 25.0)

    # ---------- Side safety ----------
    if (left_m  is not None and left_m  < SIDE_MIN_M) or \
       (right_m is not None and right_m < SIDE_MIN_M):
        # too close to a wall -> immediate stop
        set_wheels(bot, 0.0, 0.0)
        print(f"[SIDE STOP] L={left_m} R={right_m}  front={front_m:.3f}m")
        return False

    elif (left_m  is not None and left_m  < SIDE_SLOW_M) or \
         (right_m is not None and right_m < SIDE_SLOW_M):
        base_rpm = min(base_rpm, 25.0)

    # ---------- Centering PID (left-right) -> steering correction RPM ----------
    # Positive error means you're closer to the RIGHT wall (left distance larger),
    # so you should yaw LEFT: i.e., subtract corr on left, add on right.
    if left_m is None or right_m is None:
        corr = 0.0
    else:
        e_center = (left_m - right_m)
        u = center_pid.step(e_center)         # RPM units (gains tuned for meters)
        corr = clamp(u, -CORR_RPM_MAX, CORR_RPM_MAX)

    left_cmd  = base_rpm - corr
    right_cmd = base_rpm + corr
    set_wheels(bot, left_cmd, right_cmd)

    # ---------- Debug ----------
    def fmt(x): return f"{x:.3f}" if isinstance(x, float) else str(x)
    print(f"front={fmt(front_m)}m  L={fmt(left_m)} R={fmt(right_m)}  "
          f"e_center={(0.0 if (left_m is None or right_m is None) else (left_m-right_m)):+.3f}  "
          f"base={base_rpm:.1f}rpm  corr={corr:+.1f}rpm  "
          f"cmdL={left_cmd:+.1f} cmdR={right_cmd:+.1f}")
    return False


def run():
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    center_pid = PID(KP_C, KI_C, KD_C, DT)
    hold_stop = False
    try:
        while True:
            hold_stop = controller_step(bot, center_pid, hold_stop)
            time.sleep(DT)
    except KeyboardInterrupt:
        pass
    finally:
        set_wheels(bot, 0.0, 0.0)
        try:
            bot.disconnect_robot()
        except Exception:
            pass

if __name__ == "__main__":
    run()

"""


#----------------FIRST TEST---------------------
"""
Lab 2 — Task 1 (Physical robot, 0.60 m corridor)
Drive straight and stop at 1.00 m from the front wall using a PID in RPM space.
Adds side safety for the 0.60 m corridor so you don't scrape rails.

Run on robot (repo root):  PYTHONPATH=src python -m robot_systems.okhalaf_Lab2_Task1
"""
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
"""
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