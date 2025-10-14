"""
USFID_Lab2_Task1.py — Task 1: PID Forward Wall Stop (HamBot direct)

- Uses HamBot.get_range_image() (0° back, 90° left, 180° front, 270° right)
- PID computes a motor RPM command directly (no geometry needed).
- Commands both wheels to same RPM for straight approach.

Safety:
  * RPM clamped to HamBot's ±75 RPM envelope.
  * Stops if front distance is None/invalid or < safety threshold.
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