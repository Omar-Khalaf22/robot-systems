"""
Drive straight and stop at 1 m from the front wall using a PID in RPM space
*adds side safety for the 0.6 m corridor to not scrape the walls
"""
import time
from statistics import median

from robot_systems.robot import HamBot


DT = 0.032
TARGET_DIST = 1.00       
RPM_MAX = 60.0         # keeping under hardware max for smoother operations
FRWRD_SIGN = -1

#approach/safety
FRONT_SLOW_M = 1.20 
SAFETY_STOP_M = 0.20

#side safety zones (in m) for the narrow corridor
SIDE_SLOW_M = 0.12
SIDE_MIN_M  = 0.09

# PID gains (RPM/m of error)
KP = 120.0
KI = 0.0
KD = 20.0
I_MAX = 150.0
D_ALPHA = 0.25  # derivative low-pass

def clamp(x, lo, hi): #aka saturation func
    return lo if x < lo else hi if x > hi else x

def _to_meters(x): #convert to meters
    if x is None or x <= 0:
        return None  # in case the values are invalid (none, <0)
    """heuristic: because sometimes lidar gives values in mm, so logically if the lidar val is >9 
    (determined based on the longest straight dist of maze), it's probably in mm not in m. Convert in that case"""
    return x / 1000.0 if x > 9.0 else x

def _median_meters(vals): #I consulted AI for this function cuz I wasn't sure how to get the avg of a range of nearby lidar degree vals
    vals_m = [_to_meters(v) for v in vals]
    vals_m = [v for v in vals_m if v is not None]
    if not vals_m:
        return None
    return median(vals_m)

def _window(vals, center_idx, half=3): #I consulted AI for this function cuz I wasn't sure how to get the avg of a range of nearby lidar degree vals
    n = len(vals)
    idxs = [(center_idx + i) % n for i in range(-half, half + 1)]
    return [vals[k] for k in idxs]

#get the median of the lidar ranges around the front, left, and right. I'll use these vals for smoother control
def lidar_front_m(ranges): 
    return _median_meters(_window(ranges, 180, 3))
def lidar_left_m(ranges):   
    return _median_meters(_window(ranges,  90, 3))
def lidar_right_m(ranges):  
    return _median_meters(_window(ranges, 270, 3))

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
        #integral (anti-windup clamp)
        self.i += e * self.dt
        self.i = clamp(self.i, -self.i_max, self.i_max)
        #needed AI's help for derivative (low-pass filtered) calculations
        d_raw = (e - self.prev_e) / self.dt if self.dt > 0 else 0.0
        self.d = (1 - self.d_alpha) * d_raw + self.d_alpha * self.d
        self.prev_e = e
        return self.kp * e + self.ki * self.i + self.kd * self.d

def controller_step(bot, pid):
    ranges = bot.get_range_image()    #360 values, typically in mm
    front_m = lidar_front_m(ranges)
    left_m  = lidar_left_m(ranges)
    right_m = lidar_right_m(ranges)

    #safety stoppers for invalid or very close lidar readings. Robot stops
    if front_m is None or (front_m is not None and front_m < SAFETY_STOP_M):
        bot.set_left_motor_speed(0.0)
        bot.set_right_motor_speed(0.0)
        print(f"front={front_m} -> SAFETY STOP")
        return

    e = TARGET_DIST - front_m
    u_rpm = pid.step(e)

    #slowing down for approach from the front
    if front_m < FRONT_SLOW_M:
        u_rpm = clamp(u_rpm, -25.0, 25.0)

    #side safety in the narrow corridor
    if (left_m  is not None and left_m  < SIDE_MIN_M) or (right_m is not None and right_m < SIDE_MIN_M):
        u_rpm = 0.0
    elif (left_m  is not None and left_m  < SIDE_SLOW_M) or (right_m is not None and right_m < SIDE_SLOW_M):
        u_rpm = clamp(u_rpm, -25.0, 25.0)

    #final clamp (saturation) and wheel commands
    u_rpm = clamp(u_rpm, -RPM_MAX, RPM_MAX)
    cmd = FRWRD_SIGN * u_rpm #flipping motor signs to go to the right direction. I prefer this instead of directly putting negative values to make things easier when debugging
    bot.set_left_motor_speed(cmd)
    bot.set_right_motor_speed(cmd)

    print(f"front={front_m:.3f}m  e={e:+.3f}  L/R={u_rpm:+.1f} rpm  " f"sideL={left_m} sideR={right_m}") #log for debugging referencing

def run(): #main func to run the code
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
