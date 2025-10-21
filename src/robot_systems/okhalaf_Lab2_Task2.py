"""
Wall-following with a tiny FSM and a side-distance PID that outputs RPM delta. This wall following code is based off the left wall
"""

import time
from statistics import median
from robot_systems.robot import HamBot

DT = 0.032

#trigger of when to start turning away from wall. I found 1/3 of corridor dimension (0.6) works well
SIDE_TARGET_M = 0.24 #CHANGED ON TRY 1 FROM 0.2

#frwrd speeds
BASE_CRUISE_RPM = 40.0
BASE_SLOW_RPM   = 25.0
TURN_RPM        = 35.0
RPM_MAX         = 60.0

#front speeds for when slowing for corner turns
FRONT_SLOW_M  = 0.65   #start slowing
FRONT_TURN_M  = 0.32  #90 deg turn CHANGED ON TRY 1 FROM 0.35
FRONT_CLEAR_M = 0.55   #resume

#edge-wrap vals for convex corners (determined based on tests)
EDGE_RISE_M        = 0.18  #if there's a suddenly large side distance change ==>0.18 is the threshold for that
EDGE_DIAG_CLEAR_M  = 0.45  #to prevent false triggers, wall must be open enough (diagonal in degrees)to wrap

#side PID vals
KP_A = 185.0 #CHANGED ON TRY 1 FROM 220 
KI_A = 0.0
KD_A = 40.0
I_A_MAX = 200.0
D_A_ALPHA = 0.25

def clamp(x, lo, hi): #aka saturation func
    return lo if x < lo else hi if x > hi else x

def _to_meters(x):
    if x is None or x <= 0:
        return None
    return x / 1000.0 if x > 9.0 else x #same thing with task 1

def _median_meters(vals): #got this func from task 1
    vals_m = [_to_meters(v) for v in vals]
    vals_m = [v for v in vals_m if v is not None]
    if not vals_m:
        return None
    return median(vals_m)

def _window(vals, center_idx, half=3): #got this func from task 1
    n = len(vals)
    idxs = [(center_idx + i) % n for i in range(-half, half + 1)]
    return [vals[k] for k in idxs]

#I am only using the lidar front and left vals since I am following the left only for navigation
def lidar_front_m(ranges):      return _median_meters(_window(ranges, 180, 3))
def lidar_left_m(ranges):       return _median_meters(_window(ranges,  90, 3)) #am i hugging the wall
def lidar_diag_left_m(ranges):  return _median_meters(_window(ranges, 135, 2)) #is the wall going to unhug me (is there open space up front)

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


class FSM:
    FOLLOW, FRONT_TURN, EDGE_WRAP = range(3)   #3 states: track wall, turn in place, or wrap around convex corner

def controller_step(bot, pid):
    ranges = bot.get_range_image()
    d_front = lidar_front_m(ranges)            #median front distance
    d_side  = lidar_left_m(ranges)             #median side distance for left-wall follow
    d_diag  = lidar_diag_left_m(ranges)        #median front-left diagonal distance

    #init persistent state
    if not hasattr(controller_step, "state"):
        controller_step.state = FSM.FOLLOW #remember current state
        controller_step.prev_side = d_side if d_side is not None else SIDE_TARGET_M #remember previous side wall dist (for sudden increases)

    state = controller_step.state
    prev_side = controller_step.prev_side

    # ----- transitions----
    if state == FSM.FOLLOW:
        #obstacle ahead, perform in-place 90 deg turn
        if d_front is not None and d_front < FRONT_TURN_M:
            state = FSM.FRONT_TURN
        #convex corner: side distance jumped, and diagonal is open, wrap around the corner
        elif (d_side is not None and prev_side is not None and
              (d_side - prev_side) > EDGE_RISE_M and
              (d_diag is not None and d_diag > EDGE_DIAG_CLEAR_M)):
            state = FSM.EDGE_WRAP

    elif state == FSM.FRONT_TURN:
        #finish turn once front is clear and side is near target
        if (d_front is not None and d_front > FRONT_CLEAR_M and
            d_side is not None and abs(SIDE_TARGET_M - d_side) < 0.12):
            state = FSM.FOLLOW

    elif state == FSM.EDGE_WRAP:
        if d_front is not None and d_front < FRONT_TURN_M:   #CHANGED (ADDED) ON TRY 4
            state = FSM.FRONT_TURN   # <-- one change: front safety while wrapping CHANGED (ADDED) ON TRY 4
        # done wrapping once side re-acquires near the target offset"""

        #done wrapping once side re-acquires near the target offset
        if d_side is not None and abs(SIDE_TARGET_M - d_side) < 0.12:
            state = FSM.FOLLOW

    controller_step.state = state   #keep new state for next tick

    # ---control for every state -----
    if state == FSM.FOLLOW:
        #choose forward speed (cruise vs slow)
        base = BASE_CRUISE_RPM if (d_front is not None and d_front > FRONT_SLOW_M) else BASE_SLOW_RPM
        #side error: steering delta, left-wall uses +delta
        if d_side is None:
            delta = 0.0              #fallback if side reading is invalid
        else:
            e_side = (SIDE_TARGET_M - d_side)  #positive if too far from wall
            delta = pid.step(e_side) * (1.0)   #-1 cuz following the left wall CHANGED ON 2ND TRY CHANGED ON 5TH TO OG

        l_rpm = clamp(base + delta, -RPM_MAX, RPM_MAX)
        r_rpm = clamp(base - delta, -RPM_MAX, RPM_MAX)

    elif state == FSM.FRONT_TURN:
        # spin in place toward the left wall
        delta = TURN_RPM 
        l_rpm = clamp(-delta, -RPM_MAX, RPM_MAX)
        r_rpm = clamp( delta, -RPM_MAX, RPM_MAX)

    else:  #move forward slowly while turning toward the wall to round the corner
        base = BASE_SLOW_RPM
        delta = -0.75 * (TURN_RPM)   #softer turn CHANGED ON TRY 3 FROM 0.75 TO -0.75 TO ADJUST FOR THE PREVIOUS CHANGE FROM TRY 2
        l_rpm = clamp(base - delta, -RPM_MAX, RPM_MAX)
        r_rpm = clamp(base + delta, -RPM_MAX, RPM_MAX)

    bot.set_left_motor_speed(l_rpm)    # apply wheel cmds
    bot.set_right_motor_speed(r_rpm)
    controller_step.prev_side = d_side # remember last side distance for next tick
    print(f"st={state}  front={d_front}  {'L'}={d_side}  diag={d_diag}  -> L={l_rpm:+.1f} R={r_rpm:+.1f}") #log for debugging 

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