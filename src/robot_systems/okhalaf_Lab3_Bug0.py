"""
Lab 3 – Bug 0 on HamBot (PHYSICAL)
Goal-seek the yellow cylinder using the camera; when blocked, left-wall follow using LIDAR.
Transitions: GOAL_SEEK <-> WALL_FOLLOW. Stop when centered on the goal and the front LIDAR <= 0.25 m.
"""

import time
from statistics import median
from robot_systems.robot import HamBot

# ========= TIMING =========
DT = 0.032  # control period (s)

# ========= SPEED LIMITS (match prior labs) =========
RPM_MAX         = 60.0    # keep below robot clamp & for smoothness
BASE_CRUISE_RPM = 40.0
BASE_SLOW_RPM   = 25.0
TURN_RPM        = 35.0

# ========= BUG-0 THRESHOLDS (best-guess for your maze; tweak freely) =========
FRONT_BLOCK_M   = 0.35    # consider path blocked -> switch to wall follow
FRONT_RESUME_M  = 0.55    # path clear enough to try goal seeking again
GOAL_STOP_M     = 0.25    # success distance to goal (no cushion as requested)

# ========= CAMERA (yellow detection) =========
YELLOW_RGB      = (240, 220, 40)  # starting point; refine with cameraGUI
COLOR_TOL       = 0.12             # ±12% of 255
MIN_AREA_PX     = 1200             # ignore tiny blobs/noise

# Steering from image center offset (normalized)
K_BEARING       = 90.0             # rpm per 1.0 of normalized pixel error
BEARING_CLAMP   = 35.0             # cap steering delta
CENTER_OK       = 0.20             # "centered on goal" band for stopping

# ========= LEFT-WALL FOLLOW (reused shape from Lab 2, lean version) =========
SIDE_TARGET_M       = 0.27
FRONT_SLOW_M        = 0.65
FRONT_TURN_M        = 0.32
EDGE_RISE_M         = 0.18
EDGE_DIAG_CLEAR_M   = 0.45
KP_A, KI_A, KD_A    = 185.0, 0.0, 40.0
I_A_MAX, D_A_ALPHA  = 200.0, 0.25

FORWARD_SIGN=1

def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

def _to_meters(x):
    if x is None or x <= 0: return None
    # RPLidar returns mm in some configs; if it looks like mm, convert
    return x / 1000.0 if x > 9.0 else x

def _median_meters(vals):
    vals_m = [_to_meters(v) for v in vals]
    vals_m = [v for v in vals_m if v is not None]
    return None if not vals_m else median(vals_m)

def _window(vals, center_idx, half=3):
    n = len(vals)
    idxs = [(center_idx + i) % n for i in range(-half, half + 1)]
    return [vals[k] for k in idxs]

# angle map: 0=back, 90=left, 180=front, 270=right
def lidar_front_m(ranges):     return _median_meters(_window(ranges, 180, 3))
def lidar_left_m(ranges):      return _median_meters(_window(ranges,  90, 3))
def lidar_diag_left_m(ranges): return _median_meters(_window(ranges, 135, 2))

class PID:
    def __init__(self, kp, ki, kd, dt, i_max=I_A_MAX, d_alpha=D_A_ALPHA):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt; self.i = 0.0; self.prev_e = 0.0; self.d = 0.0
        self.i_max = abs(i_max); self.d_alpha = d_alpha
    def reset(self): self.i = 0.0; self.prev_e = 0.0; self.d = 0.0
    def step(self, e):
        self.i += e * self.dt
        self.i = clamp(self.i, -self.i_max, self.i_max)
        d_raw = (e - self.prev_e) / self.dt if self.dt > 0 else 0.0
        self.d = (1 - self.d_alpha) * d_raw + self.d_alpha * self.d
        self.prev_e = e
        return self.kp * e + self.ki * self.i + self.kd * self.d

class WallFSM:
    FOLLOW, FRONT_TURN, EDGE_WRAP = range(3)

def wall_follow_step(bot, pid):
    """One step of left-wall following; returns (l_rpm, r_rpm, state)"""
    ranges = bot.get_range_image()
    d_front = lidar_front_m(ranges)
    d_side  = lidar_left_m(ranges)
    d_diag  = lidar_diag_left_m(ranges)

    if not hasattr(wall_follow_step, "state"):
        wall_follow_step.state = WallFSM.FOLLOW
        wall_follow_step.prev_side = d_side if d_side is not None else SIDE_TARGET_M

    state = wall_follow_step.state
    prev_side = wall_follow_step.prev_side

    # transitions
    if state == WallFSM.FOLLOW:
        if d_front is not None and d_front < FRONT_TURN_M:
            state = WallFSM.FRONT_TURN
        elif (d_side is not None and prev_side is not None and
              (d_side - prev_side) > EDGE_RISE_M and
              (d_diag is not None and d_diag > EDGE_DIAG_CLEAR_M)):
            state = WallFSM.EDGE_WRAP

    elif state == WallFSM.FRONT_TURN:
        if (d_front is not None and d_front > FRONT_RESUME_M and
            d_side is not None and abs(SIDE_TARGET_M - d_side) < 0.12):
            state = WallFSM.FOLLOW

    else:  # EDGE_WRAP
        if d_front is not None and d_front < FRONT_TURN_M:
            state = WallFSM.FRONT_TURN
        if d_side is not None and abs(SIDE_TARGET_M - d_side) < 0.12:
            state = WallFSM.FOLLOW

    wall_follow_step.state = state

    # control
    if state == WallFSM.FOLLOW:
        base = BASE_CRUISE_RPM if (d_front is not None and d_front > FRONT_SLOW_M) else BASE_SLOW_RPM
        e_side = 0.0 if d_side is None else (SIDE_TARGET_M - d_side)  # + => steer toward wall
        delta  = pid.step(e_side)
        l_rpm  = clamp(base + delta, -RPM_MAX, RPM_MAX)
        r_rpm  = clamp(base - delta, -RPM_MAX, RPM_MAX)

    elif state == WallFSM.FRONT_TURN:
        l_rpm = clamp(-TURN_RPM, -RPM_MAX, RPM_MAX)
        r_rpm = clamp(+TURN_RPM, -RPM_MAX, RPM_MAX)

    else:  # EDGE_WRAP: slow forward + gentle steer left
        base  = BASE_SLOW_RPM
        delta = -0.75 * TURN_RPM
        l_rpm = clamp(base - delta, -RPM_MAX, RPM_MAX)
        r_rpm = clamp(base + delta, -RPM_MAX, RPM_MAX)

    wall_follow_step.prev_side = d_side
    return l_rpm, r_rpm, state

class Bug:
    GOAL_SEEK, WALL_FOLLOW = range(2)

def run():
    bot = HamBot(lidar_enabled=True, camera_enabled=True)
    # camera target
    bot.camera.set_target_colors([(255,0,166), (220,0,103)], tolerance=0.13)

    wall_pid = PID(KP_A, KI_A, KD_A, DT)

    # goal visibility hysteresis
    seen_cnt    = 0
    lost_cnt    = 0
    SEEN_TH     = 2     # frames
    LOST_TH     = 3     # frames

    mode = Bug.GOAL_SEEK

    try:
        while True:
            ranges   = bot.get_range_image()
            d_front  = lidar_front_m(ranges)

            # --- camera goal detect ---
            landmarks = bot.camera.find_landmarks(min_area=MIN_AREA_PX)
            goal_seen = len(landmarks) > 0

            if goal_seen:
                # pick largest blob
                lm = max(landmarks, key=lambda L: getattr(L, 'w', 1) * getattr(L, 'h', 1))
                # compute horizontal bearing from image center
                frame = bot.camera.get_frame(copy=False)
                W = frame.shape[1] if frame is not None else 640
                cx = getattr(lm, 'cx', None)
                if cx is None and hasattr(lm, '__dict__'):
                    # best-effort fallback if attribute names differ
                    dct = lm.__dict__
                    cx = dct.get('cx', dct.get('x', dct.get('center_x', W//2)))
                err = ((cx - (W/2.0)) / (W/2.0)) if cx is not None else 0.0  # -1..+1
                seen_cnt += 1; lost_cnt = 0
            else:
                err = 0.0
                lost_cnt += 1; seen_cnt = 0

            # --- mode transitions ---
            if mode == Bug.GOAL_SEEK:
                blocked = (d_front is not None and d_front < FRONT_BLOCK_M)
                if blocked or (lost_cnt >= LOST_TH):
                    mode = Bug.WALL_FOLLOW
            else:  # WALL_FOLLOW
                clear = (d_front is None) or (d_front > FRONT_RESUME_M)
                if (seen_cnt >= SEEN_TH) and clear:
                    mode = Bug.GOAL_SEEK

            # --- stop condition (success) ---
            if goal_seen and d_front is not None and d_front <= GOAL_STOP_M:
                # also require roughly centered to avoid stopping off-axis
                if abs(err) <= CENTER_OK:
                    bot.set_left_motor_speed(0.0)
                    bot.set_right_motor_speed(0.0)
                    print("SUCCESS: Reached goal (front ≤ 0.25 m and centered).")
                    break

            # --- control ---
            if mode == Bug.GOAL_SEEK and goal_seen:
                # forward speed: slow if something is getting closer, else cruise
                base = BASE_SLOW_RPM if (d_front is not None and d_front < FRONT_SLOW_M) else BASE_CRUISE_RPM
                delta = clamp(K_BEARING * (-err), -BEARING_CLAMP, +BEARING_CLAMP)  # -err turns toward the blob
                l_rpm = clamp(base + delta, -RPM_MAX, RPM_MAX)
                r_rpm = clamp(base - delta, -RPM_MAX, RPM_MAX)
            else:
                l_rpm, r_rpm, _ = wall_follow_step(bot, wall_pid)

            bot.set_left_motor_speed(l_rpm * FORWARD_SIGN)
            bot.set_right_motor_speed(r_rpm *FORWARD_SIGN)

            print(f"mode={'SEEK' if mode==Bug.GOAL_SEEK else 'WALL'}  "
                  f"front={d_front}  err={err:+.3f}  -> L={l_rpm*FORWARD_SIGN:+.1f} R={r_rpm*FORWARD_SIGN:+.1f}")

            time.sleep(DT)

    except KeyboardInterrupt:
        pass
    finally:
        bot.set_left_motor_speed(0.0)
        bot.set_right_motor_speed(0.0)
        try:
            bot.disconnect_robot()  # safe shutdown; internal camera stop may be handled
        except Exception:
            pass

if __name__ == "__main__":
    run()
