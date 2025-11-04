"""
Lab 3 – Bug 0 on HamBot (PHYSICAL)
Use Lab 2's left-wall follower verbatim during WALL_FOLLOW, and camera goal-seeking otherwise.
"""

import time
from robot_systems.robot import HamBot

# --- import your working Lab 2 controller (module-level constants + controller_step) ---
try:
    import okhalaf_Lab2_Task2 as wf                # running from src/robot_systems
except ImportError:
    from robot_systems import okhalaf_Lab2_Task2 as wf  # if run as a package

DT = 0.032

# === Camera target (you found pink) ===
TARGET_COLORS = [(255, 0, 166), (220, 0, 103)]  # primary + a nearby shade helps robustness
COLOR_TOL     = 0.13
MIN_AREA_PX   = 1200

# Steering from image center offset (normalized)
BASE_CRUISE_RPM = 40.0
BASE_SLOW_RPM   = 25.0
RPM_MAX         = 60.0
K_BEARING       = 90.0
BEARING_CLAMP   = 35.0
CENTER_OK       = 0.20     # how centered before stopping
GOAL_STOP_M     = 0.25     # stop when front ≤ this AND centered

# Bug-0 transitions
SEEN_TH   = 2
LOST_TH   = 3

class Bug:
    GOAL_SEEK, WALL_FOLLOW = range(2)

def run():
    bot = HamBot(lidar_enabled=True, camera_enabled=True)
    bot.camera.set_target_colors(TARGET_COLORS, tolerance=COLOR_TOL)

    # Reuse your Lab 2 PID and thresholds exactly
    pid = wf.PID(wf.KP_A, wf.KI_A, wf.KD_A, DT)

    mode = Bug.GOAL_SEEK
    seen_cnt = 0
    lost_cnt = 0

    try:
        while True:
            ranges  = bot.get_range_image()
            d_front = wf.lidar_front_m(ranges)   # use Lab 2's helper for consistency

            # --- camera goal detect ---
            landmarks = bot.camera.find_landmarks(min_area=MIN_AREA_PX)
            goal_seen = len(landmarks) > 0

            if goal_seen:
                lm = max(landmarks, key=lambda L: getattr(L, 'w', 1) * getattr(L, 'h', 1))
                frame = bot.camera.get_frame(copy=False)
                W = frame.shape[1] if frame is not None else 640
                cx = getattr(lm, 'cx', None)
                if cx is None and hasattr(lm, '__dict__'):
                    dct = lm.__dict__
                    cx = dct.get('cx', dct.get('x', dct.get('center_x', W//2)))
                err = ((cx - (W/2.0)) / (W/2.0)) if cx is not None else 0.0   # -1..+1
                seen_cnt += 1; lost_cnt = 0
            else:
                err = 0.0
                lost_cnt += 1; seen_cnt = 0

            # --- high-level mode transitions (Bug-0) ---
            if mode == Bug.GOAL_SEEK:
                blocked = (d_front is not None and d_front < wf.FRONT_TURN_M)  # same trigger Lab 2 uses to turn
                if blocked or (lost_cnt >= LOST_TH):
                    mode = Bug.WALL_FOLLOW
            else:  # WALL_FOLLOW
                clear = (d_front is None) or (d_front > wf.FRONT_CLEAR_M)      # match your Lab 2 "front clear"
                if (seen_cnt >= SEEN_TH) and clear:
                    mode = Bug.GOAL_SEEK

            # --- success stop ---
            if goal_seen and d_front is not None and d_front <= GOAL_STOP_M and abs(err) <= CENTER_OK:
                bot.set_left_motor_speed(0.0); bot.set_right_motor_speed(0.0)
                print("SUCCESS: Reached goal.")
                break

            # --- control ---
            if mode == Bug.GOAL_SEEK and goal_seen:
                # forward with camera steering; **positive RPM must be forward** (same convention as Lab 2)
                base  = BASE_SLOW_RPM if (d_front is not None and d_front < wf.FRONT_SLOW_M) else BASE_CRUISE_RPM
                delta = max(-BEARING_CLAMP, min(BEARING_CLAMP, K_BEARING * (-err)))  # steer toward blob
                l_rpm = max(-RPM_MAX, min(RPM_MAX, base + delta))
                r_rpm = max(-RPM_MAX, min(RPM_MAX, base - delta))
                bot.set_left_motor_speed(l_rpm)
                bot.set_right_motor_speed(r_rpm)
                print(f"mode=SEEK  front={d_front}  err={err:+.3f}  -> L={l_rpm:+.1f} R={r_rpm:+.1f}")
            else:
                # Defer entirely to your Lab 2 controller (it sets the motors itself and prints its own log)
                wf.controller_step(bot, pid)

            time.sleep(DT)

    except KeyboardInterrupt:
        pass
    finally:
        bot.stop_motors()
        try: bot.disconnect_robot()
        except Exception: pass

if __name__ == "__main__":
    run()
