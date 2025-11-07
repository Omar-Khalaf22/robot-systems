"""
Use Lab 2 task 2's left-wall follower during WALL_FOLLOW, and camera goal seeking otherwise using the Bug. 
drives the bot to the pink goal using the camera for bearing, and when the path ahead is blocked it hands control to the same left-wall
follower from Lab 2. A simple state machine flips between goal seek and wall follow, controls speed based on front range,
and stops when centered and ~0.25 (for some reason 0.3 worked better when testing) from the target
"""

import time
from robot_systems.robot import HamBot

# importing Lab 2 controller
try:
    import okhalaf_Lab2_Task2 as wf                # running from src/robot_systems
except ImportError:
    from robot_systems import okhalaf_Lab2_Task2 as wf  #if run as a package, just to avoid any unnecessary import errors depending on src directory

DT = 0.032

#Cam target colors (the 2 most probably shade cases)
TARGET_COLORS = [(255, 0, 166), (220, 0, 103), (252, 0, 143), (255, 0, 163), (231, 0, 108), (254, 50, 157), (246, 0, 118), (243, 0, 145),] 
COLOR_TOL     = 0.13
MIN_AREA_PX   = 1200

#steering from image center offset (normalized)
BASE_CRUISE_RPM = 40.0
BASE_SLOW_RPM   = 25.0
RPM_MAX         = 60.0
K_BEARING       = 90.0
BEARING_CLAMP   = 35.0
CENTER_OK       = 0.25     #how centered before stopping
GOAL_STOP_M     = 0.40     #stop when front ≤ this AND centered

#Bug0 transitions
SEEN_TH   = 2
LOST_TH   = 3

class Bug:
    GOAL_SEEK, WALL_FOLLOW = range(2) #the 2 cases we're dealing with

def run():
    bot = HamBot(lidar_enabled=True, camera_enabled=True)
    bot.camera.set_target_colors(TARGET_COLORS, tolerance=COLOR_TOL) #set the camera to target colors, tested by cameraGUI

    #reusing lab 2 task 2 PID threshholds
    pid = wf.PID(wf.KP_A, wf.KI_A, wf.KD_A, DT)

    mode = Bug.GOAL_SEEK
    seen_cnt = 0
    lost_cnt = 0 #init

    try:
        while True:
            ranges  = bot.get_range_image()
            d_front = wf.lidar_front_m(ranges) #using Lab 2's helper for consistency

            # camera goal detect
            #find largst color matched blob then compute horizontal
            #err => negative = target left of center, positive = right
            landmarks = bot.camera.find_landmarks(min_area=MIN_AREA_PX)
            goal_seen = len(landmarks) > 0

            if goal_seen:
                #left-wall follower (same concepts from lab 2 wall following states: front drive, convex turn, and spinning in place )
                lm = max(landmarks, key=lambda L: getattr(L, 'w', 1) * getattr(L, 'h', 1))
                frame = bot.camera.get_frame(copy=False)
                W = frame.shape[1] if frame is not None else 640
                cx = getattr(lm, 'cx', None)
                if cx is None and hasattr(lm, '__dict__'): #I used AI's help with this block of code
                    dct = lm.__dict__
                    cx = dct.get('cx', dct.get('x', dct.get('center_x', W//2)))
                err = ((cx - (W/2.0)) / (W/2.0)) if cx is not None else 0.0   #bearing error
                seen_cnt += 1; lost_cnt = 0 # to avoid flicker
            else:
                err = 0.0
                lost_cnt += 1; seen_cnt = 0

            #mode transitions (Bug0)
            if mode == Bug.GOAL_SEEK:
                blocked = (d_front is not None and d_front < wf.FRONT_TURN_M)  #same trigger Lab 2 uses to turn
                if blocked or (lost_cnt >= LOST_TH):
                    mode = Bug.WALL_FOLLOW
            else:  # WALL_FOLLOW
                clear = (d_front is None) or (d_front > wf.FRONT_CLEAR_M)  #matching Lab 2 "front clear"
                if (seen_cnt >= SEEN_TH) and clear:
                    mode = Bug.GOAL_SEEK

            #success stop
            if goal_seen and d_front is not None and d_front <= GOAL_STOP_M and abs(err) <= CENTER_OK:
                bot.set_left_motor_speed(0.0); bot.set_right_motor_speed(0.0)
                print("REACHED GOAL!!!!")
                break

            # cntrl
            if mode == Bug.GOAL_SEEK and goal_seen:
                #fwd with camera steering (same convention as Lab 2)
                base  = BASE_SLOW_RPM if (d_front is not None and d_front < wf.FRONT_SLOW_M) else BASE_CRUISE_RPM
                delta = max(-BEARING_CLAMP, min(BEARING_CLAMP, K_BEARING * (-err)))  #steer toward the target blob
                l_rpm = max(-RPM_MAX, min(RPM_MAX, base + delta))
                r_rpm = max(-RPM_MAX, min(RPM_MAX, base - delta))
                bot.set_left_motor_speed(l_rpm)
                bot.set_right_motor_speed(r_rpm)
                print(f"mode=SEEK  front={d_front}  err={err:+.3f}  -> L={l_rpm:+.1f} R={r_rpm:+.1f}") #log for debugging
            else:
                #defer entirely to lab2 cntrller (also sets the motors itself and prints its own log)
                wf.controller_step(bot, pid)

            time.sleep(DT)

    #this is just a clean way of handling interruptions that sometime happened when running tests. AI suggested it while debugging        
    except KeyboardInterrupt:
        pass
    finally:
        bot.stop_motors()
        try: bot.disconnect_robot()
        except Exception: pass

if __name__ == "__main__":
    run()
