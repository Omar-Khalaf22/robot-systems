"""
computes and prints planned kinematics before navigation
executes waypoint sequence P0 to P12 (straight segments and rotation around itself)
includes helpers and placeholders for now for final arc motion P12 → P13
"""

import time, math
from robot_systems.robot import HamBot


WHEEL_RADIUS = 0.045       # m
WHEEL_BASE = 0.184         # m
MAX_WHEEL_ANG_V = 18.0     # rad/s (theoretical)
MAX_LINEAR_SPEED = MAX_WHEEL_ANG_V * WHEEL_RADIUS   # 0.81 m/s

#converting rad/s to RPM
def rad_s_to_rpm(omega):
    return (omega * 60.0) / (2.0 * math.pi)

#given waypoints
waypts = [
    (2.0, -2.0, math.pi),             # P0
    (-1.5, -2.0, math.pi),            # P1
    (-2.0, -1.5, math.pi/2),          # P2
    (-2.0, -0.5, math.pi/2),          # P3
    (-1.0, -0.5, 3*math.pi/2),        # P4
    (-0.5, -1.0, 7*math.pi/4),        # P5
    (2.0, -1.0, 0),                   # P6
    (2.0, 0.0, math.pi/2),            # P7
    (0.0, 0.0, math.pi),              # P8
    (0.0, 1.0, math.pi/2),            # P9
    (-2.0, 1.0, math.pi),             # P10
    (-1.0, 2.0, 0),                   # P11
    (1.5, 2.0, 0)                     # P12
]

P13 = None   #I am keeping this as a placeholder now until i get final P13 value

#pre-calculate planned values for straight line waypoint
def euclidean(a, b):
    return math.hypot(b[0]-a[0], b[1]-a[1])

segments = []
total_dist = 0.0
total_time = 0.0

#calculate for each waypoint
for i in range(len(waypts)-1):
    p_from, p_to = waypts[i], waypts[i+1]
    D = euclidean(p_from, p_to)
    T = D / MAX_LINEAR_SPEED
    omega_wheel = MAX_WHEEL_ANG_V   # both wheels same angular speed
    rpm = rad_s_to_rpm(omega_wheel)
    segments.append({
        'index': i,
        'from': p_from,
        'to': p_to,
        'distance': D,
        'time': T,
        'omega_L': omega_wheel,
        'omega_R': omega_wheel,
        'rpm_L': rpm,
        'rpm_R': rpm
    })
    total_dist += D
    total_time += T

#Arc params (P12 → P13). Placeholder function for now that will be activated when P13!=NONE
def arc_params(p_start, p_end):
    """Compute radius, Omega, arc length, wheel speeds for P12→P13."""
    x1, y1, th1 = p_start
    x2, y2, th2 = p_end
    delta_theta = (th2 - th1 + math.pi) % (2*math.pi) - math.pi
    chord = euclidean(p_start, p_end)
    if abs(delta_theta) < 1e-6:
        return None
    R = chord / (2.0 * math.sin(delta_theta/2.0))
    v_plan = MAX_LINEAR_SPEED
    Omega = v_plan / R
    arc_length = abs(R * delta_theta)
    T = arc_length / v_plan
    vL = Omega * (R - WHEEL_BASE/2.0)
    vR = Omega * (R + WHEEL_BASE/2.0) 
    """DISCLAIMER: I gave Gen AI the formulas to deal with for calculating the arc parameters specifically and gave it the dimentions
    to generate the above equations in python. It felt tedious and thought AI would help with syntax
    """
    return {  #DISCLAIMER: I used Gen AI to teach me how to return multiple values 
        'R': R,
        'delta_theta': delta_theta,
        'Omega': Omega,
        'arc_length': arc_length,
        'time': T,
        'vL': vL,
        'vR': vR,
        'omega_L': vL / WHEEL_RADIUS,
        'omega_R': vR / WHEEL_RADIUS,
        'rpm_L': rad_s_to_rpm(vL / WHEEL_RADIUS),
        'rpm_R': rad_s_to_rpm(vR / WHEEL_RADIUS)
    }

arc_segment = None
if P13 != None:
    arc_segment = arc_params(waypts[-1], P13)
    total_dist += arc_segment['arc_length']
    total_time += arc_segment['time']

# Print planned values
def print_planned():
    #DISCLAIMER: I told AI to create a print func to print the parameters above to make it look organized (fed it the code so it can be tailored to this script. )
    print("=== PLANNED KINEMATIC VALUES ===")
    print("DISCLAIMER: I assumed that the max speed is 172 RPMs, but after running tests i noticed that it's restricted to a much slower speed by robot.py (I believe it's 73 rpm?)")
    print(f"{'Seg':>3} {'Dist(m)':>8} {'Time(s)':>8} {'wL(rad/s)':>10} {'wR(rad/s)':>10} {'RPM_L':>8} {'RPM_R':>8}")
    for s in segments:
        print(f"{s['index']:3d} {s['distance']:8.3f} {s['time']:8.3f} {s['omega_L']:10.3f} {s['omega_R']:10.3f} {s['rpm_L']:8.1f} {s['rpm_R']:8.1f}")
    if arc_segment:
        print("\n--- ARC SEGMENT P12→P13 ---")
        print(f"R={arc_segment['R']:.3f} m, Δθ={arc_segment['delta_theta']:.3f} rad, Ω={arc_segment['Omega']:.3f} rad/s")
        print(f"Arc length={arc_segment['arc_length']:.3f} m, Time={arc_segment['time']:.3f} s")
        print(f"Wheel speeds: rpm_L={arc_segment['rpm_L']:.1f}, rpm_R={arc_segment['rpm_R']:.1f}")
    print(f"\nTOTAL DIST: {total_dist:.3f} m")
    print(f"TOTAL TIME: {total_time:.3f} s")
    print("================================\n")

#main controller func
class Controller:
    def __init__(self):
        self.bot = HamBot(lidar_enabled=False, camera_enabled=False)

    def turn_to_heading(self, target_rad, rpm=40, tol_rad=math.radians(5)):
        while True:
            headingDeg = self.bot.get_heading()
            currentRad = math.radians(headingDeg)
            error = (target_rad - currentRad + math.pi) % (2*math.pi) - math.pi 
            if abs(error) < tol_rad:
                break
            if err > 0:
                #adjust motor speed accordingly (left motor goes backward while right goes frwrd) to correct path
                self.bot.set_left_motor_speed(-rpm)
                self.bot.set_right_motor_speed(rpm)
            else:
                #adjust motor speed accordingly (left motor goes frwrd while right goes back) to correct path
                self.bot.set_left_motor_speed(rpm)
                self.bot.set_right_motor_speed(-rpm)
            time.sleep(0.05)
            #adding delay to make it a smoother run after shakey tests
        self.bot.stop_motors()

    def drive_distance(self, distance_m, rpm=60):
        self.bot.reset_encoders()
        self.bot.set_left_motor_speed(rpm)
        self.bot.set_right_motor_speed(rpm)
        start = time.time()
        while True:
            left_rad = self.bot.get_left_encoder_reading()
            right_rad = self.bot.get_right_encoder_reading() #these 2 lines return total accumalateed angular rotation for each wheel since the robot started running
            left_m = left_rad * WHEEL_RADIUS
            right_m = right_rad * WHEEL_RADIUS #these 2 convert the angular rotation into linear dist
            avg = (left_m + right_m) / 2.0 #after some tests, I realized that depending on each wheel's accumalted encoder values could be a bit risky
            #after looking into this, averaging out the returned values seemed to be the better way of getting rid of random errors that take place
            if avg >= distance_m: #if avg dist exceeds or reachs the total distance we calculated for the path, stop the robot we probably finished the maze
                break
            time.sleep(0.01)
        elapsed = time.time() - start
        self.bot.stop_motors()
        return {'left_m': left_m, 'right_m': right_m, 'time': elapsed}

    def drive_arc(self, arc_params):
        #Execute arc motion using wheel speeds (in RPM) and arc time.
        #calculates how much each wheel needs to rotate for the robot’s center to follow a circular path of radius R over an angle theta andstops once those encoder targets are hit.
        self.bot.reset_encoders() #intialize encoders
        self.bot.set_left_motor_speed(arc_params['rpm_L'])
        self.bot.set_right_motor_speed(arc_params['rpm_R'])
        start = time.time()
        while time.time() - start < arc_params['time']:
            time.sleep(0.05)
        self.bot.stop_motors()
        elapsed = time.time() - start
        return {'time': elapsed}

    def execute_waypts(self):
        runtime_log = []
        for s in segments:
            #I used AI to format the printed outputs into an organized manner
            print(f"\n--- Segment {s['index']} ---")
            target_theta = s['to'][2]
            print(f"Rotating to {target_theta:.3f} rad")
            self.turn_to_heading(target_theta, rpm=30)
            print(f"Driving {s['distance']:.3f} m at {s['rpm_L']:.1f} RPM")
            measurement = self.drive_distance(s['distance'], rpm=60)
            print(f"Measured left_m={measurement['left_m']:.3f}, right_m={measurement['right_m']:.3f}, time={measurement['time']:.3f}")
            runtime_log.append(measurement)

        #final arc motion that won't print/execute until i provide a val for p13 (originally set to none until i am given the coordinates)
        if arc_segment:
            measurement = self.drive_arc(arc_segment)
            runtime_log.append(measurement)

        return runtime_log

def main():
    print_planned()
    ctrl = Controller()
    input("Press enter whenevr ready to start***")
    log = ctrl.execute_waypts()
    print("\n==RUNTIME LOG==")
    for i, r in enumerate(log):
        print(r)

if __name__ == "__main__":
    main()