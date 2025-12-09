"""
Wavefront path planner for Lab 5 Task 1 using the physical HamBot.

* Represents the maze as a small grid world with a blocked central cell (Maze 1 board).
* Runs a wavefront/BFS search from the goal cell to compute the shortest path.
* Prints the ordered list of cells and path length in steps.
* Then drives the HamBot along that path, one 0.6 m cell at a time, using encoders only.
"""

import time
import math
from collections import deque

from robot_systems.robot import HamBot


# --- grid / map representation -------------------------------------------------

# I am treating the physical maze as a 3x3 grid of 0.6 m cells.
# (row, col) indices are 0-based with row 0 at the "top" and col 0 at the "left".
#
# Layout (X = blocked / central obstacle):
#   (0,0)  (0,1)  (0,2)
#   (1,0)   X     (1,2)
#   (2,0)  (2,1)  (2,2)
#
# You can tweak N_ROWS, N_COLS, BLOCKED_CELLS, START_CELL, and GOAL_CELL
# if your map / goal setup is slightly different.

N_ROWS = 4
N_COLS = 4
CELL_SIZE_M = 0.60   # 60 cm between cell centers (per maze figure)
TURN_GAIN = 0.65

# cells that are not traversable at all (central obstacle)
BLOCKED_CELLS = {}

# optional explicit "wall" set for blocked edges between adjacent free cells.
# Here the only obstacle is modeled as a blocked cell, so WALLS can be empty.
# If you later want a more complex maze (like Webots maze8.xml), you can add
# entries like: WALLS.add(frozenset({(r1,c1), (r2,c2)}))
WALLS = {frozenset({(2, 3), (1, 3)}),
    frozenset({(2, 3), (2, 2)}),
    frozenset({(2, 2), (3, 2)}),
    frozenset({(2, 1), (3, 1)}),
    frozenset({(2, 2), (1, 2)}),
    frozenset({(2, 1), (1, 1)}),
    frozenset({(1, 1), (1, 0)}),
    frozenset({(1, 1), (0, 1)}),
    frozenset({(1, 2), (0, 2)})}

# start / goal cells in grid coordinates (row, col)
# For the physical robot version I'm just hard-coding one start/goal pair.
# Align the robot in the physical maze so that this matches reality.
START_CELL = (2, 3)   # bottom-left corner
GOAL_CELL  = (0, 3)   # top-right corner


def in_bounds(cell):
    """Check that cell is on the grid and not in a blocked cell."""
    (r, c) = cell
    if r < 0 or r >= N_ROWS or c < 0 or c >= N_COLS:
        return False
    if cell in BLOCKED_CELLS:
        return False
    return True


# 4-connected moves (N, E, S, W)
DIRS = [
    (-1, 0),   # N
    (0, 1),    # E
    (1, 0),    # S
    (0, -1),   # W
]


def neighbors(cell):
    """Yield all valid neighbor cells that are not separated by an internal wall."""
    (r, c) = cell
    for (dr, dc) in DIRS:
        nxt = (r + dr, c + dc)
        if not in_bounds(nxt):
            continue
        edge = frozenset({cell, nxt})
        if edge in WALLS:
            continue   # internal wall blocks this move
        yield nxt


# --- wavefront / BFS planner ---------------------------------------------------

def wavefront_plan(start, goal):
    """
    Run a wavefront (BFS) from the goal cell outward.
    This fills a distance map and a parent pointer tree.
    Then reconstruct the shortest path from start → goal.
    """
    if start in BLOCKED_CELLS:
        raise ValueError("Start cell is blocked")
    if goal in BLOCKED_CELLS:
        raise ValueError("Goal cell is blocked")

    # BFS queue seeded at the goal (this is the "wavefront" origin).
    q = deque()
    q.append(goal)

    dist = {goal: 0}     # cell -> distance (steps) to goal
    parent = {}          # child cell -> parent cell (one step closer to goal)

    while q:
        cell = q.popleft()
        d = dist[cell]
        for nb in neighbors(cell):
            if nb in dist:
                continue
            dist[nb] = d + 1
            parent[nb] = cell
            q.append(nb)

    if start not in dist:
        # no route from start to goal under the wall constraints
        return None, dist, parent

    # reconstruct shortest path by walking parent pointers from start to goal
    path = [start]
    cur = start
    while cur != goal:
        cur = parent[cur]
        path.append(cur)

    return path, dist, parent


def print_path(path):
    """Print the path and its length exactly as requested in the lab write-up."""
    if not path:
        print("No path found.")
        return
    # ordered list of cells including start and goal
    pretty = " -> ".join(f"({r},{c})" for (r, c) in path)
    print("PATH:", pretty)
    # number of steps is |path| - 1
    print(f"Path length (steps): {len(path) - 1}")


# --- low-level motion helpers (encoders only, no lidar/camera) ----------------

WHEEL_RADIUS = 0.045   # m (same as Lab 1)
WHEEL_BASE   = 0.184   # m (distance between wheel centers)

DRIVE_RPM = 45.0       # forward speed for traversing a cell
TURN_RPM  = 20.0       # wheel speed magnitude during in-place turns


def drive_straight(bot, distance_m, rpm=DRIVE_RPM):
    """
    Drive forward 'distance_m' meters using the wheel encoders.
    Assumes both wheels command the same RPM.
    """
    bot.reset_encoders()
    bot.set_left_motor_speed(rpm)
    bot.set_right_motor_speed(rpm)

    while True:
        # encoder readings are in radians of wheel rotation (per HamBot docs)
        left_rad = bot.get_left_encoder_reading()
        right_rad = bot.get_right_encoder_reading()
        left_m = left_rad * WHEEL_RADIUS
        right_m = right_rad * WHEEL_RADIUS
        avg_m = 0.5 * (left_m + right_m)

        if avg_m >= distance_m:
            break

        time.sleep(0.01)

    bot.stop_motors()


def rotate_in_place(bot, angle_rad, rpm=TURN_RPM):
    """
    Rotate the robot about its center by 'angle_rad' (positive = CCW).
    Implemented purely from encoders + kinematics (no magnetometer).
    """
    # how far each wheel must travel (along its circle) for a pure pivot:
    # d = θ * (wheel_base / 2)
    distance_per_wheel = abs(angle_rad) * (WHEEL_BASE / 2.0)
    target_wheel_rad = TURN_GAIN * distance_per_wheel / WHEEL_RADIUS

    # sign decides left/right spin: left wheel backwards, right forwards for CCW
    sign = 1.0 if angle_rad >= 0.0 else -1.0

    bot.reset_encoders()
    bot.set_left_motor_speed(-sign * rpm)
    bot.set_right_motor_speed(sign * rpm)

    while True:
        left_rad = abs(bot.get_left_encoder_reading())
        right_rad = abs(bot.get_right_encoder_reading())
        avg_rad = 0.5 * (left_rad + right_rad)
        if avg_rad >= target_wheel_rad:
            break
        time.sleep(0.01)

    bot.stop_motors()


# we keep an internal "logical" heading in {0,1,2,3} = N,E,S,W.
# no need to ask the robot for global heading as long as we start it aligned.
DIR_ORDER = ["N", "E", "S", "W"]


def direction_between(a, b):
    """Return one of 'N','E','S','W' for a single-step move a→b."""
    (r1, c1) = a
    (r2, c2) = b
    dr = r2 - r1
    dc = c2 - c1
    if dr == -1 and dc == 0:
        return "N"
    if dr == 1 and dc == 0:
        return "S"
    if dr == 0 and dc == 1:
        return "E"
    if dr == 0 and dc == -1:
        return "W"
    raise ValueError(f"Cells {a} -> {b} are not 4-connected neighbors")


def execute_path(bot, path, start_heading="N"):
    """
    Take a list of grid cells [c0, c1, ..., cK] and drive the robot along it.
    start_heading is the initial compass direction the robot is facing
    when placed in cell c0 ("N","E","S","W"). For my setup I assume "N".
    """
    if not path or len(path) == 1:
        print("Nothing to execute (trivial path).")
        return

    # track logical heading index inside DIR_ORDER
    try:
        h_idx = DIR_ORDER.index(start_heading)
    except ValueError:
        raise ValueError("start_heading must be one of 'N','E','S','W'")

    for i in range(len(path) - 1):
        cur = path[i]
        nxt = path[i + 1]
        desired_dir = direction_between(cur, nxt)

        cur_dir = DIR_ORDER[h_idx]
        if desired_dir != cur_dir:
            # compute minimal rotation in multiples of 90 degrees
            target_idx = DIR_ORDER.index(desired_dir)
            steps = (target_idx - h_idx) % 4   # +1 = right turn, +3 = left turn, +2 = U-turn

            if steps == 1:          # turn right 90°
                rotate_in_place(bot, -math.pi / 2.0)
            elif steps == 3:        # turn left 90°
                rotate_in_place(bot, +math.pi / 2.0)
            elif steps == 2:        # turn around 180°
                rotate_in_place(bot, math.pi)

            h_idx = target_idx

        # now we're aligned with nxt; move one cell forward
        drive_straight(bot, CELL_SIZE_M)

    bot.stop_motors()
    print("Reached goal cell, stopping.")


# --- main entrypoint -----------------------------------------------------------

def run():
    # plan first (pure computation, no robot motion)
    path, dist, parent = wavefront_plan(START_CELL, GOAL_CELL)

    if path is None:
        print("No valid path from start to goal under current maze constraints.")
        return

    print_path(path)   # required text output for grading

    # then bring up the physical HamBot and execute that path
    bot = HamBot(lidar_enabled=False, camera_enabled=False)

    try:
        # For my runs I assume the robot is physically placed in START_CELL,
        # facing "north" (toward decreasing row index). Adjust start_heading
        # below if you physically start it facing east/south/west instead.
        execute_path(bot, path, start_heading="S")
    except KeyboardInterrupt:
        pass
    finally:
        bot.stop_motors()
        try:
            bot.disconnect_robot()
        except Exception:
            # sometimes the underlying serial/socket is already closed; ignore
            pass


if __name__ == "__main__":
    run()
