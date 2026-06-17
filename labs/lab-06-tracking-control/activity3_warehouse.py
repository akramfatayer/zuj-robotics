"""
Lab 6: Path Tracking
Activity 3: PyBullet Warehouse Navigation (Capstone Integration)
Author: Dr. Akram Fatayer

Description:
    The capstone of the autonomy stack. This script:
      1. Re-plans the warehouse route with the A* planner from Lab 5
         (imported directly -- no need to copy a path file between labs).
      2. Drives that route with YOUR Pure Pursuit controller from Activity 2.
      3. Shows the Husky robot autonomously following the planned path in 3-D.

    This closes the loop: Plan (Lab 5) -> Track (Lab 6).

PREREQUISITE:
    Place Lab 5's `activity1_astar.py` in this folder as `lab5_astar.py`
    (this package already includes it). If it is missing, the script falls
    back to a built-in A* so the lab still runs.
"""

import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import os
import sys

# ==============================================================================
# A* PLANNER  -- imported from Lab 5
# ==============================================================================
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from lab5_astar import a_star_search
    print("[PLANNER] Using A* imported from Lab 5 (lab5_astar.py).")
except Exception:
    # Minimal fallback so the lab runs even if the Lab 5 file is missing.
    import heapq

    def a_star_search(grid, start, goal):
        """Fallback 4-connected A* (same interface as Lab 5)."""
        def h(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        explored = set()
        rows, cols = grid.shape
        while open_set:
            _, cur = heapq.heappop(open_set)
            if cur in explored:
                continue
            explored.add(cur)
            if cur == goal:
                path = []
                while cur in came_from:
                    path.append(cur)
                    cur = came_from[cur]
                path.append(start)
                path.reverse()
                return path, explored
            for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nb = (cur[0] + dr, cur[1] + dc)
                if 0 <= nb[0] < rows and 0 <= nb[1] < cols and grid[nb[0], nb[1]] == 0:
                    tg = g_score[cur] + 1
                    if nb not in g_score or tg < g_score[nb]:
                        g_score[nb] = tg
                        came_from[nb] = cur
                        heapq.heappush(open_set, (tg + h(nb, goal), nb))
        return None, explored

    print("[PLANNER] lab5_astar.py not found -- using built-in fallback A*.")


# ==============================================================================
# ROBOT PARAMETERS  (Clearpath Husky -- realistic values)
# ==============================================================================
WHEEL_RADIUS = 0.165      # Husky wheel radius (m)
WHEEL_TRACK  = 0.555      # distance between left and right wheels (m) -- the "L"
                          # in the differential-drive conversion
MAX_WHEEL_W  = 12.0       # max wheel angular velocity (rad/s)

# Pure Pursuit tuning for the warehouse scale
TARGET_V = 1.2            # forward speed command (m/s)
Ld       = 1.0            # look-ahead distance (m)

# ==============================================================================
# GRID  <->  WORLD  (must match Lab 5 Activity 3)
# ==============================================================================
CELL_SIZE = 1.0
GRID_ROWS = 12
GRID_COLS = 12


def cell_to_world(row, col):
    return col * CELL_SIZE + CELL_SIZE / 2, row * CELL_SIZE + CELL_SIZE / 2


def build_warehouse_grid():
    """Same warehouse layout as Lab 5 Activity 3 (0 = free, 1 = shelf)."""
    grid = np.zeros((GRID_ROWS, GRID_COLS), dtype=int)
    grid[3:7, 2]   = 1
    grid[3,   2:6] = 1
    grid[3:7, 8]   = 1
    grid[3,   7:9] = 1
    grid[8,   3:9] = 1
    return grid


# ==============================================================================
# PURE PURSUIT  (your Activity 2 controller)
# ==============================================================================

def get_lookahead_point(robot_pos, path, Ld):
    """First path point at least Ld away, searching forward from nearest."""
    distances   = np.linalg.norm(path - robot_pos, axis=1)
    nearest_idx = np.argmin(distances)
    for i in range(nearest_idx, len(path)):
        if distances[i] >= Ld:
            return path[i]
    return path[-1]


def pure_pursuit_control(robot_state, target_point, Ld, L):
    """
    Pure Pursuit steering (lecture Slide 7):
        delta = arctan( 2 * L * sin(alpha) / Ld )
    """
    robot_x, robot_y, theta = robot_state
    target_x, target_y      = target_point

    alpha_global = np.arctan2(target_y - robot_y, target_x - robot_x)
    alpha        = np.arctan2(np.sin(alpha_global - theta),
                              np.cos(alpha_global - theta))
    steer        = np.arctan2(2.0 * L * np.sin(alpha), Ld)
    return steer


# ==============================================================================
# LAYER 3 -- CONTROLLER: unicycle (v, omega) -> differential wheel speeds
# ==============================================================================

def steering_to_wheel_velocities(v, steer, L, wheel_radius):
    """
    Convert a unicycle command into left/right wheel ANGULAR velocities.

    From the lecture (Slide 4):
        omega = v / L * tan(delta)             (bicycle/unicycle steering)
        v_R   = v + omega * L / 2              (right wheel linear speed)
        v_L   = v - omega * L / 2              (left  wheel linear speed)
        w     = v_wheel / wheel_radius          (linear -> angular)

    Args:
        v            : forward linear velocity (m/s)
        steer        : steering angle delta (rad)
        L            : wheel track (m)
        wheel_radius : wheel radius (m)

    Returns:
        wL, wR : left and right wheel angular velocities (rad/s)
    """
    omega = (v / L) * math.tan(steer)     # body angular velocity (rad/s)

    v_left  = v - omega * L / 2.0
    v_right = v + omega * L / 2.0

    wL = v_left  / wheel_radius
    wR = v_right / wheel_radius

    wL = float(np.clip(wL, -MAX_WHEEL_W, MAX_WHEEL_W))
    wR = float(np.clip(wR, -MAX_WHEEL_W, MAX_WHEEL_W))
    return wL, wR


def apply_wheel_velocities(robot_id, wL, wR):
    """Send wheel angular velocities to the Husky joints."""
    # Husky joint indices: 2=front_left, 3=front_right, 4=rear_left, 5=rear_right
    left_wheels  = [2, 4]
    right_wheels = [3, 5]
    for j in left_wheels:
        p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL,
                                targetVelocity=wL, force=40)
    for j in right_wheels:
        p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL,
                                targetVelocity=wR, force=40)


# ==============================================================================
# PYBULLET SCENE
# ==============================================================================

def get_robot_state(robot_id):
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    yaw      = p.getEulerFromQuaternion(orn)[2]
    return np.array([pos[0], pos[1], yaw])


def setup_warehouse(grid, start_cell):
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    p.loadURDF("plane.urdf")

    # Shelf obstacles from the grid
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            if grid[r, c] == 1:
                x, y = cell_to_world(r, c)
                col = p.createCollisionShape(p.GEOM_BOX,
                                             halfExtents=[0.45, 0.45, 0.6])
                vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.45, 0.45, 0.6],
                                          rgbaColor=[0.55, 0.38, 0.18, 1])
                p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                                  baseVisualShapeIndex=vis, basePosition=[x, y, 0.6])

    # Robot at the start cell
    sx, sy = cell_to_world(*start_cell)
    robot_id = p.loadURDF("husky/husky.urdf", [sx, sy, 0.15],
                          p.getQuaternionFromEuler([0, 0, 0]))
    for _ in range(60):
        p.stepSimulation()

    p.resetDebugVisualizerCamera(cameraDistance=14, cameraYaw=30,
                                 cameraPitch=-55, cameraTargetPosition=[6, 6, 0])
    return robot_id


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    print("=" * 60)
    print("  Lab 6 Activity 3 -- Capstone: Plan (A*) + Track (Pure Pursuit)")
    print("=" * 60)

    # --- Plan with A* (imported from Lab 5) ---
    grid       = build_warehouse_grid()
    start_cell = (1, 1)
    goal_cell  = (10, 10)

    print("[PLAN] Running A* on the warehouse grid ...")
    path_cells, explored = a_star_search(grid, start_cell, goal_cell)
    if path_cells is None:
        print("[PLAN] ERROR: A* found no path. Aborting.")
        return

    # Convert grid cells to world-coordinate waypoints
    path = np.array([cell_to_world(r, c) for (r, c) in path_cells])
    print(f"[PLAN] Path found: {len(path)} waypoints, {len(explored)} cells explored.")

    # --- PyBullet ---
    print("[SIM] Launching PyBullet warehouse ...")
    robot_id = setup_warehouse(grid, start_cell)

    # Draw the A* path as a green line
    for i in range(len(path) - 1):
        p.addUserDebugLine([path[i][0], path[i][1], 0.15],
                           [path[i + 1][0], path[i + 1][1], 0.15],
                           lineColorRGB=[0.1, 0.85, 0.25], lineWidth=3)

    # Mark the goal
    goal_x, goal_y = cell_to_world(*goal_cell)
    gv = p.createVisualShape(p.GEOM_SPHERE, radius=0.3, rgbaColor=[0.1, 0.5, 1, 0.9])
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=gv,
                      basePosition=[goal_x, goal_y, 0.3])

    print("[TRACK] Driving the path with Pure Pursuit ...")
    print(f"        Ld={Ld} m, target speed={TARGET_V} m/s, "
          f"wheel track L={WHEEL_TRACK} m")

    reached = False
    for step in range(6000):
        state     = get_robot_state(robot_id)
        robot_pos = state[:2]

        # Pure Pursuit -> steering angle
        target = get_lookahead_point(robot_pos, path, Ld)
        steer  = pure_pursuit_control(state, target, Ld, WHEEL_TRACK)
        steer  = float(np.clip(steer, -np.radians(35), np.radians(35)))

        # Visualize the current look-ahead target
        p.addUserDebugLine([robot_pos[0], robot_pos[1], 0.25],
                           [target[0], target[1], 0.25],
                           lineColorRGB=[1, 0.2, 0.2], lineWidth=2, lifeTime=0.05)

        # Convert to wheel speeds and apply (Layer 3)
        wL, wR = steering_to_wheel_velocities(TARGET_V, steer,
                                              WHEEL_TRACK, WHEEL_RADIUS)
        apply_wheel_velocities(robot_id, wL, wR)

        # Goal check
        if np.linalg.norm(path[-1] - robot_pos) < 0.6:
            print(f"[TRACK] Goal reached in {step} steps!")
            apply_wheel_velocities(robot_id, 0, 0)
            reached = True
            break

        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    if not reached:
        print("[TRACK] Simulation ended before reaching the goal "
              "(try increasing Ld or step count).")

    print()
    print("REFLECTION QUESTIONS (answer in your lab report):")
    print("  Q1. The A* path is jagged (grid waypoints). How does the")
    print("      look-ahead distance Ld smooth the robot's actual motion?")
    print("  Q2. This integrates Lab 5 (Plan) with Lab 6 (Track). Which")
    print("      part is the 'global planner' and which is the 'local'?")
    print("  Q3. The robot cuts the corners of the A* path slightly. Is")
    print("      that a bug or expected Pure Pursuit behavior? Explain.")
    print()
    print("[SIM] Close the window to exit.")

    try:
        while p.isConnected():
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except Exception:
        pass
    p.disconnect()


if __name__ == '__main__':
    main()
