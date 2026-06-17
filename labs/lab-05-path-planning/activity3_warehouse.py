import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import sys
import os

# ==============================================================================
# Activity 3: PyBullet Warehouse — Path Planning Visualization
# ==============================================================================
# Goal: Use your A* implementation from Activity 1 to plan a collision-free
#       path through a 3-D warehouse, and visualize it in PyBullet.
#
# IMPORTANT — What this activity covers and what comes next:
#   THIS LAB   : Path PLANNING — computing the sequence of waypoints from
#                start to goal using A* on the warehouse occupancy grid.
#   NEXT LABS  : Path TRACKING & CONTROL — how to generate motor commands
#                that make the robot physically follow those waypoints.
#
# There is NO robot movement in this simulation. The Husky robot is placed
# at the start position and stays still. What you WILL see:
#   - The 3-D warehouse with shelving-unit obstacles
#   - The A* planned path drawn as a glowing line through the warehouse
#   - Coloured waypoint spheres along the path
#   - A matplotlib top-down summary plot saved to disk
#
# Architecture (Lecture Slide 9 — focus on Layer 1 only today):
#   LAYER 1 — Global Planner (A*)   <-- THIS ACTIVITY
#   Layer 2 — Local Planner         <-- Lab 6
#   Layer 3 — Controller            <-- Lab 6
#
# What changed from the previous version:
#   - Controller (Layer 2 & 3) removed entirely — not the topic of this lab
#   - Scenario B (local minima) removed — belongs in the control lab
#   - Focus is 100% on visualizing what A* produces in 3-D space
#   - Path is drawn as debug lines + waypoint spheres directly in PyBullet
#   - Robot is loaded purely for visual context — it does not move
# ==============================================================================

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from activity1_astar import a_star_search


# ==============================================================================
# GRID CONFIGURATION
# ==============================================================================

CELL_SIZE  = 1.0      # metres per grid cell
GRID_ROWS  = 12
GRID_COLS  = 12
WALL_HEIGHT = 1.2     # height of shelf obstacles in the 3-D scene [metres]


def cell_to_world(row, col):
    """Convert grid (row, col) to continuous world (x, y) at the cell centre."""
    return col * CELL_SIZE + CELL_SIZE / 2, row * CELL_SIZE + CELL_SIZE / 2


def build_warehouse_grid():
    """
    12x12 occupancy grid representing the warehouse shelving layout.
    0 = C_free (navigable aisle), 1 = C_obs (shelf unit).

    This grid is the ONLY input to A*. The planner has no knowledge of
    the 3-D geometry — it works purely on this 2-D abstraction.
    """
    grid = np.zeros((GRID_ROWS, GRID_COLS), dtype=int)

    # Left shelf block
    grid[3:7, 2]   = 1
    grid[3,   2:6] = 1

    # Right shelf block
    grid[3:7, 8]   = 1
    grid[3,   7:9] = 1

    # Bottom aisle divider
    grid[8, 3:9]   = 1

    return grid


# ==============================================================================
# GLOBAL PLANNER  (A* from Activity 1)
# ==============================================================================

def run_global_planner(grid, start_cell, goal_cell):
    """
    Run A* on the occupancy grid and return world-coordinate waypoints.

    Args:
        grid       (ndarray) : 2-D occupancy grid (0 = free, 1 = obstacle)
        start_cell (tuple)   : (row, col) start in grid coordinates
        goal_cell  (tuple)   : (row, col) goal  in grid coordinates

    Returns:
        waypoints (list of [x, y]) : world-coordinate path points
        explored  (set)            : all cells A* examined
    """
    print("[A*] Planning path on warehouse occupancy grid ...")
    path, explored = a_star_search(grid, start_cell, goal_cell)

    if path is None:
        print("[A*] ERROR: No path found — check start/goal are in C_free.")
        return None, explored

    waypoints = [list(cell_to_world(r, c)) for (r, c) in path]

    print(f"[A*] Path found:  {len(waypoints)} waypoints")
    print(f"[A*] Explored:    {len(explored)} / {grid.size} cells "
          f"({100*len(explored)/grid.size:.1f}%)")
    print(f"[A*] Path length: {len(waypoints) - 1} steps")
    return waypoints, explored


# ==============================================================================
# PYBULLET SCENE BUILDER
# ==============================================================================

def setup_pybullet():
    """Connect to PyBullet GUI and configure the viewer."""
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Camera: top-down view showing the full warehouse
    p.resetDebugVisualizerCamera(
        cameraDistance    = 16,
        cameraYaw         = 30,
        cameraPitch       = -50,
        cameraTargetPosition = [6, 6, 0])

    # Clean GUI — hide overlay panels
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,            0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,        1)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)

    return client


def load_ground_and_walls():
    """Load ground plane and low boundary walls."""
    p.loadURDF("plane.urdf")

    # Thin boundary walls (visual only)
    wall_color  = [0.55, 0.55, 0.60, 1.0]
    world_w = GRID_COLS * CELL_SIZE
    world_h = GRID_ROWS * CELL_SIZE

    for pos, half in [
        ([world_w/2, -0.1,       0.3], [world_w/2, 0.05, 0.3]),
        ([world_w/2, world_h+0.1, 0.3], [world_w/2, 0.05, 0.3]),
        ([-0.1,       world_h/2, 0.3], [0.05, world_h/2, 0.3]),
        ([world_w+0.1, world_h/2, 0.3], [0.05, world_h/2, 0.3]),
    ]:
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half,
                                   rgbaColor=wall_color)
        p.createMultiBody(baseMass=0,
                          baseCollisionShapeIndex=col,
                          baseVisualShapeIndex=vis,
                          basePosition=pos)


def load_warehouse_obstacles(grid):
    """
    Instantiate 3-D shelf boxes for every C_obs cell in the grid.
    Shelves are tall brown boxes with a darker top cap.
    """
    shelf_body_color = [0.55, 0.38, 0.18, 1.0]   # wood brown
    shelf_top_color  = [0.35, 0.22, 0.08, 1.0]   # darker top

    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            if grid[r, c] == 1:
                x, y = cell_to_world(r, c)

                # Main shelf body
                col = p.createCollisionShape(
                    p.GEOM_BOX,
                    halfExtents=[CELL_SIZE/2 - 0.04,
                                 CELL_SIZE/2 - 0.04,
                                 WALL_HEIGHT / 2])
                vis = p.createVisualShape(
                    p.GEOM_BOX,
                    halfExtents=[CELL_SIZE/2 - 0.04,
                                 CELL_SIZE/2 - 0.04,
                                 WALL_HEIGHT / 2],
                    rgbaColor=shelf_body_color)
                p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=col,
                                  baseVisualShapeIndex=vis,
                                  basePosition=[x, y, WALL_HEIGHT / 2])

                # Top cap (darker stripe)
                vis_top = p.createVisualShape(
                    p.GEOM_BOX,
                    halfExtents=[CELL_SIZE/2 - 0.04,
                                 CELL_SIZE/2 - 0.04,
                                 0.05],
                    rgbaColor=shelf_top_color)
                p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=-1,
                                  baseVisualShapeIndex=vis_top,
                                  basePosition=[x, y, WALL_HEIGHT + 0.05])


def load_robot(start_cell):
    """
    Place the Husky robot at the start position.
    The robot does NOT move — it is here for visual context only.
    Path tracking will be covered in Lab 6.
    """
    x, y = cell_to_world(*start_cell)
    ori  = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF("husky/husky.urdf", [x, y, 0.15], ori)

    # Let physics settle for a moment
    for _ in range(80):
        p.stepSimulation()

    print(f"[ROBOT] Husky placed at world ({x:.1f}, {y:.1f})")
    print("[ROBOT] Note: robot is stationary — path tracking is Lab 6.")
    return robot_id


# ==============================================================================
# PATH VISUALIZATION IN PYBULLET
# ==============================================================================

def visualize_path_in_pybullet(waypoints, explored, grid):
    """
    Draw the A* result directly into the 3-D PyBullet scene:

      1. Explored cells  — flat semi-transparent blue tiles on the ground
      2. Path line       — bright glowing line connecting all waypoints
      3. Waypoint spheres— small coloured spheres at each waypoint
      4. Goal marker     — larger green sphere at the destination
    """
    path_height    = 0.25    # height to draw path line above ground
    sphere_radius  = 0.15
    goal_radius    = 0.30

    # ── 1. Explored cells (subtle floor tiles) ───────────────────────────
    explored_color = [0.40, 0.65, 0.95, 0.30]
    for (r, c) in explored:
        if grid[r, c] == 0:   # only draw on free cells
            x, y = cell_to_world(r, c)
            vis  = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[CELL_SIZE/2 - 0.06,
                              CELL_SIZE/2 - 0.06,
                              0.01],
                rgbaColor=explored_color)
            p.createMultiBody(baseMass=0,
                               baseCollisionShapeIndex=-1,
                               baseVisualShapeIndex=vis,
                               basePosition=[x, y, 0.01])

    # ── 2. Path line (bright red debug lines) ────────────────────────────
    for i in range(len(waypoints) - 1):
        p.addUserDebugLine(
            lineFromXYZ = [waypoints[i][0],   waypoints[i][1],   path_height],
            lineToXYZ   = [waypoints[i+1][0], waypoints[i+1][1], path_height],
            lineColorRGB = [1.0, 0.15, 0.15],
            lineWidth    = 4)

    # ── 3. Waypoint spheres (gradient: cyan at start → orange at end) ────
    n = len(waypoints)
    for i, wp in enumerate(waypoints):
        t = i / max(n - 1, 1)   # 0.0 at start, 1.0 at goal
        r_c = t                  # red increases toward goal
        g_c = 1.0 - t * 0.7     # green fades
        b_c = 1.0 - t           # blue fades to zero

        vis = p.createVisualShape(
            p.GEOM_SPHERE,
            radius    = sphere_radius,
            rgbaColor = [r_c, g_c, b_c, 0.85])
        p.createMultiBody(baseMass=0,
                           baseCollisionShapeIndex=-1,
                           baseVisualShapeIndex=vis,
                           basePosition=[wp[0], wp[1], path_height])

    # ── 4. Goal marker (large green sphere) ──────────────────────────────
    goal_wp = waypoints[-1]
    vis_goal = p.createVisualShape(
        p.GEOM_SPHERE,
        radius    = goal_radius,
        rgbaColor = [0.10, 0.90, 0.25, 0.90])
    p.createMultiBody(baseMass=0,
                       baseCollisionShapeIndex=-1,
                       baseVisualShapeIndex=vis_goal,
                       basePosition=[goal_wp[0], goal_wp[1], path_height + 0.1])

    # ── 5. Text labels ────────────────────────────────────────────────────
    start_wp = waypoints[0]
    p.addUserDebugText("START",
                        [start_wp[0], start_wp[1], path_height + 0.5],
                        textColorRGB=[0.1, 0.9, 0.3],
                        textSize=1.2)
    p.addUserDebugText("GOAL",
                        [goal_wp[0], goal_wp[1], path_height + 0.6],
                        textColorRGB=[0.1, 0.9, 0.3],
                        textSize=1.2)
    p.addUserDebugText(
        f"A* path: {len(waypoints)} waypoints  |  "
        f"Explored: {len(explored)} cells",
        [GRID_COLS * CELL_SIZE / 2, -0.8, 0.3],
        textColorRGB=[0.9, 0.9, 0.2],
        textSize=0.9)


# ==============================================================================
# MATPLOTLIB SUMMARY PLOT
# ==============================================================================

def save_summary_plot(grid, waypoints, explored, start_cell, goal_cell):
    """
    Save a clean top-down 2-D summary plot of the A* result.
    Shows the occupancy grid, explored region, planned path, and statistics.
    Matches the visual style of Activity 1 so students can directly compare.
    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    fig, (ax_grid, ax_stats) = plt.subplots(
        1, 2, figsize=(13, 6),
        gridspec_kw={'width_ratios': [3, 1]})

    rows, cols = grid.shape
    cs = CELL_SIZE

    ax_grid.set_facecolor('#1A1A2E')
    ax_grid.set_xlim(-0.3, cols * cs + 0.3)
    ax_grid.set_ylim(-0.3, rows * cs + 0.3)
    ax_grid.set_aspect('equal')
    ax_grid.grid(True, linestyle='--', linewidth=0.3, alpha=0.25,
                  color='white')

    # Explored cells
    for (r, c) in explored:
        if grid[r, c] == 0:
            ax_grid.add_patch(plt.Rectangle(
                (c * cs + 0.05, r * cs + 0.05), cs * 0.9, cs * 0.9,
                facecolor='#2980B9', edgecolor='none', alpha=0.30))

    # Shelf obstacles
    for r in range(rows):
        for c in range(cols):
            if grid[r, c] == 1:
                ax_grid.add_patch(plt.Rectangle(
                    (c * cs, r * cs), cs, cs,
                    facecolor='#8B6914', edgecolor='#5D4309',
                    linewidth=0.8, alpha=0.95))

    # Path line
    wx = [pt[0] for pt in waypoints]
    wy = [pt[1] for pt in waypoints]
    ax_grid.plot(wx, wy, '-', color='#FF4136', linewidth=2.8,
                  zorder=5, label=f'A* path ({len(waypoints)} waypoints)')

    # Waypoint dots (colour gradient)
    n = len(waypoints)
    for i, wp in enumerate(waypoints):
        t   = i / max(n - 1, 1)
        col = [t, 1.0 - t * 0.7, 1.0 - t]
        ax_grid.scatter(wp[0], wp[1], c=[col], s=30, zorder=6, alpha=0.9)

    # Start / Goal
    sx, sy = cell_to_world(*start_cell)
    gx, gy = cell_to_world(*goal_cell)
    ax_grid.scatter(sx, sy, c='#2ECC71', s=220, zorder=7,
                     edgecolors='white', linewidths=1.2, label='Start')
    ax_grid.scatter(gx, gy, c='#2ECC71', s=320, marker='*', zorder=7,
                     edgecolors='white', linewidths=1.2, label='Goal')

    ax_grid.set_title("Activity 3 — A* Path in Warehouse\n"
                       "(Top-Down View, matches PyBullet scene)",
                       fontsize=12, fontweight='bold', color='white')
    ax_grid.set_xlabel("x (m)", color='white', fontsize=9)
    ax_grid.set_ylabel("y (m)", color='white', fontsize=9)
    ax_grid.tick_params(colors='white')

    legend_handles = [
        mpatches.Patch(facecolor='#2980B9', alpha=0.4,
                       label=f'A* explored ({len(explored)} cells)'),
        plt.Line2D([0], [0], color='#FF4136', linewidth=2.5,
                   label=f'Planned path ({len(waypoints)} waypoints)'),
        mpatches.Patch(facecolor='#8B6914',
                       label='Shelving (C_obs)'),
        mpatches.Patch(facecolor='#1A1A2E',
                       label='Aisles (C_free)'),
    ]
    ax_grid.legend(handles=legend_handles, fontsize=8, loc='upper left',
                    framealpha=0.75, labelcolor='white',
                    facecolor='#2C3E50')

    # Stats panel
    ax_stats.axis('off')
    ax_stats.set_facecolor('#F8F9FA')

    efficiency = (1 - len(explored) / grid.size) * 100

    path_length_cells = len(waypoints) - 1
    path_length_m     = path_length_cells * CELL_SIZE

    stats = [
        ("ALGORITHM",     "A* Search\nf(n) = g(n) + h(n)"),
        ("HEURISTIC",     "Manhattan distance\n(admissible)"),
        ("GRID SIZE",     f"{rows} x {cols} = {grid.size} cells"),
        ("C_OBS CELLS",   f"{int(grid.sum())} (shelves)"),
        ("EXPLORED",      f"{len(explored)} cells\n({100-efficiency:.1f}% of grid)"),
        ("CELLS SKIPPED", f"{grid.size - len(explored)}\n({efficiency:.1f}% efficiency)"),
        ("PATH LENGTH",   f"{path_length_cells} steps\n({path_length_m:.1f} m)"),
        ("WAYPOINTS",     f"{len(waypoints)}"),
        ("NEXT LAB",      "Path Tracking &\nControl (Lab 6)"),
    ]

    y = 0.97
    for label, value in stats:
        ax_stats.text(0.06, y, label, transform=ax_stats.transAxes,
                       fontsize=8, fontweight='bold', color='#555', va='top')
        y -= 0.05
        ax_stats.text(0.06, y, value, transform=ax_stats.transAxes,
                       fontsize=10, color='#1A252F', va='top')
        y -= 0.09

    ax_stats.set_title("Statistics", fontsize=11, fontweight='bold')
    fig.patch.set_facecolor('#1A1A2E')

    plt.tight_layout()
    plt.savefig("activity3_output.png", dpi=130, bbox_inches='tight')
    print("[VIZ] Summary plot saved --> activity3_output.png")
    plt.show()


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    print("=" * 65)
    print("  Activity 3 — Warehouse Path Planning (PyBullet)")
    print("  Planner  : A* (imported from activity1_astar.py)")
    print("  Heuristic: Manhattan distance (admissible)")
    print("  Simulator: PyBullet 3-D warehouse")
    print()
    print("  Focus: PATH PLANNING only.")
    print("  Path Tracking & Control --> Lab 6")
    print("=" * 65)

    # ── Build occupancy grid ──────────────────────────────────────────────
    grid       = build_warehouse_grid()
    start_cell = (1,  1)
    goal_cell  = (10, 10)

    sx, sy = cell_to_world(*start_cell)
    gx, gy = cell_to_world(*goal_cell)

    print(f"\n  Grid     : {GRID_ROWS}x{GRID_COLS} cells  "
          f"({GRID_ROWS * CELL_SIZE:.0f}m x {GRID_COLS * CELL_SIZE:.0f}m)")
    print(f"  Start    : cell {start_cell}  ->  world ({sx:.1f}, {sy:.1f})")
    print(f"  Goal     : cell {goal_cell}  ->  world ({gx:.1f}, {gy:.1f})")
    print()

    # ── Run A* ───────────────────────────────────────────────────────────
    waypoints, explored = run_global_planner(grid, start_cell, goal_cell)
    if waypoints is None:
        print("Cannot visualize — no path found.")
        return

    # ── PyBullet scene ────────────────────────────────────────────────────
    print("\n[SIM] Launching PyBullet ...")
    setup_pybullet()
    load_ground_and_walls()
    load_warehouse_obstacles(grid)

    robot_id = load_robot(start_cell)

    # ── Visualize the planned path ─────────────────────────────────────────
    print("[SIM] Drawing A* path in 3-D scene ...")
    visualize_path_in_pybullet(waypoints, explored, grid)

    # ── Step physics once to settle visuals ───────────────────────────────
    for _ in range(60):
        p.stepSimulation()
        time.sleep(1.0 / 60.0)

    print()
    print("=" * 65)
    print("  What you are seeing in PyBullet:")
    print("  - Blue floor tiles  : cells A* explored during search")
    print("  - Red line          : the planned path (sequence of waypoints)")
    print("  - Coloured spheres  : individual waypoints (cyan -> orange)")
    print("  - Green star        : goal position")
    print("  - Husky robot       : placed at start, stationary (Lab 6 topic)")
    print()
    print("  REFLECTION QUESTIONS (answer in your lab report):")
    print("  Q1. A* explored only a fraction of the grid. Why does it")
    print("      skip so many cells? What role does h(n) play?")
    print("  Q2. The path goes around the shelves even though a straight")
    print("      line from start to goal would be shorter. Why?")
    print("  Q3. Why is path planning (this lab) a separate step from")
    print("      path tracking and control (next lab)?")
    print("=" * 65)
    print()
    print("[SIM] Window open -- press Ctrl+C or close the window to exit.")

    # Keep the window open until the user closes it
    try:
        while p.isConnected():
            p.stepSimulation()
            time.sleep(1.0 / 60.0)
    except Exception:
        pass

    p.disconnect()
    print("[SIM] PyBullet closed.")

    # ── Save matplotlib summary ───────────────────────────────────────────
    print("[VIZ] Generating summary plot ...")
    save_summary_plot(grid, waypoints, explored, start_cell, goal_cell)


if __name__ == "__main__":
    main()
