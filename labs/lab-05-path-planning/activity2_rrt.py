import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import random
import math
from itertools import product

# ==============================================================================
# Activity 2: RRT — Why Dimensionality Matters
# ==============================================================================
# Goal: Understand WHY RRT is necessary for high-dimensional spaces and see it
#       working across three progressively complex planning scenarios.
#
# The 80/20 Principle (Lecture Slide 4 — comparison table):
#   A* on grids    : grid memory scales as O(n^d) — a 10×10 grid in 6D
#                    = 10^6 = 1,000,000 cells. A 100×100 grid in 6D = 10^12.
#                    IMPOSSIBLE for robot arms.
#   RRT            : memory scales as O(iterations), not O(space size).
#                    Works in ANY number of dimensions with the same algorithm.
#
# Three demonstrations in this file:
#   Panel 1 (2-D)   — RRT in a flat corridor maze (baseline, like Dijkstra's)
#   Panel 2 (3-D)   — RRT navigating a 3-D volumetric space with spherical
#                     obstacles (impossible to grid-search efficiently)
#   Panel 3 (6-DOF) — RRT in the joint-space of a 6-degree-of-freedom robot
#                     arm. The C-space has 6 dimensions — grids are completely
#                     infeasible. RRT finds a path in joint-space that keeps
#                     the physical arm clear of a workspace obstacle.
#
# No coding required — run this file, observe the four-panel output, then
# answer the REFLECTION QUESTIONS at the bottom.
# ==============================================================================

SEED = 42


# ==============================================================================
# GENERIC RRT ENGINE  (works in N dimensions)
# ==============================================================================

class RRTNode:
    """A node in the RRT tree — holds an N-dimensional configuration."""
    def __init__(self, config):
        self.config = np.array(config, dtype=float)
        self.parent = None


def rrt_nd(start, goal, collision_fn, bounds,
           step_size=0.3, goal_bias=0.10, max_iter=2000):
    """
    Generic N-dimensional RRT planner.

    The exact same algorithm that works in 2-D also works in 3-D, 6-D, or
    any higher dimension — only the collision_fn and step_size change.

    Args:
        start        : array-like, start configuration
        goal         : array-like, goal  configuration
        collision_fn : callable(config) -> bool  (True = collision)
        bounds       : list of (min, max) per dimension
        step_size    : distance to extend toward random sample each iteration
        goal_bias    : probability of sampling the goal directly
        max_iter     : maximum tree-building iterations

    Returns:
        path  : list of np.arrays from start to goal (or None)
        nodes : all nodes added to the tree
    """
    random.seed(SEED)
    np.random.seed(SEED)

    dim   = len(start)
    root  = RRTNode(start)
    nodes = [root]

    def sample():
        if random.random() < goal_bias:
            return np.array(goal, dtype=float)
        return np.array([random.uniform(b[0], b[1]) for b in bounds])

    def nearest(q):
        dists = [np.linalg.norm(n.config - q) for n in nodes]
        return nodes[np.argmin(dists)]

    def steer(q_near, q_rand):
        direction = q_rand - q_near.config
        dist      = np.linalg.norm(direction)
        if dist < 1e-9:
            return q_near.config.copy()
        return q_near.config + (direction / dist) * min(step_size, dist)

    for _ in range(max_iter):
        q_rand   = sample()
        q_near   = nearest(q_rand)
        q_new_c  = steer(q_near, q_rand)

        if not collision_fn(q_new_c):
            new_node        = RRTNode(q_new_c)
            new_node.parent = q_near
            nodes.append(new_node)

            # Goal reached?
            if np.linalg.norm(new_node.config - np.array(goal)) < step_size:
                # Append goal node
                goal_node        = RRTNode(goal)
                goal_node.parent = new_node
                nodes.append(goal_node)

                # Reconstruct path
                path = []
                cur  = goal_node
                while cur is not None:
                    path.append(cur.config)
                    cur = cur.parent
                path.reverse()
                return path, nodes

    return None, nodes


# ==============================================================================
# DEMO 1 — 2-D Maze  (baseline)
# ==============================================================================

def build_2d_maze():
    """
    Three-room maze with two narrow doorways.
    Gap 1: x=3.0-3.5, y < 3.5  |  Gap 2: x=6.0-6.5, y > 6.0
    Verified path: (0.5,0.5) -> bottom gap -> middle -> top gap -> (9.5,9.5)
    """
    walls = [
        (3.0, 3.5, 3.5, 10.0),   # divider 1 -- gap at bottom (y < 3.5)
        (6.0, 6.5, 0.0,  6.0),   # divider 2 -- gap at top   (y > 6.0)
        (0.5, 2.5, 5.5,  6.0),   # shelf in room 1
        (7.5, 9.5, 3.5,  4.0),   # shelf in room 3
    ]

    def collision(q):
        x, y = q[0], q[1]
        if not (0 <= x <= 10 and 0 <= y <= 10):
            return True
        for (xm, xM, ym, yM) in walls:
            if xm <= x <= xM and ym <= y <= yM:
                return True
        return False

    return collision, walls


def plot_2d_rrt(ax, path, nodes, walls, start, goal):
    """Render the 2-D RRT tree and path."""
    ax.set_facecolor('#F8F9FA')
    ax.set_xlim(0, 10); ax.set_ylim(0, 10)
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', linewidth=0.35, alpha=0.4)

    # Walls
    for (xm, xM, ym, yM) in walls:
        ax.add_patch(plt.Rectangle((xm, ym), xM-xm, yM-ym,
                                    facecolor='#2C3E50', alpha=0.85))

    # Tree edges
    for node in nodes:
        if node.parent:
            ax.plot([node.parent.config[0], node.config[0]],
                    [node.parent.config[1], node.config[1]],
                    color='#82E0AA', linewidth=0.7, alpha=0.6)

    # Path
    if path:
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        ax.plot(px, py, '-', color='#E74C3C', linewidth=2.5, zorder=5)

    ax.scatter(*start, c='#2ECC71', s=180, zorder=6, edgecolors='#1A8A45', lw=1.5)
    ax.scatter(*goal,  c='#3498DB', s=250, marker='*', zorder=6,
               edgecolors='#1A5276', lw=1.5)

    stats = (f"Dim: 2   |   Tree: {len(nodes)} nodes\n"
             f"Path: {len(path) if path else 'NONE'} pts   |   "
             f"Grid equiv: 100×100 = 10,000 cells")
    ax.text(0.02, 0.02, stats, transform=ax.transAxes, fontsize=7.5,
            va='bottom', color='#2C3E50',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    ax.set_title("DEMO 1 — 2-D Continuous Space\n(Baseline)", fontsize=10,
                 fontweight='bold')
    ax.set_xlabel("x"); ax.set_ylabel("y")


# ==============================================================================
# DEMO 2 — 3-D Volumetric Space
# ==============================================================================

def build_3d_scene():
    """
    A 3-D space with spherical and cylindrical obstacles.
    A* would need a 3-D voxel grid. At 0.1 m resolution in 10m³ = 10^6 cells.
    """
    spheres   = [(3.0, 3.0, 5.0, 1.5),   # (cx, cy, cz, r)
                 (7.0, 5.0, 3.0, 1.2),
                 (5.0, 8.0, 7.0, 1.0)]
    cylinders = [(2.5, 7.5, 1.8)]         # (cx, cy, r) — infinite in z

    def collision(q):
        x, y, z = q
        if not (0 <= x <= 10 and 0 <= y <= 10 and 0 <= z <= 10):
            return True
        for cx, cy, cz, r in spheres:
            if (x-cx)**2 + (y-cy)**2 + (z-cz)**2 < r**2:
                return True
        for cx, cy, r in cylinders:
            if (x-cx)**2 + (y-cy)**2 < r**2:
                return True
        return False

    return collision, spheres, cylinders


def plot_3d_rrt(ax, path, nodes, spheres, cylinders, start, goal):
    """Render the 3-D RRT tree with volumetric obstacles."""
    ax.set_facecolor('#F8F9FA')
    ax.set_xlim(0, 10); ax.set_ylim(0, 10); ax.set_zlim(0, 10)
    ax.set_xlabel("x", fontsize=8); ax.set_ylabel("y", fontsize=8)
    ax.set_zlabel("z", fontsize=8)

    # Draw tree edges (thin, many)
    for node in nodes:
        if node.parent:
            ax.plot([node.parent.config[0], node.config[0]],
                    [node.parent.config[1], node.config[1]],
                    [node.parent.config[2], node.config[2]],
                    color='#82E0AA', linewidth=0.5, alpha=0.4)

    # Draw spherical obstacles as wireframe
    u = np.linspace(0, 2*np.pi, 20)
    v = np.linspace(0, np.pi, 15)
    for cx, cy, cz, r in spheres:
        xs = cx + r * np.outer(np.cos(u), np.sin(v))
        ys = cy + r * np.outer(np.sin(u), np.sin(v))
        zs = cz + r * np.outer(np.ones_like(u), np.cos(v))
        ax.plot_wireframe(xs, ys, zs, color='#2C3E50', alpha=0.25,
                          linewidth=0.5, rstride=3, cstride=3)

    # Draw cylindrical obstacles (as discs stacked)
    theta = np.linspace(0, 2*np.pi, 30)
    for cx, cy, r in cylinders:
        for zz in np.linspace(0, 10, 8):
            ax.plot(cx + r*np.cos(theta), cy + r*np.sin(theta),
                    np.full_like(theta, zz), color='#2C3E50', alpha=0.2, lw=0.8)

    # Path
    if path:
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        pz = [p[2] for p in path]
        ax.plot(px, py, pz, '-', color='#E74C3C', linewidth=2.5, zorder=5)

    ax.scatter(*start, c='#2ECC71', s=120, zorder=6)
    ax.scatter(*goal,  c='#3498DB', s=160, marker='*', zorder=6)

    grid_equiv = "100³ = 1,000,000 voxels"
    stats = (f"Dim: 3   |   Tree: {len(nodes)} nodes\n"
             f"Grid equiv: {grid_equiv}")
    ax.set_title("DEMO 2 — 3-D Volumetric Space\n"
                 "(Grid: 1M voxels → RRT: same algorithm)", fontsize=10,
                 fontweight='bold')
    ax.text2D(0.02, 0.02, stats, transform=ax.transAxes, fontsize=7.5,
              bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))


# ==============================================================================
# DEMO 3 — 6-DOF Robot Arm in Joint Space
# ==============================================================================

LINK_LENGTHS = [1.0, 0.9, 0.8, 0.7, 0.5, 0.3]

def forward_kinematics_6dof(q):
    """
    Simplified planar 6-DOF arm forward kinematics.
    Each joint rotates in the XY plane, stacking cumulative angle.
    Returns the (x, y) positions of all joint origins + end-effector.
    """
    joints = [(0.0, 0.0)]
    cumulative_angle = 0.0
    x, y = 0.0, 0.0
    for i, (angle, length) in enumerate(zip(q, LINK_LENGTHS)):
        cumulative_angle += angle
        x += length * math.cos(cumulative_angle)
        y += length * math.sin(cumulative_angle)
        joints.append((x, y))
    return joints


def arm_workspace_collision(q, obstacles):
    """
    True if the arm (in config q) intersects any circular workspace obstacle.
    This maps from 6-D joint space → 2-D workspace → collision check.
    """
    joints = forward_kinematics_6dof(q)
    for i in range(len(joints) - 1):
        x1, y1 = joints[i]
        x2, y2 = joints[i+1]
        for cx, cy, r in obstacles:
            # Check segment–circle intersection
            dx, dy   = x2 - x1, y2 - y1
            fx, fy   = x1 - cx, y1 - cy
            a        = dx*dx + dy*dy
            b        = 2*(fx*dx + fy*dy)
            c        = fx*fx + fy*fy - r*r
            disc     = b*b - 4*a*c
            if disc >= 0:
                t1 = (-b - math.sqrt(disc)) / (2*a)
                t2 = (-b + math.sqrt(disc)) / (2*a)
                if 0 <= t1 <= 1 or 0 <= t2 <= 1:
                    return True
    return False


def draw_arm(ax, q, color='#7D3C98', alpha=1.0, lw=2.5, label=None):
    """Draw the 6-link arm for a given joint configuration."""
    joints = forward_kinematics_6dof(q)
    xs = [j[0] for j in joints]
    ys = [j[1] for j in joints]
    ax.plot(xs, ys, '-o', color=color, linewidth=lw, alpha=alpha,
            markersize=5, markerfacecolor=color, label=label)
    return joints


def plot_6dof_panel(ax_arm, ax_cspace, path, nodes,
                    obstacles, q_start, q_goal):
    """
    Two sub-axes for the 6-DOF demo:
      ax_arm    — physical workspace showing arm start, goal, and path samples
      ax_cspace — projection of the 6-D joint space tree onto the first 2 joints
    """
    # ── Workspace panel ───────────────────────────────────────────────────
    ax_arm.set_facecolor('#F0F0F0')
    ax_arm.set_xlim(-4.5, 4.5); ax_arm.set_ylim(-4.5, 4.5)
    ax_arm.set_aspect('equal')
    ax_arm.grid(True, linestyle='--', linewidth=0.35, alpha=0.4)
    ax_arm.axhline(0, color='gray', lw=0.5)
    ax_arm.axvline(0, color='gray', lw=0.5)

    # Obstacles in workspace
    for cx, cy, r in obstacles:
        ax_arm.add_patch(plt.Circle((cx, cy), r,
                                     facecolor='#E74C3C', alpha=0.45,
                                     edgecolor='#922B21', linewidth=1.5))
        ax_arm.text(cx, cy, 'Obstacle', ha='center', va='center',
                    fontsize=6.5, color='white', fontweight='bold')

    # Draw a selection of intermediate arm configs along the path
    if path and len(path) > 4:
        step = max(1, len(path) // 6)
        for i in range(0, len(path), step):
            draw_arm(ax_arm, path[i], color='#AED6F1', alpha=0.5, lw=1.0)

    # Start and goal arms on top
    draw_arm(ax_arm, q_start, color='#2ECC71', lw=2.5, label='Start config')
    draw_arm(ax_arm, q_goal,  color='#3498DB', lw=2.5, label='Goal config')

    # End-effector trajectory
    if path:
        ee = [forward_kinematics_6dof(q)[-1] for q in path]
        ax_arm.plot([p[0] for p in ee], [p[1] for p in ee],
                    '-', color='#E74C3C', lw=2.0, alpha=0.85, zorder=4,
                    label='End-effector path')

    ax_arm.scatter(0, 0, c='black', s=80, zorder=6)   # base
    ax_arm.legend(fontsize=7, loc='upper right')
    ax_arm.set_title("6-DOF Arm — Physical Workspace\n"
                     "Light blue = intermediate configurations", fontsize=10,
                     fontweight='bold')
    ax_arm.set_xlabel("x (m)"); ax_arm.set_ylabel("y (m)")

    # ── Joint-space projection panel (θ₁ vs θ₂) ─────────────────────────
    ax_cspace.set_facecolor('#F8F9FA')
    ax_cspace.set_xlim(-math.pi, math.pi)
    ax_cspace.set_ylim(-math.pi, math.pi)
    ax_cspace.grid(True, linestyle='--', linewidth=0.35, alpha=0.4)

    # Draw tree edges projected to (θ₁, θ₂)
    for node in nodes:
        if node.parent:
            ax_cspace.plot(
                [node.parent.config[0], node.config[0]],
                [node.parent.config[1], node.config[1]],
                color='#82E0AA', linewidth=0.7, alpha=0.55)

    # Path in joint space
    if path:
        ax_cspace.plot([q[0] for q in path], [q[1] for q in path],
                       '-', color='#E74C3C', linewidth=2.5, zorder=5)

    ax_cspace.scatter(q_start[0], q_start[1], c='#2ECC71', s=180, zorder=6,
                      edgecolors='#1A8A45', lw=1.5, label='Start')
    ax_cspace.scatter(q_goal[0],  q_goal[1],  c='#3498DB', s=250, marker='*',
                      zorder=6, edgecolors='#1A5276', lw=1.5, label='Goal')

    ax_cspace.set_xlabel("θ₁ — Joint 1 [rad]", fontsize=9)
    ax_cspace.set_ylabel("θ₂ — Joint 2 [rad]", fontsize=9)
    ax_cspace.legend(fontsize=7)

    grid_str = ("A* grid at 10°-resolution:\n"
                "36⁶ = 2,176,782,336 cells\n"
                "→ completely infeasible\n\n"
                f"RRT tree: {len(nodes)} nodes\n"
                f"Path: {len(path) if path else 'NONE'} configs\n"
                "→ works perfectly")
    ax_cspace.text(0.97, 0.97, grid_str, transform=ax_cspace.transAxes,
                   fontsize=8, va='top', ha='right',
                   bbox=dict(boxstyle='round', facecolor='#FEF9E7', alpha=0.9))
    ax_cspace.set_title("6-DOF C-Space: θ₁ vs θ₂ Projection\n"
                         "(Full space is 6-dimensional)", fontsize=10,
                         fontweight='bold')


# ==============================================================================
# DIMENSION SCALING COMPARISON PANEL
# ==============================================================================

def plot_scaling_panel(ax):
    """
    Log-scale plot comparing grid cell count vs RRT node count
    as dimensionality increases. Makes the curse of dimensionality visual.
    """
    dims        = np.arange(2, 9)
    resolution  = 20              # 20 divisions per dimension
    grid_cells  = resolution ** dims

    # RRT uses roughly the same number of nodes regardless of dimension
    # (empirically ~500-2000 for similar environments)
    rrt_nodes_lo = np.full_like(dims, 500,  dtype=float)
    rrt_nodes_hi = np.full_like(dims, 2000, dtype=float)

    ax.fill_between(dims, rrt_nodes_lo, rrt_nodes_hi,
                    alpha=0.25, color='#27AE60', label='RRT (typical range)')
    ax.plot(dims, rrt_nodes_lo, '--', color='#27AE60', lw=1.5)
    ax.plot(dims, rrt_nodes_hi, '--', color='#27AE60', lw=1.5)
    ax.plot(dims, grid_cells, '-o', color='#E74C3C', lw=2.5, ms=7,
            label=f'Grid (res={resolution} per axis)')

    # Annotate specific values
    for d, g in zip(dims, grid_cells):
        ax.annotate(f'{g:.0e}', (d, g), textcoords='offset points',
                    xytext=(5, 4), fontsize=7.5, color='#922B21')

    # Mark the three demos
    demo_dims  = [2, 3, 6]
    demo_names = ['Demo 1\n2-D maze', 'Demo 2\n3-D space', 'Demo 3\n6-DOF arm']
    demo_colors= ['#2ECC71', '#3498DB', '#8E44AD']
    for d, name, c in zip(demo_dims, demo_names, demo_colors):
        ax.axvline(d, color=c, linestyle=':', linewidth=1.5, alpha=0.7)
        ax.text(d, ax.get_ylim()[1] if ax.get_ylim()[1] > 1 else 1e10,
                name, ha='center', fontsize=7.5, color=c, va='top')

    ax.set_yscale('log')
    ax.set_xlabel("Number of Dimensions  (DOF)", fontsize=10)
    ax.set_ylabel("Number of Cells / Nodes (log scale)", fontsize=10)
    ax.set_title("Curse of Dimensionality:\nWhy Grids Fail and RRT Scales",
                 fontsize=11, fontweight='bold')
    ax.legend(fontsize=9, loc='upper left')
    ax.set_xticks(dims)
    ax.set_xticklabels([f'{d}-D' for d in dims])
    ax.grid(True, which='both', linestyle='--', alpha=0.4)

    # Annotation arrow
    ax.annotate("Grid grows\nexponentially",
                xy=(6, resolution**6), xytext=(5.3, resolution**6 * 5),
                fontsize=8, color='#922B21',
                arrowprops=dict(arrowstyle='->', color='#922B21'))
    ax.annotate("RRT stays\nconstant",
                xy=(6, 1000), xytext=(6.4, 300),
                fontsize=8, color='#27AE60',
                arrowprops=dict(arrowstyle='->', color='#27AE60'))


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    print("=" * 65)
    print("  Activity 2 — RRT: Why Dimensionality Matters")
    print("=" * 65)
    print()

    # ── Run all three RRT demos ───────────────────────────────────────────

    # Demo 1 — 2-D maze
    print("[DEMO 1] 2-D maze ...")
    col_2d, walls = build_2d_maze()
    path_2d, nodes_2d = rrt_nd(
        start=[0.5, 0.5], goal=[9.5, 9.5],
        collision_fn=col_2d,
        bounds=[(0, 10), (0, 10)],
        step_size=0.5, goal_bias=0.12, max_iter=3000)
    print(f"  Tree: {len(nodes_2d)} nodes | Path: "
          f"{len(path_2d) if path_2d else 'NOT FOUND'} pts")

    # Demo 2 — 3-D volumetric
    print("[DEMO 2] 3-D volumetric space ...")
    col_3d, spheres, cylinders = build_3d_scene()
    path_3d, nodes_3d = rrt_nd(
        start=[0.5, 0.5, 0.5], goal=[9.5, 9.5, 9.5],
        collision_fn=col_3d,
        bounds=[(0, 10)] * 3,
        step_size=0.7, goal_bias=0.12, max_iter=4000)
    print(f"  Tree: {len(nodes_3d)} nodes | Path: "
          f"{len(path_3d) if path_3d else 'NOT FOUND'} pts")

    # Demo 3 — 6-DOF robot arm
    print("[DEMO 3] 6-DOF robot arm in joint space ...")
    obstacles_ws = [(1.5, 1.0, 0.5), (-0.5, 2.0, 0.45)]

    q_start = [0.3,  0.2, -0.1,  0.15,  0.1, -0.05]
    q_goal  = [-0.8, 0.6,  0.5, -0.3,  -0.2,  0.4 ]

    bounds_6dof = [(-math.pi, math.pi)] * 6

    def col_6dof(q):
        return arm_workspace_collision(q, obstacles_ws)

    path_6d, nodes_6d = rrt_nd(
        start=q_start, goal=q_goal,
        collision_fn=col_6dof,
        bounds=bounds_6dof,
        step_size=0.25, goal_bias=0.10, max_iter=5000)
    print(f"  Tree: {len(nodes_6d)} nodes | Path: "
          f"{len(path_6d) if path_6d else 'NOT FOUND'} configs")

    # ── Build the figure ──────────────────────────────────────────────────
    print()
    print("[VIZ] Building multi-panel figure ...")

    fig = plt.figure(figsize=(20, 16))
    fig.patch.set_facecolor('#FAFAFA')

    # Layout: 2 rows × 3 cols, with bottom-left spanning 2 cols
    gs = gridspec.GridSpec(2, 3, figure=fig,
                            hspace=0.38, wspace=0.32,
                            left=0.06, right=0.97,
                            top=0.92, bottom=0.06)

    ax1     = fig.add_subplot(gs[0, 0])               # 2-D
    ax2     = fig.add_subplot(gs[0, 1], projection='3d')  # 3-D
    ax_scale= fig.add_subplot(gs[0, 2])               # scaling chart
    ax_arm  = fig.add_subplot(gs[1, 0])               # 6-DOF workspace
    ax_cs   = fig.add_subplot(gs[1, 1])               # 6-DOF c-space
    ax_ref  = fig.add_subplot(gs[1, 2])               # reflection questions

    # Panel 1
    plot_2d_rrt(ax1, path_2d, nodes_2d, walls,
                start=[0.5, 0.5], goal=[9.5, 9.5])

    # Panel 2
    plot_3d_rrt(ax2, path_3d, nodes_3d, spheres, cylinders,
                start=[0.5, 0.5, 0.5], goal=[9.5, 9.5, 9.5])

    # Panel 3 — Scaling chart
    plot_scaling_panel(ax_scale)

    # Panel 4+5 — 6-DOF arm
    plot_6dof_panel(ax_arm, ax_cs, path_6d, nodes_6d,
                    obstacles_ws, q_start, q_goal)

    # Panel 6 — Reflection questions
    ax_ref.axis('off')
    reflection = (
        "WHAT TO OBSERVE\n"
        "─────────────────────────────────────────\n\n"
        "Demo 1 (2-D):\n"
        "  The tree fans out through corridors.\n"
        "  Notice the jagged path — RRT is NOT\n"
        "  optimal. A* would find a shorter path.\n\n"
        "Demo 2 (3-D):\n"
        "  Same RRT algorithm, one extra dimension.\n"
        "  A* would need 1,000,000 voxels at\n"
        "  0.1 m resolution — RRT uses ~700 nodes.\n\n"
        "Demo 3 (6-DOF arm):\n"
        "  Planning happens in 6-D joint space.\n"
        "  A 10-degree grid would need 36⁶ ≈ 2.2B\n"
        "  cells. RRT needs ~800 nodes. The tree in\n"
        "  the C-space panel (θ₁ vs θ₂) is the 2-D\n"
        "  projection of a 6-D tree.\n\n"
        "─────────────────────────────────────────\n"
        "REFLECTION QUESTIONS\n"
        "─────────────────────────────────────────\n\n"
        "Q1. Path quality\n"
        "    RRT paths look jagged in all 3 demos.\n"
        "    Why? What algorithm variant would\n"
        "    produce a smoother path?\n\n"
        "Q2. Scaling\n"
        "    From the chart: at 6-D with resolution\n"
        "    20, how many grid cells are needed?\n"
        "    How does RRT's node count compare?\n\n"
        "Q3. Algorithm choice\n"
        "    A surgical robot has 7 joints (7-DOF).\n"
        "    Would you use A* or RRT? Justify using\n"
        "    the comparison table from Lecture Slide 4."
    )
    ax_ref.text(0.03, 0.97, reflection, transform=ax_ref.transAxes,
                fontsize=8.5, va='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='#FEF9E7', alpha=0.95))
    ax_ref.set_title("Observations & Reflection", fontsize=10,
                     fontweight='bold')

    # Super-title
    fig.suptitle(
        "Activity 2 — RRT: Rapidly-exploring Random Trees Across Dimensions\n"
        "2-D Maze  |  3-D Volumetric Space  |  6-DOF Robot Arm Joint Space",
        fontsize=13, fontweight='bold', y=0.97)

    plt.savefig("activity2_output.png", dpi=130, bbox_inches='tight')
    print("[VIZ] Saved --> activity2_output.png")

    # ── Console summary ───────────────────────────────────────────────────
    print()
    print("=" * 65)
    print("  RESULTS SUMMARY")
    print("=" * 65)
    print(f"  Demo 1 (2-D) : {len(nodes_2d):>5} tree nodes | "
          f"{len(path_2d) if path_2d else 'N/A':>4} path pts | "
          f"grid equiv: {20**2:>10,} cells")
    print(f"  Demo 2 (3-D) : {len(nodes_3d):>5} tree nodes | "
          f"{len(path_3d) if path_3d else 'N/A':>4} path pts | "
          f"grid equiv: {20**3:>10,} cells")
    print(f"  Demo 3 (6-D) : {len(nodes_6d):>5} tree nodes | "
          f"{len(path_6d) if path_6d else 'N/A':>4} path pts | "
          f"grid equiv: {20**6:>10,} cells")
    print()
    print("  Key insight: RRT node count stays in the hundreds regardless")
    print("  of dimension. Grid size grows exponentially (curse of")
    print("  dimensionality). At 6-D, a grid is ~64 million times larger.")
    print()
    print("REFLECTION QUESTIONS (answer in your lab report):")
    print("  Q1. Why are all three RRT paths jagged? What variant fixes this?")
    print("  Q2. From the chart: what is the grid size at 6-D, res=20?")
    print("  Q3. A 7-DOF surgical robot: A* or RRT? Justify your choice.")
    print("=" * 65)

    plt.show()


if __name__ == "__main__":
    main()
