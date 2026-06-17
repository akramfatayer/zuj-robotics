import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import heapq

# ==============================================================================
# Activity 1: A* Grid Search (The Workhorse of 2D Planning)
# ==============================================================================
# Goal: Implement the A* algorithm to find the shortest path on a 2D grid.
# The 80/20 Principle: Mastering A* unlocks 80% of practical grid navigation.
#
# KEY CONCEPTS (from lecture):
#   C-space  : All possible robot states. Here, each grid cell is one state.
#   C_free   : Cells with value 0 — the robot can safely occupy these.
#   C_obs    : Cells with value 1 — the robot must never enter these.
#   f(n) = g(n) + h(n):
#       g(n) — exact cost from start to node n
#       h(n) — heuristic estimate from n to goal (must be ADMISSIBLE: never
#               overestimates, so A* stays optimal)
# ==============================================================================

# ------------------------------------------------------------------------------
# PART A — Heuristic
# ------------------------------------------------------------------------------

def heuristic(a, b):
    """
    Manhattan distance between two grid cells.

    Why Manhattan? Our robot moves in 4 directions (up/down/left/right),
    so the true minimum cost is |Δrow| + |Δcol| — never over-estimated.
    An admissible heuristic guarantees A* finds the OPTIMAL path.

    Args:
        a (tuple): (row, col) of node a
        b (tuple): (row, col) of node b

    Returns:
        int: Manhattan distance
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# ------------------------------------------------------------------------------
# PART B — Neighbour Expansion
# ------------------------------------------------------------------------------

def get_neighbors(node, grid):
    """
    Return valid 4-connected neighbours of 'node' that lie in C_free.

    Args:
        node  (tuple): current (row, col)
        grid  (ndarray): 2-D occupancy grid (0 = free, 1 = obstacle)

    Returns:
        list of (row, col) tuples
    """
    neighbors = []
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]   # Right, Left, Down, Up

    for dr, dc in directions:
        nb = (node[0] + dr, node[1] + dc)

        # Bounds check
        if 0 <= nb[0] < grid.shape[0] and 0 <= nb[1] < grid.shape[1]:
            # C_free check: 0 = free space
            if grid[nb[0], nb[1]] == 0:
                neighbors.append(nb)

    return neighbors


# ------------------------------------------------------------------------------
# PART C — Core A* Algorithm
# ------------------------------------------------------------------------------

def a_star_search(grid, start, goal):
    """
    A* search on a 2-D occupancy grid.

    Algorithm outline:
      1. Initialise the OPEN SET (priority queue) with the start node.
      2. Pop the node with the lowest f = g + h.
      3. If it is the goal, reconstruct and return the path.
      4. Otherwise expand its neighbours, updating g and f scores.
      5. Repeat until the goal is found or the open set is empty (no path).

    Args:
        grid  (ndarray): 2-D occupancy grid (0 = free, 1 = obstacle)
        start (tuple) : (row, col) start position
        goal  (tuple) : (row, col) goal position

    Returns:
        path     (list of tuples | None): optimal path from start→goal, or None
        explored (set)                  : all nodes popped from the open set
    """
    # --- Open set: min-heap of (f_score, node) ---
    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}          # For path reconstruction
    g_score   = {start: 0} # Cost from start to each node
    explored  = set()       # Nodes already processed

    while open_set:
        # Always expand the node with the LOWEST f = g + h
        current_f, current = heapq.heappop(open_set)

        # Skip if already processed (a stale entry from a better path update)
        if current in explored:
            continue

        explored.add(current)

        # ---- Goal test ----
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, explored

        # ---- Expand neighbours ----
        for neighbor in get_neighbors(current, grid):
            tentative_g = g_score[current] + 1   # uniform edge cost = 1

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor]   = tentative_g
                f_score             = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))

    return None, explored   # No path exists


# ------------------------------------------------------------------------------
# PART D — Visualization
# ------------------------------------------------------------------------------

def visualize_grid(grid, start, goal, path=None, explored=None, title="A* Path Planning"):
    """
    Render the occupancy grid with explored nodes, final path, start, and goal.
    Also prints a statistics panel that mirrors the lecture slide metrics.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 6),
                             gridspec_kw={'width_ratios': [3, 1]})
    ax_grid, ax_stats = axes

    # ---- Grid ----
    ax_grid.imshow(grid, cmap='Greys', origin='upper', vmin=0, vmax=1)

    total_cells = grid.shape[0] * grid.shape[1]
    n_explored  = len(explored) if explored else 0
    path_len    = len(path)     if path     else 0

    # Explored nodes
    if explored:
        ex = [n[1] for n in explored]
        ey = [n[0] for n in explored]
        ax_grid.scatter(ex, ey, c='#AED6F1', marker='s', s=60,
                        alpha=0.6, label=f'Explored ({n_explored})')

    # Path
    if path:
        px = [n[1] for n in path]
        py = [n[0] for n in path]
        ax_grid.plot(px, py, c='#E74C3C', linewidth=3, label=f'Path ({path_len} steps)')

    # Start / Goal markers
    ax_grid.scatter(start[1], start[0], c='#2ECC71', marker='o',
                    s=200, zorder=5, label='Start')
    ax_grid.scatter(goal[1],  goal[0],  c='#3498DB', marker='*',
                    s=300, zorder=5, label='Goal')

    ax_grid.set_title(title, fontsize=13, fontweight='bold')
    ax_grid.legend(loc='upper left', fontsize=9)
    ax_grid.grid(True, color='gray', linestyle='-', linewidth=0.4, alpha=0.3)
    ax_grid.set_xticks(np.arange(-0.5, grid.shape[1], 1), minor=True)
    ax_grid.set_yticks(np.arange(-0.5, grid.shape[0], 1), minor=True)
    ax_grid.set_xticks([])
    ax_grid.set_yticks([])

    # C-space annotation
    ax_grid.text(0.01, 0.01,
                 "■ C_obs (obstacle)   □ C_free (navigable)",
                 transform=ax_grid.transAxes, fontsize=7,
                 color='gray', va='bottom')

    # ---- Stats panel ----
    ax_stats.axis('off')
    efficiency = (1 - n_explored / total_cells) * 100 if total_cells else 0

    stats = [
        ("PATH LENGTH",    f"{path_len} cells"   if path_len else "No path"),
        ("NODES EXPLORED", f"{n_explored}"),
        ("TOTAL CELLS",    f"{total_cells}"),
        ("EFFICIENCY",     f"{efficiency:.1f}%\n(cells skipped)"),
        ("HEURISTIC",      "Manhattan\n(admissible ✓)"),
        ("ALGORITHM",      "A* Search\nf(n) = g(n) + h(n)"),
    ]

    y = 0.97
    for label, value in stats:
        ax_stats.text(0.05, y, label, transform=ax_stats.transAxes,
                      fontsize=8, fontweight='bold', color='#555555', va='top')
        y -= 0.055
        ax_stats.text(0.05, y, value, transform=ax_stats.transAxes,
                      fontsize=11, color='#1A252F', va='top')
        y -= 0.10

    ax_stats.set_title("Statistics", fontsize=11, fontweight='bold')

    plt.tight_layout()
    plt.savefig("activity1_output.png", dpi=120, bbox_inches='tight')
    print("[VIZ] Saved → activity1_output.png")
    plt.show()


# ------------------------------------------------------------------------------
# MAIN — Demo with warehouse-style obstacle layout
# ------------------------------------------------------------------------------

if __name__ == "__main__":
    # --- Build a 20×20 occupancy grid ---
    grid = np.zeros((20, 20))

    # Warehouse shelving units (C_obs)
    grid[5:15, 5]  = 1
    grid[5,  5:15] = 1
    grid[15, 10:18] = 1
    grid[10:18, 15] = 1

    start_node = (2,  2)
    goal_node  = (18, 18)

    print("=" * 55)
    print("  Activity 1 — A* Path Planning")
    print("=" * 55)
    print(f"  Grid size : {grid.shape[0]}×{grid.shape[1]} = {grid.size} cells")
    print(f"  C_obs     : {int(grid.sum())} obstacle cells")
    print(f"  Start     : {start_node}")
    print(f"  Goal      : {goal_node}")
    print("-" * 55)
    print("  Running A* …")

    path, explored = a_star_search(grid, start_node, goal_node)

    if path:
        efficiency = (1 - len(explored) / grid.size) * 100
        print(f"  ✅ Path found  : {len(path)} steps")
        print(f"  🔍 Explored    : {len(explored)} / {grid.size} nodes")
        print(f"  ⚡ Efficiency  : {efficiency:.1f}% of grid skipped")
        print("  (Compare: Dijkstra would explore ~60-80% of all cells)")
    else:
        print("  ❌ No path found.")

    print("=" * 55)
    visualize_grid(grid, start_node, goal_node, path, explored)
