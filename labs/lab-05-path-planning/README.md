# Lab 5: Path Planning & Navigation
### Block 3 — Know Where You're Going
**Introduction to Robotics for AI & Data Science**
*Dr. Akram Fatayer · Al-Zaytoonah University of Jordan*

---

## Where You Are in the Course

```
SENSE      LOCALIZE    MAP       PLAN        TRACK      CONTROL    AUTONOMOUS
Lab 1  →   Lab 3   →  Lab 4  →  Lab 5  →   Lab 6   →  Lab 6   →  Final Project
 Done        Done       Done     YOU ARE      Next        Next
                                  HERE
```

In Lab 3 your robot learned **where it is**. In Lab 4 it built a **map** of the
environment. Now in Lab 5 it must decide **how to get somewhere** — path planning
is the bridge that connects all three pillars of autonomous navigation.

---

## Learning Objectives

By the end of this lab you will be able to:

1. **Implement** the A\* search algorithm from scratch and explain the role of
   each component: `g(n)`, `h(n)`, `f(n)`, the open set, and admissibility.
2. **Compare** graph-based planning (A\*) with sampling-based planning (RRT)
   across dimensionality, optimality, and computational cost.
3. **Explain** why RRT scales to high-dimensional spaces (robot arms) where
   grid-based A\* becomes computationally infeasible.
4. **Visualize** a planned A\* path in a 3-D PyBullet warehouse simulation and
   describe the role of the global planner in the hybrid architecture.

---

## The 80/20 Focus

Two concepts give you 80% of practical path planning capability:

1. **Master A\* implementation** — it is the undisputed workhorse of 2-D/3-D
   grid navigation. Every warehouse robot, game character, and autonomous
   ground vehicle uses a variant of it.
2. **Understand why RRT exists** — grids fail in high-dimensional spaces. A
   6-DOF robot arm would need 36⁶ ≈ 2.2 billion cells at 10° resolution. RRT
   solves the same problem with ~800 nodes.

---

## Environment Setup

This lab builds on the environment from Lab 3. Run the following to confirm
everything is in place before starting:

```bash
python -c "import numpy, matplotlib, pybullet, heapq; print('All packages ready.')"
```

**Python version:** 3.8 – 3.11 recommended.
**PyBullet note:** PyBullet does not have pre-built wheels for Python 3.12+.
If installation fails, contact the instructor for an alternative submission path
for Activity 3 (the matplotlib summary plot is accepted in place of a PyBullet
screenshot in that case).

---

## Lab Activities

### Activity 1 — A\* Grid Search `activity1_astar.py`
**Type:** Implementation (coding required)
**Time:** ~45 minutes

Implement the core A\* algorithm on a 2-D occupancy grid representing a
warehouse floor. Your robot must find the shortest collision-free path from a
start cell to a goal cell.

**Your tasks — complete the two functions marked `TODO`:**

| Function | What to implement |
|---|---|
| `heuristic(a, b)` | Manhattan distance between two grid cells |
| `a_star_search(grid, start, goal)` | Full A\* loop: open set, g/h/f scores, path reconstruction |

**Key concepts to apply:**
- `f(n) = g(n) + h(n)` — total estimated cost
- `g(n)` — exact cost from start to node `n`
- `h(n)` — admissible heuristic estimate to goal (Manhattan distance)
- Open set implemented as a `heapq` min-priority queue
- Closed set to avoid re-expanding nodes

**Expected output:** A matplotlib figure with the planned path and a statistics
panel (path length, nodes explored, efficiency %). Saved as `activity1_output.png`.

---

### Activity 2 — RRT: Why Dimensionality Matters `activity2_rrt.py`
**Type:** Observation + reflection (no coding required)
**Time:** ~25 minutes

Run this script and observe RRT working across three progressively complex
environments. The key message is not *how* RRT works — it is *why* it is
necessary.

| Demo | Space | Grid equivalent | RRT nodes used |
|---|---|---|---|
| 1 — 2-D maze | Flat corridor maze | 400 cells | ~70 |
| 2 — 3-D volumetric | Spherical obstacles | 8,000 voxels | ~60 |
| 3 — 6-DOF robot arm | Joint-space (6 dims) | 64,000,000 cells | ~40 |

**Run the script:**
```bash
python activity2_rrt.py
```

Observe the output then answer the three reflection questions printed on screen.
Your written answers are part of the submission.

**Expected output:** A 6-panel figure saved as `activity2_output.png`.

---

### Activity 3 — PyBullet Warehouse Visualization `activity3_warehouse.py`
**Type:** Demonstration (no coding required)
**Time:** ~20 minutes

Your A\* implementation from Activity 1 is imported and used to plan a path
through a 3-D warehouse loaded in PyBullet. The planned path is rendered as a
glowing line with colour-coded waypoint spheres in the simulation.

> **Important:** The Husky robot is loaded for visual context but does **not**
> move. Path *tracking* and *control* — making the robot physically follow the
> path — are the topics of **Lab 6**.

**What you will see in the PyBullet window:**

| Visual element | Meaning |
|---|---|
| Blue floor tiles | Cells A\* explored during search |
| Red glowing line | The planned path (sequence of waypoints) |
| Cyan → orange spheres | Individual waypoints from start to goal |
| Large green star | Goal position |
| Husky robot (stationary) | Start position — will move in Lab 6 |

**Run the script:**
```bash
python activity3_warehouse.py
```

Take a screenshot of the PyBullet window showing the full warehouse and path.
The summary plot is saved automatically as `activity3_output.png`.

---

## Submission Requirements

Submit the following **three items** via the course portal:

| # | Item | Filename |
|---|---|---|
| 1 | Completed A\* script | `activity1_astar.py` |
| 2 | PyBullet screenshot | `warehouse_screenshot.png` |
| 3 | Reflection answers (Q1–Q3 from Activities 2 & 3) | `reflections.txt` |

**Code quality:** Your `activity1_astar.py` must be thoroughly commented —
explain *why* each step works, not just *what* it does.

**Deadline:** As posted on the course portal.

---

## Quick Reference

```
A* FORMULA       f(n) = g(n) + h(n)
                 g(n) = exact cost from start to n
                 h(n) = admissible heuristic (never overestimates)
                 Manhattan: |Δrow| + |Δcol|  (4-connected grids)

RRT LOOP         1. Sample random config  q_rand
                 2. Find nearest node     q_near
                 3. Steer step_size toward q_rand  →  q_new
                 4. Collision check q_new
                 5. If free: add to tree. Repeat until goal reached.

HYBRID ARCH.     Layer 1 — Global Planner (A*)       runs once   ← THIS LAB
(Slide 9)        Layer 2 — Local Planner  (PF)       ~100 Hz     ← Lab 6
                 Layer 3 — Controller     (PID/MPC)  ~1000 Hz    ← Lab 6
```

---

## File Structure

```
Lab5_PathPlanning/
├── README.md                  ← this file
├── activity1_astar.py         ← IMPLEMENT THIS
├── activity2_rrt.py           ← run and observe
└── activity3_warehouse.py     ← run and screenshot
```

---

*Dr. Akram Fatayer · a.fatayer@zuj.edu.jo*
*linkedin.com/in/akram-fatayer · Al-Zaytoonah University of Jordan*
