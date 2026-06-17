# Lab 6: Path Tracking & Control
### Block 4 — Follow the Path
**Introduction to Robotics for AI & Data Science**
*Dr. Akram Fatayer · Al-Zaytoonah University of Jordan*

---

## Where You Are in the Course

```
SENSE      LOCALIZE    MAP       PLAN       TRACK       AUTONOMOUS
Lab 1  →   Lab 3   →  Lab 4  →  Lab 5  →   Lab 6   →   Final Project
 Done        Done       Done     Done       YOU ARE
                                            HERE
```

Lab 5 gave the robot a **plan** — A* computed a sequence of waypoints: optimal,
collision-free, but static. Lab 6 adds the **controller** that converts those
waypoints into real-time wheel commands so the robot physically drives the path.
This is the bridge between planning and motion — and it completes the autonomy
stack.

---

## Learning Objectives

By the end of this lab you will be able to:

1. **Define** Cross-Track Error (CTE) and Heading Error and explain why a path
   tracker must drive both toward zero.
2. **Implement** a PID controller acting on CTE and **tune** its gains to reach
   a critically-damped response.
3. **Implement** the geometric Pure Pursuit controller and explain the
   look-ahead distance trade-off.
4. **Compare** PID and Pure Pursuit on the same path, and **integrate** Pure
   Pursuit with the Lab 5 A* planner to drive a full warehouse route.

---

## The 80/20 Focus

Two ideas unlock 80% of practical path tracking:

1. **Cross-Track Error correction** — the universal signal every controller
   minimizes. Master it once (PID) and the rest are variations.
2. **Look-ahead geometry** — Pure Pursuit replaces three PID gains with a single
   intuitive parameter. It is the default local planner in the ROS Navigation
   Stack and early Tesla Autopilot.

---

## Environment Setup

This lab uses the same environment as Lab 5. Confirm it before starting:

```bash
python -c "import numpy, matplotlib, pybullet; print('All packages ready.')"
```

**Python version:** 3.8 – 3.11 recommended.
**PyBullet note:** PyBullet has no pre-built wheels for Python 3.12+. If install
fails, contact the instructor — the Activity 3 matplotlib path-plot is accepted
in place of a PyBullet screenshot.

---

## Lab Activities

### Activity 1 — PID Path Follower `activity1_pid.py`
**Type:** Implementation + tuning
**Time:** ~40 minutes

A unicycle robot follows a sine-wave path using a PID controller on the
Cross-Track Error. Tune `Kp`, `Ki`, `Kd` to achieve a critically-damped
response (smooth, no overshoot).

**Key concepts:**
- `u(t) = Kp·e(t) + Ki·∫e dt + Kd·de/dt`  where `e` = CTE
- The three damping regimes (lecture Slide 6): underdamped, overdamped,
  critically damped

**Run it:**
```bash
python activity1_pid.py            # single run with your gains
python activity1_pid.py --compare  # all three damping regimes side by side
```

> **Scope note:** this simplified PID corrects CTE only. The lecture (Slide 3)
> stresses that heading error matters too — the Stanley controller combines
> both. Here we keep PID minimal as the baseline.

**Expected output:** trajectory + CTE-over-time plot, saved as
`activity1_output.png`.

---

### Activity 2 — Pure Pursuit Controller `activity2_pure_pursuit.py`
**Type:** Implementation + comparison
**Time:** ~35 minutes

Implement the geometric look-ahead steering law and compare Pure Pursuit
against your PID controller on the same path.

**Your task — complete the formula in `pure_pursuit_control()`:**

```
δ = arctan(2 · L · sin(α) / Ld)
```

**Tuning — the single knob `Ld` (lecture Slide 8):**

| Look-ahead | Behavior |
|---|---|
| Small `Ld` | Tight tracking, but oscillates at speed |
| Large `Ld` | Smooth, but cuts corners on sharp turns |
| Adaptive `Ld = k·v` | Best of both — set `USE_ADAPTIVE = True` |

**Run it:**
```bash
python activity2_pure_pursuit.py
```

**Expected output:** Pure Pursuit vs PID trajectory + error plot, saved as
`activity2_output.png`. Answer the three reflection questions printed on screen.

---

### Activity 3 — PyBullet Warehouse Navigation `activity3_warehouse.py`
**Type:** Capstone integration
**Time:** ~25 minutes

The autonomy stack, end to end. A* (Lab 5) plans the warehouse route, your Pure
Pursuit controller (Activity 2) drives it, and the Husky robot follows the path
in the 3-D simulation.

> **Integration:** this script imports A* directly from Lab 5
> (`lab5_astar.py`, included in this package) and re-plans the route — no path
> file needs to be copied between labs. If `lab5_astar.py` is missing, a
> built-in fallback A* keeps the lab running.

**What you will see:**

| Visual element | Meaning |
|---|---|
| Green line | A* planned path (from Lab 5) |
| Red line | Current Pure Pursuit look-ahead target |
| Blue sphere | Goal position |
| Husky robot | Driving the path autonomously |

**Run it:**
```bash
python activity3_warehouse.py
```

Take a screenshot of the robot following the path. Answer the three reflection
questions printed on exit.

---

## Validate Before Submitting

```bash
python check_my_lab.py
```

All 4 tests (straight line, 90° curve, Ld sensitivity, left/right symmetry)
must show `[OK] PASS`.

---

## Submission Requirements

| # | Item | Filename |
|---|---|---|
| 1 | Completed PID script (with your tuned gains) | `activity1_pid.py` |
| 2 | Completed Pure Pursuit script | `activity2_pure_pursuit.py` |
| 3 | PyBullet screenshot of Activity 3 | `warehouse_run.png` |
| 4 | Reflection answers (Activities 2 & 3) | `reflections.txt` |
| 5 | Validation output (all tests passing) | `validation_output.txt` |

---

## Quick Reference

```
PID LAW        u(t) = Kp·e + Ki·∫e dt + Kd·de/dt    e = Cross-Track Error
               underdamped  : Kp high / Kd low   -> oscillates
               overdamped   : Kp low  / Kd high  -> sluggish, cuts corners
               critically   : balanced           -> smooth, ideal

PURE PURSUIT   δ = arctan(2·L·sin(α) / Ld)         one knob: Ld
               small Ld -> tight but oscillates
               large Ld -> smooth but cuts corners
               adaptive : Ld = k·v

UNICYCLE       ẋ = v·cos(θ)   ẏ = v·sin(θ)   θ̇ = ω
-> DIFF DRIVE  vR = v + ω·L/2     vL = v − ω·L/2     w = v_wheel / r

CONTROLLERS    PID          : 3 gains, ignores geometry      -> simple AGVs
(Slide 10)     Pure Pursuit : 1 knob, geometric, smooth      -> highway AVs
               Stanley      : combines CTE + heading, stable  -> urban AVs
```

---

## File Structure

```
Lab6_PathTracking/
├── README.md                      ← this file
├── activity1_pid.py               ← IMPLEMENT + TUNE
├── activity2_pure_pursuit.py      ← IMPLEMENT the look-ahead formula
├── activity3_warehouse.py         ← run and screenshot (capstone)
├── lab5_astar.py                  ← A* imported from Lab 5 (provided)
└── check_my_lab.py                ← validate before submitting
```

---

*Dr. Akram Fatayer · a.fatayer@zuj.edu.jo*
*linkedin.com/in/akram-fatayer · Al-Zaytoonah University of Jordan*
