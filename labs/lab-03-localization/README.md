# Lab 3: Robot Localization and Sensor Fusion

> **MATLAB track:** the same lab, with the same questions, as MATLAB live scripts using the Sensor Fusion and Tracking Toolbox: see [`matlab/README.md`](matlab/README.md).

## Mission Context
In the previous labs, you learned how to command the R2D2 robot to move (Lab 1) and how to use a camera to detect objects (Lab 2). However, for a robot to be truly autonomous, it must answer the most fundamental question: **"Where am I?"**

Your R2D2 robot has two sensors to help it answer this question:
1. **Odometry (Wheel Encoders):** Measures how fast the wheels are turning. It updates very quickly (10Hz) but accumulates small errors over time, leading to **drift**.
2. **GPS:** Measures absolute position. It does not drift, but it is **noisy** and updates slowly (1Hz).

**Your Mission:** Neither sensor is perfect on its own. You will implement a **Kalman Filter** to fuse the data from both sensors, creating a smooth, accurate, and drift-free estimate of the robot's position. This is the foundation of the "Know Where You Are" block of our Autonomous Car project.

---

## Environment Setup
You will use the same `zuj-robotics` environment as in Lab 1. There is no need to reinstall anything!

1. On a lab PC, double-click **Robotics Lab** on the desktop. On your own laptop, open your terminal or Miniforge/Anaconda Prompt.
2. Activate the environment (lab PCs do this for you):
   ```bash
   conda activate zuj-robotics
   ```
3. Navigate to the folder containing this lab's files.

---

## Lab Activities

This lab is divided into three progressive activities. You must complete them in order.

### Activity 1: The Math (1D Kalman Filter)
**File:** `activity1_straight_1d.py`
Before applying the Kalman Filter to a complex robot, you will learn the core mathematics in a simple 1D scenario. A train moves along a straight track. You have noisy velocity readings and noisy position readings.
- **Task:** Run the script and analyze the output plot. Observe how the Kalman Filter estimate (purple line) is smoother and more accurate than either sensor alone.

### Activity 2: The Robot (Kalman Filter in 2D with Matrices)
**File:** `activity2_pybullet_2d_v3.py`
In 2D the state is the position [x, y]ᵀ, so every number in the filter becomes a vector or a matrix. Odometry is the control input of the predict step (u = the displacement it reports, F = I, B = I); a sensor updates the estimate.
- **Part A:** the script works through Lecture 2's 2D example matrix by matrix: predict with odometry, then update with GPS (H = I) or with a wall sensor that measures only x (H = [1, 0]). The numbers match section 5 of the [interactive module](https://akramfatayer.github.io/courses/robotics/learn/2).
- **Part B:** the same filter runs on R2D2 driving an L-shaped path in PyBullet. The plot (`activity2_pybullet_2d.png`) shows the path with 2σ uncertainty ellipses, and σx, σy and P₁₂ over time.
- **Task:** Check Part A by hand for at least K, then run Part B with `SENSOR = "gps"` and `SENSOR = "wall"` and answer the Activity 2 questions below.

### Activity 3: Odometry + GPS in a Straight Aisle
**File:** `activity3_pybullet_fusion.py`
R2D2 drives 60 m down a straight aisle. Odometry (10 Hz) over-reads slightly and drifts about 6 m; GPS (1 Hz, σ = 3 m) never drifts but each fix is about 3 m off. The script compares four estimates, each averaged over 20 runs:
1. **Odometry alone** (dead reckoning)
2. **GPS alone** (hold the last fix)
3. **The gated reset** from Lecture 2: follow odometry, and jump to a GPS fix when it lies outside a gate
4. **The Kalman filter**: predict with odometry (the control input), update with GPS

It is the same scenario as sections 3 and 7 of the [interactive module](https://akramfatayer.github.io/courses/robotics/learn/2).
- **Task:** Run the script, then change `GATE`, `Q` and `GPS_STD` in the **EXPERIMENT** block at the top and run it again. Answer the questions below.
- **Goal:** With the default settings, the Kalman filter's error is about 1 m, against about 3 m for odometry alone, GPS alone and the gated reset.

---

## What to Submit
Zip the following files and upload them to the Moodle submission portal:
1. `activity1_straight_1d.png`
2. `activity2_pybullet_2d.png`
3. `activity3_pybullet_fusion.png`
4. A short text file (`answers.txt`) answering these questions.

   **Activity 2**
   1. **By hand.** Compute K for the GPS update in Part A and check it against the script.
   2. **The wall sensor.** In Part A the wall sensor measures only x, yet the y estimate changes. Why? What would happen if P⁻₁₂ were 0?
   3. **Ellipses.** Run Part B with `SENSOR = "wall"`. Why do the ellipses get long in y after the turn, while σx stays small?

   **Activity 3**
   1. **The gated reset.** Find a jump in Plot A. How far from the truth was the estimate just before the jump, and just after it? Run `GATE = 2`, `4` and `10` and report the error each time. Why can the gated reset never be much better than GPS alone, whatever the gate?
   2. **Tuning Q.** Run `Q = 0.0001`, `0.03` and `3`. Report the Kalman filter's error for each and explain what goes wrong when Q is too small and when it is too large.
   3. **A worse GPS.** Set `GPS_STD = 6`. Which is better on its own now, odometry or GPS? Find the Q that gives the lowest Kalman error. How can a sensor that is worse on its own still improve the estimate?
   4. **The sawtooth.** Why does the variance P (Plot C) look like a sawtooth? What makes it grow, and what makes it drop?

---

## Download Lab Files

Run this lab online with one click, or download the Python files to run locally:

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/akramfatayer/zuj-robotics/blob/main/labs/lab-03-localization/Lab_03_Localization.ipynb)
&nbsp;&nbsp;**or download individual files below:**

| File | Description |
|------|-------------|
| [`activity1_straight_1d.py`](https://github.com/akramfatayer/zuj-robotics/raw/main/labs/lab-03-localization/activity1_straight_1d.py) | Activity 1 — 1-D Kalman filter on a straight run |
| [`activity2_pybullet_2d_v3.py`](https://github.com/akramfatayer/zuj-robotics/raw/main/labs/lab-03-localization/activity2_pybullet_2d_v3.py) | Activity 2 — 2-D Kalman filter with noisy odometry |
| [`activity3_pybullet_fusion.py`](https://github.com/akramfatayer/zuj-robotics/raw/main/labs/lab-03-localization/activity3_pybullet_fusion.py) | Activity 3 — Sensor fusion (odometry + GPS) |

> **Download everything at once:** [Download full repository as ZIP](https://github.com/akramfatayer/zuj-robotics/archive/refs/heads/main.zip) — extract and navigate to `zuj-robotics-main/labs/lab-03-localization/`

---
