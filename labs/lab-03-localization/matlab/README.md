# Lab 3: Robot Localization and Sensor Fusion — MATLAB Track

This is the MATLAB version of [Lab 3](../README.md). The mission, the activities, the questions and the grading are the same as in the Python track; only the tools differ.

| | Python track | MATLAB track |
|---|---|---|
| The Kalman filter | You write the equations with NumPy | You use `trackingKF` from the Sensor Fusion and Tracking Toolbox, then tune it and judge it by its error |
| The robot | R2D2 in PyBullet | The same motion, computed directly (no 3D simulator needed) |
| Files | `.py` scripts and a Colab notebook | Live scripts (`.m`): text, code and plots in one document |

Both tracks run exactly the same filters: given the same sensor readings, they give the same estimates. Your numbers differ slightly from a Python student's only because MATLAB's random numbers differ from NumPy's.

---

## Setup

You need **MATLAB R2025a or later** with the **Sensor Fusion and Tracking Toolbox**. The university licence includes both.

- **MATLAB Online (no installation):** click **Open in MATLAB Online** on the [course page](https://akramfatayer.github.io/courses/robotics) and sign in with your university MathWorks account. MATLAB copies the course repository into your MATLAB Drive and opens the activity.
- **MATLAB on a lab PC or your laptop:** download the three `.m` files below into one folder, open that folder in MATLAB, and double-click an activity. It opens in the Live Editor.

To run an activity, click **Run** on the **Live Editor** tab. Each activity saves its figure as a PNG in the current folder.

---

## Lab Activities

Complete them in order.

### Activity 1: The Kalman Filter in 1D
**File:** `activity1_straight_1d.m`
A robot drives along a straight line; a sensor measures its position once per second, with noise. `trackingKF` with the constant-velocity model combines the two.
- **Task:** Run the script. Compare the filter's error with the sensor's, and check that the true position stays inside the filter's ±2σ band about 95% of the time.

### Activity 2: The Kalman Filter in 2D, with Matrices
**File:** `activity2_kalman_2d.m`
- **Part A:** the lecture's 2D example with `trackingKF`: predict with odometry, then update with GPS (`H = eye(2)`) or with a wall sensor that measures only x (`H = [1 0]`). The script also computes K by hand, so you can see what `correct` does inside. The numbers match section 5 of the [interactive module](https://akramfatayer.github.io/courses/robotics/learn/2).
- **Part B:** the same filter while the robot drives an L-shaped path. The figure shows the path with 2σ uncertainty ellipses, and σx, σy and P12 over time.
- **Task:** Check Part A by hand for at least K, then run Part B with `SENSOR = "gps"` and `SENSOR = "wall"` and answer the Activity 2 questions below.

### Activity 3: Odometry + GPS in a Straight Aisle
**File:** `activity3_odometry_gps.m`
The robot drives 60 m down a straight aisle. Odometry (10 Hz) over-reads slightly and drifts about 6 m; GPS (1 Hz, σ = 3 m) never drifts but each fix is about 3 m off. The script compares four estimates, each averaged over 20 runs: odometry alone, GPS alone, the gated reset from Lecture 2, and the Kalman filter.
- **Task:** Run the script, then change `GATE`, `Q` and `GPS_STD` in the **Experiment** section and run it again. Answer the questions below.
- **Goal:** With the default settings, the Kalman filter's error is about 1 m, against about 3 m for odometry alone, GPS alone and the gated reset.

---

## What to Submit
Zip the following files and upload them to the Moodle submission portal:
1. `activity1_straight_1d.png`
2. `activity2_kalman_2d.png`
3. `activity3_odometry_gps.png`
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

| File | Open | Description |
|------|------|-------------|
| [`activity1_straight_1d.m`](https://github.com/akramfatayer/zuj-robotics/raw/main/labs/lab-03-localization/matlab/activity1_straight_1d.m) | [![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=akramfatayer/zuj-robotics&file=labs/lab-03-localization/matlab/activity1_straight_1d.m) | Activity 1 — 1D Kalman filter |
| [`activity2_kalman_2d.m`](https://github.com/akramfatayer/zuj-robotics/raw/main/labs/lab-03-localization/matlab/activity2_kalman_2d.m) | [![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=akramfatayer/zuj-robotics&file=labs/lab-03-localization/matlab/activity2_kalman_2d.m) | Activity 2 — 2D Kalman filter with matrices |
| [`activity3_odometry_gps.m`](https://github.com/akramfatayer/zuj-robotics/raw/main/labs/lab-03-localization/matlab/activity3_odometry_gps.m) | [![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=akramfatayer/zuj-robotics&file=labs/lab-03-localization/matlab/activity3_odometry_gps.m) | Activity 3 — Odometry + GPS fusion |
