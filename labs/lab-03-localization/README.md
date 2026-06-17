# Lab 3: Robot Localization and Sensor Fusion

## Mission Context
In the previous labs, you learned how to command the R2D2 robot to move (Lab 1) and how to use a camera to detect objects (Lab 2). However, for a robot to be truly autonomous, it must answer the most fundamental question: **"Where am I?"**

Your R2D2 robot has two sensors to help it answer this question:
1. **Odometry (Wheel Encoders):** Measures how fast the wheels are turning. It updates very quickly (10Hz) but accumulates small errors over time, leading to **drift**.
2. **GPS:** Measures absolute position. It does not drift, but it is **noisy** and updates slowly (1Hz).

**Your Mission:** Neither sensor is perfect on its own. You will implement a **Kalman Filter** to fuse the data from both sensors, creating a smooth, accurate, and drift-free estimate of the robot's position. This is the foundation of the "Know Where You Are" block of our Autonomous Car project.

---

## Environment Setup
You will use the same Conda environment you created in Lab 1. There is no need to reinstall anything!

1. Open your terminal or Anaconda Prompt.
2. Activate your environment:
   ```bash
   conda activate robotics_YourFirstName
   ```
3. Navigate to the folder containing this lab's files.

---

## Lab Activities

This lab is divided into three progressive activities. You must complete them in order.

### Activity 1: The Math (1D Kalman Filter)
**File:** `activity1_straight_1d.py`
Before applying the Kalman Filter to a complex robot, you will learn the core mathematics in a simple 1D scenario. A train moves along a straight track. You have noisy velocity readings and noisy position readings.
- **Task:** Run the script and analyze the output plot. Observe how the Kalman Filter estimate (purple line) is smoother and more accurate than either sensor alone.

### Activity 2: The Robot (2D Kalman Filter with PyBullet)
**File:** `activity2_pybullet_2d.py`
Now we apply the Kalman Filter to the R2D2 robot in PyBullet. The robot moves in an L-shape pattern. We simulate noisy odometry and noisy GPS.
- **Task:** Run the script. Look at the generated plot (`activity2_pybullet_2d.png`). Notice how the "Odometry Only" path drifts away from the true path, while the "Fused" path stays close to the truth.

### Activity 3: Sensor Fusion (The Full Pipeline)
**File:** `activity3_pybullet_fusion.py`
This is the complete sensor fusion pipeline. The robot moves in a continuous circle. Odometry updates at 10Hz, but GPS only updates at 1Hz (every 10 steps).
- **Task:** Run the script. The terminal will output the Root Mean Square Error (RMSE) for Odometry, GPS, and the Fused estimate.
- **Goal:** The Fused RMSE must be lower than both the Odometry RMSE and the GPS RMSE. This proves that fusing two bad sensors creates one good estimate!

---

## Verification and Submission

Before submitting your work, run the automated self-grading script to ensure everything is correct:

```bash
python check_my_lab.py
```

If all tests pass, you will see a congratulatory message.

### What to Submit
Zip the following files and upload them to the Moodle submission portal:
1. `activity1_straight_1d.png`
2. `activity2_pybullet_2d.png`
3. `activity3_pybullet_fusion.png`
4. A short text file (`answers.txt`) answering this question: *In Activity 3, why does the position uncertainty (Plot D) look like a sawtooth pattern? What causes it to grow, and what causes it to drop?*
