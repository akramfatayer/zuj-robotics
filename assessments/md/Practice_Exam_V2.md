---
title: "Practice Exam"
---

# Practice Exam

**Course:** Introduction to Robotics for AI and Data Science (0135343)  
**Instructor:** Dr. Akram Fatayer · [a.fatayer@zuj.edu.jo](mailto:a.fatayer@zuj.edu.jo)

---

**Introduction to Robotics — PRACTICE Final Exam**

Course: Introduction to Robotics for AI & Data Science  |  Code: 0142351

Instructor: Dr. Akram Fatayer  |  Duration: 2 hours  |  Total: 40 points


*This practice exam mirrors the structure of the real final. The exam is organized by the robot's own pipeline — Sensing, Localization, Mapping & Planning, Tracking & Control, and Integration — so it walks the same path you followed all semester.*


**Instructions:**

- Closed book. A formula sheet is provided.

- Answer all questions. Show your work on calculation problems for partial credit.

- For multiple choice, circle the single best answer.


**Section A — Foundations & Sensing   (7 points)**

**Multiple Choice (3 x 1.5 = 4.5 points)**

**A1. Which of the following is NOT one of the three components of the Sense–Plan–Act paradigm?**

   A) Sensing

   B) Planning

   C) Power supply

   D) Acting

**A2. A wheeled robot that can drive forward and turn, but cannot move directly sideways, is subject to a:**

   A) holonomic constraint

   B) non-holonomic constraint

   C) kinematic singularity

   D) steady-state error

**A3. Which sensor is most appropriate for directly measuring the distance to nearby obstacles for mapping?**

   A) Wheel encoder

   B) IMU

   C) LiDAR

   D) Battery monitor

**Short Answer (2.5 points)**

**A4. Define the robot 'pose' and explain why orientation (theta) matters for navigation, not just position (x, y). (2-3 sentences)**

_______________________________________________________________________________________________

_______________________________________________________________________________________________

_______________________________________________________________________________________________

**Section B — Localization   (9 points)**

**Multiple Choice (2 x 1.5 = 3 points)**

**B1. Odometry drift occurs because:**

   A) the battery voltage drops over time

   B) small measurement errors accumulate over time

   C) the map resolution is too low

   D) the goal position keeps changing

**B2. In a Kalman filter, the PREDICT step uses the motion model to project the state forward. What happens to the estimate's uncertainty during predict (before any measurement)?**

   A) it shrinks

   B) it grows

   C) it stays exactly the same

   D) it becomes zero

**Short Answer (3 points)**

**B3. A robot has wheel odometry (smooth but drifts over time) and a noisy GPS-style sensor (jumpy but does not drift). Explain why fusing both with a Kalman filter gives a better pose estimate than using either one alone. (3-4 sentences)**

_______________________________________________________________________________________________

_______________________________________________________________________________________________

_______________________________________________________________________________________________

_______________________________________________________________________________________________

**Problem B4 — Kalman Filter, one step (3 points)**

A robot moves along a line. Its state is [position, velocity], and it uses a constant-velocity motion model with time step dt = 1 s. Use the Kalman filter equations from the formula sheet.

**Given:**

- Current state estimate: position = 2.0 m, velocity = 0.5 m/s

- Predicted covariance has already been computed: P_pred[position] = 1.5

- A new position measurement arrives: z = 2.0 m

- Measurement noise: R = 0.5


a) (1 pt) Compute the predicted position after the predict step.

_______________________________________________________________________________________________

b) (1 pt) Compute the innovation y.

_______________________________________________________________________________________________

c) (1 pt) Compute the innovation covariance S, the Kalman gain K for position, and the corrected position.

_______________________________________________________________________________________________

_______________________________________________________________________________________________

**Section C — Mapping & Planning   (12 points)**

**Multiple Choice (3 x 1.5 = 4.5 points)**

**C1. In an occupancy grid, a cell value of 1 typically represents:**

   A) free space

   B) an obstacle

   C) unknown / unexplored

   D) the goal

**C2. Why do we INFLATE obstacles in the occupancy grid before planning?**

   A) to make the map look clearer

   B) to account for the robot's physical size so its body does not clip obstacles

   C) to make A* run faster

   D) to increase the grid resolution

**C3. A* would be a POOR choice (and RRT a better one) for which of the following?**

   A) a 2-D warehouse floor grid

   B) planning for a 6-joint robot arm in high-dimensional space

   C) a small known static map

   D) finding the shortest path on a road network

**Short Answer (2.5 points)**

**C4. What does it mean for an A* heuristic to be ADMISSIBLE, and why does admissibility matter? (2-3 sentences)**

_______________________________________________________________________________________________

_______________________________________________________________________________________________

_______________________________________________________________________________________________

**Problem C5 — A* Path Planning (5 points)**

Consider the 5x5 grid below. 'S' is the start at (row 0, col 0), 'G' is the goal at (row 4, col 4), and 'X' are obstacles. Use 4-connected movement (up/down/left/right), step cost 1, and the Manhattan heuristic.

        col0 col1 col2 col3 col4
      +----+----+----+----+----+
  row0|  S |    |    |    |    |
      +----+----+----+----+----+
  row1|    |  X |  X |    |    |
      +----+----+----+----+----+
  row2|    |  X |    |    |    |
      +----+----+----+----+----+
  row3|    |    |    |  X |    |
      +----+----+----+----+----+
  row4|    |    |    |    |  G |
      +----+----+----+----+----+

a) (1 pt) Compute h(S), the Manhattan heuristic at the start.

_______________________________________________________________________________________________

b) (2 pts) After expanding S, list its valid neighbours and compute f(n) for each.

_______________________________________________________________________________________________

_______________________________________________________________________________________________

_______________________________________________________________________________________________

c) (2 pts) Give one optimal path from S to G and state its total cost.

_______________________________________________________________________________________________

_______________________________________________________________________________________________

_______________________________________________________________________________________________

**Section D — Tracking & Control   (10 points)**

**Multiple Choice (2 x 1.5 = 3 points)**

**D1. Which PID term is primarily responsible for eliminating steady-state error?**

   A) Proportional (P)

   B) Integral (I)

   C) Derivative (D)

   D) none of them

**D2. In Pure Pursuit, making the look-ahead distance Ld too SMALL tends to cause:**

   A) cutting corners on turns

   B) oscillation / jerky motion

   C) the robot to stop

   D) the robot to ignore the path

**Short Answer (3 points)**

**D3. A path tracker tries to drive two errors to zero: cross-track error (CTE) and heading error. Explain the difference between them, and why driving only CTE to zero is not enough. (3-4 sentences)**

_______________________________________________________________________________________________

_______________________________________________________________________________________________

_______________________________________________________________________________________________

_______________________________________________________________________________________________

**Problem D4 — Pure Pursuit (4 points)**

A mobile robot at position (1.0, 1.0) has heading 0 degrees (facing the +x direction) and linear velocity v = 0.5 m/s. The look-ahead point on the path is at (3.0, 2.0).

a) (1.5 pts) Compute the angle to the look-ahead point and the heading error alpha. (Hint: heading is 0, so alpha equals the angle to the point.)

_______________________________________________________________________________________________

_______________________________________________________________________________________________

b) (1.5 pts) The look-ahead distance is Ld = sqrt(5) ~ 2.236 m. Compute the curvature kappa.

_______________________________________________________________________________________________

_______________________________________________________________________________________________

c) (1 pt) Compute the angular velocity omega.

_______________________________________________________________________________________________

**Section E — System Integration   (2 points)**

**E1. In the full autonomy pipeline (Sense -> Localize -> Map -> Plan -> Track), explain how a poor result in ONE early stage damages a later stage. Give one concrete example. (2-3 sentences)**

_______________________________________________________________________________________________

_______________________________________________________________________________________________

_______________________________________________________________________________________________

_______________________________________________________________________________________________


**End of Practice Exam**
