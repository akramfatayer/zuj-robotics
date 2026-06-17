---
title: "Practice Exam Answer Key"
---

# Practice Exam Answer Key

**Course:** Introduction to Robotics for AI and Data Science (0135343)  
**Instructor:** Dr. Akram Fatayer · [a.fatayer@zuj.edu.jo](mailto:a.fatayer@zuj.edu.jo)

---

**PRACTICE Final Exam — Answer Key**

*Introduction to Robotics for AI & Data Science*

*For instructor use and student self-study.*


**Section A — Foundations & Sensing**

A1: C  — A power supply is not part of Sense-Plan-Act (the three are Sense, Plan, Act).

A2: B  — Inability to move sideways is the non-holonomic constraint.

A3: C  — LiDAR measures distance directly using laser time-of-flight.


A4 (model answer): Pose is the robot's position (x, y) together with its orientation theta. Orientation matters because two robots at the same (x, y) facing different directions will move to completely different places when they drive forward — heading determines the direction of motion, so navigation and control both depend on it.

Grading: pose = position + orientation (1 pt); correct reasoning about why heading matters (1.5 pts).

**Section B — Localization**

B1: B  — Drift is the accumulation of small errors over time.

B2: B  — During predict, with no new measurement, uncertainty GROWS (the +Q term).


B3 (model answer): Odometry is smooth but its error grows without bound (drift); the GPS-style sensor does not drift but is noisy and jumpy. A Kalman filter fuses them: it uses odometry between measurements to stay smooth, and periodically corrects the accumulated drift using the absolute fix. The result is more accurate and stable than either sensor alone — the strengths of one cover the weakness of the other.

Grading: identifies odometry drift (1 pt); identifies GPS noisy/no-drift (1 pt); explains fusion gives best of both (1 pt).


**Problem B4 — Kalman Filter**

**a) position_pred = position + velocity * dt = 2.0 + 0.5 * 1 = 2.5 m**

**b) innovation y = z - position_pred = 2.0 - 2.5 = -0.5**

**c) S = P_pred + R = 1.5 + 0.5 = 2.0**

**   K = P_pred / S = 1.5 / 2.0 = 0.75**

**   corrected position = 2.5 + 0.75 * (-0.5) = 2.5 - 0.375 = 2.125 m**

*Interpretation: the measurement (2.0) pulled the prediction (2.5) down toward it, but only partway, because the gain 0.75 balances trust between prediction and measurement.*

Grading: a) 1 pt; b) 1 pt; c) S, K, and corrected position 1 pt total (partial credit for method).

**Section C — Mapping & Planning**

C1: B  — 1 = occupied (obstacle).

C2: B  — Inflation accounts for the robot's physical size.

C3: B  — A 6-joint arm lives in high-dimensional space where grids are infeasible; RRT is the right tool.


C4 (model answer): An admissible heuristic never overestimates the true remaining cost to the goal. It matters because admissibility is the condition that guarantees A* returns the optimal (shortest) path; if the heuristic overestimates, A* may return a sub-optimal path.

Grading: definition of admissible (1.5 pts); guarantees optimal path (1 pt).


**Problem C5 — A* Path Planning**

**a) h(S) = |0-4| + |0-4| = 8**

**b) Valid neighbours of S=(0,0):**

- (0,1): g=1, h=|0-4|+|1-4|=7, f=8

- (1,0): g=1, h=|1-4|+|0-4|=7, f=8

   Both have f = 8 (a tie; A* may expand either first).

**c) One optimal path (cost 8):**

        col0 col1 col2 col3 col4
  row0|  S ->  ->  ->  ->  v |
  row1|    | X  | X |    | v |
  row2|    | X  |   |    | v |
  row3|    |    |   | X  | v |
  row4|    |    |   |    | G |

Path: (0,0)->(0,1)->(0,2)->(0,3)->(0,4)->(1,4)->(2,4)->(3,4)->(4,4). Total cost = 8 (8 moves x 1). Other equal-cost paths exist.

Grading: a) 1 pt; b) both neighbours with correct f 2 pts; c) any valid cost-8 path 2 pts.

**Section D — Tracking & Control**

D1: B  — The Integral term eliminates steady-state error.

D2: B  — Too-small Ld causes oscillation / jerky motion.


D3 (model answer): Cross-track error (CTE) is the perpendicular distance from the robot to the path; heading error is the angle between the robot's heading and the path's direction. Driving only CTE to zero is not enough because the robot can sit ON the path but point AWAY from it — it will immediately drift off again. Both must reach zero for stable tracking.

Grading: defines CTE (1 pt); defines heading error (1 pt); explains why CTE alone is insufficient (1 pt).


**Problem D4 — Pure Pursuit**

**a) angle to look-ahead = atan2(2.0-1.0, 3.0-1.0) = atan2(1, 2) = 26.57 deg = 0.4636 rad.**

**   Heading is 0, so alpha = 0.4636 rad (26.57 deg).**

**b) kappa = 2 sin(alpha) / Ld = 2 * sin(0.4636) / 2.236 = 2 * 0.4472 / 2.236 = 0.400**

**c) omega = v * kappa = 0.5 * 0.400 = 0.200 rad/s**

Grading: a) angle and alpha 1.5 pts; b) curvature 1.5 pts; c) omega 1 pt (partial credit for method).

**Section E — System Integration**

E1 (model answer): Each stage consumes the output of the one before it, so an early error propagates. Example: if the Kalman filter (Localize) reports a wrong pose, the LiDAR scan is placed in the wrong grid cells (Map), A* plans a path from the wrong start (Plan), and the controller drives the wrong route (Track). Any single weak stage degrades everything downstream.

Grading: states stages depend on each other (1 pt); gives a concrete propagation example (1 pt).


**End of Answer Key**
