---
title: "Course Summary"
---

# Course Summary

**Course:** Introduction to Robotics for AI and Data Science (0135343)  
**Instructor:** Dr. Akram Fatayer · [a.fatayer@zuj.edu.jo](mailto:a.fatayer@zuj.edu.jo)

---

**Introduction to Robotics — Course Summary**

**Course: Introduction to Robotics for AI & Data Science**

Instructor: Dr. Akram Fatayer

Final Exam Coverage: Labs 1, 3, 4, 5, 6 and the integration project


*How to use this summary: the course follows one robot through a single pipeline - Sense, Localize, Map, Plan, Track. Each numbered section below is one stage of that pipeline. For every stage, ask yourself three questions: (1) What problem does this stage solve? (2) What is the core method? (3) How does its output feed the next stage? If you can answer those three for all five stages, you understand the course.*


**The Autonomy Pipeline - The Big Picture**

  SENSE        LOCALIZE       MAP          PLAN         TRACK
  Lab 1   -->  Lab 3    -->   Lab 4   -->  Lab 5   -->  Lab 6
  spawn,       Kalman         LiDAR ->     A* / RRT     PID /
  sensors      filter         occupancy    path         Pure Pursuit
               (fuse)         grid         planning     control
     ^                                                      |
     +------------------ feedback loop --------------------+

Think of the pipeline as a chain of questions the robot must answer in order. First: 'What is around me?' (Sense). Then 'Where am I?' (Localize). Then 'What does the whole environment look like?' (Map). Then 'What route should I take?' (Plan). Finally 'How do I physically drive that route?' (Track). The output of each stage is the input to the next - which is why an error early in the chain corrupts everything after it.

The midterm project wired all five stages into one closed loop and added a dynamic obstacle that forced the robot to re-plan mid-route. The final exam tests the concepts behind each stage and your ability to do the light calculations that power them.

**1. Foundations & Sensing (Lab 1)**

**What is a robot?**

A robot is a programmable machine that senses its environment, processes that information to make decisions, and acts on the environment through actuators. The defining feature is the closed loop between perception and action: a robot does not blindly execute a fixed script - it continuously adapts to what it perceives. This is what separates a robot from, say, a washing machine running a timed cycle.

- Sensing - perceiving the environment (cameras, LiDAR, IMU, encoders)

- Processing - deciding what to do based on sensor data (the algorithms in this course)

- Acting - affecting the environment through motors, wheels, or grippers

**The Sense-Plan-Act paradigm**

Sense-Plan-Act is the classic architecture for organizing a robot's behaviour. The robot senses the world, plans an action based on that information, then acts - and the cycle repeats many times per second. The feedback arrow is the key: because the robot re-senses after acting, it can correct its own mistakes.

  +--------+     +--------+     +--------+
  | SENSE  | --> |  PLAN  | --> |  ACT   |
  +--------+     +--------+     +--------+
      ^                             |
      +------- feedback loop -------+

Open-loop vs closed-loop: an OPEN-LOOP system acts without checking the result (e.g. 'drive forward for 3 seconds'). A CLOSED-LOOP system uses sensor feedback to adjust as it goes (e.g. 'drive forward until the sensor says I have arrived'). Closed-loop control is more robust because it corrects for disturbances - a wheel slipping, a slope, a push.

**Common sensors and what they measure**

- Camera - captures images/light; used for object detection and visual navigation cues

- LiDAR - measures distance by timing reflected laser pulses; the workhorse for mapping and obstacle detection

- IMU (Inertial Measurement Unit) - measures acceleration and angular rotation; used to estimate orientation and motion

- Wheel encoders - count wheel rotations; used for odometry (estimating how far the robot has driven)

A useful way to group them: PROPRIOCEPTIVE sensors measure the robot's own internal state (encoders, IMU), while EXTEROCEPTIVE sensors measure the outside world (camera, LiDAR). Good localization usually fuses both kinds.

**Key terms you must know**

- Pose - the robot's position (x, y) together with its orientation (theta). In 2-D, a full pose is three numbers.

- Degrees of Freedom (DOF) - the number of independent parameters needed to fully describe the robot's configuration. A mobile robot on a plane has 3 (x, y, theta); a 6-joint arm has 6.

- Configuration space (C-space) - the set of all poses the robot can take. Planning happens in this space.

- Non-holonomic constraint - a restriction on how a robot can move. A car or differential-drive robot can drive forward and turn, but cannot slide directly sideways. This is why parallel parking takes several moves rather than one sideways slide.

Common misconception: 'more DOF is always better'. More DOF means more flexibility but also a much larger configuration space to search - which is exactly why high-DOF robot arms need RRT instead of grid-based A* (Section 4).

**2. Localization - 'Where am I?' (Lab 3)**

Localization is the problem of estimating the robot's pose from noisy, imperfect sensor data. It sounds simple - 'just read the position' - but no real sensor gives a perfect, drift-free position. The art is combining imperfect sensors into a good estimate.

**Odometry and drift**

Odometry estimates position by integrating wheel motion: if you know the wheel sizes and how much each wheel turned, you can compute how far and in what direction the robot moved. Odometry is always available and updates very fast, which is its strength.

Its fatal weakness is DRIFT. Every measurement has a tiny error (wheel slip on a smooth floor, slightly uneven tyre pressure, rounding). Because odometry ADDS each step's motion to the previous estimate, these tiny errors ACCUMULATE. After a long drive, the estimate can be metres off, and it only ever gets worse - odometry never corrects itself. Think of it like compounding interest, but on error.

**Why fuse sensors?**

No single sensor is both accurate and drift-free. Odometry is smooth and fast but drifts without bound. An absolute sensor like GPS does not drift (it is anchored to the real world) but is noisy and jumpy from moment to moment. Sensor fusion combines them so the strengths of one cover the weakness of the other: use odometry for smooth short-term motion, and use the absolute fix to periodically pull the estimate back to reality.

AI/DS connection: this is the same idea as ENSEMBLE methods in machine learning. A single weak model is unreliable, but combining several models whose errors are independent gives a stronger prediction than any one alone. The Kalman filter is, in effect, a principled weighted average of two noisy estimates.

**The Kalman Filter - the core idea**

The Kalman filter is the standard tool for fusing a motion model with noisy measurements. It maintains two things: a STATE estimate (the robot's best guess of its pose and velocity) and a COVARIANCE (how UNCERTAIN it is about that guess). It repeats two steps forever:

- PREDICT - use the motion model to project the state forward in time. This produces the 'prior'. Because the model is imperfect, uncertainty GROWS during predict.

- UPDATE - fold in a new measurement to correct the prediction. This produces the 'posterior'. Because we just gained information, uncertainty SHRINKS during update.

The genius is the KALMAN GAIN, which decides how much to trust the new measurement versus the prediction. If the measurement is very noisy (large R), the gain is small and the filter mostly trusts its prediction. If the model is very uncertain (large process noise Q), the gain is large and the filter leans on the measurement. The gain is computed automatically every step from the current uncertainties.

  PREDICT:  x_pred = F x            (project the state forward)
            P_pred = F P F^T + Q    (uncertainty grows by Q)
  UPDATE:   y = z - H x_pred        (innovation = measurement - prediction)
            S = H P_pred H^T + R    (innovation covariance)
            K = P_pred H^T S^-1     (Kalman gain)
            x = x_pred + K y        (corrected estimate)
            P = (I - K H) P_pred    (uncertainty shrinks)

The INNOVATION (y) is worth understanding: it is the surprise - the gap between what the sensor reported and what the filter expected. If the innovation is zero, the measurement told us nothing new and the estimate does not change. The bigger the surprise, the bigger the correction.

Worked intuition (1-D): suppose the filter predicts the robot is at 2.5 m, and a measurement says 2.0 m. The innovation is 2.0 - 2.5 = -0.5. If the gain works out to 0.75, the corrected estimate is 2.5 + 0.75 x (-0.5) = 2.125 m - the estimate moved most of the way toward the measurement, but not all the way, because the filter still partly trusts its prediction.

*Exam note: you will NOT multiply 4x4 matrices by hand. You should be able to plug numbers into the formulas for a simple 1-D case, compute an innovation and a corrected estimate, and explain in words what predict and update each do.*

**3. Mapping - 'What does the world look like?' (Lab 4)**

Once the robot knows where it is, it needs a model of the environment to plan through. Mapping builds that model from sensor data as the robot explores.

**The occupancy grid**

An occupancy grid divides the world into a grid of square cells, where each cell stores the probability that it is occupied by an obstacle. It is the most common map representation for mobile robots because it is simple, and it plugs directly into grid-based planners like A*.

- 0 - free space (the robot can safely pass through)

- 1 - occupied (an obstacle; the robot must avoid)

- unknown - not yet observed by any sensor

In practice cells store a probability or a 'log-odds' value rather than a hard 0/1, so that repeated observations can strengthen or weaken belief in a cell. Log-odds is used because it lets you UPDATE belief by simple addition rather than repeated probability multiplication.

**Building the grid from LiDAR**

Each LiDAR beam carries two pieces of information. First, the cell where the beam STOPPED is probably occupied - something reflected the laser back. Second, every cell the beam PASSED THROUGH on the way is probably free - the laser travelled through empty space to get there. So for each beam we trace its path across the grid and nudge the cells: free cells along the ray become more 'free', and the endpoint cell becomes more 'occupied'.

Tracing the straight line of cells a beam crosses is done with Bresenham's line algorithm - an efficient, integer-only way to step from the robot's cell to the hit cell. You do not need to reproduce Bresenham on the exam, but you should know WHY it is used: to mark every free cell along a beam.

**Coordinate conversion (world <-> grid)**

Sensors report distances in metres (the 'world' frame), but the grid is indexed by integer rows and columns. You constantly convert between them.

  World -> Grid:   col = (x - origin_x) / resolution
                   row = (y - origin_y) / resolution
  Grid -> World:   x = col * resolution + origin_x
                   y = row * resolution + origin_y

Worked example: with a resolution of 0.25 m and the world origin at the grid centre, a point at x = 2.0 m lands in column 2.0 / 0.25 = 8 columns from the centre. Mixing up this conversion is a classic bug - it places obstacles in the wrong cells and ruins the map.

**Obstacle inflation**

The planner treats the robot as a single point with no size. But a real robot has a body, and a path that grazes the corner of a shelf will crash it. The fix is to INFLATE every obstacle by the robot's radius (plus a safety margin) before planning. The robot can then be planned as a point through the inflated map and its body will still clear real obstacles.

Trade-off: too little inflation risks collisions; too much inflation can seal off narrow gaps and make the planner report 'no path' even though one physically exists. (This exact failure can occur in the project if the inflation radius is set too large.)

**Resolution trade-off**

- High resolution (small cells) - accurate, captures narrow gaps, but uses more memory and makes planning slower

- Low resolution (large cells) - fast and memory-light, but may merge nearby obstacles and miss tight passages

Choosing resolution is a practical engineering decision: fine enough to represent the gaps the robot must fit through, coarse enough to plan in reasonable time.

**4. Path Planning - 'How do I get from A to B?' (Lab 5)**

Planning finds a collision-free route from the start to the goal through the map. The two methods in this course represent two different worlds: A* for discrete grids, RRT for continuous high-dimensional spaces.

**A* search**

A* (pronounced 'A-star') is the workhorse of grid path planning. It is a best-first search: it always expands the node that looks most promising, judged by the cost function:

  f(n) = g(n) + h(n)

- g(n) - the ACTUAL cost of the path from the start to node n (known exactly, because we have already travelled it)

- h(n) - the HEURISTIC estimate of the remaining cost from n to the goal (a guess about the future)

- f(n) - the total estimated cost of a path that goes through n; A* always expands the node with the smallest f

Intuition: g(n) pulls the search to stay efficient (don't wander), while h(n) pulls it toward the goal (don't explore backwards). Balancing the past cost and the estimated future cost is what makes A* both correct and fast.

**Heuristics and admissibility**

A heuristic is ADMISSIBLE if it NEVER OVERESTIMATES the true remaining cost. Admissibility is the crucial property: if h is admissible, A* is GUARANTEED to find the optimal (shortest) path. If h overestimates, A* may confidently return a path that is not actually the shortest.

- Manhattan distance |dx| + |dy| - admissible for 4-connected grids (movement up/down/left/right only)

- Euclidean distance sqrt(dx^2 + dy^2) - admissible for 8-connected or any-angle movement

If you set h = 0 everywhere, A* loses all sense of direction and explores outward uniformly in every direction - this special case is exactly Dijkstra's algorithm. So A* can be seen as 'Dijkstra plus a sense of direction'. A good heuristic is what lets A* explore far fewer cells.

Worked example: on a 4-connected grid, the Manhattan heuristic from (0, 0) to (4, 3) is |0-4| + |0-3| = 7. If the straight Manhattan route is not blocked, the optimal path cost is also 7.

**A* properties**

- Complete - if a path exists, A* will find it

- Optimal - it finds the SHORTEST path, provided the heuristic is admissible

- Efficient - the heuristic steers the search toward the goal, so it expands far fewer cells than uninformed search

**RRT - when grids are not enough**

A* requires a grid, and grids explode in high dimensions. Consider a robot arm with 6 joints: to grid each joint angle into just 10 buckets would require 10^6 = one million cells; finer resolution or more joints quickly becomes astronomically large. This is the CURSE OF DIMENSIONALITY, and it makes grid planning hopeless for arms.

RRT (Rapidly-exploring Random Tree) sidesteps the grid entirely. It grows a tree outward from the start by repeatedly sampling random points in the continuous space and steering toward them, skipping samples that would cause a collision, until the tree reaches the goal.

- Probabilistically complete - it will find a solution if one exists, GIVEN ENOUGH TIME (not instantly guaranteed like A*)

- NOT optimal - the path it returns is valid but usually jagged and longer than necessary (the variant RRT* improves this toward optimal)

- Scales to high dimensions - its cost depends on the number of samples, not on the size of a grid, so it handles 6-DOF arms where A* cannot

Rule of thumb to remember: use A* for 2-D/3-D grids and known maps where you want the shortest path; use RRT for high-dimensional configuration spaces like robot arms where a grid is infeasible.

**5. Path Tracking & Control - 'How do I drive the path?' (Lab 6)**

Planning gives a sequence of waypoints, but a robot cannot teleport between them - it must physically drive the path with its motors. A tracking controller turns the planned path into real-time steering commands, continuously correcting as the robot drifts.

**Two errors every tracker minimizes**

- Cross-Track Error (CTE) - the perpendicular distance from the robot to the nearest point on the path. It answers 'how far OFF the path am I sideways?'

- Heading Error (psi) - the angle between the robot's heading and the path's direction. It answers 'am I POINTING the right way?'

Every tracking controller drives BOTH toward zero. This is a subtle but important point: a robot can sit exactly ON the path (CTE = 0) while pointing 90 degrees away from it - and it will immediately drive off again. Correcting position alone is not enough; heading must also be corrected.

**The unicycle model**

To design controllers independently of the specific robot hardware, we model the robot as a 'unicycle' with state [x, y, theta] and two control inputs: linear velocity v and angular velocity omega.

  x_dot     = v cos(theta)    (how x changes over time)
  y_dot     = v sin(theta)    (how y changes over time)
  theta_dot = omega           (how heading changes over time)

The controller outputs (v, omega), and these are then converted to actual wheel speeds for a two-wheel (differential drive) robot with wheelbase L:

  v_R = v + omega * L / 2     (right wheel speed)
  v_L = v - omega * L / 2     (left wheel speed)

Intuition: to turn, one wheel spins faster than the other. The +/- omega term makes the outer wheel faster and the inner wheel slower, which rotates the robot. If omega = 0, both wheels turn at speed v and the robot drives straight.

Worked example: for v = 1.0 m/s, omega = 0.5 rad/s, L = 0.5 m: v_R = 1.0 + 0.5 x 0.25 = 1.125 m/s and v_L = 1.0 - 0.5 x 0.25 = 0.875 m/s. The right wheel is faster, so the robot curves left.

**PID control**

PID is the universal feedback controller, used far beyond robotics. Applied to path tracking, the error e is the cross-track error and the output u is a steering command.

  u(t) = Kp e(t) + Ki integral(e dt) + Kd de/dt

- P (Proportional) - reacts to the CURRENT error. Larger error -> stronger correction. Too high a Kp causes overshoot and oscillation.

- I (Integral) - accumulates PAST error over time. It removes steady-state error from a constant bias (e.g. a slightly misaligned wheel). Too high a Ki causes instability and 'windup'.

- D (Derivative) - reacts to the RATE OF CHANGE of error, anticipating the future. It damps oscillation by counter-steering as the robot approaches the path. Too high a Kd amplifies sensor noise.

AI/DS connection: PID is conceptually close to gradient descent with momentum - the P term is like the gradient step, the D term is like momentum/damping, and the I term corrects persistent bias.

Tuning and damping regimes - a well-tuned controller is 'critically damped'. The three behaviours to recognize:

- Underdamped - Kp too high or Kd too low; the robot oscillates and weaves across the path

- Overdamped - Kp too low or Kd too high; the robot is sluggish and cuts corners, taking too long to reach the path

- Critically damped - well tuned; the robot converges to the path quickly and smoothly without overshooting

A common tuning recipe: raise Kp until the robot just begins to oscillate, add Kd to damp the oscillation, then add a little Ki only if a steady-state offset remains.

**Pure Pursuit**

Pure Pursuit is a GEOMETRIC tracker. Instead of computing error derivatives like PID, it picks a 'look-ahead point' a fixed distance Ld ahead on the path and steers the robot toward it - like a driver fixing their eyes on a point down the road, or a dog chasing a thrown ball.

  alpha = atan2(Ly - Ry, Lx - Rx) - robot_heading   (heading to look-ahead)
  kappa = 2 sin(alpha) / Ld                          (path curvature to follow)
  omega = v * kappa                                  (resulting turn rate)

Its great advantage is having only ONE tuning knob, the look-ahead distance Ld, versus PID's three gains. But that knob involves a trade-off:

- Small Ld - the robot tracks tightly but tends to oscillate, especially at speed

- Large Ld - the robot moves smoothly but cuts corners on sharp turns

- Adaptive Ld = k * v - scale the look-ahead with speed: short at low speed for precision, long at high speed for smoothness. Used by the ROS Navigation Stack and early Tesla Autopilot.

Worked example: a robot at (1, 1) facing along +x, with a look-ahead point at (3, 2) and Ld = sqrt(5). The heading to the point is atan2(1, 2) = 26.6 degrees, so alpha = 0.4636 rad. Then kappa = 2 sin(0.4636)/2.236 = 0.40, and at v = 0.5 m/s, omega = 0.5 x 0.40 = 0.20 rad/s.

**Stanley controller**

The Stanley controller was developed by Stanford's team for the autonomous car that won the 2005 DARPA Grand Challenge (a 212 km desert race). Unlike Pure Pursuit, it EXPLICITLY combines both heading error and cross-track error in its steering law, which makes it excellent at tracking tight, complex paths accurately. Its proven robustness made it a standard in the autonomous-vehicle field.

**Choosing a controller**

- PID - simple and universal, but ignores the path's geometry and is sensitive to speed changes; good for simple automated guided vehicles (AGVs)

- Pure Pursuit - one intuitive tuning knob, smooth at high speed, but cuts corners; good for highway-style driving

- Stanley - combines heading and cross-track error, tracks tight paths precisely, but needs accurate heading estimation; good for urban driving

**How the Stages Connect (and Fail Together)**

The single most important idea in the course is that the stages form a chain, and each depends on the one before it. The exam's integration question tests exactly this.

- If LOCALIZATION reports the wrong pose, the LiDAR scan gets placed in the wrong grid cells, so the MAP is wrong.

- If the MAP is wrong, A* plans a path around obstacles that are not really there, or through ones that are - so the PLAN is wrong.

- If the PLAN is wrong, the controller faithfully drives a bad route - so TRACKING the path perfectly still takes the robot to the wrong place.

This is why a robotics system is only as strong as its weakest stage, and why real systems close the loop: they keep sensing and re-localizing, and they re-plan when the world changes (as in the project's dynamic-obstacle scenario).

**Study Tips**

Understand the WHY, not just the formula. The exam is weighted toward reasoning and light calculation, not heavy mathematics.

- For each stage, be able to state the problem it solves, its core method, and what it passes to the next stage.

- Connect ideas to your AI/DS background: A* is informed search; sensor fusion is like ensembling; PID resembles gradient descent with momentum; the curse of dimensionality appears in both RRT and machine learning.

- Practice the light calculations until they are automatic: one A* expansion (compute g, h, f for a node), one Pure Pursuit angle and curvature, the wheel-speed conversion, one PID output, and one Kalman update (innovation and corrected estimate).

- Memorize the small comparison tables: A* vs RRT, and PID vs Pure Pursuit vs Stanley. These map directly to short-answer and multiple-choice questions.

- Draw the diagrams by hand: the pipeline, the Sense-Plan-Act loop, the cross-track/heading error geometry, and the three PID damping curves.

- Watch your units and signs: radians vs degrees, and which side of the path a positive cross-track error means.


**Good luck on your exam!**
