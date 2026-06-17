---
title: "Formula Sheet"
---

# Formula Sheet

**Course:** Introduction to Robotics for AI and Data Science (0135343)  
**Instructor:** Dr. Akram Fatayer · [a.fatayer@zuj.edu.jo](mailto:a.fatayer@zuj.edu.jo)

---

**Introduction to Robotics — Formula Sheet**

*This sheet is provided during the final exam. No other materials are permitted.*


**Geometry & Trigonometry**

  Euclidean distance:   d = sqrt( (x2 - x1)^2 + (y2 - y1)^2 )
  Angle to a point:     theta = atan2( y2 - y1 , x2 - x1 )
  Angle normalization:  wrap any angle into [-pi, pi]
                        atan2( sin(angle) , cos(angle) )

**Occupancy Grid (Mapping)**

  World -> Grid:   col = (x - origin_x) / resolution
                   row = (y - origin_y) / resolution
  Grid -> World:   x = col * resolution + origin_x
                   y = row * resolution + origin_y

**Localization — Kalman Filter**

  PREDICT (prior):
     x_pred = F x
     P_pred = F P F^T + Q
  UPDATE (posterior):
     y = z - H x_pred           (innovation)
     S = H P_pred H^T + R       (innovation covariance)
     K = P_pred H^T S^-1        (Kalman gain)
     x = x_pred + K y           (corrected state)

x = state, P = covariance, F = motion model, H = measurement model, Q = process noise, R = measurement noise, z = measurement, K = gain.

**Path Planning — A***

  Cost function:   f(n) = g(n) + h(n)
     g(n) = actual cost from start to node n
     h(n) = heuristic estimate from n to goal

Heuristics:

  Manhattan (4-connected):  h = |x1 - x2| + |y1 - y2|
  Euclidean (8-connected):  h = sqrt( (x1-x2)^2 + (y1-y2)^2 )

Movement cost (8-connected grid): horizontal/vertical = 1, diagonal = sqrt(2) ~ 1.414

**Control — PID**

  Continuous:  u(t) = Kp e(t) + Ki integral(e dt) + Kd de/dt
  Discrete:    e[k]      = setpoint - measurement[k]
               integral  = integral + e[k] * dt
               derivative = ( e[k] - e[k-1] ) / dt
               u[k]      = Kp e[k] + Ki integral + Kd derivative

**Tracking — Pure Pursuit**

  Heading to look-ahead:  alpha = atan2(Ly - Ry, Lx - Rx) - robot_heading
  Curvature:              kappa = 2 sin(alpha) / Ld
  Angular velocity:       omega = v * kappa

Ld = look-ahead distance, v = linear velocity, omega = angular velocity.

**Kinematics — Unicycle & Differential Drive**

  Unicycle model:
     x_dot = v cos(theta)
     y_dot = v sin(theta)
     theta_dot = omega
  Unicycle -> wheel speeds (wheelbase L):
     v_R = v + omega * L / 2     (right wheel)
     v_L = v - omega * L / 2     (left wheel)

**Useful Constants**

  pi        = 3.14159...
  sqrt(2)   = 1.41421...
  sin(45 deg) = cos(45 deg) = 0.7071
  sin(30 deg) = 0.5      cos(30 deg) = 0.8660
  degrees -> radians: multiply by pi/180
  radians -> degrees: multiply by 180/pi


*End of Formula Sheet*
