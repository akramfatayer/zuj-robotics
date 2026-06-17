"""
Lab 6: Path Tracking
Activity 2: Pure Pursuit Controller
Author: Dr. Akram Fatayer

Description:
    A unicycle robot follows a sine-wave path using the geometric Pure Pursuit
    controller. Your task is to complete the look-ahead steering formula and
    explore how the look-ahead distance Ld affects tracking.

    Pure Pursuit (lecture Slides 7-8) has a SINGLE tuning knob -- the look-ahead
    distance Ld -- versus PID's three gains. This script also overlays the PID
    result from Activity 1 so you can compare the two controllers on the same
    path, exactly as the lecture's comparison table (Slide 10) describes.
"""

import numpy as np
import matplotlib.pyplot as plt

# --- ROBOT PARAMETERS ---
DT = 0.1          # Time step (seconds)
V  = 2.0          # Constant forward velocity (m/s)
L  = 2.5          # Wheelbase of the robot (meters)

# --- PURE PURSUIT PARAMETER (TUNE THIS) ---
# TODO: Tune the look-ahead distance and observe the trade-off (lecture Slide 8):
#   small Ld -> tight tracking but oscillation
#   large Ld -> smooth but cuts corners
Ld = 3.0          # Look-ahead distance (meters)

# Optional: adaptive look-ahead Ld = k * v  (lecture Slide 8).
# Set USE_ADAPTIVE = True to scale Ld with velocity. With constant V this just
# scales Ld, but it shows the mechanism used by Tesla Autopilot / ROS Nav Stack.
USE_ADAPTIVE = False
ADAPTIVE_K   = 1.5    # time constant (seconds); Ld = k * v


def generate_path():
    """Generates a sine wave path for the robot to follow."""
    x = np.linspace(0, 50, 500)
    y = 5 * np.sin(x / 5.0)
    return np.column_stack((x, y))


def get_lookahead_point(robot_pos, path, Ld):
    """
    Finds the first path point that is at least Ld away from the robot,
    searching forward from the nearest point.
    """
    distances   = np.linalg.norm(path - robot_pos, axis=1)
    nearest_idx = np.argmin(distances)

    target_idx = nearest_idx
    for i in range(nearest_idx, len(path)):
        if distances[i] >= Ld:
            target_idx = i
            break
    else:
        target_idx = len(path) - 1   # reached the end -> aim at last point

    return path[target_idx]


def pure_pursuit_control(robot_state, target_point, Ld, L):
    """
    Calculates the steering angle using the Pure Pursuit geometric model.

    Args:
        robot_state  : [x, y, theta]
        target_point : [x, y] look-ahead point
        Ld           : look-ahead distance
        L            : wheelbase

    Returns:
        steer : steering angle command (radians)
    """
    robot_x, robot_y, theta = robot_state
    target_x, target_y      = target_point

    # 1. Global angle to the target point
    alpha_global = np.arctan2(target_y - robot_y, target_x - robot_x)

    # 2. Angle to target RELATIVE to robot heading (normalized to [-pi, pi])
    alpha = np.arctan2(np.sin(alpha_global - theta),
                       np.cos(alpha_global - theta))

    # 3. Pure Pursuit steering formula (lecture Slide 7):
    #        delta = arctan( 2 * L * sin(alpha) / Ld )
    # TODO: implement this formula
    steer = np.arctan2(2.0 * L * np.sin(alpha), Ld)

    return steer


def run_simulation(Ld, steps=400, use_adaptive=False, adaptive_k=1.5):
    """Simulate Pure Pursuit tracking. Returns (path, hx, hy, hcte)."""
    path = generate_path()
    robot_state = np.array([0.0, 2.0, 0.0])

    history_x   = [robot_state[0]]
    history_y   = [robot_state[1]]
    history_cte = []

    for _ in range(steps):
        robot_pos = robot_state[:2]
        theta     = robot_state[2]

        # Adaptive look-ahead scales with speed (constant here, shown for concept)
        Ld_eff = adaptive_k * V if use_adaptive else Ld

        target_point = get_lookahead_point(robot_pos, path, Ld_eff)
        steer        = pure_pursuit_control(robot_state, target_point, Ld_eff, L)

        max_steer = np.radians(30)
        steer     = np.clip(steer, -max_steer, max_steer)

        # Track CTE for comparison metrics
        d = np.linalg.norm(path - robot_pos, axis=1)
        history_cte.append(d.min())

        # Unicycle kinematics
        robot_state[0] += V * np.cos(theta) * DT
        robot_state[1] += V * np.sin(theta) * DT
        robot_state[2] += (V / L) * np.tan(steer) * DT

        history_x.append(robot_state[0])
        history_y.append(robot_state[1])

        if robot_state[0] >= path[-1, 0]:
            break

    return path, history_x, history_y, history_cte


def try_import_pid():
    """Attempt to run the Activity 1 PID for an overlay comparison."""
    try:
        from activity1_pid import run_simulation as pid_sim, Kp, Ki, Kd
        path, hx, hy, hcte = pid_sim(Kp, Ki, Kd)
        return (hx, hy, hcte, f"PID (Kp={Kp}, Kd={Kd})")
    except Exception:
        return None


def main():
    print("Starting Pure Pursuit simulation...")
    mode = "adaptive (Ld = k*v)" if USE_ADAPTIVE else f"fixed Ld = {Ld} m"
    print(f"  Look-ahead mode: {mode}")

    path, hx, hy, hcte = run_simulation(Ld, use_adaptive=USE_ADAPTIVE,
                                        adaptive_k=ADAPTIVE_K)
    print("Simulation complete.")

    steady = [abs(c) for c in hcte[50:]] or hcte
    print(f"  Steady-state distance-to-path: max={max(steady):.3f} m, "
          f"mean={np.mean(steady):.3f} m")

    # Optional PID overlay
    pid = try_import_pid()

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    ax1.plot(path[:, 0], path[:, 1], 'k--', linewidth=1.5, label='Reference path')
    ax1.plot(hx, hy, '-', color='#2980B9', linewidth=2,
             label=f'Pure Pursuit (Ld={Ld}m)')
    if pid:
        pid_hx, pid_hy, _, pid_label = pid
        ax1.plot(pid_hx, pid_hy, '-', color='#E67E22', linewidth=1.6,
                 alpha=0.8, label=pid_label)
    ax1.set_title('Pure Pursuit vs PID Path Following',
                  fontsize=12, fontweight='bold')
    ax1.set_xlabel('x (m)'); ax1.set_ylabel('y (m)')
    ax1.legend(fontsize=9); ax1.grid(True, alpha=0.3); ax1.axis('equal')

    ax2.plot(hcte, '-', color='#2980B9', linewidth=1.5, label='Pure Pursuit')
    if pid:
        _, _, pid_hcte, _ = pid
        ax2.plot([abs(c) for c in pid_hcte], '-', color='#E67E22',
                 linewidth=1.4, alpha=0.8, label='PID')
    ax2.set_title('Distance to Path over Time', fontsize=12, fontweight='bold')
    ax2.set_xlabel('Time step'); ax2.set_ylabel('|error| (m)')
    ax2.legend(fontsize=9); ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("activity2_output.png", dpi=130, bbox_inches='tight')
    print("[VIZ] Saved -> activity2_output.png")
    plt.show()

    print()
    print("REFLECTION QUESTIONS (answer in your lab report):")
    print("  Q1. Pure Pursuit has ONE tuning knob (Ld); PID has THREE.")
    print("      What does this trade off? When is each preferable?")
    print("  Q2. Re-run with Ld=1.0 then Ld=8.0. Describe the oscillation")
    print("      vs corner-cutting trade-off you observe (lecture Slide 8).")
    print("  Q3. Set USE_ADAPTIVE=True. Why does scaling Ld with speed help")
    print("      a real autonomous vehicle? (lecture Slide 8: Ld = k*v)")


if __name__ == '__main__':
    main()
