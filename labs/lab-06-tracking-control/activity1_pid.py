"""
Lab 6: Path Tracking
Activity 1: PID Path Follower
Author: Dr. Akram Fatayer

Description:
    A unicycle robot follows a sine-wave path using a PID controller acting on
    the Cross-Track Error (CTE). Your task is to tune the gains to achieve a
    critically-damped response.

NOTE ON SCOPE (see lecture Slide 3):
    The lecture explains that a path tracker must drive BOTH the Cross-Track
    Error (CTE) and the Heading Error (psi) to zero. This activity uses a
    simplified PID that corrects CTE only -- it is the universal baseline.
    The Stanley controller (lecture Slide 9) is the one that explicitly
    combines CTE and heading error; we compare against Pure Pursuit in
    Activity 2.
"""

import numpy as np
import matplotlib.pyplot as plt

# --- ROBOT PARAMETERS ---
DT = 0.1          # Time step (seconds)
V  = 2.0          # Constant forward velocity (m/s)
L  = 2.5          # Wheelbase of the robot (meters)

# --- PID GAINS (TUNE THESE) ---
# TODO: Tune these gains to achieve smooth, critically-damped path following.
# Start from the presets in PRESETS below and observe the three damping regimes.
Kp = 0.8   # Proportional gain
Ki = 0.0   # Integral gain
Kd = 2.0   # Derivative gain

# Preset gain sets that demonstrate the three damping regimes from lecture
# Slide 6. Try each by copying its values into Kp/Ki/Kd above, or run
# `python activity1_pid.py --compare` to see all three at once.
PRESETS = {
    "underdamped":       dict(Kp=2.0,  Ki=0.0, Kd=0.0),  # oscillates / weaves
    "overdamped":        dict(Kp=0.05, Ki=0.0, Kd=0.5),  # sluggish, lags / cuts corners
    "critically_damped": dict(Kp=0.8,  Ki=0.0, Kd=2.0),  # smooth, ideal
}


def generate_path():
    """Generates a sine wave path for the robot to follow."""
    x = np.linspace(0, 50, 500)
    y = 5 * np.sin(x / 5.0)
    return np.column_stack((x, y))


def calculate_cte(robot_pos, path):
    """
    Calculates the signed Cross-Track Error (CTE).

    Magnitude = distance from the robot to the nearest path point.
    Sign tells us which side of the path the robot is on:
        cross(path_tangent, robot_offset) > 0  ->  robot LEFT of path  -> CTE negative
        cross(path_tangent, robot_offset) < 0  ->  robot RIGHT of path -> CTE positive

    This sign convention matters: the control law below is written to match it.
    """
    distances   = np.linalg.norm(path - robot_pos, axis=1)
    nearest_idx = np.argmin(distances)
    cte         = distances[nearest_idx]

    if nearest_idx < len(path) - 1:
        path_vec = path[nearest_idx + 1] - path[nearest_idx]
    else:
        path_vec = path[nearest_idx] - path[nearest_idx - 1]

    robot_vec  = robot_pos - path[nearest_idx]
    # 2-D scalar cross product (avoids NumPy 2.0 deprecation of np.cross on 2-vectors)
    cross_prod = path_vec[0] * robot_vec[1] - path_vec[1] * robot_vec[0]

    if cross_prod > 0:
        cte = -cte    # robot is to the LEFT -> negative CTE

    return cte


def run_simulation(Kp, Ki, Kd, steps=400):
    """
    Simulates the robot following the path with the given PID gains.
    Returns (path, history_x, history_y, history_cte).
    """
    path = generate_path()

    # Initial state [x, y, theta] -- start off the path so there is an error.
    robot_state = np.array([0.0, 2.0, 0.0])

    int_cte  = 0.0
    prev_cte = 0.0

    history_x   = [robot_state[0]]
    history_y   = [robot_state[1]]
    history_cte = []

    for _ in range(steps):
        robot_pos = robot_state[:2]
        theta     = robot_state[2]

        # 1. Error
        cte = calculate_cte(robot_pos, path)
        history_cte.append(cte)

        # 2. PID control law
        #
        #   u(t) = Kp*e(t) + Ki * INT e(t) dt + Kd * de(t)/dt     (lecture Slide 5)
        #
        # u is the steering angle (delta). With the CTE sign convention in
        # calculate_cte() (negative when the robot is LEFT of the path), a
        # POSITIVE steering angle turns the robot left -- so the terms are
        # added with a POSITIVE sign to steer back toward the path.
        int_cte += cte * DT
        diff_cte = (cte - prev_cte) / DT
        steer    = Kp * cte + Ki * int_cte + Kd * diff_cte

        # Physical steering limit (+/- 30 degrees)
        max_steer = np.radians(30)
        steer     = np.clip(steer, -max_steer, max_steer)

        prev_cte = cte

        # 3. Unicycle kinematics update
        robot_state[0] += V * np.cos(theta) * DT
        robot_state[1] += V * np.sin(theta) * DT
        robot_state[2] += (V / L) * np.tan(steer) * DT

        history_x.append(robot_state[0])
        history_y.append(robot_state[1])

        if robot_state[0] >= path[-1, 0]:
            break

    return path, history_x, history_y, history_cte


def plot_single(Kp, Ki, Kd):
    """Run and plot a single PID configuration."""
    print(f"Running PID simulation (Kp={Kp}, Ki={Ki}, Kd={Kd}) ...")
    path, hx, hy, hcte = run_simulation(Kp, Ki, Kd)
    print("Simulation complete.")

    steady = [abs(c) for c in hcte[50:]] or [abs(c) for c in hcte]
    print(f"  Steady-state |CTE|: max={max(steady):.3f} m, "
          f"mean={np.mean(steady):.3f} m")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    ax1.plot(path[:, 0], path[:, 1], 'k--', linewidth=1.5, label='Reference path')
    ax1.plot(hx, hy, '-', color='#2980B9', linewidth=2, label='Robot trajectory')
    ax1.set_title(f'PID Path Following\n(Kp={Kp}, Ki={Ki}, Kd={Kd})',
                  fontsize=12, fontweight='bold')
    ax1.set_xlabel('x (m)'); ax1.set_ylabel('y (m)')
    ax1.legend(); ax1.grid(True, alpha=0.3); ax1.axis('equal')

    ax2.plot(hcte, '-', color='#E74C3C', linewidth=1.5)
    ax2.axhline(0, color='gray', linewidth=0.8, linestyle='--')
    ax2.set_title('Cross-Track Error over Time', fontsize=12, fontweight='bold')
    ax2.set_xlabel('Time step'); ax2.set_ylabel('CTE (m)')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("activity1_output.png", dpi=130, bbox_inches='tight')
    print("[VIZ] Saved -> activity1_output.png")
    plt.show()


def plot_comparison():
    """
    Run all three damping presets side by side -- makes lecture Slide 6
    (underdamped / overdamped / critically damped) concrete and tunable.
    """
    print("Running all three damping regimes for comparison ...")

    fig, axes = plt.subplots(2, 3, figsize=(16, 8))
    colors = {'underdamped': '#E67E22',
              'overdamped': '#7F8C8D',
              'critically_damped': '#27AE60'}
    titles = {'underdamped': 'Underdamped (oscillating)',
              'overdamped': 'Overdamped (sluggish)',
              'critically_damped': 'Critically damped (ideal)'}

    for col, (name, gains) in enumerate(PRESETS.items()):
        path, hx, hy, hcte = run_simulation(**gains)
        c = colors[name]

        ax_traj = axes[0, col]
        ax_traj.plot(path[:, 0], path[:, 1], 'k--', linewidth=1.2, label='Reference')
        ax_traj.plot(hx, hy, '-', color=c, linewidth=2, label='Robot')
        ax_traj.set_title(f"{titles[name]}\nKp={gains['Kp']}, Kd={gains['Kd']}",
                          fontsize=11, fontweight='bold', color=c)
        ax_traj.set_xlabel('x (m)'); ax_traj.set_ylabel('y (m)')
        ax_traj.legend(fontsize=8); ax_traj.grid(True, alpha=0.3)
        ax_traj.set_ylim(-12, 12)

        ax_cte = axes[1, col]
        ax_cte.plot(hcte, '-', color=c, linewidth=1.3)
        ax_cte.axhline(0, color='gray', linewidth=0.8, linestyle='--')
        ax_cte.set_title('Cross-Track Error', fontsize=10)
        ax_cte.set_xlabel('Time step'); ax_cte.set_ylabel('CTE (m)')
        ax_cte.grid(True, alpha=0.3)
        ax_cte.set_ylim(-8, 8)

    plt.suptitle("PID Damping Regimes (lecture Slide 6)",
                 fontsize=13, fontweight='bold', y=1.00)
    plt.tight_layout()
    plt.savefig("activity1_output.png", dpi=130, bbox_inches='tight')
    print("[VIZ] Saved -> activity1_output.png")
    plt.show()


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == '--compare':
        plot_comparison()
    else:
        plot_single(Kp, Ki, Kd)
        print()
        print("TIP: run  'python activity1_pid.py --compare'  to see all")
        print("     three damping regimes side by side (lecture Slide 6).")
