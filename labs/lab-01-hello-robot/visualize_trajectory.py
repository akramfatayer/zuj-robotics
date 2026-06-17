# visualize_trajectory_v4.py
#
# Description:
# This script combines all previous concepts and adds data analysis and visualization.
# It runs the simulation in the background (DIRECT mode) for speed, records the
# robot's trajectory, and then uses Matplotlib to plot the results.
# This is a critical skill for debugging and verifying robot performance.
#
# Key Concepts:
#   - DIRECT Mode: Running simulation without GUI for faster-than-real-time execution.
#   - Data Logging: Storing state information at each time step for later analysis.
#   - Visualization: Using plots to understand and validate robot behavior.

import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import math

def run_simulation_and_get_trajectory():
    """Runs the simulation in DIRECT mode and returns the trajectory data."""
    
    # --- 1. Initialization in DIRECT mode ---
    # DIRECT mode runs without a GUI window, which is much faster.
    # This is ideal for batch processing, data collection, and automated testing.
    physicsClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")
    robotId = p.loadURDF("r2d2.urdf", [0, 0, 0.5], p.getQuaternionFromEuler([0, 0, 0]))
    print("Simulation started in DIRECT mode...")

    # --- 2. Data Logging Setup ---
    # We'll store the robot's position at each step in a list.
    trajectory = []

    # --- 3. Execute Movement (same logic as move_robot.py) ---
    side_length = 2.0
    linear_speed = 1.0
    turn_speed_rad = math.pi / 2

    move_duration = side_length / linear_speed
    turn_duration = (math.pi / 2) / turn_speed_rad

    for i in range(4):
        # Move Forward
        for _ in range(int(move_duration * 240)):
            _, orientation = p.getBasePositionAndOrientation(robotId)
            euler = p.getEulerFromQuaternion(orientation)
            yaw = euler[2]
            world_vx = linear_speed * math.cos(yaw)
            world_vy = linear_speed * math.sin(yaw)
            p.resetBaseVelocity(robotId, linearVelocity=[world_vx, world_vy, 0], angularVelocity=[0, 0, 0])
            pos, _ = p.getBasePositionAndOrientation(robotId)
            trajectory.append(pos)
            p.stepSimulation()

        # Turn Left
        for _ in range(int(turn_duration * 240)):
            p.resetBaseVelocity(robotId, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, turn_speed_rad])
            pos, _ = p.getBasePositionAndOrientation(robotId)
            trajectory.append(pos)
            p.stepSimulation()

    p.disconnect()
    print("Simulation finished. Trajectory recorded.")
    return np.array(trajectory)

def analyze_and_plot_trajectory(trajectory):
    """Takes trajectory data and creates plots for analysis."""
    print("Analyzing and plotting trajectory...")
    
    # --- 1. Create a time vector ---
    time = np.arange(len(trajectory)) / 240.0

    # --- 2. Create the Main Plot Figure ---
    # We create a figure with 2x2 subplots for detailed analysis.
    fig, axs = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Robot Trajectory Analysis", fontsize=16)

    # --- Plot 1: Top-Down Trajectory (X vs Y) ---
    # This is the most important plot. A correct square should be clearly visible.
    ax = axs[0, 0]
    ax.plot(trajectory[:, 0], trajectory[:, 1], label="Robot Path")
    ax.plot(trajectory[0, 0], trajectory[0, 1], "go", markersize=10, label="Start")
    ax.plot(trajectory[-1, 0], trajectory[-1, 1], "rs", markersize=10, label="End")
    ax.set_title("Top-Down Trajectory")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.grid(True)
    ax.axis("equal")
    ax.legend()

    # --- Plot 2: X Position vs Time ---
    ax = axs[0, 1]
    ax.plot(time, trajectory[:, 0], label="X Position")
    ax.set_title("X Position over Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("X Position (m)")
    ax.grid(True)
    ax.legend()

    # --- Plot 3: Y Position vs Time ---
    ax = axs[1, 0]
    ax.plot(time, trajectory[:, 1], label="Y Position")
    ax.set_title("Y Position over Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Y Position (m)")
    ax.grid(True)
    ax.legend()

    # --- Plot 4: Z Position vs Time (for stability check) ---
    # If the robot is stable, Z should remain roughly constant.
    ax = axs[1, 1]
    ax.plot(time, trajectory[:, 2], label="Z Position")
    ax.set_title("Z Position (Height) over Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Z Position (m)")
    ax.grid(True)
    ax.legend()
    ax.set_ylim(0, 1.0)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig("robot_trajectory_analysis.png")
    print("Saved detailed analysis plot to robot_trajectory_analysis.png")
    plt.show()

def main():
    """Main function to run simulation and visualize results."""
    trajectory_data = run_simulation_and_get_trajectory()
    analyze_and_plot_trajectory(trajectory_data)

if __name__ == "__main__":
    main()
