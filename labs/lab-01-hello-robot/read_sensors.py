# read_sensors_v2.py
#
# Description:
# This script builds upon spawn_robot.py by introducing the "Sense" part of the
# Sense-Plan-Act cycle. Its purpose is to demonstrate how to query the physics
# engine for the state of a robot.
#
# Key Concepts:
#   - Robot State: A robot's state includes its position, orientation, and velocities.
#   - Proprioceptive Sensing: Sensing the internal state of the robot (as opposed to
#     exteroceptive sensing, which is sensing the external world, like with a camera).
#   - getBasePositionAndOrientation: The core PyBullet function for this task.

import pybullet as p
import time
import pybullet_data

def main():
    """Initializes simulation, spawns a robot, and reads its sensor data."""
    
    # --- 1. Initialization (Same as spawn_robot.py) ---
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    planeId = p.loadURDF("plane.urdf")
    startPos = [0, 0, 0.1]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    robotId = p.loadURDF("r2d2.urdf", startPos, startOrientation)
    print(f"Robot with ID {robotId} spawned.")

    # --- 2. Simulation Loop with Sensing ---
    # We will run the simulation for 5 seconds and print the robot's state every 0.5 seconds.
    print("\n--- Reading Robot State for 5 seconds ---")
    for i in range(int(5 * 240)):
        p.stepSimulation()

        # We only print every 120 steps (0.5 seconds) to avoid flooding the console.
        if i % 120 == 0:
            # --- Get the robot's position and orientation ---
            # This is the core function for proprioceptive sensing of the robot's base.
            # It returns the position (x, y, z) and orientation (a quaternion).
            pos, orn = p.getBasePositionAndOrientation(robotId)
            
            # --- Get the robot's velocity ---
            # This function returns the linear velocity (vx, vy, vz) and angular velocity (wx, wy, wz).
            linear_vel, angular_vel = p.getBaseVelocity(robotId)

            # --- Print the results in a readable format ---
            print(f"Time: {i/240.0:.2f}s")
            print(f"  Position (x,y,z):    ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
            
            # Convert quaternion to Euler for easier interpretation
            orn_euler = p.getEulerFromQuaternion(orn)
            print(f"  Orientation (r,p,y): ({orn_euler[0]:.3f}, {orn_euler[1]:.3f}, {orn_euler[2]:.3f})")
            
            print(f"  Linear Velocity:     ({linear_vel[0]:.3f}, {linear_vel[1]:.3f}, {linear_vel[2]:.3f})")
            print(f"  Angular Velocity:    ({angular_vel[0]:.3f}, {angular_vel[1]:.3f}, {angular_vel[2]:.3f})\n")

    print("Simulation finished.")
    p.disconnect()

if __name__ == "__main__":
    main()
