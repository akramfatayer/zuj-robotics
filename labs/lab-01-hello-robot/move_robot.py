# move_robot_v4.py
#
# Description:
# This script demonstrates a simpler, more direct way to control robot motion
# using resetBaseVelocity. Instead of controlling individual wheel joints, we
# command the robot's body (its "base") directly with linear and angular velocities.
# This is less physically realistic but much easier for beginners to understand and control.
#
# Key Concepts:
#   - Base Control: Commanding the entire robot body's velocity.
#   - resetBaseVelocity: A PyBullet function to directly set the linear and angular
#     velocity of a robot's base link.
#   - Body-to-World Frame Conversion: Converting desired forward/turn speeds into
#     global X, Y, Z velocities based on the robot's current orientation.

import pybullet as p
import time
import pybullet_data
import math

def move_square(robotId):
    """Drives the robot in a 2x2 meter square using resetBaseVelocity."""
    
    # --- Define Movement Parameters ---
    side_length = 2.0  # meters
    linear_speed = 1.0 # m/s
    turn_speed_rad = math.pi / 2 # 90 deg/s in radians

    # --- Calculate Durations ---
    move_duration = side_length / linear_speed # 2 seconds
    turn_duration = (math.pi / 2) / turn_speed_rad # 1 second for a 90-degree turn

    print("\n--- Starting to move in a square ---")
    for i in range(4):
        print(f"Side {i+1}/4: Moving forward...")
        # --- Move Forward ---
        for _ in range(int(move_duration * 240)):
            _, orientation = p.getBasePositionAndOrientation(robotId)
            euler = p.getEulerFromQuaternion(orientation)
            yaw = euler[2]
            world_vx = linear_speed * math.cos(yaw)
            world_vy = linear_speed * math.sin(yaw)
            p.resetBaseVelocity(robotId, linearVelocity=[world_vx, world_vy, 0], angularVelocity=[0, 0, 0])
            p.stepSimulation()
            time.sleep(1./240.)

        print(f"Side {i+1}/4: Turning left...")
        # --- Turn Left ---
        for _ in range(int(turn_duration * 240)):
            p.resetBaseVelocity(robotId, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, turn_speed_rad])
            p.stepSimulation()
            time.sleep(1./240.)

    # --- Stop the robot ---
    print("Finished square. Stopping robot.")
    p.resetBaseVelocity(robotId, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])
    for _ in range(240):
        p.stepSimulation()

def main():
    """Initializes simulation, spawns a robot, and commands it to move."""
    
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    planeId = p.loadURDF("plane.urdf")
    
    # Using R2D2 because it works well with resetBaseVelocity
    startPos = [0, 0, 0.5]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    robotId = p.loadURDF("r2d2.urdf", startPos, startOrientation)
    print(f"R2D2 robot with ID {robotId} spawned.")

    move_square(robotId)

    print("Simulation finished.")
    p.disconnect()

if __name__ == "__main__":
    main()
