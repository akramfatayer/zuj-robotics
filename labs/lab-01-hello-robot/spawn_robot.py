# spawn_robot_v2.py
#
# Description:
# This script serves as the most basic "Hello, World!" for robotics simulation.
# Its purpose is to demonstrate the fundamental steps of:
#   1. Connecting to the physics engine (PyBullet).
#   2. Loading the simulation environment (a ground plane).
#   3. Loading a robot model from a URDF file.
#   4. Running the simulation for a fixed duration.
#
# Every robotics simulation you build will start with these core steps.

import pybullet as p
import time
import pybullet_data

def main():
    """Initializes the simulation and spawns a robot."""
    
    # --- 1. Connect to the Physics Engine ---
    # p.GUI starts the simulation with a graphical user interface.
    # p.DIRECT would run the simulation without a window (useful for batch processing).
    print("Starting simulation...")
    physicsClient = p.connect(p.GUI)
    print(f"Connected to physics server with ID: {physicsClient}")

    # --- 2. Set up the Simulation Environment ---
    # Set the search path for PyBullet to find its default assets (like the plane).
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Set the gravity of the world. The default is (0, 0, -9.8).
    p.setGravity(0, 0, -9.8)
    
    # Load a ground plane. The URDF (Unified Robot Description Format) is an XML file
    # that describes all elements of a robot or, in this case, a simple plane.
    print("Loading ground plane...")
    planeId = p.loadURDF("plane.urdf")
    print(f"Loaded plane with ID: {planeId}")

    # --- 3. Load the Robot ---
    # Define the robot's starting position and orientation.
    # Position is in meters (x, y, z).
    # Orientation is a quaternion (x, y, z, w), which represents rotation.
    # p.getQuaternionFromEuler converts from human-readable Euler angles (roll, pitch, yaw).
    startPos = [0, 0, 0.1]  # Start slightly above the ground to avoid initial collisions.
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    
    # Load the R2D2 model. PyBullet will search for this file in its data path.
    # The function returns a unique integer ID for the robot body.
    print("Loading robot model...")
    robotId = p.loadURDF("r2d2.urdf", startPos, startOrientation)
    print(f"Loaded robot with ID: {robotId}")

    # --- 4. Run the Simulation ---
    # The simulation runs in a loop. In each step, we advance the physics.
    # 1/240 is the default time step (240 Hz). The simulation will run for 5 seconds.
    print("Running simulation for 5 seconds...")
    for i in range(int(5 * 240)):
        p.stepSimulation()
        # time.sleep(1./240.) # Optional: slow down to real-time. Not needed for simple sims.

    print("Simulation finished.")

    # --- 5. Disconnect from the Physics Engine ---
    # It's good practice to clean up and close the connection.
    p.disconnect()
    print("Disconnected from physics server.")

if __name__ == "__main__":
    main()
