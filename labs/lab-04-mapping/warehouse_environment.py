"""
Warehouse Environment for Autonomous Robot Project

This module provides a simulated warehouse environment for the course project.
Students will extend this base environment throughout the semester.

Course: Introduction to Robotics for AI and Data Science
Institution: Al-Zaytoonah University of Jordan
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import os

class WarehouseEnvironment:
    """
    Simulated warehouse environment with shelves, boxes, and obstacles.
    
    This class manages the PyBullet simulation, including:
    - Loading the warehouse layout
    - Spawning objects (boxes, obstacles)
    - Managing the robot
    - Providing sensor interfaces
    """
    
    def __init__(self, gui=True, warehouse_size=(10, 10)):
        """
        Initialize the warehouse environment.
        
        Args:
            gui (bool): If True, show 3D visualization window
            warehouse_size (tuple): (width, length) of warehouse in meters
        """
        self.gui = gui
        self.warehouse_size = warehouse_size
        self.robot_id = None
        self.objects = {}  # Dictionary to store object IDs
        self.target_items = []  # List of items to retrieve
        
        # Connect to physics server
        if gui:
            self.client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # Disable GUI controls
        else:
            self.client = p.connect(p.DIRECT)
        
        # Set up simulation
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1./240.)
        
        # Load environment
        self._load_environment()
        
        print(f"Warehouse environment initialized: {warehouse_size[0]}m x {warehouse_size[1]}m")
    
    def _load_environment(self):
        """Load the warehouse layout (floor, walls, shelves)."""
        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf")
        
        # Create warehouse boundaries (walls)
        self._create_walls()
        
        # Create shelving units
        self._create_shelves()
        
        print("Warehouse layout loaded")
    
    def _create_walls(self):
        """Create walls around the warehouse perimeter."""
        width, length = self.warehouse_size
        wall_height = 2.0
        wall_thickness = 0.1
        
        # Wall positions: [x, y, z]
        # North wall
        self._create_wall([0, length/2, wall_height/2], 
                         [width/2, wall_thickness/2, wall_height/2])
        # South wall
        self._create_wall([0, -length/2, wall_height/2], 
                         [width/2, wall_thickness/2, wall_height/2])
        # East wall
        self._create_wall([width/2, 0, wall_height/2], 
                         [wall_thickness/2, length/2, wall_height/2])
        # West wall
        self._create_wall([-width/2, 0, wall_height/2], 
                         [wall_thickness/2, length/2, wall_height/2])
    
    def _create_wall(self, position, half_extents):
        """Create a single wall using a box collision shape."""
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents,
                                          rgbaColor=[0.7, 0.7, 0.7, 1.0])
        wall_id = p.createMultiBody(baseMass=0,  # Static object
                                    baseCollisionShapeIndex=collision_shape,
                                    baseVisualShapeIndex=visual_shape,
                                    basePosition=position)
        return wall_id
    
    def _create_shelves(self):
        """Create shelving units in the warehouse."""
        # Simple shelves: vertical posts with horizontal platforms
        shelf_positions = [
            [2, 2, 0],
            [2, -2, 0],
            [-2, 2, 0],
            [-2, -2, 0]
        ]
        
        for pos in shelf_positions:
            self._create_shelf(pos)
    
    def _create_shelf(self, position):
        """Create a single shelf unit."""
        # Vertical post
        post_height = 1.5
        post_radius = 0.10
        collision_shape = p.createCollisionShape(p.GEOM_CYLINDER, 
                                                 radius=post_radius, 
                                                 height=post_height)
        visual_shape = p.createVisualShape(p.GEOM_CYLINDER,
                                          radius=post_radius,
                                          length=post_height,
                                          rgbaColor=[0.6, 0.4, 0.2, 1.0])
        
        post_pos = [position[0], position[1], post_height/2]
        shelf_id = p.createMultiBody(baseMass=0,
                                    baseCollisionShapeIndex=collision_shape,
                                    baseVisualShapeIndex=visual_shape,
                                    basePosition=post_pos)
        return shelf_id
    
    def spawn_robot(self, robot_type="husky", position=None):
        """
        Spawn a robot in the warehouse.
        
        Args:
            robot_type (str): Type of robot ("husky", "r2d2", etc.)
            position (list): [x, y, z] starting position, or None for center
        
        Returns:
            int: Robot ID
        """
        if position is None:
            position = [0, 0, 0.1]
        
        orientation = p.getQuaternionFromEuler([0, 0, 0])
        
        if robot_type == "husky":
            self.robot_id = p.loadURDF("husky/husky.urdf", position, orientation)
        elif robot_type == "r2d2":
            self.robot_id = p.loadURDF("r2d2.urdf", position, orientation)
        else:
            raise ValueError(f"Unknown robot type: {robot_type}")
        
        print(f"Robot spawned: {robot_type} at position {position}")
        return self.robot_id
    
    def spawn_box(self, position, color="red", size=0.5, name=None):
        """
        Spawn a colored box (item to retrieve).
        
        Args:
            position (list): [x, y, z] position
            color (str): Color name ("red", "blue", "green", "yellow")
            size (float): Box size in meters
            name (str): Optional name for the box
        
        Returns:
            int: Box ID
        """
        # Color mapping
        colors = {
            "red": [1, 0, 0, 1],
            "blue": [0, 0, 1, 1],
            "green": [0, 1, 0, 1],
            "yellow": [1, 1, 0, 1],
            "orange": [1, 0.5, 0, 1],
            "purple": [0.5, 0, 0.5, 1]
        }
        
        rgba = colors.get(color, [0.5, 0.5, 0.5, 1])
        
        # Create box
        half_extents = [size/2, size/2, size/2]
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents,
                                          rgbaColor=rgba)
        
        box_id = p.createMultiBody(baseMass=0.1,  # Light weight
                                   baseCollisionShapeIndex=collision_shape,
                                   baseVisualShapeIndex=visual_shape,
                                   basePosition=position)
        
        # Store box info
        if name is None:
            name = f"{color}_box_{len(self.objects)}"
        
        self.objects[name] = {
            'id': box_id,
            'type': 'box',
            'color': color,
            'position': position,
            'size': size
        }
        
        print(f"Box spawned: {name} ({color}) at position {position}")
        return box_id
    
    def spawn_random_boxes(self, num_boxes=3):
        """
        Spawn random colored boxes in the warehouse.
        
        Args:
            num_boxes (int): Number of boxes to spawn
        """
        colors = ["red", "blue", "green", "yellow", "orange"]
        width, length = self.warehouse_size
        
        for i in range(num_boxes):
            # Random position (avoid center where robot spawns)
            x = np.random.uniform(-width/3, width/3)
            y = np.random.uniform(-length/3, length/3)
            z = 0.1  # Just above ground
            
            # Ensure not too close to center
            if abs(x) < 1.0 and abs(y) < 1.0:
                x += 1.5 * np.sign(x) if x != 0 else 1.5
            
            color = colors[i % len(colors)]
            self.spawn_box([x, y, z], color=color, name=f"box_{i}")
    
    def get_robot_state(self):
        """
        Get current robot state (position, orientation, velocity).
        
        Returns:
            dict: Robot state information
        """
        if self.robot_id is None:
            return None
        
        position, orientation = p.getBasePositionAndOrientation(self.robot_id)
        linear_vel, angular_vel = p.getBaseVelocity(self.robot_id)
        euler = p.getEulerFromQuaternion(orientation)
        
        return {
            'position': position,
            'orientation': orientation,
            'euler': euler,  # (roll, pitch, yaw)
            'linear_velocity': linear_vel,
            'angular_velocity': angular_vel
        }
    
    def move_robot(self, linear_velocity, angular_velocity):
        """
        Command robot to move with specified velocities.
        
        Args:
            linear_velocity (float): Forward speed in m/s
            angular_velocity (float): Turning rate in rad/s
        """
        if self.robot_id is None:
            print("No robot spawned!")
            return
        
        # For Husky robot: control wheel joints directly
        # Husky has 4 wheels: front_left, front_right, rear_left, rear_right
        # Wheel radius: ~0.165m, wheelbase: ~0.555m
        wheel_radius = 0.165
        wheelbase = 0.555
        
        # Calculate wheel velocities for differential drive
        # v_left = v - (w * L / 2)
        # v_right = v + (w * L / 2)
        v_left = linear_velocity - (angular_velocity * wheelbase / 2.0)
        v_right = linear_velocity + (angular_velocity * wheelbase / 2.0)
        
        # Convert linear wheel velocity to angular velocity (rad/s)
        wheel_vel_left = v_left / wheel_radius
        wheel_vel_right = v_right / wheel_radius
        
        # Get number of joints
        num_joints = p.getNumJoints(self.robot_id)
        
        # Find wheel joints and set their velocities
        for joint_index in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, joint_index)
            joint_name = joint_info[1].decode('utf-8')
            
            # Set velocity for each wheel
            if 'left' in joint_name.lower() and 'wheel' in joint_name.lower():
                p.setJointMotorControl2(self.robot_id, joint_index,
                                       p.VELOCITY_CONTROL,
                                       targetVelocity=wheel_vel_left,
                                       force=20)
            elif 'right' in joint_name.lower() and 'wheel' in joint_name.lower():
                p.setJointMotorControl2(self.robot_id, joint_index,
                                       p.VELOCITY_CONTROL,
                                       targetVelocity=wheel_vel_right,
                                       force=20)
    
    def get_object_positions(self):
        """
        Get positions of all objects in the warehouse.
        
        Returns:
            dict: Object names mapped to positions
        """
        positions = {}
        for name, obj_info in self.objects.items():
            pos, _ = p.getBasePositionAndOrientation(obj_info['id'])
            positions[name] = pos
        return positions
    
    def step(self, sleep_time=None):
        """
        Step the simulation forward by one time step.
        
        Args:
            sleep_time (float): Optional sleep time for real-time visualization
        """
        p.stepSimulation()
        if sleep_time is not None:
            time.sleep(sleep_time)
    
    def run(self, duration=5.0, real_time=True):
        """
        Run simulation for a specified duration.
        
        Args:
            duration (float): Simulation duration in seconds
            real_time (bool): If True, run at real-time speed
        """
        steps = int(duration * 240)  # 240 Hz
        sleep_time = 1./240. if real_time else None
        
        for _ in range(steps):
            self.step(sleep_time)
    
    def close(self):
        """Disconnect from physics server and cleanup."""
        p.disconnect()
        print("Warehouse environment closed")
    
    def reset(self):
        """Reset the simulation to initial state."""
        p.resetSimulation()
        self.objects = {}
        self.target_items = []
        self._load_environment()
        print("Environment reset")


# Example usage
if __name__ == "__main__":
    # Create warehouse environment
    env = WarehouseEnvironment(gui=True, warehouse_size=(10, 10))
    
    # Spawn robot
    env.spawn_robot(robot_type="husky", position=[0, 0, 0.1])
    
    # Spawn some colored boxes
    env.spawn_random_boxes(num_boxes=5)
    
    # Run simulation for 10 seconds
    print("\nRunning simulation for 10 seconds...")
    env.run(duration=10.0, real_time=True)
    
    # Get final positions
    print("\nFinal object positions:")
    positions = env.get_object_positions()
    for name, pos in positions.items():
        print(f"  {name}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
    
    # Cleanup
    env.close()

