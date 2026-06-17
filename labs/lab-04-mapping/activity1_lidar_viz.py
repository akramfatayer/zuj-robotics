"""
Week 4 Lab - Activity 1: Visualize LiDAR Data
==============================================

Learning Objectives:
- Understand LiDAR data format (angle, distance)
- Convert polar to Cartesian coordinates
- Visualize point clouds

Author: Manus AI
Course: AI & Robotics
"""

import numpy as np
import matplotlib.pyplot as plt

# ============================================================================
# SIMULATED LIDAR DATA
# ============================================================================

def generate_lidar_scan(robot_x=0.0, robot_y=0.0, robot_theta=0.0):
    """
    Generate a simulated LiDAR scan in a simple environment.
    
    Environment: Rectangular room with obstacles
    - Room: 20m x 20m
    - Obstacles: Two boxes
    
    Parameters:
    -----------
    robot_x, robot_y : float
        Robot position in world frame (meters)
    robot_theta : float
        Robot orientation in world frame (radians)
    
    Returns:
    --------
    angles : np.array
        Scan angles in radians (robot frame)
    ranges : np.array
        Measured distances in meters
    """
    # LiDAR parameters
    num_beams = 360  # 360 beams (1 degree resolution)
    max_range = 15.0  # Maximum range: 15 meters
    noise_std = 0.05  # Measurement noise: 5cm standard deviation
    
    # Scan angles (robot frame): -180° to +180°
    angles = np.linspace(-np.pi, np.pi, num_beams)
    
    # Initialize ranges to max_range
    ranges = np.ones(num_beams) * max_range
    
    # Define environment obstacles (in world frame)
    # Obstacle 1: Box at (5, 3) with size 2m x 2m
    # Obstacle 2: Box at (-4, -5) with size 3m x 1.5m
    obstacles = [
        {'x': 5.0, 'y': 3.0, 'width': 2.0, 'height': 2.0},
        {'x': -4.0, 'y': -5.0, 'width': 3.0, 'height': 1.5},
    ]
    
    # Room boundaries
    room_size = 10.0  # ±10m
    
    # For each beam, ray cast to find nearest obstacle
    for i, angle in enumerate(angles):
        # Beam direction in world frame
        world_angle = robot_theta + angle
        dx = np.cos(world_angle)
        dy = np.sin(world_angle)
        
        min_dist = max_range
        
        # Check room boundaries
        # Right wall (x = room_size)
        if dx > 0:
            t = (room_size - robot_x) / dx
            if t > 0:
                y_hit = robot_y + t * dy
                if abs(y_hit) <= room_size:
                    min_dist = min(min_dist, t)
        
        # Left wall (x = -room_size)
        if dx < 0:
            t = (-room_size - robot_x) / dx
            if t > 0:
                y_hit = robot_y + t * dy
                if abs(y_hit) <= room_size:
                    min_dist = min(min_dist, t)
        
        # Top wall (y = room_size)
        if dy > 0:
            t = (room_size - robot_y) / dy
            if t > 0:
                x_hit = robot_x + t * dx
                if abs(x_hit) <= room_size:
                    min_dist = min(min_dist, t)
        
        # Bottom wall (y = -room_size)
        if dy < 0:
            t = (-room_size - robot_y) / dy
            if t > 0:
                x_hit = robot_x + t * dx
                if abs(x_hit) <= room_size:
                    min_dist = min(min_dist, t)
        
        # Check obstacles (simplified: treat as rectangles)
        for obs in obstacles:
            # Check four edges of the obstacle
            x_min = obs['x'] - obs['width'] / 2
            x_max = obs['x'] + obs['width'] / 2
            y_min = obs['y'] - obs['height'] / 2
            y_max = obs['y'] + obs['height'] / 2
            
            # Right edge
            if dx > 0:
                t = (x_max - robot_x) / dx
                if t > 0:
                    y_hit = robot_y + t * dy
                    if y_min <= y_hit <= y_max:
                        min_dist = min(min_dist, t)
            
            # Left edge
            if dx < 0:
                t = (x_min - robot_x) / dx
                if t > 0:
                    y_hit = robot_y + t * dy
                    if y_min <= y_hit <= y_max:
                        min_dist = min(min_dist, t)
            
            # Top edge
            if dy > 0:
                t = (y_max - robot_y) / dy
                if t > 0:
                    x_hit = robot_x + t * dx
                    if x_min <= x_hit <= x_max:
                        min_dist = min(min_dist, t)
            
            # Bottom edge
            if dy < 0:
                t = (y_min - robot_y) / dy
                if t > 0:
                    x_hit = robot_x + t * dx
                    if x_min <= x_hit <= x_max:
                        min_dist = min(min_dist, t)
        
        ranges[i] = min_dist
    
    # Add measurement noise
    ranges += np.random.normal(0, noise_std, num_beams)
    ranges = np.clip(ranges, 0.1, max_range)  # Clip to valid range
    
    return angles, ranges


def polar_to_cartesian(angles, ranges):
    """
    Convert polar coordinates (angle, range) to Cartesian (x, y).
    
    Parameters:
    -----------
    angles : np.array
        Angles in radians
    ranges : np.array
        Distances in meters
    
    Returns:
    --------
    x, y : np.array
        Cartesian coordinates in meters
    """
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    return x, y


# ============================================================================
# VISUALIZATION
# ============================================================================

def visualize_lidar_scan(angles, ranges, robot_x=0.0, robot_y=0.0):
    """
    Visualize LiDAR scan as a point cloud.
    
    Parameters:
    -----------
    angles : np.array
        Scan angles in radians
    ranges : np.array
        Measured distances in meters
    robot_x, robot_y : float
        Robot position for plotting
    """
    # Convert to Cartesian
    x, y = polar_to_cartesian(angles, ranges)
    
    # Offset by robot position
    x += robot_x
    y += robot_y
    
    # Create figure
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Plot 1: Polar plot (range vs angle)
    ax1.plot(np.degrees(angles), ranges, 'b-', linewidth=0.5)
    ax1.scatter(np.degrees(angles), ranges, c='blue', s=1, alpha=0.5)
    ax1.set_xlabel('Angle (degrees)', fontsize=12)
    ax1.set_ylabel('Range (meters)', fontsize=12)
    ax1.set_title('LiDAR Scan: Range vs Angle', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(-180, 180)
    
    # Plot 2: Cartesian point cloud
    ax2.scatter(x, y, c='red', s=2, alpha=0.6, label='LiDAR points')
    ax2.plot(robot_x, robot_y, 'go', markersize=10, label='Robot')
    ax2.set_xlabel('X Position (m)', fontsize=12)
    ax2.set_ylabel('Y Position (m)', fontsize=12)
    ax2.set_title('Point Cloud Visualization', fontsize=14, fontweight='bold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    ax2.set_xlim(-12, 12)
    ax2.set_ylim(-12, 12)
    
    plt.tight_layout()
    plt.savefig('activity1_lidar_viz.png', dpi=150, bbox_inches='tight')
    print("✓ Saved: activity1_lidar_viz.png")
    plt.show()


# ============================================================================
# MAIN EXECUTION
# ============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("Activity 1: Visualize LiDAR Data")
    print("=" * 60)
    
    # Robot pose
    robot_x = 0.0
    robot_y = 0.0
    robot_theta = 0.0  # Facing right (+x direction)
    
    print(f"\nRobot pose: ({robot_x:.1f}, {robot_y:.1f}), θ = {np.degrees(robot_theta):.1f}°")
    
    # Generate LiDAR scan
    print("\nGenerating simulated LiDAR scan...")
    angles, ranges = generate_lidar_scan(robot_x, robot_y, robot_theta)
    
    print(f"  Number of beams: {len(angles)}")
    print(f"  Angular resolution: {np.degrees(angles[1] - angles[0]):.2f}°")
    print(f"  Range: {ranges.min():.2f}m to {ranges.max():.2f}m")
    
    # Visualize
    print("\nVisualizing LiDAR scan...")
    visualize_lidar_scan(angles, ranges, robot_x, robot_y)
    
    print("\n" + "=" * 60)
    print("Activity 1 Complete!")
    print("=" * 60)
    print("\nKey Observations:")
    print("  • LiDAR provides 360° coverage around the robot")
    print("  • Point cloud shows room boundaries and obstacles")
    print("  • Polar representation shows range vs angle")
    print("  • Cartesian representation shows spatial layout")

