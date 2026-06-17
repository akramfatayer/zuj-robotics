"""
Activity 3: Real-Time Warehouse Mapping with PyBullet

This activity integrates everything you've learned:
- LiDAR sensor simulation (Activity 1)
- Occupancy grid mapping (Activity 2)
- PyBullet warehouse environment (Week 2)

Your robot will explore the warehouse and build an occupancy grid map in real-time.

Course: Introduction to Robotics for AI and Data Science
Week 4: Distance Sensing and Occupancy Grid Mapping
"""

import pybullet as p
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import time
from warehouse_environment import WarehouseEnvironment

class LiDARSensor:
    """
    Simulated 2D LiDAR sensor using PyBullet ray casting.
    
    This sensor shoots rays in a 360-degree pattern and measures
    distances to obstacles.
    """
    
    def __init__(self, num_rays=360, max_range=10.0, min_range=0.01):
        """
        Initialize LiDAR sensor.
        
        Args:
            num_rays (int): Number of laser rays (angular resolution)
            max_range (float): Maximum detection range in meters
            min_range (float): Minimum detection range in meters
        """
        self.num_rays = num_rays
        self.max_range = max_range
        self.min_range = min_range
        self.angles = np.linspace(0, 2*np.pi, num_rays, endpoint=False)
        
    def scan(self, robot_id, sensor_height=0.5):
        """
        Perform a LiDAR scan from the robot's current position.
        
        Args:
            robot_id (int): PyBullet robot ID
            sensor_height (float): Height of sensor above robot base
            
        Returns:
            tuple: (ranges, angles) where ranges is array of distances
        """
        # Get robot position and orientation
        pos, orn = p.getBasePositionAndOrientation(robot_id)
        euler = p.getEulerFromQuaternion(orn)
        yaw = euler[2]  # Robot's heading angle
        
        # Sensor position (on top of robot)
        sensor_pos = [pos[0], pos[1], pos[2] + sensor_height]
        
        # Prepare ray casting
        ray_from = []
        ray_to = []
        
        for angle in self.angles:
            # Global angle = robot yaw + sensor angle
            global_angle = yaw + angle
            
            # Calculate ray endpoints
            from_point = sensor_pos
            to_point = [
                sensor_pos[0] + self.max_range * np.cos(global_angle),
                sensor_pos[1] + self.max_range * np.sin(global_angle),
                sensor_pos[2]
            ]
            
            ray_from.append(from_point)
            ray_to.append(to_point)
        
        # Perform batch ray casting (efficient!)
        results = p.rayTestBatch(ray_from, ray_to)
        
        # Extract ranges
        ranges = []
        for i, result in enumerate(results):
            object_id, link_index, hit_fraction, hit_position, hit_normal = result
            
            if object_id == -1:  # No hit
                ranges.append(self.max_range)
            else:
                # Calculate actual distance
                distance = hit_fraction * self.max_range
                ranges.append(max(distance, self.min_range))
        
        return np.array(ranges), self.angles


class OccupancyGridMapper:
    """
    Real-time occupancy grid mapper using log-odds representation.
    
    This class builds a 2D occupancy grid map from LiDAR scans.
    """
    
    def __init__(self, width=20.0, height=20.0, resolution=0.01):
        """
        Initialize occupancy grid.
        
        Args:
            width (float): Map width in meters
            height (float): Map height in meters
            resolution (float): Grid cell size in meters
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        
        # Calculate grid dimensions
        self.grid_width = int(width / resolution)
        self.grid_height = int(height / resolution)
        
        # Initialize log-odds grid (0 = unknown)
        self.log_odds = np.zeros((self.grid_height, self.grid_width))
        
        # Log-odds update values
        self.l_occ = 0.85   # Log-odds for occupied
        self.l_free = -0.4  # Log-odds for free
        
        # Clamping limits
        self.l_min = -5.0
        self.l_max = 5.0
        
        # Map origin (center of grid)
        self.origin_x = width / 2.0
        self.origin_y = height / 2.0
        
        print(f"Occupancy grid initialized: {self.grid_width}x{self.grid_height} cells")
        print(f"Resolution: {resolution}m, Coverage: {width}m x {height}m")
    
    def world_to_grid(self, x, y):
        """
        Convert world coordinates to grid indices.
        
        Args:
            x, y (float): World coordinates in meters
            
        Returns:
            tuple: (row, col) grid indices, or None if out of bounds
        """
        # Transform to grid coordinates
        grid_x = int((x + self.origin_x) / self.resolution)
        grid_y = int((y + self.origin_y) / self.resolution)
        
        # Check bounds
        if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
            return (grid_y, grid_x)  # Note: row=y, col=x
        return None
    
    def bresenham_line(self, x0, y0, x1, y1):
        """
        Bresenham's line algorithm to get all cells along a line.
        
        Args:
            x0, y0: Start grid coordinates
            x1, y1: End grid coordinates
            
        Returns:
            list: List of (row, col) tuples along the line
        """
        cells = []
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            cells.append((y, x))  # (row, col)
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            
            if e2 > -dy:
                err -= dy
                x += sx
            
            if e2 < dx:
                err += dx
                y += sy
        
        return cells
    
    def update_from_scan(self, robot_x, robot_y, ranges, angles):
        """
        Update occupancy grid from a LiDAR scan.
        
        Args:
            robot_x, robot_y (float): Robot position in world coordinates
            ranges (array): Array of range measurements
            angles (array): Array of corresponding angles (global frame)
        """
        # Get robot grid position
        robot_grid = self.world_to_grid(robot_x, robot_y)
        if robot_grid is None:
            return  # Robot outside map
        
        robot_row, robot_col = robot_grid
        
        # Process each ray
        for r, angle in zip(ranges, angles):
            # Skip invalid measurements
            if r >= 9.9:  # Max range (no obstacle detected)
                continue
            
            # Calculate endpoint in world coordinates
            end_x = robot_x + r * np.cos(angle)
            end_y = robot_y + r * np.sin(angle)
            
            # Convert to grid coordinates
            end_grid = self.world_to_grid(end_x, end_y)
            if end_grid is None:
                continue  # Endpoint outside map
            
            end_row, end_col = end_grid
            
            # Get all cells along the ray using Bresenham
            ray_cells = self.bresenham_line(robot_col, robot_row, end_col, end_row)
            
            # Update cells along the ray
            for i, (row, col) in enumerate(ray_cells):
                # Check bounds
                if not (0 <= row < self.grid_height and 0 <= col < self.grid_width):
                    continue
                
                if i == len(ray_cells) - 1:
                    # Last cell = occupied (obstacle detected)
                    self.log_odds[row, col] += self.l_occ
                else:
                    # Intermediate cells = free space
                    self.log_odds[row, col] += self.l_free
                
                # Clamp to limits
                self.log_odds[row, col] = np.clip(self.log_odds[row, col], 
                                                   self.l_min, self.l_max)
    
    def get_probability_grid(self):
        """
        Convert log-odds to probability (for visualization).
        
        Returns:
            array: Probability grid (0 to 1)
        """
        # Convert log-odds to probability: p = 1 - 1/(1 + exp(l))
        prob = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
        return prob
    
    def get_occupancy_grid(self, threshold=0.5):
        """
        Get binary occupancy grid (occupied/free/unknown).
        
        Args:
            threshold (float): Probability threshold for occupied
            
        Returns:
            array: Grid with values: -1 (unknown), 0 (free), 1 (occupied)
        """
        prob = self.get_probability_grid()
        
        grid = np.full_like(prob, -1)  # Unknown
        grid[prob < 0.4] = 0   # Free
        grid[prob > 0.6] = 1   # Occupied
        
        return grid


class WarehouseMapper:
    """
    Main class for warehouse mapping demo.
    
    Integrates PyBullet simulation, LiDAR sensor, and occupancy grid mapping.
    """
    
    def __init__(self):
        """Initialize warehouse mapper."""
        # Create warehouse environment
        self.env = WarehouseEnvironment(gui=True, warehouse_size=(10, 10))
        
        # Spawn robot
        self.robot_id = self.env.spawn_robot(robot_type="husky", position=[-4, -4, 0.1])
        
        # Spawn obstacles (boxes)
        self.env.spawn_random_boxes(num_boxes=6)
        
        # Create LiDAR sensor
        self.lidar = LiDARSensor(num_rays=360, max_range=10.0)
        
        # Create occupancy grid mapper
        self.mapper = OccupancyGridMapper(width=12.0, height=12.0, resolution=0.1)
        
        # Setup visualization
        self.setup_visualization()
        
        print("\nWarehouse Mapper initialized!")
        print("Controls: The robot will explore automatically")
        print("Watch the map build in real-time!\n")
    
    def setup_visualization(self):
        """Setup matplotlib figure for real-time map visualization."""
        plt.ion()  # Interactive mode
        self.fig, self.axes = plt.subplots(1, 2, figsize=(14, 6))
        
        # Left plot: Occupancy grid
        self.ax_map = self.axes[0]
        self.ax_map.set_title('Occupancy Grid Map', fontsize=14, fontweight='bold')
        self.ax_map.set_xlabel('X Position (m)')
        self.ax_map.set_ylabel('Y Position (m)')
        self.ax_map.set_aspect('equal')
        
        # Right plot: Statistics
        self.ax_stats = self.axes[1]
        self.ax_stats.set_title('Mapping Statistics', fontsize=14, fontweight='bold')
        self.ax_stats.axis('off')
        
        plt.tight_layout()
    
    def update_visualization(self, scan_count):
        """Update the visualization with current map."""
        # Get probability grid
        prob_grid = self.mapper.get_probability_grid()
        
        # Clear and redraw map
        self.ax_map.clear()
        self.ax_map.set_title('Occupancy Grid Map', fontsize=14, fontweight='bold')
        self.ax_map.set_xlabel('X Position (m)')
        self.ax_map.set_ylabel('Y Position (m)')
        
        # Display map (flip vertically for correct orientation)
        extent = [-self.mapper.origin_x, self.mapper.origin_x,
                 -self.mapper.origin_y, self.mapper.origin_y]
        
        im = self.ax_map.imshow(prob_grid, cmap='gray_r', origin='lower',
                                extent=extent, vmin=0, vmax=1)
        
        # Add colorbar
        if not hasattr(self, 'colorbar'):
            self.colorbar = plt.colorbar(im, ax=self.ax_map, fraction=0.046, pad=0.04)
            self.colorbar.set_label('Occupancy Probability', rotation=270, labelpad=15)
        
        # Get robot position
        state = self.env.get_robot_state()
        if state:
            rx, ry = state['position'][0], state['position'][1]
            self.ax_map.plot(rx, ry, 'ro', markersize=10, label='Robot')
            
            # Draw robot orientation
            yaw = state['euler'][2]
            arrow_len = 0.5
            self.ax_map.arrow(rx, ry, 
                            arrow_len * np.cos(yaw), 
                            arrow_len * np.sin(yaw),
                            head_width=0.2, head_length=0.15, fc='r', ec='r')
        
        self.ax_map.legend()
        self.ax_map.grid(True, alpha=0.3)
        
        # Update statistics
        self.ax_stats.clear()
        self.ax_stats.axis('off')
        
        # Calculate statistics
        occupancy = self.mapper.get_occupancy_grid()
        total_cells = occupancy.size
        unknown = np.sum(occupancy == -1)
        free = np.sum(occupancy == 0)
        occupied = np.sum(occupancy == 1)
        
        unknown_pct = 100 * unknown / total_cells
        free_pct = 100 * free / total_cells
        occupied_pct = 100 * occupied / total_cells
        
        stats_text = f"""
        Mapping Progress
        ═══════════════════════════
        
        Scans Completed: {scan_count}
        
        Map Coverage:
        ─────────────────────────
        Unknown:   {unknown_pct:5.1f}%  ({unknown:6d} cells)
        Free:      {free_pct:5.1f}%  ({free:6d} cells)
        Occupied:  {occupied_pct:5.1f}%  ({occupied:6d} cells)
        
        Total Cells: {total_cells:,}
        Resolution: {self.mapper.resolution} m/cell
        
        Map Size: {self.mapper.width}m × {self.mapper.height}m
        """
        
        self.ax_stats.text(0.1, 0.5, stats_text, fontsize=11, 
                          verticalalignment='center', family='monospace')
        
        plt.pause(0.01)
    
    def explore_pattern(self, duration=60.0):
        """
        Make robot explore the warehouse in a pattern.
        
        Args:
            duration (float): Exploration duration in seconds
        """
        print("Starting exploration...")
        
        start_time = time.time()
        scan_count = 0
        last_scan_time = 0
        scan_interval = 0.5  # Scan every 0.5 seconds
        
        # Exploration pattern: move forward, turn, repeat
        phase = 0
        phase_start = time.time()
        phase_duration = 3.0  # Each phase lasts 3 seconds
        
        while time.time() - start_time < duration:
            current_time = time.time()
            
            # Control robot based on phase
            if phase == 0:  # Move forward
                self.env.move_robot(linear_velocity=0.5, angular_velocity=0.0)
                if current_time - phase_start > phase_duration:
                    phase = 1
                    phase_start = current_time
            elif phase == 1:  # Turn
                self.env.move_robot(linear_velocity=0.0, angular_velocity=0.8)
                if current_time - phase_start > 2.0:  # Turn for 2 seconds
                    phase = 0
                    phase_start = current_time
            
            # Step simulation
            self.env.step(sleep_time=1./240.)
            
            # Perform LiDAR scan periodically
            if current_time - last_scan_time >= scan_interval:
                # Get robot state
                state = self.env.get_robot_state()
                robot_x = state['position'][0]
                robot_y = state['position'][1]
                robot_yaw = state['euler'][2]
                
                # Perform scan
                ranges, sensor_angles = self.lidar.scan(self.robot_id)
                
                # Convert sensor angles to global angles
                global_angles = sensor_angles + robot_yaw
                
                # Update map
                self.mapper.update_from_scan(robot_x, robot_y, ranges, global_angles)
                
                scan_count += 1
                last_scan_time = current_time
                
                # Update visualization
                if scan_count % 2 == 0:  # Update every 2 scans
                    self.update_visualization(scan_count)
                
                print(f"Scan {scan_count}: Robot at ({robot_x:.2f}, {robot_y:.2f})")
        
        # Stop robot
        self.env.move_robot(0, 0)
        
        print(f"\nExploration complete! Total scans: {scan_count}")
    
    def save_map(self, filename='warehouse_map.png'):
        """Save the final map to file."""
        prob_grid = self.mapper.get_probability_grid()
        
        plt.figure(figsize=(10, 10))
        plt.imshow(prob_grid, cmap='gray_r', origin='lower')
        plt.title('Final Warehouse Occupancy Grid Map', fontsize=16, fontweight='bold')
        plt.xlabel('Grid X')
        plt.ylabel('Grid Y')
        plt.colorbar(label='Occupancy Probability')
        plt.tight_layout()
        plt.savefig(filename, dpi=150)
        print(f"Map saved to {filename}")
    
    def run(self):
        """Run the complete mapping demo."""
        try:
            # Explore warehouse
            self.explore_pattern(duration=30.0)
            
            # Final visualization update
            self.update_visualization(scan_count=999)
            
            # Save map
            self.save_map('activity3_warehouse_map.png')
            
            print("\n" + "="*50)
            print("Mapping demo complete!")
            print("="*50)
            print("\nPress Enter to close...")
            input()
            
        finally:
            plt.ioff()
            plt.close('all')
            self.env.close()


# Main execution
if __name__ == "__main__":
    print("="*60)
    print("Activity 3: Real-Time Warehouse Mapping")
    print("="*60)
    print("\nThis demo shows:")
    print("  1. LiDAR sensor simulation in PyBullet")
    print("  2. Real-time occupancy grid mapping")
    print("  3. Robot exploration in warehouse environment")
    print("\nThe robot will explore for 30 seconds.")
    print("Watch the map build in real-time!")
    print("="*60)
    print()
    
    # Create and run mapper
    mapper = WarehouseMapper()
    mapper.run()

