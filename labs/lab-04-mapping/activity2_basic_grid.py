"""
Week 4 Lab - Activity 2: Basic Occupancy Grid
==============================================

Learning Objectives:
- Implement occupancy grid data structure
- Convert world coordinates to grid indices
- Mark occupied cells from LiDAR endpoints
- Visualize grid as grayscale image

Author: Manus AI
Course: AI & Robotics
"""

import numpy as np
import matplotlib.pyplot as plt
from activity1_lidar_viz import generate_lidar_scan, polar_to_cartesian

# ============================================================================
# OCCUPANCY GRID CLASS
# ============================================================================

class OccupancyGrid:
    """
    2D Occupancy Grid for robot mapping.
    
    Attributes:
    -----------
    width, height : int
        Grid dimensions in cells
    resolution : float
        Cell size in meters
    origin_x, origin_y : float
        World coordinates of grid[0][0]
    grid : np.array
        2D array storing occupancy probabilities [0, 1]
    """
    
    def __init__(self, width, height, resolution, origin_x, origin_y):
        """
        Initialize empty occupancy grid.
        
        Parameters:
        -----------
        width, height : int
            Grid dimensions in cells
        resolution : float
            Cell size in meters (e.g., 0.1 = 10cm)
        origin_x, origin_y : float
            World coordinates of grid[0][0] (bottom-left corner)
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        
        # Initialize grid: 0.5 = unknown
        self.grid = np.ones((height, width)) * 0.5
        
        print(f"Initialized grid: {width}x{height} cells")
        print(f"  Resolution: {resolution}m per cell")
        print(f"  Coverage: {width*resolution}m x {height*resolution}m")
        print(f"  Origin: ({origin_x}, {origin_y})")
    
    def world_to_grid(self, x, y):
        """
        Convert world coordinates to grid indices.
        
        Parameters:
        -----------
        x, y : float or np.array
            World coordinates in meters
        
        Returns:
        --------
        gx, gy : int or np.array
            Grid indices (column, row)
        """
        gx = ((x - self.origin_x) / self.resolution).astype(int)
        gy = ((y - self.origin_y) / self.resolution).astype(int)
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        """
        Convert grid indices to world coordinates (cell center).
        
        Parameters:
        -----------
        gx, gy : int or np.array
            Grid indices (column, row)
        
        Returns:
        --------
        x, y : float or np.array
            World coordinates in meters
        """
        x = self.origin_x + (gx + 0.5) * self.resolution
        y = self.origin_y + (gy + 0.5) * self.resolution
        return x, y
    
    def is_valid(self, gx, gy):
        """
        Check if grid indices are within bounds.
        
        Parameters:
        -----------
        gx, gy : int or np.array
            Grid indices
        
        Returns:
        --------
        valid : bool or np.array
            True if indices are within grid bounds
        """
        return (gx >= 0) & (gx < self.width) & (gy >= 0) & (gy < self.height)
    
    def mark_occupied(self, x, y):
        """
        Mark a cell as occupied (probability = 1.0).
        
        Parameters:
        -----------
        x, y : float or np.array
            World coordinates in meters
        """
        gx, gy = self.world_to_grid(x, y)
        
        # Handle arrays
        if isinstance(gx, np.ndarray):
            valid = self.is_valid(gx, gy)
            gx = gx[valid]
            gy = gy[valid]
        else:
            if not self.is_valid(gx, gy):
                return
        
        # Mark as occupied
        self.grid[gy, gx] = 1.0
    
    def visualize(self, robot_x=None, robot_y=None, title="Occupancy Grid"):
        """
        Visualize occupancy grid as grayscale image.
        
        Parameters:
        -----------
        robot_x, robot_y : float, optional
            Robot position to plot
        title : str
            Plot title
        """
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Display grid (flip vertically for correct orientation)
        ax.imshow(np.flipud(self.grid), cmap='gray', origin='lower', 
                  extent=[self.origin_x, self.origin_x + self.width * self.resolution,
                          self.origin_y, self.origin_y + self.height * self.resolution],
                  vmin=0, vmax=1)
        
        # Plot robot position
        if robot_x is not None and robot_y is not None:
            ax.plot(robot_x, robot_y, 'ro', markersize=10, label='Robot')
            ax.legend()
        
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # Add colorbar
        cbar = plt.colorbar(ax.images[0], ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label('Occupancy Probability', fontsize=12)
        
        plt.tight_layout()
        return fig, ax


# ============================================================================
# MAPPING FUNCTION
# ============================================================================

def build_basic_map(robot_poses, num_scans=10):
    """
    Build a basic occupancy grid by marking LiDAR endpoints as occupied.
    
    Parameters:
    -----------
    robot_poses : list of tuples
        List of (x, y, theta) robot poses
    num_scans : int
        Number of scans to process
    
    Returns:
    --------
    grid : OccupancyGrid
        Resulting occupancy grid
    """
    # Initialize grid
    grid = OccupancyGrid(
        width=200,      # 200 cells
        height=200,     # 200 cells
        resolution=0.1,  # 10cm per cell
        origin_x=-10.0,  # Grid covers -10m to +10m
        origin_y=-10.0
    )
    
    print(f"\nProcessing {num_scans} scans...")
    
    for i, (robot_x, robot_y, robot_theta) in enumerate(robot_poses[:num_scans]):
        # Generate LiDAR scan
        angles, ranges = generate_lidar_scan(robot_x, robot_y, robot_theta)
        
        # Convert to Cartesian (robot frame)
        x_robot, y_robot = polar_to_cartesian(angles, ranges)
        
        # Transform to world frame
        cos_theta = np.cos(robot_theta)
        sin_theta = np.sin(robot_theta)
        x_world = robot_x + x_robot * cos_theta - y_robot * sin_theta
        y_world = robot_y + x_robot * sin_theta + y_robot * cos_theta
        
        # Mark endpoints as occupied
        grid.mark_occupied(x_world, y_world)
        
        if (i + 1) % 5 == 0:
            print(f"  Processed {i + 1}/{num_scans} scans")
    
    print("✓ Mapping complete!")
    return grid


# ============================================================================
# MAIN EXECUTION
# ============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("Activity 2: Basic Occupancy Grid")
    print("=" * 60)
    
    # Define robot trajectory (stationary for now)
    robot_poses = [
        (0.0, 0.0, 0.0),           # Scan 1: Center, facing right
        (0.0, 0.0, np.pi/4),       # Scan 2: Center, rotated 45°
        (0.0, 0.0, np.pi/2),       # Scan 3: Center, facing up
        (0.0, 0.0, 3*np.pi/4),     # Scan 4: Center, rotated 135°
        (0.0, 0.0, np.pi),         # Scan 5: Center, facing left
        (2.0, 0.0, 0.0),           # Scan 6: Moved right
        (-2.0, 0.0, np.pi),        # Scan 7: Moved left
        (0.0, 2.0, -np.pi/2),      # Scan 8: Moved up
        (0.0, -2.0, np.pi/2),      # Scan 9: Moved down
        (3.0, 3.0, -3*np.pi/4),    # Scan 10: Corner
    ]
    
    # Build map
    grid = build_basic_map(robot_poses, num_scans=10)
    
    # Visualize
    print("\nVisualizing occupancy grid...")
    fig, ax = grid.visualize(robot_x=0.0, robot_y=0.0, 
                              title="Basic Occupancy Grid (Endpoints Only)")
    plt.savefig('activity2_basic_grid.png', dpi=150, bbox_inches='tight')
    print("✓ Saved: activity2_basic_grid.png")
    plt.show()
    
    # Statistics
    occupied_cells = np.sum(grid.grid == 1.0)
    unknown_cells = np.sum(grid.grid == 0.5)
    total_cells = grid.width * grid.height
    
    print("\n" + "=" * 60)
    print("Grid Statistics:")
    print("=" * 60)
    print(f"  Total cells: {total_cells}")
    print(f"  Occupied cells: {occupied_cells} ({100*occupied_cells/total_cells:.2f}%)")
    print(f"  Unknown cells: {unknown_cells} ({100*unknown_cells/total_cells:.2f}%)")
    print(f"  Free cells: 0 (not marked yet)")
    
    print("\n" + "=" * 60)
    print("Activity 2 Complete!")
    print("=" * 60)
    print("\nKey Observations:")
    print("  • Endpoints marked as occupied (black)")
    print("  • Most of grid remains unknown (gray)")
    print("  • No free space marked yet (need ray casting)")
    print("  • Next: Activity 3 will add inverse sensor model")

