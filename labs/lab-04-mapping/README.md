# Week 4 Lab: Distance Sensing and Occupancy Grid Mapping

**Course:** AI & Robotics  
**Topic:** LiDAR Sensing and Occupancy Grid Mapping  
**Project Integration:** Autonomous Warehouse Robot

---

## Quick Start

### Installation

```bash
# Install required packages فقط في حالة عدم تنزيلها سابقا ولمرة واحدة
conda install numpy matplotlib pybullet

# Verify installation
python -c "import numpy, matplotlib, pybullet; print('✓ All packages installed')"
```

### Running the Activities

**Activity 1: LiDAR Visualization** (Standalone, ~20 minutes)
```bash
python activity1_lidar_viz.py
```
Output: `activity1_lidar_viz.png`

**Activity 2: Basic Occupancy Grid** (Standalone, ~30 minutes)
```bash
python activity2_basic_grid.py
```
Output: `activity2_basic_grid.png`

**Activity 3: Warehouse Mapping** (PyBullet Integration, ~90 minutes)
```bash
python activity3_warehouse_mapping.py
```
Output: `activity3_warehouse_map.png` + PyBullet 3D visualization

---

## Lab Structure

### Activity 1: LiDAR Visualization
- **Goal:** Understand LiDAR data format
- **Type:** Standalone simulation
- **Duration:** 20 minutes
- **Key Concepts:** Polar coordinates, coordinate transformation, point clouds

### Activity 2: Basic Occupancy Grid
- **Goal:** Implement grid data structure
- **Type:** Standalone simulation
- **Duration:** 30 minutes
- **Key Concepts:** Discretization, world-to-grid conversion, endpoint marking

### Activity 3: Real-Time Warehouse Mapping
- **Goal:** Integrate LiDAR + mapping with warehouse robot
- **Type:** PyBullet simulation (integrated with course project)
- **Duration:** 90 minutes (30 min run + 60 min experiments)
- **Key Concepts:** Ray casting, log-odds, real-time mapping, exploration

---

## Project Integration

This lab builds on your autonomous warehouse robot project:

**Week 1:** Basic robot control in PyBullet  
**Week 2:** Computer vision and object detection  
**Week 3:** Localization with Kalman filtering  
**Week 4 (This Week):** Mapping with LiDAR → **Building toward SLAM**

Activity 3 uses:
- `warehouse_environment.py` from Week 2
- R2D2 robot model from Week 1
- Localization concepts from Week 3

---

## Troubleshooting

### Issue: "ModuleNotFoundError: No module named 'pybullet'"
**Solution:**
```bash
conda install pybullet
```

### Issue: PyBullet window doesn't appear
**Solution:** 
- Check if running in headless environment (SSH without X11)
- Modify `activity3_warehouse_mapping.py`:
  ```python
  # Line 423: Change gui=True to gui=False
  self.env = WarehouseEnvironment(gui=False, warehouse_size=(10, 10))
  ```

### Issue: "ImportError: cannot import name 'WarehouseEnvironment'"
**Solution:** Ensure `warehouse_environment.py` is in the same directory as `activity3_warehouse_mapping.py`

### Issue: Robot doesn't move in Activity 3
**Solution:** 
- Check console for error messages
- Verify robot spawned successfully (check console output)
- Try reducing exploration duration for faster testing

### Issue: Map is mostly unknown (gray)
**Solution:**
- Robot may not have explored enough
- Increase exploration duration in `activity3_warehouse_mapping.py`:
  ```python
  # Line 499: Change duration
  self.explore_pattern(duration=60.0)  # Try 60 seconds instead of 30
  ```

---

## Expected Results

### Activity 1:
- **Execution Time:** ~2 seconds
- **Output:** Dual plot showing polar and Cartesian LiDAR data
- **Key Observation:** Point cloud forms a room with obstacles

### Activity 2:
- **Execution Time:** ~5 seconds
- **Output:** Occupancy grid with only endpoints marked
- **Key Observation:** ~97% of grid remains unknown (gray)

### Activity 3:
- **Execution Time:** ~30-40 seconds
- **Output:** Complete warehouse map with walls, shelves, boxes
- **Key Observation:** 50-70% coverage, clear free space and obstacles
- **Visual:** PyBullet 3D window + Matplotlib real-time map

---

## Tips for Success

### Understanding the Code:
1. **Start with Activities 1-2** to understand fundamentals
2. **Read through Activity 3 code** before running (15 minutes)
3. **Trace the data flow:** Robot → LiDAR → Map → Visualization

### Experimentation:
1. **Try different exploration strategies** (spiral, wall-following)
2. **Vary LiDAR parameters** (rays, range) and observe effects
3. **Change grid resolution** and analyze trade-offs
4. **Document your experiments** with screenshots and brief notes

### Debugging:
1. **Check console output** for errors and progress
2. **Print intermediate values** to understand what's happening
3. **Visualize single scans** before full exploration
4. **Start with shorter durations** (10s) for faster testing

---

## Common Questions

**Q: Why does Activity 3 take so long?**  
A: PyBullet simulation runs in real-time (30 seconds of exploration = 30 seconds wall-clock time). This is realistic!

**Q: Can I speed up Activity 3?**  
A: Yes, reduce `duration` parameter or increase `sleep_time` in simulation step (less realistic but faster).

**Q: Why is my map different from others?**  
A: Random box placement! Each run spawns boxes in different locations. Walls and shelves should be consistent.

**Q: Do I need to modify the code?**  
A: Not required for basic completion. Modifications are for experimentation (Part 3) and optional extensions.

**Q: Can I use a different robot?**  
A: Yes! Change `robot_type="r2d2"` to `robot_type="husky"` in Activity 3. Husky is larger and may produce different maps.

---

## Resources

### LiDAR Technology:
- [How LiDAR Works (Video)](https://www.youtube.com/watch?v=EYbhNSUnIdU)
- [Velodyne LiDAR Sensors](https://velodynelidar.com/)

### Occupancy Grid Mapping:
- Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press. Chapter 9.
- [ROS Navigation Stack](http://wiki.ros.org/navigation)

### PyBullet:
- [PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/)
- [PyBullet Examples](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/examples)

---

## Support

**Course Forum:** [Link to be provided]  
**Office Hours:** [Schedule to be provided]  
**Email:** a.fatayer@zuj.edu.jo

---

**Good luck with your mapping implementation!** 🤖🗺️

---

## Download Lab Files

Run this lab online with one click, or download the Python files to run locally:

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/fatayer8891-boop/zuj-robotics/blob/main/labs/lab-04-mapping/notebook.ipynb)
&nbsp;&nbsp;**or download individual files below:**

| File | Description |
|------|-------------|
| [`activity1_lidar_viz.py`](https://github.com/fatayer8891-boop/zuj-robotics/raw/main/labs/lab-04-mapping/activity1_lidar_viz.py) | Activity 1 — Visualise raw LiDAR scans |
| [`activity2_basic_grid.py`](https://github.com/fatayer8891-boop/zuj-robotics/raw/main/labs/lab-04-mapping/activity2_basic_grid.py) | Activity 2 — Build a basic occupancy grid |
| [`activity3_warehouse_mapping.py`](https://github.com/fatayer8891-boop/zuj-robotics/raw/main/labs/lab-04-mapping/activity3_warehouse_mapping.py) | Activity 3 — Map the full warehouse environment |
| [`warehouse_environment.py`](https://github.com/fatayer8891-boop/zuj-robotics/raw/main/labs/lab-04-mapping/warehouse_environment.py) | Helper — Warehouse environment definition |

> **Download everything at once:** [Download full repository as ZIP](https://github.com/fatayer8891-boop/zuj-robotics/archive/refs/heads/main.zip) — extract and navigate to `zuj-robotics-main/labs/lab-04-mapping/`

---
