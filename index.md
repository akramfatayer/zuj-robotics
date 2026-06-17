# Introduction to Robotics for AI and Data Science

**Course Code:** 0135343  
**Instructor:** Dr. Akram Fatayer  
**Email:** [a.fatayer@zuj.edu.jo](mailto:a.fatayer@zuj.edu.jo)  
**LinkedIn:** [linkedin.com/in/akram-fatayer](https://www.linkedin.com/in/akram-fatayer/)

---

## Course Overview

This course takes a data-first, project-based approach to intelligent mobile robotics. Rather than studying robotics theory in isolation, you will progressively build a **fully autonomous warehouse robot** — one module at a time — integrating perception, localization, mapping, planning, and control into a single working system.

The course is designed for AI and Data Science students who are proficient in Python and machine learning but new to robotics. You will discover that robotics is applied AI: sensor fusion is ensemble learning, path planning is graph search, and control is optimization — concepts you already know, applied to physical systems.

**The 80/20 Principle:** To master autonomous mobile robotics, you need to master these core concepts: the Sense-Plan-Act cycle, Kalman filtering for state estimation, occupancy grid mapping, A* search for path planning, and PID/Pure Pursuit for path tracking. These five ideas unlock the ability to build any autonomous navigation system.

---

## The Autonomous Warehouse Robot Project

Your semester-long mission is to build an autonomous warehouse robot that can:

1. **Sense** its environment using LiDAR and cameras
2. **Localize** itself using a Kalman filter (fusing odometry + GPS)
3. **Map** the warehouse using occupancy grid mapping
4. **Plan** collision-free paths using A* and RRT
5. **Track** those paths smoothly using PID and Pure Pursuit controllers
6. **Replan** when dynamic obstacles appear

Each lab builds one layer of this system. The final project integrates all layers into a complete autonomous pipeline.

---

## Course Structure

The course is organized into **four progressive project blocks**, each building on the previous:

| Block | Title | Lecture Topic | Lab |
|-------|-------|---------------|-----|
| 1 | See the World (Perception) | Introduction to Robotics; Computer Vision for Robots | Lab 1: Hello, Robot! |
| 2 | Know Where You Are (Localization & Mapping) | Localization & State Estimation; Distance Sensing & Occupancy Grids | Lab 3: Localization; Lab 4: Mapping |
| 3 | Find Your Path (Planning) | Path Planning and Navigation | Lab 5: Path Planning (A*, RRT) |
| 4 | Move Smoothly (Control) | Path Tracking and Control | Lab 6: Tracking (PID, Pure Pursuit) |
| Final | Full Integration | — | Autonomous Warehouse Project |

---

## Learning Outcomes

| Code | Outcome |
|------|---------|
| **MK1** | Build and program autonomous robotic systems using Python and physics simulation |
| **MK2** | Apply AI fundamentals (state estimation, search algorithms, control theory) to robotics |
| **MS1** | Process and fuse sensor data (LiDAR, odometry, GPS) for robot perception |
| **MS2** | Implement machine learning concepts (Kalman filtering, A* search, PID control) in robotic systems |
| **MC1** | Design and build a complete autonomous navigation pipeline for a warehouse robot |
| **MC2** | Solve novel robotics challenges through creative integration of learned techniques |
| **MT1** | Collaborate effectively on the final integration project |

---

## Assessment

| Component | Weight | Description |
|-----------|--------|-------------|
| **Labs & Projects (PBL)** | 60% | Progressive lab assignments + final integration project |
| **Final Exam** | 40% | Conceptual understanding and problem-solving |

---

## Course Materials

### Lectures

| # | Topic | Format |
|---|-------|--------|
| 1 | [Introduction to Robotics](lectures/pdf/Introduction_to_Robotics.pdf) | PDF |
| 2 | [Computer Vision for Warehouse Robots](lectures/pdf/Computer_Vision_for_Warehouse_Robots.pdf) | PDF |
| 3 | [Robot Localization and State Estimation](lectures/pdf/Robot_Localization_and_State_Estimation.pdf) | PDF |
| 4 | [Distance Sensing and Occupancy Grid Mapping](lectures/pdf/Distance_Sensing_and_Occupancy_Grid_Mapping.pdf) | PDF |
| 5 | [Path Planning and Navigation](lectures/pdf/Lab_5_Path_Planning_and_Navigation.pdf) | PDF |
| 6 | [Path Tracking and Control](lectures/pdf/Lab_6_Path_Tracking_and_Control.pdf) | PDF |

### Labs

| Lab | Topic | Key Concepts |
|-----|-------|-------------|
| [Lab 1: Hello, Robot!](labs/lab-01-hello-robot/README.md) | Spawn, sense, and move a robot | PyBullet, URDF, velocity commands |
| [Lab 3: Localization](labs/lab-03-localization/README.md) | Kalman filter for state estimation | Predict-update cycle, sensor fusion |
| [Lab 4: Mapping](labs/lab-04-mapping/README.md) | Occupancy grid from LiDAR | Ray casting, log-odds, grid maps |
| [Lab 5: Path Planning](labs/lab-05-path-planning/README.md) | A* and RRT algorithms | Graph search, sampling-based planning |
| [Lab 6: Tracking & Control](labs/lab-06-tracking-control/README.md) | PID and Pure Pursuit controllers | Cross-track error, look-ahead steering |

### Final Project

| Resource | Description |
|----------|-------------|
| [Project Package](project/V2_Project_Student_Release.zip) | Complete student release with all modules |

### Assessments

| Resource | Description |
|----------|-------------|
| [Course Summary](assessments/md/Course_Summary_V2.md) | Complete course review |
| [Formula Sheet](assessments/md/Formula_Sheet_V2.md) | Reference formulas for the exam |
| [Practice Exam](assessments/md/Practice_Exam_V2.md) | Sample exam questions |
| [Practice Exam — Answer Key](assessments/md/Practice_Exam_V2_Answer_Key.md) | Solutions to practice exam |

---

## Tools & Environment

| Tool | Purpose | Installation |
|------|---------|-------------|
| **Python 3.11** | Programming language | Via Anaconda |
| **Anaconda** | Environment management | [anaconda.com](https://www.anaconda.com/) |
| **PyBullet** | Physics simulation | `conda install -c conda-forge pybullet` |
| **NumPy** | Numerical computation | `conda install numpy` |
| **Matplotlib** | Visualization | `conda install matplotlib` |
| **OpenCV** | Computer vision | `conda install opencv` |

---

## Getting Started

1. Install Anaconda and create your course environment (see Lab 1 README)
2. Read the [Introduction to Robotics](lectures/Introduction_to_Robotics_for_AI_and_Data_Science.pdf) overview
3. Complete [Lab 1: Hello, Robot!](labs/lab-01-hello-robot/README.md) to meet your robot
4. Follow the project blocks in sequence — each builds on the previous

---

*Dr. Akram Fatayer · [a.fatayer@zuj.edu.jo](mailto:a.fatayer@zuj.edu.jo) · [LinkedIn](https://www.linkedin.com/in/akram-fatayer/) · Al-Zaytoonah University of Jordan*
