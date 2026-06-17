# zuj-robotics — Introduction to Robotics for AI and Data Science

**Course Code:** 0135343  
**University:** Al-Zaytoonah University of Jordan  
**Department:** Artificial Intelligence  
**Instructor:** Dr. Akram Fatayer ([a.fatayer@zuj.edu.jo](mailto:a.fatayer@zuj.edu.jo))

---

## About

This repository contains all course materials for "Introduction to Robotics for AI and Data Science" (0135343). The course takes a data-first, project-based approach where students progressively build a fully autonomous warehouse robot using Python and PyBullet simulation.

## Repository Structure

```
zuj-robotics/
├── index.md                    # Course landing page (for MkDocs site)
├── COURSE_CONTEXT.md           # Living course state (for AI assistants)
├── ROADMAP.md                  # Development roadmap
├── lectures/                   # Lecture slides (PPTX + PDF)
│   ├── Introduction_to_Robotics.pptx
│   ├── Computer_Vision_for_Warehouse_Robots.pptx
│   ├── Robot_Localization_and_State_Estimation.pptx
│   ├── Distance_Sensing_and_Occupancy_Grid_Mapping.pptx
│   ├── Lab_5_Path_Planning_and_Navigation.pdf
│   └── Lab_6_Path_Tracking_and_Control.pdf
├── labs/                       # Hands-on lab packages
│   ├── lab-01-hello-robot/     # Spawn, sense, move
│   ├── lab-03-localization/    # Kalman filter
│   ├── lab-04-mapping/         # Occupancy grids
│   ├── lab-05-path-planning/   # A*, RRT
│   └── lab-06-tracking-control/# PID, Pure Pursuit
├── project/                    # Final integration project
│   └── V2_Project_Student_Release.zip
├── assessments/                # Exam materials
│   ├── Course_Summary_V2.docx
│   ├── Formula_Sheet_V2.docx
│   ├── Practice_Exam_V2.docx
│   └── Practice_Exam_V2_Answer_Key.docx
└── resources/                  # Supplementary materials
```

## Course Blocks

| Block | Title | Focus |
|-------|-------|-------|
| 1 | See the World | Perception — sensors, cameras, basic motion |
| 2 | Know Where You Are | Localization & Mapping — Kalman filter, LiDAR, grids |
| 3 | Find Your Path | Planning — A*, RRT, global/local planning |
| 4 | Move Smoothly | Control — PID, Pure Pursuit, path tracking |
| Final | Full Integration | Autonomous warehouse robot pipeline |

## Tools Required

- Python 3.11 (via Anaconda)
- PyBullet, NumPy, Matplotlib, OpenCV

## Part of the ZUJ Hub

This repo is mounted as a submodule in [zuj-hub](https://github.com/fatayer8891-boop/zuj-hub), which builds a unified course website at [fatayer8891-boop.github.io/zuj-hub](https://fatayer8891-boop.github.io/zuj-hub/).

## Working Principle

No AI commits autonomously. Assistants propose changes; a human reviews and commits. See `COURSE_CONTEXT.md` for current state.
