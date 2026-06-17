# Labs Overview

The labs in this course form a **progressive integration pipeline**. Each lab builds one layer of an autonomous warehouse robot system. By the end of the course, you will have implemented every component needed for full autonomous navigation.

---

## Run Labs Online

Every lab is available as a **Jupyter Notebook** that you can run directly in Google Colab — no installation required. Just click the badge next to each lab:

| Lab | Topic | Run Online |
|-----|-------|:----------:|
| Lab 1 | Hello, Robot! | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/fatayer8891-boop/zuj-robotics/blob/main/labs/lab-01-hello-robot/notebook.ipynb) |
| Lab 3 | Localization | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/fatayer8891-boop/zuj-robotics/blob/main/labs/lab-03-localization/notebook.ipynb) |
| Lab 4 | Mapping | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/fatayer8891-boop/zuj-robotics/blob/main/labs/lab-04-mapping/notebook.ipynb) |
| Lab 5 | Path Planning | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/fatayer8891-boop/zuj-robotics/blob/main/labs/lab-05-path-planning/notebook.ipynb) |
| Lab 6 | Tracking & Control | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/fatayer8891-boop/zuj-robotics/blob/main/labs/lab-06-tracking-control/notebook.ipynb) |

!!! info "Colab vs Local"
    **Google Colab** runs PyBullet in headless mode (no 3D GUI window). You'll see matplotlib plots of trajectories and sensor data instead of the live 3D simulation. For the full interactive 3D experience, run the labs locally using your Conda environment.

---

## The Integration Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                    AUTONOMOUS WAREHOUSE ROBOT                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│   Lab 1          Lab 3           Lab 4          Lab 5    Lab 6   │
│  ┌──────┐     ┌──────────┐    ┌────────┐    ┌───────┐ ┌──────┐ │
│  │SENSE │ ──► │LOCALIZE  │ ──►│  MAP   │ ──►│ PLAN  │►│TRACK │ │
│  │& ACT │     │(Kalman)  │    │(LiDAR) │    │(A*/RRT)│ │(PID/ │ │
│  └──────┘     └──────────┘    └────────┘    └───────┘ │ PP)  │ │
│                                                         └──────┘ │
│                                                                   │
│   Final Project: Wire ALL layers into one autonomous system       │
└─────────────────────────────────────────────────────────────────┘
```

---

## Lab Progression

| Lab | Title | What You Build | Builds On |
|-----|-------|----------------|-----------|
| [Lab 1](lab-01-hello-robot/README.md) | Hello, Robot! | Spawn a robot, read sensors, drive in a square | — (foundation) |
| [Lab 3](lab-03-localization/README.md) | Localization | Kalman filter fusing odometry + GPS | Lab 1 (sensors) |
| [Lab 4](lab-04-mapping/README.md) | Mapping | Occupancy grid from LiDAR scans | Lab 1 (sensors) + Lab 3 (pose) |
| [Lab 5](lab-05-path-planning/README.md) | Path Planning | A* and RRT on occupancy grids | Lab 4 (map) |
| [Lab 6](lab-06-tracking-control/README.md) | Tracking & Control | PID and Pure Pursuit path following | Lab 5 (path) |

---

## Assessment

Each lab is assessed through:

- **Code submission** — your completed Python scripts
- **Validation output** — automated tests confirming correctness
- **Reflection** — brief written answers demonstrating understanding
- **Screenshots/video** — visual evidence of working system

All labs contribute to the **60% PBL component** of your final grade.

---

## Environment Setup

All labs use the same Python environment. Set it up once in Lab 1:

```bash
conda create -n robotics_YourFirstName python=3.11 -y
conda activate robotics_YourFirstName
conda install -c conda-forge pybullet numpy matplotlib opencv -y
```

Activate this environment at the start of every lab session:

```bash
conda activate robotics_YourFirstName
```

---

*Dr. Akram Fatayer · [a.fatayer@zuj.edu.jo](mailto:a.fatayer@zuj.edu.jo) · Al-Zaytoonah University of Jordan*
