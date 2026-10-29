# Lab 1: Hello, Robot! — MATLAB Track (Simulink)

This is the MATLAB version of [Lab 1](../README.md). The mission is the same as in the Python track: create a robot, read its state, drive it in a 2 m square, and analyse the path. Here the robot is a **Simulink model**, and you can watch it drive at three levels.

| Level | Model | What you see | Needs |
|---|---|---|---|
| 1 | `lab1_square_2d.slx` | The robot from above, with its path, drawn while it drives | Simulink, Robotics System Toolbox |
| 2 | `lab1_square_3d.slx` | A 3D robot in a `robotScenario` | Simulink, Robotics System Toolbox |
| 3 | `lab1_square_unreal.slx` | The robot in an Unreal Engine 3D scene | Simulink 3D Animation and a good graphics card. Not available in MATLAB Online |

All three run the same model: a **Square commander** (MATLAB Function block) plans the motion and converts it to wheel speeds, a **Right wheel error** gain can make one wheel slightly wrong, and the **Differential Drive Kinematic Model** block turns the wheel speeds into the pose [x, y, θ]. Only the viewer differs.

---

## Setup

You need **MATLAB R2025b or later** with **Simulink** and the **Robotics System Toolbox** (and **Simulink 3D Animation** for Level 3). The university licence includes all of them.

- **MATLAB Online (no installation, Levels 1 and 2):** click **Open in MATLAB Online** on the [course page](https://akramfatayer.github.io/courses/robotics) and sign in with your university MathWorks account.
- **MATLAB on a lab PC or your laptop:** download the [full repository as a ZIP](https://github.com/akramfatayer/zuj-robotics/archive/refs/heads/main.zip), extract it, and open `labs/lab-01-hello-robot/matlab/` in MATLAB. Keep all the files of this folder together: the models need the two viewer files.

Open `lab1_hello_robot.m` (it opens in the Live Editor) and click **Run**.

> **Level 3, the first time:** the 3D scene takes about 30 seconds to start. Windows may ask whether **Simulation 3D Viewer** may use the network: it is MATLAB's own 3D window, and the simulation works either way.

---

## Activities

The live script `lab1_hello_robot.m` walks you through the four parts of the Python lab:

1. **Create the robot:** open the model and find its blocks and parameters.
2. **Read the robot's state:** the pose and speeds every 0.5 s.
3. **Drive the square:** run the model and watch the robot at the level you chose (`LEVEL = 1`, `2` or `3`).
4. **Analyse the path:** four plots, saved as `robot_trajectory_analysis.png`.

Then change `WHEEL_ERROR` in the **Experiment** section (try `0.02` and `-0.02`) and answer the **Explore** questions.

---

## Deliverables

1. **Figure:** `robot_trajectory_analysis.png` with `WHEEL_ERROR = 0.02`.
2. **Video:** a short screen recording of your robot driving the square, at any level.
3. **Report** (1–2 pages): the figure, your answers to the three **Explore** questions in the live script, and a brief reflection on the challenges you faced.

---

## Files

| File | What it is |
|------|-------------|
| [`lab1_hello_robot.m`](https://github.com/akramfatayer/zuj-robotics/raw/main/labs/lab-01-hello-robot/matlab/lab1_hello_robot.m) | The live script: start here |
| [`lab1_square_2d.slx`](https://github.com/akramfatayer/zuj-robotics/raw/main/labs/lab-01-hello-robot/matlab/lab1_square_2d.slx) | Level 1 model |
| [`lab1_square_3d.slx`](https://github.com/akramfatayer/zuj-robotics/raw/main/labs/lab-01-hello-robot/matlab/lab1_square_3d.slx) | Level 2 model |
| [`lab1_square_unreal.slx`](https://github.com/akramfatayer/zuj-robotics/raw/main/labs/lab-01-hello-robot/matlab/lab1_square_unreal.slx) | Level 3 model |
| [`RobotViewer2D.m`](https://github.com/akramfatayer/zuj-robotics/raw/main/labs/lab-01-hello-robot/matlab/RobotViewer2D.m), [`RobotViewer3D.m`](https://github.com/akramfatayer/zuj-robotics/raw/main/labs/lab-01-hello-robot/matlab/RobotViewer3D.m) | The Level 1 and Level 2 viewers (System objects used by the models) |
| [`build_lab1_models.m`](https://github.com/akramfatayer/zuj-robotics/raw/main/labs/lab-01-hello-robot/matlab/build_lab1_models.m) | Rebuilds all three models from one shared core (for the instructor) |

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=akramfatayer/zuj-robotics&file=labs/lab-01-hello-robot/matlab/lab1_hello_robot.m)
