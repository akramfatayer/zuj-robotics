%[text] # Lab 1 · Hello, Robot! (Simulink)
%[text] Intelligent Mobile Robots (0135343) · **MATLAB track** · follows Lecture 1 and [Module 1](https://akramfatayer.github.io/courses/robotics/learn/1)
%[text] The same mission as the Python track: create a robot, read its state, drive it in a 2 m square, and analyse the path. Here the robot is a **Simulink model**: a square commander turns the plan into wheel speeds, and the **Differential Drive Kinematic Model** block (Robotics System Toolbox) turns the wheel speeds into motion.
%[text] You can watch the robot at three levels. All three run the same model; only the viewer changes:
%[text:table]
%[text] | Level | Model | What you see | Needs |
%[text] | --- | --- | --- | --- |
%[text] | 1 | `lab1_square_2d.slx` | The robot from above, with its path | Simulink, Robotics System Toolbox |
%[text] | 2 | `lab1_square_3d.slx` | A 3D robot in a `robotScenario` | the same |
%[text] | 3 | `lab1_square_unreal.slx` | The robot in an Unreal Engine 3D scene | Simulink 3D Animation, a good graphics card; not in MATLAB Online |
%[text:table]
%%
%[text] ## Experiment
%[text] Change these values and run the script again.
LEVEL = 1;           % 1 (2D), 2 (3D) or 3 (Unreal)
WHEEL_ERROR = 0;     % the right wheel turns this fraction too fast (try 0.02 and -0.02)
REAL_TIME = true;    % true: the robot moves in real time, so you can watch it
%%
%[text] ## Part 1: Create the robot
%[text] Open the model and look at it: the **Square commander** (a MATLAB Function block) plans the motion, the **Right wheel error** gain makes one wheel slightly wrong, and the **Differential drive** block integrates the wheel speeds into the pose $[x, y, \\theta]$.
here = fileparts(mfilename("fullpath"));
if here ~= ""
    cd(here)   % work in this lab's folder, where the models and viewers are
end
models = ["lab1_square_2d", "lab1_square_3d", "lab1_square_unreal"];
model = models(LEVEL);
open_system(model)
%[text] The robot's parameters live in the model's workspace (**Modeling** tab, **Model Explorer**):
load_system(model)
mws = get_param(model, "ModelWorkspace");
params = ["SIDE", "SPEED", "TURN_RATE", "WHEEL_RADIUS", "TRACK"];
values = arrayfun(@(p) getVariable(mws, p), params);
disp(table(params', values', VariableNames=["Parameter", "Value"]))
%%
%[text] ## Part 2 and 3: Drive the square, and read the robot's state
%[text] The commander drives 2 m at 1 m/s, turns 90° left at 90°/s, four times, then stops. It is **open loop**: it never checks where the robot is. Level 3 takes about 30 s to start the 3D scene the first time.
in = Simulink.SimulationInput(model);
in = setVariable(in, "WHEEL_ERROR", WHEEL_ERROR, Workspace=model);
if ~REAL_TIME
    in = setModelParameter(in, EnablePacing="off");
end
out = sim(in);
t = out.tout;
pose = out.pose;        % [x, y, theta] at every 0.01 s step
poseDot = out.poseDot;  % [dx/dt, dy/dt, dtheta/dt]
%[text] The robot's state every 0.5 s for the first 5 s, like `read_sensors.py` in the Python track:
k = find(mod(round(t * 100), 50) == 0 & t <= 5);
speed = hypot(poseDot(:, 1), poseDot(:, 2));
disp(table(t(k), pose(k, 1), pose(k, 2), rad2deg(pose(k, 3)), speed(k), rad2deg(poseDot(k, 3)), ...
    VariableNames=["t (s)", "x (m)", "y (m)", "heading (deg)", "speed (m/s)", "turn rate (deg/s)"]))
%%
%[text] ## Part 4: Analyse the path
endError = hypot(pose(end, 1), pose(end, 2));
fprintf("WHEEL_ERROR = %g: the robot ends %.2f m from where it started, turned %.1f° too far.\n", ...
    WHEEL_ERROR, endError, rad2deg(pose(end, 3)) - 360);
fig = figure(Position=[100 100 1200 850]);
lightTheme(fig);
tiledlayout(fig, 2, 2, TileSpacing="compact");
nexttile
s = getVariable(mws, "SIDE");
plot([0 s s 0 0], [0 0 s s 0], "--", Color=[0.6 0.6 0.6], DisplayName="Commanded square");
hold on
plot(pose(:, 1), pose(:, 2), LineWidth=2, DisplayName="Robot path");
plot(pose(1, 1), pose(1, 2), "go", MarkerSize=10, LineWidth=2, DisplayName="Start");
plot(pose(end, 1), pose(end, 2), "rs", MarkerSize=10, LineWidth=2, DisplayName="End");
hold off
axis equal; grid on; legend(Location="best");
xlabel("x (m)"); ylabel("y (m)"); title("Top-down trajectory");
nexttile
plot(t, pose(:, 1), LineWidth=1.5); grid on
xlabel("Time (s)"); ylabel("x (m)"); title("x position over time");
nexttile
plot(t, pose(:, 2), LineWidth=1.5); grid on
xlabel("Time (s)"); ylabel("y (m)"); title("y position over time");
nexttile
plot(t, rad2deg(pose(:, 3)), LineWidth=1.5); grid on
xlabel("Time (s)"); ylabel("Heading θ (deg)"); title("Heading over time: four 90° turns");
sgtitle(fig, sprintf("Robot trajectory analysis (WHEEL\\_ERROR = %g)", WHEEL_ERROR), FontWeight="bold");
exportgraphics(fig, "robot_trajectory_analysis.png", Resolution=150);
disp("Saved robot_trajectory_analysis.png")
%%
%[text] ## Explore (for your report)
%[text] 1. With `WHEEL_ERROR = 0`, how far from the start does the robot end? Why is it not exactly zero?
%[text] 2. Set `WHEEL_ERROR = 0.02`. How far from the start does it end now, and which way did it drift? Then try `-0.02`.
%[text] 3. A 2% error in one wheel gives a much bigger error at the end of the square. Why does the error grow? Why can this robot not correct it by itself? \
%[text] Lab 3 gives the robot a way to know where it is.
%%
%[text] ## Local functions
function lightTheme(fig)
% Light figure even when MATLAB uses the dark theme (R2025a+), so the saved PNG is readable.
if exist("theme", "file")
    theme(fig, "light");
end
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline"}
%---
