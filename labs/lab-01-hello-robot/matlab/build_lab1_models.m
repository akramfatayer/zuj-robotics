function build_lab1_models
% BUILD_LAB1_MODELS  Builds the three Simulink models of Lab 1 (MATLAB track).
%
%   lab1_square_2d.slx      Level 1: the robot seen from above, drawn while it drives
%   lab1_square_3d.slx      Level 2: a 3D robot in a robotScenario (Robotics System Toolbox)
%   lab1_square_unreal.slx  Level 3: the robot in an Unreal Engine scene (Simulink 3D Animation)
%
% All three share the same core: a square commander, the wheel speeds, a right wheel
% that can be made slightly wrong (WHEEL_ERROR), and the Differential Drive Kinematic
% Model block. Only the viewer differs.
%
% Students do not need to run this: the .slx files are already in this folder.
% Run it after changing the core, to rebuild all three models the same way.

here = fileparts(mfilename("fullpath"));
buildModel("lab1_square_2d", "2d", here);
buildModel("lab1_square_3d", "3d", here);
buildModel("lab1_square_unreal", "unreal", here);
end

function buildModel(name, level, folder)
if bdIsLoaded(name)
    close_system(name, 0);
end
file = fullfile(folder, name + ".slx");
if isfile(file)
    delete(file);   % rebuild from scratch (and avoid a "name shadows a file" warning)
end
new_system(name);
load_system("simulink");
load_system("robotmobilelib");

% Parameters live in the model workspace; the live script overrides them with setVariable
mws = get_param(name, "ModelWorkspace");
assignin(mws, "SIDE", 2);              % side of the square, m
assignin(mws, "SPEED", 1);             % forward speed, m/s
assignin(mws, "TURN_RATE", pi/2);      % turning rate, rad/s (90°/s)
assignin(mws, "WHEEL_RADIUS", 0.05);   % m
assignin(mws, "TRACK", 0.3);           % distance between the wheels, m
assignin(mws, "WHEEL_ERROR", 0);       % the right wheel turns this fraction too fast (try 0.02)
set_param(name, Solver="ode4", FixedStep="0.01", StopTime="13", ...
    EnablePacing="on", PacingRate="1");   % run in real time, so students can watch

% ---------- The core: commander -> wheels -> differential drive ----------
add_block("simulink/Sources/Clock", name + "/Clock", Position=[30 95 60 125]);
add_block("simulink/User-Defined Functions/MATLAB Function", name + "/Square commander", ...
    Position=[110 75 270 145]);
setCommanderCode(name + "/Square commander");
add_block("simulink/Math Operations/Gain", name + "/Right wheel error", Gain="1 + WHEEL_ERROR", ...
    Position=[320 120 420 150]);
add_block("robotmobilelib/Differential Drive Kinematic Model", name + "/Differential drive", ...
    WheelRadius="WHEEL_RADIUS", TrackWidth="TRACK", Position=[470 75 640 150]);
add_block("simulink/Sinks/To Workspace", name + "/Log pose", VariableName="pose", ...
    SaveFormat="Array", Position=[760 30 850 60]);
add_block("simulink/Sinks/To Workspace", name + "/Log velocity", VariableName="poseDot", ...
    SaveFormat="Array", Position=[760 165 850 195]);
add_line(name, "Clock/1", "Square commander/1");
add_line(name, "Square commander/1", "Differential drive/1", "autorouting", "smart");
add_line(name, "Square commander/2", "Right wheel error/1");
add_line(name, "Right wheel error/1", "Differential drive/2");
add_line(name, "Differential drive/1", "Log pose/1", "autorouting", "smart");
add_line(name, "Differential drive/2", "Log velocity/1", "autorouting", "smart");

% ---------- The viewer ----------
switch level
    case {"2d", "3d"}
        add_block("simulink/Discrete/Zero-Order Hold", name + "/Draw at 20 Hz", SampleTime="0.05", ...
            Position=[700 95 760 125]);
        add_block("simulink/User-Defined Functions/MATLAB System", name + "/Robot viewer", ...
            Position=[810 85 960 135]);
        if level == "2d"
            set_param(name + "/Robot viewer", System="RobotViewer2D");
        else
            set_param(name + "/Robot viewer", System="RobotViewer3D");
        end
        set_param(name + "/Robot viewer", SimulateUsing="Interpreted execution", Side="SIDE");
        add_line(name, "Differential drive/1", "Draw at 20 Hz/1", "autorouting", "smart");
        add_line(name, "Draw at 20 Hz/1", "Robot viewer/1");
    case "unreal"
        % The camera looks along +x from behind the square, from 5 m up, pitched 50 degrees down
        add_block("sim3dlib/Simulation 3D Scene Configuration", name + "/3D scene", ...
            SceneDesc="Open surface", Ts="0.02", Position=[810 220 960 280], ...
            FreeCameraDefaultInitialTranslation="[-2.5, -1, 4.5]", FreeCameraDefaultInitialRotation="[0, -50, 0]", ...
            FreeCameraInitialTranslation="[-2.5, -1, 4.5]", FreeCameraInitialRotation="[0, -50, 0]");
        add_block("sim3dlib/Simulation 3D Actor", name + "/Square markers", ActorName="Markers", ...
            InitScriptText=markersActorScript(), SampleTime="0.02", Position=[990 220 1110 280]);
        add_block("simulink/Discrete/Zero-Order Hold", name + "/Send at 50 Hz", SampleTime="0.02", ...
            Position=[700 95 760 125]);
        add_block("simulink/User-Defined Functions/MATLAB Function", name + "/Pose to 3D", ...
            Position=[800 85 930 135]);
        setPoseTo3DCode(name + "/Pose to 3D");
        add_block("sim3dlib/Simulation 3D Actor", name + "/Robot", ActorName="Robot", ...
            InitScriptText=robotActorScript(), SampleTime="0.02", Position=[990 80 1110 140]);
        set_param(name + "/Robot", InputsText=sprintf("Robot.Translation\nRobot.Rotation"));
        add_line(name, "Differential drive/1", "Send at 50 Hz/1", "autorouting", "smart");
        add_line(name, "Send at 50 Hz/1", "Pose to 3D/1");
        add_line(name, "Pose to 3D/1", "Robot/1");
        add_line(name, "Pose to 3D/2", "Robot/2");
end

note = Simulink.Annotation(name + "/About");
note.Text = sprintf(['Lab 1 (MATLAB track): drive a %g m square, open loop.\n' ...
    'Parameters are in the Model Workspace (Modeling > Model Explorer). ' ...
    'Try WHEEL_ERROR = 0.02 to see the drift.'], 2);
note.Position = [30 240 600 290];

save_system(name, fullfile(folder, name + ".slx"));
close_system(name);
fprintf("Built %s.slx\n", name);
end

function setCommanderCode(block)
chart = find(sfroot, "-isa", "Stateflow.EMChart", "Path", block);
chart.Script = join([
    "function [wL, wR] = commander(t, SIDE, SPEED, TURN_RATE, WHEEL_RADIUS, TRACK)"
    "% Open-loop square, as in the Python lab: drive SIDE metres, turn 90 degrees left,"
    "% four times, then stop. No sensor is used: the robot does not know where it is."
    "tSide = SIDE / SPEED;            % time to drive one side, s"
    "tTurn = (pi/2) / TURN_RATE;      % time to turn 90 degrees, s"
    "period = tSide + tTurn;"
    "if t >= 4 * period"
    "    v = 0; w = 0;                % finished: stop"
    "elseif mod(t, period) < tSide"
    "    v = SPEED; w = 0;            % drive forward"
    "else"
    "    v = 0; w = TURN_RATE;        % turn left on the spot"
    "end"
    "% Inverse kinematics of a differential drive: body speeds -> wheel speeds (rad/s)"
    "wL = (v - w * TRACK / 2) / WHEEL_RADIUS;"
    "wR = (v + w * TRACK / 2) / WHEEL_RADIUS;"
    ], newline);
for p = ["SIDE", "SPEED", "TURN_RATE", "WHEEL_RADIUS", "TRACK"]
    d = find(chart, "-isa", "Stateflow.Data", "Name", p);
    d.Scope = "Parameter";
end
chart.SampleTime = "0.01";   % hold each command for a whole step
end

function setPoseTo3DCode(block)
chart = find(sfroot, "-isa", "Stateflow.EMChart", "Path", block);
chart.Script = join([
    "function [translation, rotation] = poseTo3D(pose)"
    "% pose = [x; y; theta] from the drive block -> the actor's position and rotation."
    "% Unreal's world is left-handed (y points right), so y and the heading change sign:"
    "% otherwise every left turn would look like a right turn on the screen."
    "translation = [pose(1), -pose(2), 0.25];  % lift the body so it stands on the floor, m"
    "rotation = [0, 0, -pose(3)];              % roll, pitch, yaw, rad"
    ], newline);
end

function s = robotActorScript()
% Initialization script of the Unreal robot: an R2D2-like robot built from shapes.
% Actor and World are reserved keywords of the Simulation 3D Actor block.
s = join([
    "% The robot: a cylinder body, a dome and a red nose that shows its heading"
    "createShape(Actor, 'cylinder', [0.4 0.4 0.5]);"
    "Actor.Color = [0.92 0.92 0.95];"
    "dome = sim3d.Actor(ActorName='Dome');"
    "createShape(dome, 'sphere', [0.4 0.4 0.4]);"
    "dome.Color = [0.35 0.5 0.85];"
    "dome.Translation = [0 0 0.25];"
    "add(World, dome, Actor);"
    "nose = sim3d.Actor(ActorName='Nose');"
    "createShape(nose, 'box', [0.12 0.12 0.12]);"
    "nose.Color = [0.85 0.1 0.1];"
    "nose.Translation = [0.2 0 0.3];"
    "add(World, nose, Actor);"
    ], newline);
end

function s = markersActorScript()
% Initialization script of a fixed actor that marks the 2 m square: white lines on the
% floor and an orange post at each corner (children of this actor, which stays at the origin).
s = join([
    "% A tiny base at the origin; the lines and posts are its children"
    "createShape(Actor, 'box', [0.01 0.01 0.01]);"
    "corners = [0 0; 2 0; 2 -2; 0 -2];   % the square in Unreal's frame (y flipped)"
    "for k = 1:4"
    "    post = sim3d.Actor(ActorName=sprintf('Post%d', k));"
    "    createShape(post, 'cylinder', [0.08 0.08 0.4]);"
    "    post.Color = [0.95 0.55 0.1];"
    "    post.Translation = [corners(k, :), 0.2];"
    "    add(World, post, Actor);"
    "    a = corners(k, :);"
    "    b = corners(mod(k, 4) + 1, :);"
    "    edge = sim3d.Actor(ActorName=sprintf('Edge%d', k));"
    "    createShape(edge, 'box', [norm(b - a) + 0.05, 0.05, 0.01]);"
    "    edge.Color = [1 1 1];"
    "    edge.Translation = [(a + b) / 2, 0.005];"
    "    edge.Rotation = [0, 0, atan2(b(2) - a(2), b(1) - a(1))];"
    "    add(World, edge, Actor);"
    "end"
    ], newline);
end
