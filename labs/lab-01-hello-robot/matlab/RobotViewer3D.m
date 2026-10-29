classdef RobotViewer3D < matlab.System
    % RobotViewer3D  Level 2 viewer: a 3D ground robot in a robotScenario
    %   (Robotics System Toolbox), drawn while the model runs.
    %   Used by the MATLAB System block in lab1_square_3d.slx.
    %   Input: pose [x; y; theta] in metres and radians (the drive block's state output).

    properties (Nontunable)
        Side = 2   % side of the commanded square, m (corner posts mark it)
    end

    properties (Access = private)
        Scenario   % robotScenario: the floor, the posts and the robot
        Robot      % robotPlatform for the robot
        Ax         % axes that show3D draws into
        Trail      % the path driven so far
    end

    methods (Access = protected)
        function setupImpl(obj)
            s = obj.Side;
            obj.Scenario = robotScenario(UpdateRate=20);
            addMesh(obj.Scenario, "Plane", Position=[s/2 s/2 0], Size=[s + 3, s + 3], Color=[0.85 0.85 0.82]);
            for corner = [0 0; s 0; s s; 0 s]'   % a post at each corner of the square
                addMesh(obj.Scenario, "Cylinder", Position=[corner' 0.15], Size=[0.05 0.3], Color=[0.9 0.5 0.1]);
            end
            obj.Robot = robotPlatform("robot", obj.Scenario);
            updateMesh(obj.Robot, "GroundVehicle", Scale=0.7, Color=[0.2 0.4 0.8]);
            setup(obj.Scenario);
            fig = figure(Name="Robot view: Level 2 (3D)", NumberTitle="off", Position=[100 100 900 650]);
            if exist("theme", "file")
                theme(fig, "light");
            end
            obj.Ax = show3D(obj.Scenario, Parent=axes(fig));
            set(findobj(obj.Ax, Type="patch"), EdgeColor="none");   % no mesh edges across the floor
            camlight(obj.Ax, "headlight");
            lighting(obj.Ax, "gouraud");
            material(obj.Ax, "dull");
            hold(obj.Ax, "on");
            obj.Trail = animatedline(obj.Ax, Color=[0 0.45 0.74], LineWidth=2);
            view(obj.Ax, -35, 35); axis(obj.Ax, "equal");
            xlim(obj.Ax, [-1.5, s + 1.5]); ylim(obj.Ax, [-1.5, s + 1.5]); zlim(obj.Ax, [0 1]);
            title(obj.Ax, "Level 2: the robot in a 3D scenario (orange posts = the square)");
        end

        function stepImpl(obj, pose)
            if ~isvalid(obj.Ax)   % the student closed the window: keep simulating
                return
            end
            q = compact(quaternion([pose(3) 0 0], "euler", "ZYX", "frame"));
            % move() takes [position, velocity, acceleration, orientation quaternion, angular velocity]
            move(obj.Robot, "base", [pose(1) pose(2) 0, 0 0 0, 0 0 0, q, 0 0 0]);
            advance(obj.Scenario);
            show3D(obj.Scenario, Parent=obj.Ax, FastUpdate=true);
            addpoints(obj.Trail, pose(1), pose(2), 0.02);
            drawnow limitrate
        end

        function n = getNumOutputsImpl(~)
            n = 0;
        end
    end
end
