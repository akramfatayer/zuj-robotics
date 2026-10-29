classdef RobotViewer2D < matlab.System
    % RobotViewer2D  Level 1 viewer: the robot seen from above, drawn while the model runs.
    %   Used by the MATLAB System block in lab1_square_2d.slx.
    %   Input: pose [x; y; theta] in metres and radians (the drive block's state output).

    properties (Nontunable)
        Side = 2   % side of the commanded square, m (drawn as a dashed target)
    end

    properties (Access = private)
        Fig      % the viewer window
        Body     % hgtransform that carries the robot drawing
        Trail    % the path driven so far
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.Fig = figure(Name="Robot view: Level 1 (2D)", NumberTitle="off", Position=[80 80 720 680]);
            if exist("theme", "file")
                theme(obj.Fig, "light");
            end
            ax = axes(obj.Fig);
            hold(ax, "on"); grid(ax, "on"); axis(ax, "equal");
            s = obj.Side;
            plot(ax, [0 s s 0 0], [0 0 s s 0], "--", Color=[0.6 0.6 0.6], LineWidth=1.5, DisplayName="Commanded square");
            obj.Trail = animatedline(ax, Color=[0 0.45 0.74], LineWidth=2, DisplayName="Path driven");
            % The robot, drawn at the origin facing +x; the hgtransform moves it
            obj.Body = hgtransform(ax);
            r = 0.18;
            a = linspace(0, 2*pi, 48);
            patch(Parent=obj.Body, XData=r*cos(a), YData=r*sin(a), FaceColor=[0.88 0.88 0.9], ...
                EdgeColor=[0.2 0.2 0.2], LineWidth=1.5);
            for side = [-1 1]   % the two wheels
                patch(Parent=obj.Body, XData=[-0.07 0.07 0.07 -0.07], YData=side*[0.15 0.15 0.2 0.2], ...
                    FaceColor=[0.15 0.15 0.15], EdgeColor="none");
            end
            line(obj.Body, [0 r], [0 0], Color=[0.85 0.1 0.1], LineWidth=3);   % heading
            xlim(ax, [-1, s + 1]); ylim(ax, [-1, s + 1]);
            xlabel(ax, "x (m)"); ylabel(ax, "y (m)");
            title(ax, "Level 1: the robot from above (red line = heading)");
            legend(ax, Location="northeast");
        end

        function stepImpl(obj, pose)
            if ~isvalid(obj.Fig)   % the student closed the window: keep simulating
                return
            end
            obj.Body.Matrix = makehgtform("translate", [pose(1) pose(2) 0], "zrotate", pose(3));
            addpoints(obj.Trail, pose(1), pose(2));
            drawnow limitrate
        end

        function n = getNumOutputsImpl(~)
            n = 0;
        end
    end
end
