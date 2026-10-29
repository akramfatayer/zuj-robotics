%[text] # Lab 3 · Activity 1: The Kalman Filter in 1D
%[text] Intelligent Mobile Robots (0135343) · **MATLAB track** · follows Lecture 2 and [Module 2](https://akramfatayer.github.io/courses/robotics/learn/2)
%[text] A robot drives along a straight line at about 1 m/s. Once per second a sensor measures its position, with noise of σ = 1 m. A **Kalman filter** combines a motion model (constant velocity) with these noisy readings. It reports an estimate *and* how sure it is: a ±2σ band.
%[text] In the MATLAB track you do not type the filter equations yourself. You use `trackingKF` from the Sensor Fusion and Tracking Toolbox, then judge it by its error. The Python track writes the same filter by hand.
%%
%[text] ## The scenario
%[text] Keep these fixed. `rng(42)` gives everyone the same random numbers, so everyone sees the same figures.
NUM_STEPS = 60;     % 60 s
DT = 1;             % one reading per second, s
SPEED = 1;          % m/s
DRIFT_STD = 0.05;   % the true speed wanders a little each step, m
MEAS_STD = 1;       % sensor noise σ, m
ACCEL_STD = 0.1;    % the filter's process noise: how much it expects the speed to change, m/s²
rng(42)
%%
%[text] ## Step 1: Simulate the robot and the sensor
[truePos, z] = simulateStraight1D(NUM_STEPS, SPEED, DT, DRIFT_STD, MEAS_STD);
fprintf("The robot drove %.1f m in %d s.\n", truePos(end), NUM_STEPS * DT);
%%
%[text] ## Step 2: Create the filter
%[text] The state is $x = [p, v]^T$: position and velocity. With `MotionModel="1D Constant Velocity"`, `trackingKF` builds the lecture's model for you:
%[text] - motion model `F = [1 Δt; 0 1]`: the position moves by $v \\cdot \\Delta t$ each step
%[text] - measurement model $H = [1, 0]$: the sensor sees the position only
%[text] - process noise $Q$ from `ProcessNoise` (the acceleration variance), measurement noise $R = \\sigma^2$ \
%[text] We start at $p = 0, v = 0$ with a large covariance (25), because we do not know the speed yet.
kf = trackingKF(MotionModel="1D Constant Velocity", State=[0; 0], StateCovariance=diag([25 25]), ...
    ProcessNoise=ACCEL_STD^2, MeasurementNoise=MEAS_STD^2)
%%
%[text] ## Step 3: Run the filter
%[text] Every second: `predict` moves the state forward by $\\Delta t$ and grows $P$; `correct` pulls the estimate towards the reading and shrinks $P$.
[est, sd] = runFilter(kf, z, DT);
%%
%[text] ## Step 4: Judge the filter
%[text] Two checks a data scientist would make: is the filter more accurate than the raw sensor, and is its ±2σ band honest (about 95% of the true errors inside it)?
fprintf("Position error (RMSE):  sensor alone %.2f m,  Kalman filter %.2f m\n", rmse(z, truePos), rmse(est, truePos));
inside = mean(abs(est - truePos) <= 2 * sd);
fprintf("True position inside the ±2σ band: %.0f%% of the time (an honest filter: about 95%%)\n", 100 * inside);
%%
%[text] ## Step 5: Plot
t = (1:NUM_STEPS)' * DT;
fig = figure(Position=[100 100 1100 750]);
lightTheme(fig);
tiledlayout(fig, 2, 1, TileSpacing="compact");
nexttile
fill([t; flipud(t)], [est - 2*sd; flipud(est + 2*sd)], [0.3 0.3 1], FaceAlpha=0.2, EdgeColor="none", DisplayName="±2σ band");
hold on
plot(t, truePos, "g-", LineWidth=2, DisplayName="True position");
scatter(t, z, 18, "r", "filled", MarkerFaceAlpha=0.5, DisplayName="Noisy measurements");
plot(t, est, "b-", LineWidth=2, DisplayName="Kalman filter");
hold off
xlabel("Time (s)"); ylabel("Position (m)"); grid on; legend(Location="northwest");
title("Activity 1: 1D constant-velocity Kalman filter");
nexttile
fill([t; flipud(t)], [-2*sd; flipud(2*sd)], [0.3 0.3 1], FaceAlpha=0.2, EdgeColor="none", DisplayName="±2σ band");
hold on
plot(t, est - truePos, "k-", LineWidth=1.5, DisplayName="Kalman filter error");
yline(0, "g-", HandleVisibility="off");
hold off
xlabel("Time (s)"); ylabel("Error: estimate − true (m)"); grid on; legend(Location="northeast");
title("Estimation error over time, inside the filter's own ±2σ band");
exportgraphics(fig, "activity1_straight_1d.png", Resolution=200);
disp("Saved activity1_straight_1d.png")
%%
%[text] ## Local functions
function [truePos, z] = simulateStraight1D(numSteps, speed, dt, driftStd, measStd)
% True position: constant speed plus a small random wander. z: noisy position readings.
truePos = cumsum(speed * dt + driftStd * randn(numSteps, 1));
z = truePos + measStd * randn(numSteps, 1);
end
function [est, sd] = runFilter(kf, z, dt)
% Runs predict + correct for every reading. Returns the position estimate and its σ.
kf = clone(kf);   % work on a copy: the filter in the workspace stays at its start state
est = zeros(size(z));
sd = zeros(size(z));
for k = 1:numel(z)
    predict(kf, dt);                % predict: x⁻ = F·x,  P⁻ = F·P·Fᵀ + Q
    [x, P] = correct(kf, z(k));     % update:  K = P⁻Hᵀ(HP⁻Hᵀ + R)⁻¹,  x = x⁻ + K(z − Hx⁻)
    est(k) = x(1);
    sd(k) = sqrt(P(1, 1));
end
end
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
