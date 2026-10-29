%[text] # Lab 3 · Activity 2: The Kalman Filter in 2D, with Matrices
%[text] Intelligent Mobile Robots (0135343) · **MATLAB track** · follows Lecture 2 (slides 13–18) and section 5 of [Module 2](https://akramfatayer.github.io/courses/robotics/learn/2)
%[text] In 2D the state is the position $[x, y]^T$, so every number in the filter becomes a vector or a matrix. Odometry is the **control input** of the predict step: `u` is the displacement it reports, so `F = I` and `B = I`. A sensor then updates the estimate:
%[text] - a GPS measures both coordinates: `H = eye(2)`, `R = σ²·eye(2)`
%[text] - a wall sensor measures only x: `H = [1 0]`, `R = σ_wall²` \
%[text] **Part A** works through the lecture's example with `trackingKF`, and opens the box to check `K` by hand. **Part B** runs the same filter while the robot drives an L-shaped path.
%%
%[text] ## Experiment
%[text] Change these values and run the script again.
SENSOR = "gps";     % "gps" (measures x and y) or "wall" (measures x only)
GPS_STD = 1.5;      % GPS noise σ, m
WALL_STD = 0.5;     % wall-sensor noise σ, m
Q_ALONG = 0.004;    % odometry's error per step ALONG the direction of travel, m²
Q_ACROSS = 0.0005;  % ... and ACROSS it (wheels slip mostly along the direction of travel)
RUNS = 20;          % the error is averaged over RUNS runs: one run is mostly luck
%%
%[text] ## Part A: the lecture's example
%[text] The robot was at `x = [3; 2.5]` with uncertainty `P`. Odometry reports that it moved `u = [1; 0.5]`. `Q` is stretched along the direction of travel, so it has an off-diagonal term.
x = [3; 2.5];
P = [1.5 0.75; 0.75 1.5];
u = [1; 0.5];
Q = [0.5 0.25; 0.25 0.5];
kf = trackingKF(MotionModel="Custom", StateTransitionModel=eye(2), ControlModel=eye(2), ...
    MeasurementModel=eye(2), ProcessNoise=Q, MeasurementNoise=eye(2), State=x, StateCovariance=P);
%[text] **1. Predict with odometry.** `predict(kf, u)` computes $\\hat{x}^- = F x + B u$ and $P^- = F P F^T + Q$.
[xPred, PPred] = predict(kf, u)
%[text] **2. Update with GPS:** `z = [5; 2.5]`, `H = eye(2)`, `R = eye(2)` (σ = 1 m). Before calling `correct`, open the box: compute the Kalman gain by hand.
z = [5; 2.5];
H = eye(2);
R = eye(2);
S = H * PPred * H' + R          % innovation covariance
K = PPred * H' / S              % Kalman gain: how far to move towards the reading
[xGps, PGps] = correct(kf, z)   % the toolbox's answer
xByHand = xPred + K * (z - H * xPred)   % the same, by hand
%[text] **3. Update with a wall sensor instead:** `z = 5`, `H = [1 0]`, `R = 0.25` (σ = 0.5 m). The measurement model is part of the filter, so the wall sensor gets its own `trackingKF`, starting from the same prediction.
kfWall = trackingKF(MotionModel="Custom", StateTransitionModel=eye(2), ControlModel=eye(2), ...
    MeasurementModel=[1 0], ProcessNoise=Q, MeasurementNoise=0.25, State=xPred, StateCovariance=PPred);
[xWall, PWall] = correct(kfWall, 5)
fprintf("The sensor measured only x, yet y changed too (3 → %.3f m).\n", xWall(2));
%[text] $P^-$ has an off-diagonal term, so errors in x and y move together. With $P^-_{12} = 0$, y would not change.
%%
%[text] ## Part B: the robot drives an L-shaped path
%[text] The robot drives 15 s along `[2 1]/√5` (the lecture's direction), turns 90° left, and drives 15 s more, at 1 m/s. Odometry runs at 10 Hz and over-reads the distance by 8%, so it drifts. The sensor gives one reading per second.
DT = 0.1;            % odometry period: 10 Hz
SPEED = 1;           % m/s
LEG_STEPS = 150;     % each leg of the L lasts 15 s
FIX_EVERY = 10;      % one sensor reading every 10 steps: 1 Hz
ODOM_SCALE = 1.08;   % odometry over-reads the distance by 8% → drift
ODOM_STD = 0.01;     % odometry noise per step and axis, m
LEG1 = [2; 1] / sqrt(5);
LEG2 = [-1; 2] / sqrt(5);
truth = lPath(LEG1, LEG2, LEG_STEPS, SPEED, DT);
[H, R] = sensorModel(SENSOR, GPS_STD, WALL_STD);
fprintf("The robot drove %.1f m. Sensor: %s.\n", 2 * LEG_STEPS * SPEED * DT, SENSOR);
%[text] Each step the filter predicts with the odometry reading. Its `ProcessNoise` is set from the direction of that reading, so $Q$ is stretched along the direction of travel.
errOdom = zeros(RUNS, 1);
errSensor = zeros(RUNS, 1);
errKf = zeros(RUNS, 1);
for run = 1:RUNS
    rng(run)
    [odometry, fixes] = makeSensors(truth, H, R, FIX_EVERY, ODOM_SCALE, ODOM_STD);
    est = runFilter(odometry, fixes, H, R, Q_ALONG, Q_ACROSS, LEG1);
    errOdom(run) = rmse2(cumsum(odometry, 2), truth);
    errKf(run) = rmse2(est, truth);
    if SENSOR == "gps"
        errSensor(run) = rmse2(fillmissing(fixes, "previous", 2, EndValues=0), truth);
    end
end
fprintf("Mean position error (RMSE) over %d runs:\n", RUNS);
fprintf("  Odometry alone:  %.2f m\n", mean(errOdom));
if SENSOR == "gps"
    fprintf("  GPS alone:       %.2f m\n", mean(errSensor));
end
fprintf("  Kalman filter:   %.2f m\n", mean(errKf));
%[text] Run 1 again, keeping the covariance after every step for the plots.
rng(1)
[odometry, fixes] = makeSensors(truth, H, R, FIX_EVERY, ODOM_SCALE, ODOM_STD);
[est, covs] = runFilter(odometry, fixes, H, R, Q_ALONG, Q_ACROSS, LEG1);
QLeg1 = qMatrix(LEG1, Q_ALONG, Q_ACROSS)
kMid = 79;   % the step just before a reading halfway along the first leg
PBeforeFix = covs(:, :, kMid)
disp("→ the ellipse is stretched along the direction of travel.")
%%
%[text] ## Plot
fig = figure(Position=[100 100 1300 600]);
lightTheme(fig);
tiledlayout(fig, 1, 2, TileSpacing="compact");
nexttile
plot(truth(1, :), truth(2, :), "g-", LineWidth=3, DisplayName="True path");
hold on
odomPath = cumsum(odometry, 2);
plot(odomPath(1, :), odomPath(2, :), Color=[1 0.5 0], LineWidth=1.8, DisplayName="Odometry alone");
hasFix = ~isnan(fixes(1, :));
if SENSOR == "gps"
    scatter(fixes(1, hasFix), fixes(2, hasFix), 40, "r", "x", DisplayName="GPS readings");
else
    xline(fixes(1, hasFix), Color=[1 0 0], Alpha=0.12, HandleVisibility="off");
    plot(NaN, NaN, Color=[1 0.6 0.6], DisplayName="Wall-sensor readings (x only)");
end
plot(est(1, :), est(2, :), Color=[0.5 0 0.5], LineWidth=2.2, DisplayName="Kalman filter");
for k = FIX_EVERY-1:2*FIX_EVERY:size(est, 2)
    [ex, ey] = ellipse2sigma(est(:, k), covs(:, :, k));
    patch(ex, ey, [0.5 0 0.5], FaceAlpha=0.15, EdgeColor=[0.5 0 0.5], HandleVisibility="off");
end
hold off
axis equal; grid on; legend(Location="northeast");
xlabel("x (m)"); ylabel("y (m)");
if SENSOR == "gps"
    title("A. 2D Kalman filter with GPS");
else
    title("A. 2D Kalman filter with a wall sensor (x only)");
end
nexttile
t = (1:size(est, 2)) * DT;
plot(t, sqrt(squeeze(covs(1, 1, :))), LineWidth=1.5, DisplayName="σx = √P11");
hold on
plot(t, sqrt(squeeze(covs(2, 2, :))), LineWidth=1.5, DisplayName="σy = √P22");
plot(t, squeeze(covs(1, 2, :)), "--", LineWidth=1.5, DisplayName="P12 (x–y covariance)");
xline(LEG_STEPS * DT, ":", Color=[0.5 0.5 0.5], DisplayName="The robot turns");
hold off
grid on; legend(Location="northwest");
xlabel("Time (s)"); ylabel("m  (P12 in m²)");
title("B. Uncertainty over time");
sgtitle(fig, "Activity 2: Kalman filter in 2D. Odometry predicts, the sensor updates", FontWeight="bold");
exportgraphics(fig, "activity2_kalman_2d.png", Resolution=200);
disp("Saved activity2_kalman_2d.png")
%%
%[text] ## Local functions
function truth = lPath(leg1, leg2, legSteps, speed, dt)
% True position (2×N, one column per 0.1 s step) along the two legs of the L.
v = [repmat(leg1 * speed, 1, legSteps), repmat(leg2 * speed, 1, legSteps)];
truth = cumsum(v * dt, 2);
end
function [H, R] = sensorModel(sensor, gpsStd, wallStd)
if sensor == "gps"
    H = eye(2);
    R = gpsStd^2 * eye(2);
elseif sensor == "wall"
    H = [1 0];
    R = wallStd^2;
else
    error("SENSOR must be ""gps"" or ""wall""")
end
end
function [odometry, fixes] = makeSensors(truth, H, R, fixEvery, odomScale, odomStd)
% odometry(:, k): the displacement reported for step k. fixes(:, k): a reading every
% fixEvery-th step, NaN in between.
trueDisp = diff([zeros(2, 1), truth], 1, 2);
odometry = trueDisp * odomScale + odomStd * randn(size(truth));
fixes = NaN(size(H, 1), size(truth, 2));
for k = fixEvery:fixEvery:size(truth, 2)
    fixes(:, k) = H * truth(:, k) + chol(R, "lower") * randn(size(H, 1), 1);
end
end
function Q = qMatrix(direction, qAlong, qAcross)
% Large variance along the direction of travel, small across it. Driving diagonally
% gives Q an off-diagonal term.
d = direction / norm(direction);
n = [-d(2); d(1)];
Q = qAlong * (d * d') + qAcross * (n * n');
end
function [est, covs] = runFilter(odometry, fixes, H, R, qAlong, qAcross, firstLeg)
kf = trackingKF(MotionModel="Custom", StateTransitionModel=eye(2), ControlModel=eye(2), ...
    MeasurementModel=H, ProcessNoise=qMatrix(firstLeg, qAlong, qAcross), MeasurementNoise=R, ...
    State=[0; 0], StateCovariance=0.01 * eye(2));   % we know where the robot starts
n = size(odometry, 2);
est = zeros(2, n);
covs = zeros(2, 2, n);
for k = 1:n
    u = odometry(:, k);
    if norm(u) > 0
        kf.ProcessNoise = qMatrix(u, qAlong, qAcross);
    end
    predict(kf, u);
    if ~isnan(fixes(1, k))
        correct(kf, fixes(:, k));
    end
    est(:, k) = kf.State;
    covs(:, :, k) = kf.StateCovariance;
end
end
function e = rmse2(est, truth)
% RMSE of the 2D position: the mean squared distance from the truth.
e = sqrt(mean(sum((est - truth).^2, 1)));
end
function [ex, ey] = ellipse2sigma(mu, P)
% Points on the 2σ ellipse of the covariance P around mu.
[V, D] = eig((P + P') / 2);
a = linspace(0, 2*pi, 60);
pts = mu + 2 * V * sqrt(max(D, 0)) * [cos(a); sin(a)];
ex = pts(1, :);
ey = pts(2, :);
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
