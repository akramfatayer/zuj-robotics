%[text] # Lab 3 · Activity 3: Odometry + GPS in a Straight Aisle
%[text] Intelligent Mobile Robots (0135343) · **MATLAB track** · follows Lecture 2 and sections 3 and 7 of [Module 2](https://akramfatayer.github.io/courses/robotics/learn/2)
%[text] The robot drives 60 m down a straight warehouse aisle at 1 m/s. It has two sensors, and they fail in opposite ways:
%[text] - **Odometry** (wheel encoders), 10 Hz: smooth, but it over-reads by 0.1 m/s, so its position **drifts** (about 6 m off after 60 s)
%[text] - **GPS**, 1 Hz: never drifts, but every fix is **noisy** (about 3 m off, σ = 3 m) \
%[text] You will compare four ways to estimate the position, each averaged over 20 runs:
%[text] 1. **Odometry alone** (dead reckoning: add up the reported motion)
%[text] 2. **GPS alone** (hold the last fix)
%[text] 3. **The gated reset** from Lecture 2: follow odometry, and jump to a GPS fix when it lies outside a gate
%[text] 4. **The Kalman filter**: `trackingKF` predicts with odometry (the control input) and corrects with GPS \
%[text] Then change `GATE`, `Q` and `GPS_STD` below and explain what you see (the questions are in `README.md`).
%%
%[text] ## Experiment
%[text] Change these values and run the script again.
GPS_STD = 3;   % GPS noise σ, m (try 1.5 and 6)
GATE = 6;      % gated reset: jump when the GPS fix is more than GATE metres away (try 2, 4, 10)
Q = 0.03;      % Kalman filter: odometry's drift per step, m² (try 0.0001, 0.003, 0.3, 3)
RUNS = 20;     % every number is averaged over RUNS runs: one run is mostly luck
%%
%[text] ## The scenario
%[text] Keep these fixed.
DT = 0.1;          % odometry period: 10 Hz
STEPS = 600;       % 60 s
SPEED = 1;         % m/s
ODOM_BIAS = 0.1;   % odometry over-reads by 0.1 m/s → drift
ODOM_STD = 0.05;   % odometry noise, m/s
GPS_EVERY = 10;    % one GPS fix every 10 steps: 1 Hz
truth = SPEED * DT * (1:STEPS)';   % true position after each 0.1 s step
sim = struct(dt=DT, bias=ODOM_BIAS, odomStd=ODOM_STD, gpsEvery=GPS_EVERY);
%%
%[text] ## The Kalman filter
%[text] A 1D `trackingKF` with odometry as the control input. Every step, `predict(kf, u)` with $u = v_{odom} \\cdot \\Delta t$ computes $\\hat{x}^- = \\hat{x} + u$ and $P^- = P + Q$. At each GPS fix, `correct(kf, z)` moves the estimate part of the way towards it:
%[text]{"align":"center"} $K = \\frac{P^-}{P^- + R}, \\quad \\hat{x} = \\hat{x}^- + K (z - \\hat{x}^-), \\quad P = (1 - K) P^-, \\quad R = \\sigma_{GPS}^2$
kf = trackingKF(MotionModel="Custom", StateTransitionModel=1, ControlModel=1, MeasurementModel=1, ...
    ProcessNoise=Q, MeasurementNoise=GPS_STD^2, State=0, StateCovariance=0.01)   % we know where the robot starts
%%
%[text] ## Score every method over RUNS runs
s = score(truth, kf, GATE, GPS_STD, RUNS, sim);
fprintf("MEAN ERROR (RMSE) OVER %d RUNS, GPS σ = %g m\n", RUNS, GPS_STD);
fprintf("  1. Odometry alone:                %6.2f m\n", s.odometry);
fprintf("  2. GPS alone (hold last fix):     %6.2f m\n", s.gps);
fprintf("  3. Gated reset (gate %g m):       %6.2f m   largest jump %5.1f m\n", GATE, s.reset, s.resetJump);
fprintf("  4. Kalman filter (Q = %g m²):   %6.2f m   largest jump %5.1f m\n", Q, s.kalman, s.kalmanJump);
%[text] The Kalman gain at the fixes of run 1: small at first, then it settles.
rng(1)
[odomVel, gps] = makeSensors(truth, GPS_STD, sim);
[kfEst, kfVar, gains] = runKalman(kf, odomVel, gps, DT);
fprintf("Kalman gain K: first fix %.3f, 10th fix %.3f, last fix %.3f\n", gains(1), gains(10), gains(end));
fprintf("→ each GPS fix moves the estimate only %.0f%% of the way towards it.\n", 100 * gains(end));
%[text] The gate sweep: the gated reset's error for every gate.
gates = [1 2 3 4 5 6 8 10 12 15];
sweep = zeros(size(gates));
for i = 1:numel(gates)
    si = score(truth, kf, gates(i), GPS_STD, RUNS, sim);
    sweep(i) = si.reset;
end
disp(table(gates', sweep', VariableNames=["Gate (m)", "Gated reset RMSE (m)"]))
fprintf("GPS alone is %.2f m: no gate gets far below it.\n", s.gps);
%%
%[text] ## Plot
%[text] A and B show run 1 as the error over time (estimate − truth). C zooms in on the variance P. D is the gate sweep.
odom = deadReckoning(odomVel, DT);
[reset, jumps] = gatedReset(odomVel, gps, GATE, DT);
t = (1:STEPS)' * DT;
fix = ~isnan(gps);
lim = max(10, 3.5 * GPS_STD);
fig = figure(Position=[100 100 1400 1000]);
lightTheme(fig);
tiledlayout(fig, 2, 2, TileSpacing="compact");
nexttile
errorAxes(t, odom, gps, truth, fix, lim);
plot(t, reset - truth, Color=[0.12 0.47 0.71], LineWidth=2, DisplayName="Gated reset");
scatter(t(jumps), reset(jumps) - truth(jumps), 90, [0.12 0.47 0.71], LineWidth=1.5, DisplayName="Jumps");
hold off
legend(Location="northwest");
title(sprintf("A. Gated reset (gate %g m): %d jumps", GATE, numel(jumps)));
nexttile
errorAxes(t, odom, gps, truth, fix, lim);
plot(t, reset - truth, "--", Color=[0.12 0.47 0.71 0.6], LineWidth=1, DisplayName="Gated reset");
plot(t, kfEst - truth, Color=[0.5 0 0.5], LineWidth=2.5, DisplayName="Kalman filter");
hold off
legend(Location="northwest");
title(sprintf("B. Kalman filter (Q = %g m²)", Q));
nexttile
plot(t, kfVar, Color=[0.5 0 0.5], LineWidth=2, DisplayName="Variance P (m²)");
hold on
scatter(t(fix), kfVar(fix), 40, "r", "v", "filled", DisplayName="GPS update");
hold off
xlim([0 20]); grid on; legend(Location="southeast");
xlabel("Time (s)"); ylabel("P (m²)");
title("C. Uncertainty sawtooth (first 20 s)");
nexttile
plot(gates, sweep, "o-", Color=[0.12 0.47 0.71], LineWidth=2, DisplayName="Gated reset");
hold on
yline(s.gps, "r--", DisplayName="GPS alone");
yline(s.odometry, "--", Color=[1 0.5 0], DisplayName="Odometry alone");
yline(s.kalman, "-", Color=[0.5 0 0.5], LineWidth=2, DisplayName=sprintf("Kalman filter (Q = %g)", Q));
hold off
ylim([0 inf]); grid on; legend(Location="southeast");
xlabel("Gate (m)"); ylabel("RMSE (m)");
title(sprintf("D. Mean error over %d runs vs gate", RUNS));
sgtitle(fig, sprintf("Activity 3: Odometry + GPS in a 60 m aisle (GPS σ = %g m)", GPS_STD), FontWeight="bold");
arrayfun(@(ax) set(ax.Toolbar, Visible="off"), findall(fig, Type="axes"));   % keep the toolbar out of the PNG
exportgraphics(fig, "activity3_odometry_gps.png", Resolution=200);
disp("Saved activity3_odometry_gps.png")
%%
%[text] ## Local functions
function [odomVel, gps] = makeSensors(truth, gpsStd, sim)
% odomVel(i): the velocity odometry reports during step i (true velocity + bias + noise).
% gps(i): a GPS position at every gpsEvery-th step, NaN in between.
trueVel = diff([0; truth]) / sim.dt;
odomVel = trueVel + sim.bias + sim.odomStd * randn(size(truth));
gps = NaN(size(truth));
fixes = sim.gpsEvery:sim.gpsEvery:numel(truth);
gps(fixes) = truth(fixes) + gpsStd * randn(numel(fixes), 1);
end
function est = deadReckoning(odomVel, dt)
% 1. Odometry alone: add up the reported motion. It drifts.
est = cumsum(odomVel * dt);
end
function est = gpsHold(gps)
% 2. GPS alone: hold the last fix until the next one (0 before the first).
est = fillmissing(gps, "previous", EndValues=0);
end
function [est, jumps] = gatedReset(odomVel, gps, gate, dt)
% 3. The gated reset: follow odometry; at each GPS fix, if the fix is more than gate metres
% from the estimate, JUMP to the fix. Returns the estimate and the steps where it jumped.
x = 0;
est = zeros(size(odomVel));
jumps = [];
for i = 1:numel(odomVel)
    x = x + odomVel(i) * dt;                    % follow odometry
    if ~isnan(gps(i)) && abs(gps(i) - x) > gate
        x = gps(i);                             % all or nothing: copy the fix
        jumps(end + 1) = i; %#ok<AGROW>
    end
    est(i) = x;
end
end
function [est, var, gains] = runKalman(kf, odomVel, gps, dt)
% 4. The Kalman filter: predict with odometry every step, correct at every GPS fix.
% Returns the estimate, the variance P after each step, and K at each fix.
kf = clone(kf);   % work on a copy: the filter in the workspace stays at its start state
est = zeros(size(odomVel));
var = zeros(size(odomVel));
gains = [];
for i = 1:numel(odomVel)
    predict(kf, odomVel(i) * dt);               % x⁻ = x + u,  P⁻ = P + Q
    if ~isnan(gps(i))
        gains(end + 1) = kf.StateCovariance / (kf.StateCovariance + kf.MeasurementNoise); %#ok<AGROW>
        correct(kf, gps(i));                    % move part of the way towards the fix
    end
    est(i) = kf.State;
    var(i) = kf.StateCovariance;
end
end
function s = score(truth, kf, gate, gpsStd, runs, sim)
% Mean RMSE (and mean largest jump) of each method over runs runs.
r = zeros(runs, 6);
for run = 1:runs
    rng(run)
    [odomVel, gps] = makeSensors(truth, gpsStd, sim);
    reset = gatedReset(odomVel, gps, gate, sim.dt);
    kfEst = runKalman(kf, odomVel, gps, sim.dt);
    r(run, :) = [rmse(deadReckoning(odomVel, sim.dt), truth), rmse(gpsHold(gps), truth), ...
        rmse(reset, truth), rmse(kfEst, truth), max(abs(diff(reset))), max(abs(diff(kfEst)))];
end
m = mean(r, 1);
s = struct(odometry=m(1), gps=m(2), reset=m(3), kalman=m(4), resetJump=m(5), kalmanJump=m(6));
end
function errorAxes(t, odom, gps, truth, fix, lim)
% The parts plots A and B share: the zero line, odometry's drift and the GPS fixes.
yline(0, "g-", LineWidth=2, DisplayName="True position (error 0)");
hold on
plot(t, odom - truth, Color=[1 0.5 0], LineWidth=1.5, DisplayName="Odometry alone");
scatter(t(fix), gps(fix) - truth(fix), 30, "r", "x", DisplayName="GPS fixes");
xlabel("Time (s)"); ylabel("Error: estimate − true position (m)");
ylim([-lim lim]); grid on;
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
