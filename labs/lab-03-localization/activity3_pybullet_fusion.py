"""
Activity 3 — Odometry + GPS in a Straight Aisle (PyBullet)
Version 3.0 — follows Lecture 2 and Module 2 (sections 3 and 7)

=== Mission Context ===
The R2D2 robot drives 60 m down a straight warehouse aisle at 1 m/s.
It has TWO sensors, and they fail in opposite ways:
  - Odometry (wheel encoders), 10 Hz: smooth, but it over-reads by 0.1 m/s,
    so its position DRIFTS (about 6 m off after 60 s).
  - GPS, 1 Hz: never drifts, but every fix is NOISY (about 3 m off, σ = 3 m).

You will compare four ways to estimate the robot's position:
  1. Odometry alone   (dead reckoning: add up the reported motion)
  2. GPS alone        (hold the last fix)
  3. The gated reset  (follow odometry; jump to a GPS fix that lies outside a gate)
  4. The Kalman filter (predict with odometry, update with GPS)

Then you change GATE, Q and GPS_STD in the EXPERIMENT block and explain what
you see (see the questions in README.md).

Learning goals:
1) See why the gated reset can never beat GPS: each jump copies one noisy fix.
2) See how the Kalman filter moves only part of the way (K) towards each fix.
3) Tune Q, and see the uncertainty sawtooth: P grows every step, shrinks at every fix.
"""
import numpy as np
import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data

# ============================================================
# EXPERIMENT — change these and re-run
# ============================================================
GPS_STD = 3.0   # GPS noise σ in metres (try 1.5 and 6)
GATE = 6.0      # gated reset: jump when the GPS fix is more than GATE metres away (try 2, 4, 10)
Q = 0.03        # Kalman filter: odometry's drift per step, in m² (try 0.0001, 0.003, 0.3, 3)
RUNS = 20       # every number is averaged over RUNS runs: one run is mostly luck

# The scenario (keep these fixed)
DT = 0.1          # odometry period: 10 Hz
STEPS = 600       # 60 s
SPEED = 1.0       # commanded speed, m/s
ODOM_BIAS = 0.1   # odometry over-reads by 0.1 m/s → drift
ODOM_STD = 0.05   # odometry noise, m/s
GPS_EVERY = 10    # one GPS fix every 10 steps: 1 Hz


# ============================================================
# STEP 1: PyBullet — the TRUE motion of the robot
# ============================================================
def simulate_aisle_truth(steps=STEPS, dt=DT, speed=SPEED):
    """
    Drives R2D2 along the x axis in PyBullet and returns its true position
    (metres from the start) after each 0.1 s step.

    PyBullet's physics step is 1/240 s, so each 0.1 s step runs 24 physics steps.
    """
    p.connect(p.DIRECT)  # headless: we only need the numbers
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.loadURDF("plane.urdf")
    robot = p.loadURDF("r2d2.urdf", [0, 0, 0.5], p.getQuaternionFromEuler([0, 0, 0]))

    physics_dt = 1.0 / 240.0
    p.setTimeStep(physics_dt)
    substeps = round(dt / physics_dt)

    # Let the robot settle on the floor before it starts
    for _ in range(240):
        p.stepSimulation()
    x0 = p.getBasePositionAndOrientation(robot)[0][0]

    truth = np.zeros(steps)
    for i in range(steps):
        for _ in range(substeps):
            # Velocity command: drive straight along x at `speed`
            p.resetBaseVelocity(robot, linearVelocity=[speed, 0, 0], angularVelocity=[0, 0, 0])
            p.stepSimulation()
        truth[i] = p.getBasePositionAndOrientation(robot)[0][0] - x0
    p.disconnect()
    return truth


# ============================================================
# STEP 2: The two sensors
# ============================================================
def make_sensors(truth, rng, gps_std=GPS_STD, dt=DT):
    """
    odom_vel[i]: the velocity odometry reports during step i (true velocity + bias + noise)
    gps[i]:      a GPS position at every 10th step, NaN in between
    """
    true_vel = np.diff(truth, prepend=0.0) / dt
    odom_vel = true_vel + ODOM_BIAS + rng.normal(0, ODOM_STD, len(truth))
    gps = np.full(len(truth), np.nan)
    fixes = np.arange(GPS_EVERY - 1, len(truth), GPS_EVERY)  # t = 1 s, 2 s, ...
    gps[fixes] = truth[fixes] + rng.normal(0, gps_std, len(fixes))
    return odom_vel, gps


# ============================================================
# STEP 3: Four ways to estimate the position
# ============================================================
def dead_reckoning(odom_vel, dt=DT):
    """1. Odometry alone: add up the reported motion. It drifts."""
    return np.cumsum(odom_vel * dt)


def gps_hold(gps):
    """2. GPS alone: hold the last fix until the next one (0 before the first)."""
    est = np.zeros(len(gps))
    last = 0.0
    for i, g in enumerate(gps):
        if not np.isnan(g):
            last = g
        est[i] = last
    return est


def gated_reset(odom_vel, gps, gate, dt=DT):
    """
    3. The gated reset (Lecture 2's first idea):
       follow odometry; at each GPS fix, if the fix is more than `gate` metres
       from the estimate, JUMP to the fix and carry on from there.
    Returns the estimate and the steps at which it jumped.
    """
    x = 0.0
    est = np.zeros(len(odom_vel))
    jumps = []
    for i in range(len(odom_vel)):
        x += odom_vel[i] * dt                 # follow odometry
        if not np.isnan(gps[i]) and abs(gps[i] - x) > gate:
            x = gps[i]                        # all or nothing: copy the fix
            jumps.append(i)
        est[i] = x
    return est, jumps


def kalman_filter(odom_vel, gps, Q, gps_std, dt=DT):
    """
    4. The Kalman filter of Lecture 2, with odometry as the control input:
         predict (every step):  x̂⁻ = x + u,  u = v_odom · Δt,   P⁻ = P + Q
         update  (GPS fix):     K = P⁻ / (P⁻ + R),   R = σ_GPS²
                                x̂ = x̂⁻ + K · (z − x̂⁻),   P = (1 − K) · P⁻
    Returns the estimate, the variance P after each step, and K at each fix.
    """
    R = gps_std ** 2
    x, P = 0.0, 0.01                          # we know where the robot starts
    est, var, gains = np.zeros(len(odom_vel)), np.zeros(len(odom_vel)), []
    for i in range(len(odom_vel)):
        # ======== PREDICT with odometry ========
        x = x + odom_vel[i] * dt
        P = P + Q
        # ======== UPDATE with GPS (only when a fix arrives) ========
        if not np.isnan(gps[i]):
            K = P / (P + R)                   # how far to move towards the fix
            x = x + K * (gps[i] - x)          # move only part of the way
            P = (1 - K) * P
            gains.append((i, K))
        est[i], var[i] = x, P
    return est, var, gains


def rmse(est, truth):
    return float(np.sqrt(np.mean((est - truth) ** 2)))


def max_jump(est):
    """Largest change of the estimate in one step (the robot really moves 0.1 m)."""
    return float(np.max(np.abs(np.diff(est))))


# ============================================================
# STEP 4: Score every method over RUNS runs
# ============================================================
def score(truth, gate, Q, gps_std, runs=RUNS):
    """Mean RMSE (and mean largest jump) of each method over `runs` runs."""
    rows = {"odometry": [], "gps": [], "reset": [], "kalman": [], "reset_jump": [], "kalman_jump": []}
    for seed in range(1, runs + 1):
        odom_vel, gps = make_sensors(truth, np.random.default_rng(seed), gps_std)
        reset, _ = gated_reset(odom_vel, gps, gate)
        kf, _, _ = kalman_filter(odom_vel, gps, Q, gps_std)
        rows["odometry"].append(rmse(dead_reckoning(odom_vel), truth))
        rows["gps"].append(rmse(gps_hold(gps), truth))
        rows["reset"].append(rmse(reset, truth))
        rows["kalman"].append(rmse(kf, truth))
        rows["reset_jump"].append(max_jump(reset))
        rows["kalman_jump"].append(max_jump(kf))
    return {k: float(np.mean(v)) for k, v in rows.items()}


# ============================================================
# STEP 5: Plots
# ============================================================
def plot_results(truth, gate, Q, gps_std, sweep):
    """Run 1 over time (A, B, C) and the gate sweep (D)."""
    odom_vel, gps = make_sensors(truth, np.random.default_rng(1), gps_std)
    odom = dead_reckoning(odom_vel)
    reset, jumps = gated_reset(odom_vel, gps, gate)
    kf, var, _ = kalman_filter(odom_vel, gps, Q, gps_std)
    t = (np.arange(len(truth)) + 1) * DT
    fix = ~np.isnan(gps)
    lim = max(10, 3.5 * gps_std)

    fig, axes = plt.subplots(2, 2, figsize=(15, 11))

    def error_axes(ax, title):
        ax.axhline(0, color="green", lw=2, label="True position (error 0)")
        ax.plot(t, odom - truth, color="orange", lw=1.5, label="Odometry alone")
        ax.scatter(t[fix], (gps - truth)[fix], color="red", marker="x", s=30, zorder=3, label="GPS fixes")
        ax.set_title(title, fontsize=13, fontweight="bold")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Error: estimate − true position (m)")
        ax.set_ylim(-lim, lim)
        ax.grid(True, alpha=0.3)

    # --- A. The gated reset ---
    ax = axes[0, 0]
    error_axes(ax, f"A. Gated reset (gate {gate:g} m): {len(jumps)} jumps")
    ax.plot(t, reset - truth, color="tab:blue", lw=2, label="Gated reset")
    ax.scatter(t[jumps], (reset - truth)[jumps], facecolors="none", edgecolors="tab:blue", s=90, zorder=4, label="Jumps")
    ax.legend(loc="upper left", fontsize=9)

    # --- B. The Kalman filter ---
    ax = axes[0, 1]
    error_axes(ax, f"B. Kalman filter (Q = {Q:g} m²)")
    ax.plot(t, reset - truth, color="tab:blue", lw=1, ls="--", alpha=0.6, label="Gated reset")
    ax.plot(t, kf - truth, color="purple", lw=2.5, label="Kalman filter")
    ax.legend(loc="upper left", fontsize=9)

    # --- C. The uncertainty sawtooth ---
    ax = axes[1, 0]
    ax.plot(t, var, color="purple", lw=2, label="Variance P (m²)")
    ax.scatter(t[fix], var[fix], color="red", marker="v", s=40, zorder=3, label="GPS update")
    ax.set_xlim(0, 20)  # the first 20 s: the pattern repeats after that
    ax.set_title("C. Uncertainty Sawtooth (first 20 s)", fontsize=13, fontweight="bold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("P (m²)")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.3)

    # --- D. No gate beats GPS ---
    ax = axes[1, 1]
    gates = [g for g, _ in sweep]
    ax.plot(gates, [s["reset"] for _, s in sweep], "o-", color="tab:blue", lw=2, label="Gated reset")
    ax.axhline(sweep[0][1]["gps"], color="red", ls="--", label="GPS alone")
    ax.axhline(sweep[0][1]["odometry"], color="orange", ls="--", label="Odometry alone")
    ax.axhline(sweep[0][1]["kalman"], color="purple", ls="-", lw=2, label=f"Kalman filter (Q = {Q:g})")
    ax.set_title(f"D. Mean Error over {RUNS} Runs vs Gate", fontsize=13, fontweight="bold")
    ax.set_xlabel("Gate (m)")
    ax.set_ylabel("RMSE (m)")
    ax.set_ylim(0, None)
    ax.legend(loc="lower right", fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.suptitle(f"Activity 3: Odometry + GPS in a 60 m Aisle (GPS σ = {gps_std:g} m)", fontsize=15, fontweight="bold")
    plt.tight_layout()
    plt.savefig("activity3_pybullet_fusion.png", dpi=200, bbox_inches="tight")
    print("Saved activity3_pybullet_fusion.png")


# ============================================================
# MAIN
# ============================================================
def main():
    print("Running PyBullet: R2D2 drives 60 m down a straight aisle...")
    truth = simulate_aisle_truth()
    print(f"  The robot drove {truth[-1]:.1f} m in {STEPS * DT:.0f} s.")

    s = score(truth, GATE, Q, GPS_STD)
    print("\n" + "=" * 62)
    print(f"  MEAN ERROR (RMSE) OVER {RUNS} RUNS   GPS σ = {GPS_STD:g} m")
    print("=" * 62)
    print(f"  1. Odometry alone:                {s['odometry']:6.2f} m")
    print(f"  2. GPS alone (hold last fix):     {s['gps']:6.2f} m")
    print(f"  3. Gated reset (gate {GATE:g} m):      {s['reset']:6.2f} m   largest jump {s['reset_jump']:5.1f} m")
    print(f"  4. Kalman filter (Q = {Q:g} m²):  {s['kalman']:6.2f} m   largest jump {s['kalman_jump']:5.1f} m")
    print("=" * 62)

    # K at the fixes of run 1: small at first, then it settles
    odom_vel, gps = make_sensors(truth, np.random.default_rng(1), GPS_STD)
    _, _, gains = kalman_filter(odom_vel, gps, Q, GPS_STD)
    ks = [k for _, k in gains]
    print(f"\n  Kalman gain K: first fix {ks[0]:.3f}, 10th fix {ks[9]:.3f}, last fix {ks[-1]:.3f}")
    print(f"  → each GPS fix moves the estimate only {ks[-1]:.0%} of the way towards it.")

    print("\n  Gate sweep (gated reset, mean over runs):")
    sweep = []
    for g in [1, 2, 3, 4, 5, 6, 8, 10, 12, 15]:
        sg = score(truth, g, Q, GPS_STD)
        sweep.append((g, sg))
        print(f"    gate {g:4.1f} m → {sg['reset']:.2f} m")
    print(f"  GPS alone is {s['gps']:.2f} m: no gate gets far below it.")

    plot_results(truth, GATE, Q, GPS_STD, sweep)


if __name__ == "__main__":
    main()
