"""
Activity 2 — Kalman Filter in 2D: Working with Matrices (PyBullet)
Version 4.0 — follows Lecture 2 (slides 13–18) and Module 2, section 5

The state is now the 2D position x = [x, y]ᵀ, so x, P, F, B, Q, H, R and K
become vectors and matrices. The equations are exactly the ones from the lecture:

    predict (every step):   x̂⁻ = F·x + B·u          P⁻ = F·P·Fᵀ + Q
    update  (sensor fix):   K  = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
                            x̂  = x̂⁻ + K·(z − H·x̂⁻)   P  = (I − K·H)·P⁻

  - u = [Δx, Δy]: the displacement odometry reports (the control input), so F = I and B = I
  - GPS measures both coordinates:         H = I,       R = σ²·I
  - A wall sensor measures only x:         H = [1, 0],  R = σ_wall²

Part A works through the lecture's example by hand, matrix by matrix.
Part B runs the same filter on R2D2 driving an L-shaped path in PyBullet.
Change the EXPERIMENT block and re-run to answer the questions in README.md.
"""
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import pybullet_data
from matplotlib.patches import Ellipse

# ============================================================
# EXPERIMENT — change these and re-run
# ============================================================
SENSOR = "gps"     # "gps" (measures x and y) or "wall" (measures x only)
GPS_STD = 1.5      # GPS noise σ, m
WALL_STD = 0.5     # wall-sensor noise σ, m
Q_ALONG = 0.004    # odometry's error per step ALONG the direction of travel, m²
Q_ACROSS = 0.0005  # ... and ACROSS it (wheels slip mostly along the travel direction)
RUNS = 20          # RMSE is averaged over RUNS runs: one run is mostly luck
USE_GUI = "google.colab" not in sys.modules  # watch R2D2 drive (not available on Colab)

# The scenario (keep these fixed)
DT = 0.1           # odometry period: 10 Hz
SPEED = 1.0        # m/s
LEG_STEPS = 150    # each leg of the L lasts 15 s
FIX_EVERY = 10     # one sensor fix every 10 steps: 1 Hz
ODOM_SCALE = 1.08  # odometry over-reads the distance travelled by 8% → drift
ODOM_STD = 0.01    # odometry noise per step and axis, m
# First leg along u = [1, 0.5] (the lecture's example), then a 90° left turn
LEG1 = np.array([2.0, 1.0]) / np.sqrt(5.0)
LEG2 = np.array([-1.0, 2.0]) / np.sqrt(5.0)


# ============================================================
# The Kalman filter in matrix form (any H, any R)
# ============================================================
def kf_predict(x, P, u, F, B, Q):
    x = F @ x + B @ u
    P = F @ P @ F.T + Q
    return x, P


def kf_update(x, P, z, H, R):
    S = H @ P @ H.T + R                    # innovation covariance
    K = P @ H.T @ np.linalg.inv(S)         # Kalman gain
    x = x + K @ (z - H @ x)                # move part of the way towards the reading
    P = (np.eye(len(x)) - K @ H) @ P
    return x, P, K


def q_matrix(direction, q_along=Q_ALONG, q_across=Q_ACROSS):
    """Q with a large variance along the direction of travel and a small one across it.
    When the robot drives diagonally, this gives Q an off-diagonal term."""
    d = direction / np.linalg.norm(direction)
    n = np.array([-d[1], d[0]])
    return q_along * np.outer(d, d) + q_across * np.outer(n, n)


def sensor_model(sensor):
    if sensor == "gps":
        return np.eye(2), np.eye(2) * GPS_STD**2
    if sensor == "wall":
        return np.array([[1.0, 0.0]]), np.array([[WALL_STD**2]])
    raise ValueError("SENSOR must be 'gps' or 'wall'")


# ============================================================
# PART A: the lecture's example, by hand
# ============================================================
def show(name, m):
    rows = np.atleast_2d(m)
    text = np.array2string(rows, precision=4, suppress_small=True, floatmode="maxprec")
    print(f"  {name} =\n" + "\n".join("      " + line for line in text.splitlines()))


def part_a():
    print("=" * 64)
    print("PART A — The lecture's 2D example (Module 2, section 5)")
    print("=" * 64)
    x = np.array([3.0, 2.5])                          # where the robot was
    P = np.array([[1.5, 0.75], [0.75, 1.5]])          # its uncertainty
    u = np.array([1.0, 0.5])                          # odometry: it moved 1 m in x, 0.5 m in y
    F, B = np.eye(2), np.eye(2)
    Q = np.array([[0.5, 0.25], [0.25, 0.5]])          # slip mostly along the travel direction

    print("\n1) PREDICT with odometry:  x̂⁻ = F·x + B·u,   P⁻ = F·P·Fᵀ + Q")
    x_pred, P_pred = kf_predict(x, P, u, F, B, Q)
    show("x̂⁻", x_pred)
    show("P⁻", P_pred)

    print("\n2) UPDATE with GPS:  z = [5, 2.5],  H = I,  R = I (σ = 1 m)")
    z, H, R = np.array([5.0, 2.5]), np.eye(2), np.eye(2)
    show("S = H·P⁻·Hᵀ + R", H @ P_pred @ H.T + R)
    x_gps, P_gps, K = kf_update(x_pred, P_pred, z, H, R)
    show("K", K)
    show("innovation z − H·x̂⁻", z - H @ x_pred)
    show("x̂", x_gps)
    show("P", P_gps)

    print("\n3) UPDATE with a wall sensor instead:  z = [5],  H = [1, 0],  R = [0.25] (σ = 0.5 m)")
    z, H, R = np.array([5.0]), np.array([[1.0, 0.0]]), np.array([[0.25]])
    x_wall, P_wall, K = kf_update(x_pred, P_pred, z, H, R)
    show("K", K)
    show("x̂", x_wall)
    show("P", P_wall)
    print("  The sensor measured only x, yet y changed too (3 → %.3f m): P⁻ has an off-diagonal" % x_wall[1])
    print("  term, so errors in x and y move together. With P⁻₁₂ = 0, y would not change.\n")


# ============================================================
# PART B: R2D2 in PyBullet
# ============================================================
def simulate_truth(use_gui=USE_GUI):
    """Drives R2D2 along an L-shaped path and returns its true x-y position after each step."""
    p.connect(p.GUI if use_gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    physics_dt = 1.0 / 240.0
    p.setTimeStep(physics_dt)
    substeps = round(DT / physics_dt)          # 24 physics steps per 0.1 s step
    if use_gui:
        p.resetDebugVisualizerCamera(cameraDistance=18, cameraYaw=0, cameraPitch=-70, cameraTargetPosition=[6, 9, 0])
    p.loadURDF("plane.urdf")
    robot = p.loadURDF("r2d2.urdf", [0, 0, 0.5], p.getQuaternionFromEuler([0, 0, 0]))
    for _ in range(240):                       # let it settle on the floor
        p.stepSimulation()
    start = np.array(p.getBasePositionAndOrientation(robot)[0][:2])

    truth = np.zeros((2 * LEG_STEPS, 2))
    for k in range(2 * LEG_STEPS):
        v = SPEED * (LEG1 if k < LEG_STEPS else LEG2)
        for _ in range(substeps):
            p.resetBaseVelocity(robot, linearVelocity=[v[0], v[1], 0], angularVelocity=[0, 0, 0])
            p.stepSimulation()
            if use_gui:
                time.sleep(physics_dt)
        truth[k] = np.array(p.getBasePositionAndOrientation(robot)[0][:2]) - start
    p.disconnect()
    return truth


def make_sensors(truth, rng, sensor=SENSOR):
    """odometry[k]: displacement reported for step k.  fixes[k]: a reading every 10th step, else None."""
    true_disp = np.diff(truth, axis=0, prepend=np.zeros((1, 2)))
    odometry = true_disp * ODOM_SCALE + rng.normal(0, ODOM_STD, truth.shape)
    H, R = sensor_model(sensor)
    fixes = [None] * len(truth)
    for k in range(FIX_EVERY - 1, len(truth), FIX_EVERY):
        fixes[k] = H @ truth[k] + rng.multivariate_normal(np.zeros(len(R)), R)
    return odometry, fixes


def run_filter(odometry, fixes, sensor=SENSOR):
    F, B = np.eye(2), np.eye(2)
    H, R = sensor_model(sensor)
    x, P = np.zeros(2), np.eye(2) * 0.01      # we know where the robot starts
    est, covs = np.zeros((len(odometry), 2)), []
    for k, u in enumerate(odometry):
        x, P = kf_predict(x, P, u, F, B, q_matrix(u if np.linalg.norm(u) > 0 else LEG1))
        if fixes[k] is not None:
            x, P, _ = kf_update(x, P, fixes[k], H, R)
        est[k] = x
        covs.append(P.copy())
    return est, covs


def rmse(est, truth):
    return float(np.sqrt(np.mean(np.sum((est - truth) ** 2, axis=1))))


def add_ellipse(ax, P, mean, color, n_std=2.0):
    """2σ ellipse of a 2×2 covariance matrix."""
    vals, vecs = np.linalg.eigh(P)
    angle = np.degrees(np.arctan2(vecs[1, 1], vecs[0, 1]))
    w, h = 2 * n_std * np.sqrt(np.maximum(vals[::-1], 0))
    ax.add_patch(Ellipse(mean, w, h, angle=angle, facecolor=color, edgecolor=color, alpha=0.18, lw=1.2))


def part_b():
    print("=" * 64)
    print(f"PART B — R2D2 on an L-shaped path, sensor = {SENSOR}")
    print("=" * 64)
    truth = simulate_truth()
    print(f"  The robot drove {np.sum(np.linalg.norm(np.diff(truth, axis=0), axis=1)):.1f} m.")

    scores = {"odometry": [], "gps": [], "kalman": []}
    for seed in range(1, RUNS + 1):
        odometry, fixes = make_sensors(truth, np.random.default_rng(seed))
        est, _ = run_filter(odometry, fixes)
        scores["odometry"].append(rmse(np.cumsum(odometry, axis=0), truth))
        scores["kalman"].append(rmse(est, truth))
        if SENSOR == "gps":
            held, last = np.zeros_like(truth), np.zeros(2)
            for k, f in enumerate(fixes):
                last = f if f is not None else last
                held[k] = last
            scores["gps"].append(rmse(held, truth))
    print(f"\n  Mean position error (RMSE) over {RUNS} runs:")
    print(f"    Odometry alone:   {np.mean(scores['odometry']):.2f} m")
    if SENSOR == "gps":
        print(f"    GPS alone:        {np.mean(scores['gps']):.2f} m")
    print(f"    Kalman filter:    {np.mean(scores['kalman']):.2f} m")

    # Run 1 for the plots
    odometry, fixes = make_sensors(truth, np.random.default_rng(1))
    odom_path = np.cumsum(odometry, axis=0)
    est, covs = run_filter(odometry, fixes)
    # The step just before a fix halfway along the first leg
    k_mid = next(k for k in range(LEG_STEPS // 2, LEG_STEPS) if fixes[k] is not None) - 1
    print("\n  Q on the first leg (driving along [1, 0.5]):")
    show("Q", q_matrix(LEG1))
    print("  P just before a fix halfway along the first leg:")
    show("P⁻", covs[k_mid])
    print("  → the ellipse is stretched along the direction of travel.")

    fig, axes = plt.subplots(1, 2, figsize=(16, 7.5))
    ax = axes[0]
    ax.plot(truth[:, 0], truth[:, 1], "g-", lw=3, label="True path (PyBullet)")
    ax.plot(odom_path[:, 0], odom_path[:, 1], color="orange", lw=1.8, label="Odometry alone")
    if SENSOR == "gps":
        pts = np.array([f for f in fixes if f is not None])
        ax.scatter(pts[:, 0], pts[:, 1], c="red", marker="x", s=40, label="GPS fixes")
    else:
        for k, f in enumerate(fixes):
            if f is not None:
                ax.axvline(f[0], ymin=0, ymax=1, color="red", alpha=0.08)
        ax.plot([], [], color="red", alpha=0.4, label="Wall-sensor readings (x only)")
    ax.plot(est[:, 0], est[:, 1], color="purple", lw=2.2, label="Kalman filter")
    for k in range(FIX_EVERY - 2, len(est), 2 * FIX_EVERY):
        add_ellipse(ax, covs[k], est[k], "purple")
    ax.set_title(f"A. 2D Kalman filter with {'GPS' if SENSOR == 'gps' else 'a wall sensor (x only)'}", fontsize=13, fontweight="bold")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)

    ax = axes[1]
    t = (np.arange(len(est)) + 1) * DT
    ax.plot(t, [np.sqrt(c[0, 0]) for c in covs], label="σx = √P₁₁")
    ax.plot(t, [np.sqrt(c[1, 1]) for c in covs], label="σy = √P₂₂")
    ax.plot(t, [c[0, 1] for c in covs], ls="--", label="P₁₂ (x–y covariance)")
    ax.axvline(LEG_STEPS * DT, color="gray", ls=":", label="The robot turns")
    ax.set_title("B. Uncertainty over time", fontsize=13, fontweight="bold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("m  (P₁₂ in m²)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)

    plt.suptitle("Activity 2: Kalman Filter in 2D — Odometry Predicts, the Sensor Updates", fontsize=15, fontweight="bold")
    plt.tight_layout()
    plt.savefig("activity2_pybullet_2d.png", dpi=200, bbox_inches="tight")
    print("\nSaved activity2_pybullet_2d.png")


def main():
    part_a()
    part_b()
    plt.show()  # open the plot window (in Colab it appears under the cell)


if __name__ == "__main__":
    main()
