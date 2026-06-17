
"""
Activity 2 — 2D Kalman Filter with PyBullet Simulation (Instructor Version)
Version 3.0

Instructor goals:
1) Make the prediction step visible (prior vs posterior).
2) Keep the robot motion slow enough to watch in PyBullet.
3) Use baseline parameters that clearly show the role of Q and R.
4) Provide numerical metrics (RMSE) so students can verify performance.

State model:
    x = [x, y, vx, vy]^T
Measurement:
    z = [x, y]^T  (simulated noisy GPS)
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

import pybullet as p
import pybullet_data


# ============================================================
# Helper functions
# ============================================================
def add_ellipse(ax, Pxy, mean_xy, n_std=2.0,
                facecolor='purple', edgecolor='purple',
                alpha=0.16, lw=1.2):
    """Draw a 2D covariance ellipse from a 2x2 covariance matrix."""
    vals, vecs = np.linalg.eigh(Pxy)
    order = np.argsort(vals)[::-1]
    vals = vals[order]
    vecs = vecs[:, order]

    lam1 = max(float(vals[0]), 0.0)
    lam2 = max(float(vals[1]), 0.0)

    width = 2.0 * n_std * np.sqrt(lam1)
    height = 2.0 * n_std * np.sqrt(lam2)
    angle = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))

    e = Ellipse(
        xy=mean_xy,
        width=width,
        height=height,
        angle=angle,
        facecolor=facecolor,
        edgecolor=edgecolor,
        alpha=alpha,
        lw=lw,
    )
    ax.add_patch(e)
    return e


def compute_rmse(est, truth):
    """Root Mean Squared Euclidean position error."""
    return np.sqrt(np.mean(np.sum((est - truth) ** 2, axis=1)))


# ============================================================
# PyBullet simulation
# ============================================================
def run_pybullet_simulation(
    num_steps=60,
    dt=0.2,
    gps_noise_std_true=0.35,
    use_gui=True,
    speed=1.0,
    start_height=0.5,
    camera_distance=6.0,
    camera_yaw=45,
    camera_pitch=-55,
):
    """
    Run a simple L-shaped trajectory in PyBullet using R2D2.

    Returns:
        true_pos: (N, 2) true x-y positions
        gps_pos:  (N, 2) noisy GPS-like measurements
        times:    (N,) time stamps
    """
    mode = p.GUI if use_gui else p.DIRECT
    client = p.connect(mode)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    # Use a small physics step for smooth motion.
    physics_dt = 1.0 / 240.0
    p.setTimeStep(physics_dt)
    substeps = max(1, int(round(dt / physics_dt)))

    # Optional: make the GUI camera friendly for class demos.
    if use_gui:
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=camera_yaw,
            cameraPitch=camera_pitch,
            cameraTargetPosition=[2.0, 2.0, 0.0],
        )

    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(
        "r2d2.urdf",
        [0.0, 0.0, start_height],
        p.getQuaternionFromEuler([0.0, 0.0, 0.0]),
    )

    true_pos = np.zeros((num_steps, 2))
    gps_pos = np.zeros((num_steps, 2))
    times = np.arange(num_steps) * dt

    # L-shape: first half along +x, second half along +y
    for k in range(num_steps):
        if k < num_steps // 2:
            cmd_v = [speed, 0.0, 0.0]
        else:
            cmd_v = [0.0, speed, 0.0]

        p.resetBaseVelocity(robot_id, linearVelocity=cmd_v, angularVelocity=[0, 0, 0])

        for _ in range(substeps):
            p.stepSimulation()
            if use_gui:
                time.sleep(physics_dt)

        pos, _ = p.getBasePositionAndOrientation(robot_id)
        true_xy = np.array([pos[0], pos[1]])
        noise = np.random.normal(0.0, gps_noise_std_true, size=2)
        meas_xy = true_xy + noise

        true_pos[k] = true_xy
        gps_pos[k] = meas_xy

    p.disconnect(client)
    return true_pos, gps_pos, times


# ============================================================
# Kalman filter
# ============================================================
def run_kf_2d(
    measurements,
    dt=0.2,
    accel_noise_std=0.8,
    gps_noise_std_filter=0.8,
    dropout_range=None,
):
    """
    2D Constant-Velocity Kalman Filter.

    State:
        x = [x, y, vx, vy]^T

    This instructor version stores BOTH:
    - prediction (prior): x_{k|k-1}
    - corrected estimate (posterior): x_{k|k}

    Returns a dictionary with states, covariances, innovations, and gains.
    """
    n = len(measurements)

    # Initialize from first measurement for a cleaner classroom demonstration.
    x = np.array([measurements[0, 0], measurements[0, 1], 0.0, 0.0], dtype=float)
    P = np.diag([gps_noise_std_filter**2, gps_noise_std_filter**2, 4.0, 4.0])

    F = np.array([
        [1.0, 0.0, dt,  0.0],
        [0.0, 1.0, 0.0, dt ],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])

    H = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
    ])

    # White-acceleration process noise for CV model.
    s2 = accel_noise_std ** 2
    Q = s2 * np.array([
        [dt**4 / 4.0, 0.0,          dt**3 / 2.0, 0.0],
        [0.0,          dt**4 / 4.0, 0.0,         dt**3 / 2.0],
        [dt**3 / 2.0,  0.0,         dt**2,       0.0],
        [0.0,          dt**3 / 2.0, 0.0,         dt**2],
    ])

    R = np.eye(2) * (gps_noise_std_filter ** 2)
    I = np.eye(4)

    pred_state = np.zeros((n, 4))
    pred_pos = np.zeros((n, 2))
    pred_cov_xy = []

    upd_state = np.zeros((n, 4))
    upd_pos = np.zeros((n, 2))
    upd_cov_xy = []

    innovations = np.full((n, 2), np.nan)
    gains = np.zeros((n, 4, 2))
    used_measurement = np.ones(n, dtype=bool)

    for k in range(n):
        # ---------------- Predict ----------------
        x_pred = F @ x
        P_pred = F @ P @ F.T + Q

        pred_state[k] = x_pred
        pred_pos[k] = x_pred[:2]
        pred_cov_xy.append(P_pred[:2, :2].copy())

        # Optional dropout window
        z = measurements[k].copy()
        if dropout_range is not None:
            start, end = dropout_range
            if start <= k < end:
                z[:] = np.nan
                used_measurement[k] = False

        # ---------------- Update ----------------
        if np.all(np.isfinite(z)):
            y = z - H @ x_pred                 # innovation / residual
            S = H @ P_pred @ H.T + R
            K = P_pred @ H.T @ np.linalg.inv(S)

            x = x_pred + K @ y

            # Joseph-form covariance update (numerically safer)
            IKH = I - K @ H
            P = IKH @ P_pred @ IKH.T + K @ R @ K.T

            innovations[k] = y
            gains[k] = K
        else:
            # No measurement available => posterior equals prior
            x = x_pred
            P = P_pred
            innovations[k] = np.array([np.nan, np.nan])
            gains[k] = 0.0

        upd_state[k] = x
        upd_pos[k] = x[:2]
        upd_cov_xy.append(P[:2, :2].copy())

    return {
        "pred_state": pred_state,
        "pred_pos": pred_pos,
        "pred_cov_xy": pred_cov_xy,
        "upd_state": upd_state,
        "upd_pos": upd_pos,
        "upd_cov_xy": upd_cov_xy,
        "innovations": innovations,
        "gains": gains,
        "used_measurement": used_measurement,
        "F": F,
        "H": H,
        "Q": Q,
        "R": R,
    }


# ============================================================
# Plotting
# ============================================================
def plot_tracking_results(
    true_pos,
    gps_pos,
    pred_pos,
    upd_pos,
    upd_cov_xy,
    used_measurement,
    ellipse_stride=5,
    save_path="activity2_pybullet_kf_v3.png",
):
    """Main trajectory plot showing truth, measurement, prediction, and update."""
    fig, ax = plt.subplots(figsize=(10, 8))

    ax.plot(true_pos[:, 0], true_pos[:, 1], 'g-', lw=2.5, label='True Path (PyBullet)')
    ax.scatter(gps_pos[:, 0], gps_pos[:, 1], c='r', s=25, alpha=0.5, label='Noisy GPS')
    ax.plot(pred_pos[:, 0], pred_pos[:, 1], color='orange', ls='--', lw=2.0, label='KF Prediction (Prior)')
    ax.plot(upd_pos[:, 0], upd_pos[:, 1], 'b-', lw=2.5, label='KF Update (Posterior)')

    # Draw small correction segments so students can SEE update = correction of prediction.
    for k in range(0, len(upd_pos), 2):
        ax.plot(
            [pred_pos[k, 0], upd_pos[k, 0]],
            [pred_pos[k, 1], upd_pos[k, 1]],
            color='gray', alpha=0.35, lw=1.0,
        )

    # Show covariance ellipses only on posterior for visual clarity.
    for k in range(0, len(upd_pos), ellipse_stride):
        color = 'purple' if used_measurement[k] else 'gray'
        add_ellipse(
            ax,
            upd_cov_xy[k],
            upd_pos[k],
            n_std=2.0,
            facecolor=color,
            edgecolor=color,
            alpha=0.14,
            lw=1.2,
        )

    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('2D Kalman Filter in PyBullet: Truth vs Measurement vs Prediction vs Update')
    ax.legend(loc='best')
    ax.grid(True)
    ax.axis('equal')

    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    print(f'Saved {save_path}')
    return fig, ax


def plot_innovations(times, innovations, save_path="activity2_innovations_v3.png"):
    """Plot measurement innovations (residuals)."""
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(times, innovations[:, 0], label='Innovation in x')
    ax.plot(times, innovations[:, 1], label='Innovation in y')
    ax.axhline(0.0, color='k', lw=1.0)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Residual')
    ax.set_title('Kalman Filter Innovation (Measurement Residual)')
    ax.grid(True)
    ax.legend(loc='best')
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    print(f'Saved {save_path}')
    return fig, ax


def plot_position_errors(times, true_pos, gps_pos, pred_pos, upd_pos,
                         save_path="activity2_position_error_v3.png"):
    """Plot Euclidean position errors over time."""
    gps_err = np.linalg.norm(gps_pos - true_pos, axis=1)
    pred_err = np.linalg.norm(pred_pos - true_pos, axis=1)
    upd_err = np.linalg.norm(upd_pos - true_pos, axis=1)

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(times, gps_err, 'r-', alpha=0.7, label='GPS error')
    ax.plot(times, pred_err, color='orange', ls='--', lw=2.0, label='Prediction error')
    ax.plot(times, upd_err, 'b-', lw=2.0, label='Updated KF error')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Euclidean position error (m)')
    ax.set_title('Position Error Over Time')
    ax.grid(True)
    ax.legend(loc='best')
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    print(f'Saved {save_path}')
    return fig, ax


# ============================================================
# Main demo
# ============================================================
def main():
    np.random.seed(42)  # reproducible measurement noise

    # --------------------------------------------------------
    # Baseline parameters chosen for teaching clarity
    # --------------------------------------------------------
    num_steps = 60
    dt = 0.2
    speed = 1.0

    gps_noise_std_true = 0.35      # actual GPS noise in simulation
    gps_noise_std_filter = 0.35    # R used by the filter
    accel_noise_std = 0.8          # Q tuning parameter

    use_gui = True                 # set False for fast batch runs

    # Optional sensor dropout: e.g., (20, 28) or None
    dropout_range = None

    print('Running PyBullet simulation...')
    true_pos, gps_pos, times = run_pybullet_simulation(
        num_steps=num_steps,
        dt=dt,
        gps_noise_std_true=gps_noise_std_true,
        use_gui=use_gui,
        speed=speed,
    )

    print('Running Kalman filter...')
    kf = run_kf_2d(
        gps_pos,
        dt=dt,
        accel_noise_std=accel_noise_std,
        gps_noise_std_filter=gps_noise_std_filter,
        dropout_range=dropout_range,
    )

    pred_pos = kf['pred_pos']
    upd_pos = kf['upd_pos']
    upd_cov_xy = kf['upd_cov_xy']
    innovations = kf['innovations']
    used_measurement = kf['used_measurement']

    # --------------------------------------------------------
    # Metrics
    # --------------------------------------------------------
    gps_rmse = compute_rmse(gps_pos, true_pos)
    pred_rmse = compute_rmse(pred_pos, true_pos)
    upd_rmse = compute_rmse(upd_pos, true_pos)

    print('\n========== RMSE Summary =========')
    print(f'GPS RMSE           : {gps_rmse:.3f} m')
    print(f'Prediction RMSE    : {pred_rmse:.3f} m')
    print(f'Updated KF RMSE    : {upd_rmse:.3f} m')
    print('=================================\n')

    print('Filter matrices used:')
    print('F =\n', kf['F'])
    print('Q =\n', kf['Q'])
    print('R =\n', kf['R'])

    print('Plotting results...')
    plot_tracking_results(
        true_pos=true_pos,
        gps_pos=gps_pos,
        pred_pos=pred_pos,
        upd_pos=upd_pos,
        upd_cov_xy=upd_cov_xy,
        used_measurement=used_measurement,
        ellipse_stride=5,
        save_path='activity2_pybullet_kf_v3.png',
    )

    plot_innovations(
        times=times,
        innovations=innovations,
        save_path='activity2_innovations_v3.png',
    )

    plot_position_errors(
        times=times,
        true_pos=true_pos,
        gps_pos=gps_pos,
        pred_pos=pred_pos,
        upd_pos=upd_pos,
        save_path='activity2_position_error_v3.png',
    )

    plt.show()


if __name__ == '__main__':
    main()
