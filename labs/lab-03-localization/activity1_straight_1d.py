
"""
Activity 1 — Straight-Line Motion (1‑D) with Constant‑Velocity Kalman Filter
Teaching version (well‑commented) — clean plots, no extra plot text boxes.

Learning goals (for students):
1) Understand predict → update cycle in a 1‑D setting.
2) See why a motion model (constant‑velocity) enables **prediction** between measurements.
3) Read uncertainty as a ±2σ (standard deviation) **band** around the estimate.

Key ideas in code:
- State x = [p, v]ᵀ (position and velocity along a line).
- Motion model (F) assumes constant velocity over each dt.
- Process noise (Q) comes from small random accelerations (white acceleration model).
- Measurement model (H) directly observes position (p) with noise variance R.
"""
import numpy as np
import matplotlib.pyplot as plt

# ---------------- Simulation ----------------
def simulate_straight_1d(num_steps=60, v=1.0, dt=1.0,
                          process_noise_std=0.05, meas_noise_std=1.0):
    """Generate 1‑D straight‑line motion with noisy position measurements.
    True motion includes a small perturbation so students see Q in action.
    """
    p = 0.0
    true_p = np.zeros(num_steps)
    z = np.zeros(num_steps)
    for k in range(num_steps):
        # True position update: p <- p + v*dt + process noise (small)
        p = p + v*dt + np.random.normal(0.0, process_noise_std)
        true_p[k] = p
        # Noisy position measurement
        z[k] = p + np.random.normal(0.0, meas_noise_std)
    return true_p, z

# ---------------- KF (CV model) ----------------
def run_kf_cv_1d(measurements, dt=1.0, p0=0.0, v0=0.0,
                 P0_pos_var=25.0, P0_vel_var=25.0,
                 accel_noise_std=0.1, meas_noise_std=1.0):
    """Run a 1‑D constant‑velocity KF on a sequence of position measurements.

    Parameters
    ----------
    measurements : array-like
        Noisy position z_k.
    dt : float
        Sampling period.
    p0, v0 : float
        Initial guesses for position and velocity.
    P0_pos_var, P0_vel_var : float
        Initial covariance (variance) for p and v (start large to allow learning).
    accel_noise_std : float
        Std of white acceleration driving Q (higher → more agile predictions).
    meas_noise_std : float
        Std of measurement noise (sets R = σ_z^2).
    """
    n = len(measurements)
    # State and covariance init
    x = np.array([p0, v0], dtype=float)
    P = np.diag([P0_pos_var, P0_vel_var])

    # Models: CV dynamics and position-only measurement
    F = np.array([[1.0, dt],
                  [0.0, 1.0]], dtype=float)
    H = np.array([1.0, 0.0], dtype=float)  # observe position only

    # Process noise from white acceleration (constant acceleration over dt approx.)
    s2 = accel_noise_std**2
    Q = s2 * np.array([[dt**4/4, dt**3/2],
                       [dt**3/2, dt**2  ]], dtype=float)
    R = meas_noise_std**2

    est = np.zeros(n)   # estimated positions
    var = np.zeros(n)   # position variances P[0,0]

    for k, zk in enumerate(measurements):
        # ---- Predict ----
        x = F @ x
        P = F @ P @ F.T + Q

        # ---- Update ---- (scalar measurement)
        y = zk - H @ x                 # innovation (residual)
        S = H @ P @ H.T + R            # innovation variance (scalar)
        K = P @ H / S                  # Kalman gain (2x1)
        x = x + K * y                  # state correction
        P = (np.eye(2) - np.outer(K, H)) @ P  # Joseph form not needed here

        est[k] = x[0]
        var[k] = P[0, 0]

    return est, var

# ---------------- Plotting ----------------
def plot_activity1(true_p, z, est, var, out_file='activity1_straight_1d.png'):
    """One figure, two clean plots: trajectory with ±2σ band, and error vs time.
    Keep visuals minimal (legend + grid) for a first exposure.
    """
    t = np.arange(len(true_p))
    std = np.sqrt(var)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 9))

    # Plot 1: Trajectory with ±2σ band
    ax1.plot(t, true_p, 'g-', lw=2, label='True Position')
    ax1.scatter(t, z, c='r', s=18, alpha=0.5, label='Noisy Measurements')
    ax1.plot(t, est, 'b-', lw=2, label='KF Estimate')
    ax1.fill_between(t, est - 2*std, est + 2*std, color='blue', alpha=0.2, label='±2σ band')
    ax1.set_xlabel('Time step'); ax1.set_ylabel('Position (m)')
    ax1.set_title('Activity 1: 1‑D CV Kalman Filter — Straight Line')
    ax1.grid(True, alpha=0.3); ax1.legend()

    # Plot 2: Estimation error (for intuition about bias and noise)
    ax2.plot(t, est - true_p, 'k-')
    ax2.set_xlabel('Time step'); ax2.set_ylabel('Position Error (m)')
    ax2.set_title('Activity 1: Estimation Error over Time')
    ax2.grid(True, alpha=0.3)

    fig.tight_layout(); fig.savefig(out_file, dpi=200)
    print('Saved', out_file)

# ---------------- Main ----------------

def main():
    # Deterministic seed so everyone sees the same figures in class.
    np.random.seed(42)

    # Simulation configuration (safe defaults for teaching)
    num_steps = 60
    dt = 1.0
    process_noise_std = 0.05
    measurement_noise_std = 1.0

    # Generate data
    true_p, z = simulate_straight_1d(num_steps=num_steps, v=1.0, dt=dt,
                                      process_noise_std=process_noise_std,
                                      meas_noise_std=measurement_noise_std)

    # Run KF
    est, var = run_kf_cv_1d(z, dt=dt, p0=0.0, v0=0.0,
                             P0_pos_var=25.0, P0_vel_var=25.0,
                             accel_noise_std=0.1,
                             meas_noise_std=measurement_noise_std)

    # Visualize
    plot_activity1(true_p, z, est, var)
    plt.show()  # open the plot window (in Colab it appears under the cell)

if __name__ == '__main__':
    main()
