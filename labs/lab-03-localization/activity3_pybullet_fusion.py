"""
Activity 3 — Sensor Fusion (Odometry + GPS) with PyBullet
Version 2.0 - Integrated with the Autonomous Car Project

=== Mission Context ===
Your R2D2 robot has TWO sensors, but neither is perfect:
  - Odometry (wheel encoders): Fast updates (10Hz), but drifts over time.
  - GPS: Drift-free, but noisy and slow (1Hz — only every 10 steps).

Your mission: Fuse both sensors using a Kalman Filter to get the BEST estimate.

Learning goals:
1) Understand the complementary nature of sensors (Odometry drifts, GPS is noisy).
2) Implement a Kalman Filter that fuses two different measurement sources.
3) See how uncertainty grows between GPS updates and shrinks when GPS arrives.

Key ideas:
- The robot moves in a circle in PyBullet.
- Odometry has a small constant bias → causes drift over time.
- GPS arrives only every 10 steps → sparse but drift-free.
- The KF combines both: smooth trajectory from odometry + corrections from GPS.
"""
import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

# ============================================================
# HELPER: Draw a 2D covariance ellipse on a matplotlib axis
# ============================================================
def add_ellipse(ax, Pxy, mean_xy, n_std=2.0, facecolor='purple',
                edgecolor='purple', alpha=0.15, lw=1.0):
    """
    Draws a 2D covariance ellipse representing ±n_std standard deviations.
    Pxy: 2x2 covariance matrix (position only).
    mean_xy: [x, y] center of the ellipse.
    """
    vals, vecs = np.linalg.eigh(Pxy)
    order = np.argsort(vals)[::-1]
    vals = vals[order]; vecs = vecs[:, order]
    a = 2.0 * n_std * np.sqrt(max(float(vals[0]), 0.0))
    b = 2.0 * n_std * np.sqrt(max(float(vals[1]), 0.0))
    theta_deg = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))
    e = Ellipse(xy=mean_xy, width=a, height=b, angle=theta_deg,
                facecolor=facecolor, edgecolor=edgecolor, alpha=alpha, lw=lw)
    ax.add_patch(e)

# ============================================================
# STEP 1: PyBullet Simulation — Robot moves in a circle
# ============================================================
def run_pybullet_simulation(num_steps=120, dt=0.1, odom_bias=0.05,
                            gps_noise_std=3.0, gps_freq=10):
    """
    Runs R2D2 in a circle using PyBullet.
    
    Returns:
        true_pos:  (N, 2) ground truth positions from PyBullet
        odom_vel:  (N, 2) noisy odometry velocity readings (with bias)
        gps_pos:   (N, 2) noisy GPS position readings (NaN when unavailable)
        gps_avail: (N,)   boolean array — True when GPS reading is available
    """
    # --- Connect to PyBullet (headless mode for data collection) ---
    physicsClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    
    # --- Load environment and robot ---
    p.loadURDF("plane.urdf")
    
    # Circle parameters
    radius = 5.0
    omega = 2 * np.pi / (num_steps * dt)  # Angular velocity for one full circle
    
    # Start at the right edge of the circle (radius, 0)
    startPos = [radius, 0, 0.5]
    startOri = p.getQuaternionFromEuler([0, 0, 0])
    robotId = p.loadURDF("r2d2.urdf", startPos, startOri)
    
    # --- Allocate storage ---
    true_pos  = np.zeros((num_steps, 2))
    odom_vel  = np.zeros((num_steps, 2))
    gps_pos   = np.full((num_steps, 2), np.nan)
    gps_avail = np.zeros(num_steps, dtype=bool)
    
    # --- Simulation loop ---
    for i in range(num_steps):
        # Compute commanded velocity for circular motion
        angle = omega * i * dt
        vx_cmd = -omega * radius * np.sin(angle)
        vy_cmd =  omega * radius * np.cos(angle)
        
        # Apply velocity to robot
        p.resetBaseVelocity(robotId, linearVelocity=[vx_cmd, vy_cmd, 0],
                            angularVelocity=[0, 0, 0])
        p.stepSimulation()
        
        # --- Ground Truth (what PyBullet knows) ---
        pos, _ = p.getBasePositionAndOrientation(robotId)
        true_pos[i] = [pos[0], pos[1]]
        
        # --- Simulated Odometry Sensor ---
        # WHY bias? Real wheel encoders accumulate small systematic errors
        # (e.g., slightly different wheel diameters). This causes DRIFT.
        odom_noise = np.random.normal(0, 0.05, 2)
        odom_vel[i] = [vx_cmd + odom_bias + odom_noise[0],
                       vy_cmd + odom_bias + odom_noise[1]]
        
        # --- Simulated GPS Sensor ---
        # WHY sparse? Real GPS updates at ~1Hz, while odometry runs at 10-100Hz.
        # WHY noisy? GPS has ~3m accuracy in consumer-grade receivers.
        if i % gps_freq == 0:
            gps_noise = np.random.normal(0, gps_noise_std, 2)
            gps_pos[i] = [pos[0] + gps_noise[0], pos[1] + gps_noise[1]]
            gps_avail[i] = True
            
    p.disconnect()
    return true_pos, odom_vel, gps_pos, gps_avail

# ============================================================
# STEP 2: Sensor Fusion Kalman Filter
# ============================================================
def run_fusion_kf(odom_vel, gps_pos, gps_avail, dt=0.1,
                  accel_noise_std=5.0, odom_noise_std=0.05, gps_noise_std=3.0):
    """
    Fuses Odometry (velocity) and GPS (position) using a Kalman Filter.
    
    State vector: x = [px, py, vx, vy]
    Odometry measures: [vx, vy] (fast, biased)
    GPS measures: [px, py] (slow, noisy, drift-free)
    """
    n = len(odom_vel)
    
    # --- Initial state and covariance ---
    # We start at (5, 0) — the edge of the circle
    x = np.array([5.0, 0.0, 0.0, 0.0])
    P = np.eye(4) * 10.0  # High initial uncertainty (we're not sure)
    
    # --- State Transition Matrix (Constant Velocity model) ---
    # x_new = x_old + vx * dt
    # y_new = y_old + vy * dt
    F = np.array([
        [1, 0, dt, 0],
        [0, 1, 0, dt],
        [0, 0, 1,  0],
        [0, 0, 0,  1]
    ])
    
    # --- Process Noise Q ---
    # WHY large Q? The robot is moving in a CIRCLE, but our model assumes
    # STRAIGHT lines (constant velocity). Large Q tells the filter:
    # "My model is imperfect — trust measurements more."
    s2 = accel_noise_std**2
    Q = s2 * np.array([
        [dt**4/4, 0,       dt**3/2, 0      ],
        [0,       dt**4/4, 0,       dt**3/2],
        [dt**3/2, 0,       dt**2,   0      ],
        [0,       dt**3/2, 0,       dt**2  ]
    ])
    # Extra position noise to help track curves
    Q[0, 0] += 0.1
    Q[1, 1] += 0.1
    
    # --- Odometry Measurement Model ---
    # Odometry observes velocity: z_odom = [vx, vy]
    H_odom = np.array([[0, 0, 1, 0],
                       [0, 0, 0, 1]])
    R_odom = np.eye(2) * (odom_noise_std**2)
    
    # --- GPS Measurement Model ---
    # GPS observes position: z_gps = [px, py]
    H_gps = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]])
    R_gps = np.eye(2) * (gps_noise_std**2)
    
    # --- Storage ---
    est_pos = np.zeros((n, 2))
    covariances = []
    
    for i in range(n):
        # ======== PREDICT ========
        x = F @ x
        P = F @ P @ F.T + Q
        
        # ======== UPDATE 1: Odometry (always available, every step) ========
        z_odom = odom_vel[i]
        y_odom = z_odom - H_odom @ x                    # Innovation
        S_odom = H_odom @ P @ H_odom.T + R_odom          # Innovation covariance
        K_odom = P @ H_odom.T @ np.linalg.inv(S_odom)    # Kalman Gain
        x = x + K_odom @ y_odom                           # Correct state
        P = (np.eye(4) - K_odom @ H_odom) @ P             # Correct covariance
        
        # ======== UPDATE 2: GPS (only when available) ========
        if gps_avail[i]:
            z_gps = gps_pos[i]
            y_gps = z_gps - H_gps @ x
            S_gps = H_gps @ P @ H_gps.T + R_gps
            K_gps = P @ H_gps.T @ np.linalg.inv(S_gps)
            x = x + K_gps @ y_gps
            P = (np.eye(4) - K_gps @ H_gps) @ P
            
        est_pos[i] = [x[0], x[1]]
        covariances.append(P[0:2, 0:2].copy())
        
    return est_pos, covariances

# ============================================================
# STEP 3: Baseline Methods (for comparison)
# ============================================================
def dead_reckoning(odom_vel, start_pos, dt=0.1):
    """
    Dead reckoning: integrates velocity to get position.
    This WILL drift because odometry has a bias.
    """
    n = len(odom_vel)
    pos = np.zeros((n, 2))
    pos[0] = start_pos
    for i in range(1, n):
        pos[i] = pos[i-1] + odom_vel[i] * dt
    return pos

def gps_hold(gps_pos, gps_avail, start_pos):
    """
    Holds the last known GPS position until a new one arrives.
    This is what you'd get if you ONLY had GPS — jumpy and sparse.
    """
    n = len(gps_pos)
    pos = np.zeros((n, 2))
    last = np.array(start_pos)
    for i in range(n):
        if gps_avail[i]:
            last = gps_pos[i].copy()
        pos[i] = last
    return pos

# ============================================================
# STEP 4: Visualization
# ============================================================
def plot_fusion_results(true_pos, odom_only, gps_only, fused_pos,
                        covariances, gps_avail):
    """Creates a 2x2 figure comparing all three approaches + uncertainty."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    
    # --- A. Odometry Only (Dead Reckoning) ---
    ax = axes[0, 0]
    ax.plot(true_pos[:, 0], true_pos[:, 1], 'g-', lw=3, label='True Path', zorder=3)
    ax.plot(odom_only[:, 0], odom_only[:, 1], 'b--', lw=2, label='Odometry (Drift)', zorder=2)
    ax.plot(true_pos[0, 0], true_pos[0, 1], 'go', ms=10, zorder=4)
    ax.set_title('A. Odometry Only (Dead Reckoning)', fontsize=13, fontweight='bold')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.legend(loc='upper left'); ax.grid(True, alpha=0.3); ax.axis('equal')
    
    # --- B. GPS Only ---
    ax = axes[0, 1]
    ax.plot(true_pos[:, 0], true_pos[:, 1], 'g-', lw=3, label='True Path', zorder=3)
    idx = np.where(gps_avail)[0]
    ax.scatter(gps_only[idx, 0], gps_only[idx, 1], c='r', marker='x', s=100,
               zorder=4, label='GPS Fixes')
    ax.plot(gps_only[:, 0], gps_only[:, 1], 'r-', lw=1, alpha=0.4, label='GPS Hold')
    ax.plot(true_pos[0, 0], true_pos[0, 1], 'go', ms=10, zorder=4)
    ax.set_title('B. GPS Only (Noisy & Sparse)', fontsize=13, fontweight='bold')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.legend(loc='upper left'); ax.grid(True, alpha=0.3); ax.axis('equal')
    
    # --- C. Sensor Fusion (The Winner!) ---
    ax = axes[1, 0]
    ax.plot(true_pos[:, 0], true_pos[:, 1], 'g-', lw=3, label='True Path', zorder=3)
    ax.plot(fused_pos[:, 0], fused_pos[:, 1], color='purple', lw=2,
            label='Fused (KF)', zorder=2)
    # Draw uncertainty ellipses every 10 steps (less cluttered)
    for i in range(0, len(fused_pos), 10):
        add_ellipse(ax, covariances[i], fused_pos[i], n_std=2.0)
    ax.plot(true_pos[0, 0], true_pos[0, 1], 'go', ms=10, zorder=4)
    ax.set_title('C. Sensor Fusion (Odom + GPS)', fontsize=13, fontweight='bold')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.legend(loc='upper left'); ax.grid(True, alpha=0.3); ax.axis('equal')
    
    # --- D. Uncertainty Evolution (Sawtooth Pattern) ---
    ax = axes[1, 1]
    t = np.arange(len(covariances))
    trP = np.array([np.trace(C) for C in covariances])
    ax.plot(t, trP, 'k-', lw=2, label='Uncertainty (Trace P)')
    ax.scatter(idx, trP[idx], c='red', marker='v', s=80, zorder=3, label='GPS Update')
    ax.set_title('D. Uncertainty Evolution (Sawtooth Pattern)', fontsize=13, fontweight='bold')
    ax.set_xlabel('Time Step'); ax.set_ylabel('Position Uncertainty (m²)')
    ax.legend(); ax.grid(True, alpha=0.3)
    
    plt.suptitle('Activity 3: Sensor Fusion — Odometry + GPS on PyBullet Robot',
                 fontsize=15, fontweight='bold', y=1.01)
    plt.tight_layout()
    plt.savefig('activity3_pybullet_fusion.png', dpi=300, bbox_inches='tight')
    print("Saved activity3_pybullet_fusion.png")

# ============================================================
# MAIN
# ============================================================
def main():
    np.random.seed(42)  # Reproducible results for all students
    
    # --- Configuration ---
    dt = 0.1
    gps_noise_std = 3.0   # GPS is noisy (~3m accuracy)
    odom_bias = 0.05      # Small constant bias in odometry
    start_pos = [5.0, 0.0]
    
    # --- Run Simulation ---
    print("Running PyBullet simulation (R2D2 moving in a circle)...")
    true_pos, odom_vel, gps_pos, gps_avail = run_pybullet_simulation(
        num_steps=120, dt=dt, odom_bias=odom_bias, gps_noise_std=gps_noise_std,
        gps_freq=10)
    
    # --- Compute Baselines ---
    print("Computing baselines (Dead Reckoning and GPS-only)...")
    odom_only = dead_reckoning(odom_vel, start_pos, dt=dt)
    gps_only  = gps_hold(gps_pos, gps_avail, start_pos)
    
    # --- Run Sensor Fusion KF ---
    print("Running Sensor Fusion Kalman Filter...")
    fused_pos, covariances = run_fusion_kf(
        odom_vel, gps_pos, gps_avail, dt=dt,
        accel_noise_std=5.0, odom_noise_std=0.05, gps_noise_std=gps_noise_std)
    
    # --- Visualize ---
    print("Generating comparison plots...")
    plot_fusion_results(true_pos, odom_only, gps_only, fused_pos,
                        covariances, gps_avail)
    
    # --- RMSE Comparison ---
    rmse_odom = np.sqrt(np.mean(np.sum((odom_only - true_pos)**2, axis=1)))
    rmse_gps  = np.sqrt(np.mean(np.sum((gps_only  - true_pos)**2, axis=1)))
    rmse_kf   = np.sqrt(np.mean(np.sum((fused_pos - true_pos)**2, axis=1)))
    
    print("\n" + "="*50)
    print("  RMSE COMPARISON")
    print("="*50)
    print(f"  Odometry only (Dead Reckoning): {rmse_odom:.3f} m  ← Worst (drift)")
    print(f"  GPS only (Hold):                {rmse_gps:.3f} m  ← Noisy")
    print(f"  Kalman Filter (Fused):          {rmse_kf:.3f} m  ← BEST!")
    print("="*50)
    print(f"\n  Fusion improved over GPS by: {(1 - rmse_kf/rmse_gps)*100:.1f}%")
    print(f"  Fusion improved over Odom by: {(1 - rmse_kf/rmse_odom)*100:.1f}%")

if __name__ == '__main__':
    main()
