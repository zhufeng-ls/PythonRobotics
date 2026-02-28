import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import math

def plot_covariance_ellipse(x, y, P, ax, color, label):
    """
    Plots the covariance ellipse of the uncertainty.
    """
    eig_val, eig_vec = np.linalg.eig(P[0:2, 0:2])

    if eig_val[0] >= eig_val[1]:
        big_ind = 0
        small_ind = 1
    else:
        big_ind = 1
        small_ind = 0

    a = math.sqrt(eig_val[big_ind])
    b = math.sqrt(eig_val[small_ind])
    angle = math.atan2(eig_vec[1, big_ind], eig_vec[0, big_ind])

    ell = Ellipse(xy=(x, y), width=a * 4, height=b * 4, angle=np.rad2deg(angle),
                  color=color, alpha=0.3, label=label)
    ax.add_patch(ell)
    ax.scatter(x, y, c=color, s=50)

def main():
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # 1. Initial State (Green)
    x_init = np.array([[0.0], [0.0], [0.0], [0.0]]) # x, y, yaw, v
    P_init = np.diag([0.1, 0.1, 0.1, 0.1]) ** 2
    plot_covariance_ellipse(x_init[0, 0], x_init[1, 0], P_init, ax, 'green', 'Initial State')
    
    # 2. Prediction Step (Blue) -> Uncertainty Increases
    # Move 1 meter to the right
    dt = 1.0
    v = 1.0
    u = np.array([[v], [0.0]])
    
    # Simple linear motion model for visualization
    F = np.eye(4)
    F[0, 3] = dt # x += v * dt
    
    # Process Noise
    Q = np.diag([0.2, 0.2, 0.1, 0.2]) ** 2
    
    x_pred = x_init.copy()
    x_pred[0] += v * dt
    
    P_pred = F @ P_init @ F.T + Q
    plot_covariance_ellipse(x_pred[0, 0], x_pred[1, 0], P_pred, ax, 'blue', 'Prediction (Uncertainty Increases)')
    
    # 3. Update Step (Red) -> Uncertainty Decreases
    # Observe x, y
    z = np.array([[1.1], [0.1]]) # Observation is slightly off
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])
    R = np.diag([0.1, 0.1]) ** 2 # Observation Noise (Very accurate GPS)
    
    y = z - H @ x_pred
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)
    
    x_est = x_pred + K @ y
    P_est = (np.eye(4) - K @ H) @ P_pred
    
    plot_covariance_ellipse(x_est[0, 0], x_est[1, 0], P_est, ax, 'red', 'Update (Uncertainty Decreases)')
    
    # Plot Observation
    ax.scatter(z[0], z[1], marker='x', c='black', s=100, label='Observation')

    ax.set_xlim(-1, 3)
    ax.set_ylim(-2, 2)
    ax.set_aspect('equal')
    ax.legend()
    ax.set_title("EKF: Prediction vs Update", fontsize=16)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True)
    
    output_path = "/home/zhufeng/code/PythonRobotics/PythonRoboticsLearning/notes/img/ekf_prediction_update.png"
    plt.savefig(output_path, dpi=300)
    print(f"Image saved to {output_path}")

if __name__ == '__main__':
    main()
