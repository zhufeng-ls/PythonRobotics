"""
LQR Speed and Steering Control - Visual Demo
A simple visualization to understand LQR path tracking control
"""
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import FancyArrowPatch
from matplotlib.widgets import Button
import scipy.linalg as la
import sys
import pathlib
import copy

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
from utils.angle import angle_mod
from PathPlanning.CubicSpline import cubic_spline_planner

# === Parameters ===
dt = 0.1  # time tick [s]
L = 0.5   # wheel base [m]
max_steer = np.deg2rad(45.0)

# LQR parameters
Q = np.eye(5)
R = np.eye(2)


class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta):
    """Update vehicle state using bicycle model"""
    if delta >= max_steer:
        delta = max_steer
    if delta <= -max_steer:
        delta = -max_steer

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt

    return state


def solve_dare(A, B, Q, R):
    """Solve Discrete Algebraic Riccati Equation"""
    x = Q
    x_next = Q
    max_iter = 150
    eps = 0.01

    for i in range(max_iter):
        x_next = A.T @ x @ A - A.T @ x @ B @ \
                 la.inv(R + B.T @ x @ B) @ B.T @ x @ A + Q
        if (abs(x_next - x)).max() < eps:
            break
        x = x_next

    return x_next


def dlqr(A, B, Q, R):
    """Solve discrete-time LQR controller"""
    X = solve_dare(A, B, Q, R)
    K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)
    return K, X


def calc_nearest_index(state, cx, cy, cyaw):
    """Find nearest point on reference path"""
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)
    ind = d.index(mind)
    mind = math.sqrt(mind)

    # Calculate sign of lateral error
    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y
    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def pi_2_pi(angle):
    return angle_mod(angle)


def lqr_speed_steering_control(state, cx, cy, cyaw, ck, pe, pth_e, sp, Q, R):
    """LQR controller for combined speed and steering control"""
    ind, e = calc_nearest_index(state, cx, cy, cyaw)
    tv = sp[ind]
    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    # System matrix A (5x5)
    A = np.zeros((5, 5))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt
    A[4, 4] = 1.0

    # Control matrix B (5x2)
    B = np.zeros((5, 2))
    B[3, 0] = v / L
    B[4, 1] = dt

    # Calculate LQR gain
    K, _ = dlqr(A, B, Q, R)

    # State vector: [e, e_dot, th_e, th_e_dot, delta_v]
    x = np.zeros((5, 1))
    x[0, 0] = e
    x[1, 0] = (e - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt
    x[4, 0] = v - tv

    # Control input: [delta, accel]
    ustar = -K @ x

    # Steering angle with feedforward + feedback
    ff = math.atan2(L * k, 1)
    fb = pi_2_pi(ustar[0, 0])
    delta = ff + fb

    # Acceleration
    accel = ustar[1, 0]

    return delta, ind, e, th_e, accel, K


def draw_vehicle(ax, x, y, yaw, steer, color='blue'):
    """Draw vehicle as a rectangle with direction indicator"""
    # Vehicle body (simple rectangle)
    length = 0.4
    width = 0.2

    # Vehicle corners
    corners = np.array([
        [length/2, width/2],
        [length/2, -width/2],
        [-length/2, -width/2],
        [-length/2, width/2],
        [length/2, width/2]
    ])

    # Rotate and translate
    R = np.array([
        [math.cos(yaw), -math.sin(yaw)],
        [math.sin(yaw), math.cos(yaw)]
    ])
    corners = corners @ R.T
    corners[:, 0] += x
    corners[:, 1] += y

    ax.plot(corners[:, 0], corners[:, 1], color=color, linewidth=2)

    # Draw heading direction
    arrow_len = 0.3
    dx = arrow_len * math.cos(yaw)
    dy = arrow_len * math.sin(yaw)
    ax.arrow(x, y, dx, dy, head_width=0.1,
             head_length=0.05, fc=color, ec=color, alpha=0.6)


class LQRSimulation:
    """Interactive LQR path tracking simulation with step-by-step controls"""

    def __init__(self):
        print("LQR Speed and Steering Control - Visual Demo")
        print("=" * 50)

        # Generate reference path
        self.ax_wp = [0.0, 6.0, 12.5, 10.0, 17.5, 20.0, 25.0]
        self.ay_wp = [0.0, -3.0, -5.0, 6.5, 3.0, 0.0, 0.0]
        self.cx, self.cy, self.cyaw, self.ck, self.s = cubic_spline_planner.calc_spline_course(
            self.ax_wp, self.ay_wp, ds=0.1)

        # Speed profile
        self.target_speed = 10.0 / 3.6
        self.speed_profile = [self.target_speed] * len(self.cyaw)

        # Create figure with subplots
        self.fig = plt.figure(figsize=(16, 10))
        self.gs = gridspec.GridSpec(3, 3, figure=self.fig,
                                    height_ratios=[1, 1, 0.2])

        # Main path tracking view
        self.ax_main = self.fig.add_subplot(self.gs[0:2, 0:2])
        self.ax_main.set_title("LQR Path Tracking Control", fontsize=14, fontweight='bold')
        self.ax_main.set_xlabel("X [m]")
        self.ax_main.set_ylabel("Y [m]")
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.axis('equal')

        # State variables display
        self.ax_state = self.fig.add_subplot(self.gs[0, 2])
        self.ax_state.axis('off')
        self.state_text = self.ax_state.text(0.1, 0.5, "", transform=self.ax_state.transAxes,
                                             fontsize=10, verticalalignment='center',
                                             fontfamily='monospace')

        # Control inputs display
        self.ax_control = self.fig.add_subplot(self.gs[1, 2])
        self.ax_control.axis('off')
        self.control_text = self.ax_control.text(0.1, 0.5, "", transform=self.ax_control.transAxes,
                                                 fontsize=10, verticalalignment='center',
                                                 fontfamily='monospace')

        # LQR gain display
        self.ax_gain = self.fig.add_subplot(self.gs[2, 0])
        self.ax_gain.axis('off')
        self.gain_text = self.ax_gain.text(0.1, 0.5, "", transform=self.ax_gain.transAxes,
                                           fontsize=8, verticalalignment='center',
                                           fontfamily='monospace')

        # Lateral error plot
        self.ax_error = self.fig.add_subplot(self.gs[2, 1])
        self.ax_error.set_title("Lateral Error [m]", fontsize=10)
        self.ax_error.grid(True, alpha=0.3)
        self.error_data = []
        self.error_line, = self.ax_error.plot([], [], 'b-', linewidth=1.5)
        self.ax_error.set_ylim(-1.5, 1.5)

        # Speed plot
        self.ax_speed = self.fig.add_subplot(self.gs[2, 2])
        self.ax_speed.set_title("Speed [m/s]", fontsize=10)
        self.ax_speed.grid(True, alpha=0.3)
        self.speed_data = []
        self.target_speed_data = []
        self.speed_line, = self.ax_speed.plot([], [], 'g-', linewidth=1.5, label='Actual')
        self.target_line, = self.ax_speed.plot([], [], 'r--', linewidth=1, label='Target')
        self.ax_speed.legend(fontsize=8)
        self.ax_speed.set_ylim(0, self.target_speed * 1.5)

        # Plot reference path
        self.ax_main.plot(self.cx, self.cy, "-r", linewidth=2, label="Reference Path", alpha=0.7)

        # Plot waypoints
        self.ax_main.plot(self.ax_wp, self.ay_wp, "ks", markersize=8, label="Waypoints", alpha=0.5)

        # Initialize trajectory lines
        self.traj_x, self.traj_y = [], []
        self.traj_line, = self.ax_main.plot([], [], "-g", linewidth=2, label="Vehicle Trajectory")
        self.target_point, = self.ax_main.plot([], [], "rx", markersize=10,
                                               markeredgewidth=2, label="Target Point")

        # Initialize vehicle
        self.vehicle_patch, = self.ax_main.plot([], [], 'bo', markersize=8, label="Vehicle")

        self.ax_main.legend(loc='upper right', fontsize=9)

        # Control button area
        self.ax_button_area = self.fig.add_axes([0.1, 0.02, 0.8, 0.08])
        self.ax_button_area.axis('off')

        # Create buttons
        self.create_buttons()

        # Initialize simulation state
        self.reset_simulation()

        # Timer for auto play
        self.timer = None
        self.auto_play_speed = 50  # ms between steps

        print("Click 'Step' to advance one step, 'Auto' to auto-play, or 'Reset' to restart.")
        print("Press ESC key to exit.")

        # Connect key press event
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)

        plt.ion()
        plt.show()

    def create_buttons(self):
        """Create control buttons"""
        # Step button
        ax_step = self.fig.add_axes([0.15, 0.03, 0.15, 0.05])
        self.btn_step = Button(ax_step, 'Step Forward',
                               color='lightblue', hovercolor='blue')
        self.btn_step.on_clicked(self.step_forward)

        # Auto button
        ax_auto = self.fig.add_axes([0.42, 0.03, 0.15, 0.05])
        self.btn_auto = Button(ax_auto, 'Auto Play',
                               color='lightgreen', hovercolor='green')
        self.btn_auto.on_clicked(self.toggle_auto_play)

        # Reset button
        ax_reset = self.fig.add_axes([0.69, 0.03, 0.15, 0.05])
        self.btn_reset = Button(ax_reset, 'Reset',
                                color='lightcoral', hovercolor='red')
        self.btn_reset.on_clicked(self.reset)

    def reset_simulation(self):
        """Reset simulation to initial state"""
        self.state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)
        self.time = 0.0
        self.e, self.e_th = 0.0, 0.0
        self.goal_dis = 0.3
        self.stop_speed = 0.05
        self.target_ind = 0
        self.goal_reached = False

        # Clear trajectory
        self.traj_x = []
        self.traj_y = []
        self.error_data = []
        self.speed_data = []
        self.target_speed_data = []

        # Update display
        self.update_visualization(first_step=True)

    def step_forward(self, event=None):
        """Execute one simulation step"""
        if self.goal_reached:
            print("Goal already reached! Press Reset to start over.")
            return

        # LQR control
        delta, self.target_ind, self.e, self.e_th, accel, K = lqr_speed_steering_control(
            self.state, self.cx, self.cy, self.cyaw, self.ck, self.e, self.e_th,
            self.speed_profile, Q, R)

        # Update vehicle state
        self.state = update(self.state, accel, delta)

        if abs(self.state.v) <= self.stop_speed:
            self.target_ind += 1

        self.time += dt

        # Store trajectory
        self.traj_x.append(self.state.x)
        self.traj_y.append(self.state.y)

        # Store data for plots
        self.error_data.append(self.e)
        self.speed_data.append(self.state.v)
        self.target_speed_data.append(self.speed_profile[min(self.target_ind, len(self.speed_profile)-1)])

        # Update visualization
        self.update_visualization(delta, accel, K)

        # Check goal
        dx = self.state.x - self.ax_wp[-1]
        dy = self.state.y - self.ay_wp[-1]
        if math.hypot(dx, dy) <= self.goal_dis:
            self.goal_reached = True
            print(f"\n✓ Goal reached at t={self.time:.1f}s with {len(self.traj_x)} steps!")
            if self.timer is not None:
                self.stop_auto_play()

    def update_visualization(self, delta=0.0, accel=0.0, K=None, first_step=False):
        """Update visualization with current state"""
        # Update trajectory
        self.traj_line.set_data(self.traj_x, self.traj_y)

        # Update vehicle position
        self.vehicle_patch.set_data([self.state.x], [self.state.y])

        # Update target point
        if len(self.cx) > 0 and len(self.cy) > 0:
            self.target_point.set_data([self.cx[self.target_ind]], [self.cy[self.target_ind]])

        # Update error plot
        if self.error_data:
            self.ax_error.set_xlim(0, max(10, len(self.error_data)))
            self.error_line.set_data(range(len(self.error_data)), self.error_data)

        # Update speed plot
        if self.speed_data:
            self.ax_speed.set_xlim(0, max(10, len(self.speed_data)))
            self.speed_line.set_data(range(len(self.speed_data)), self.speed_data)
            self.target_line.set_data(range(len(self.target_speed_data)), self.target_speed_data)

        # Calculate derivatives
        if not first_step and self.error_data:
            e_dot = (self.e - self.error_data[-2]) / dt if len(self.error_data) > 1 else 0.0
            v_target = self.speed_profile[min(self.target_ind, len(self.speed_profile)-1)]

            # Update state display
            state_str = f"""STATE VARIABLES
{'─'*25}
Position: ({self.state.x:.2f}, {self.state.y:.2f}) m
Heading:  {np.rad2deg(self.state.yaw):.1f}°
Speed:    {self.state.v:.2f} m/s
Target:   {v_target:.2f} m/s
━━━━━━━━━━━━━━━━━━━━━━━━
LQR State Vector x:
[0] e (lateral):   {self.e:.3f} m
[1] ė (e_dot):     {e_dot:.3f} m/s
[2] θ_e (heading): {np.rad2deg(self.e_th):.2f}°
[4] Δv (speed):    {self.state.v - v_target:.3f} m/s"""
            self.state_text.set_text(state_str)

            # Update control display
            control_str = f"""CONTROL INPUTS
{'─'*25}
Steering Angle δ:
  {np.rad2deg(delta):.2f}° (max: ±45°)

Acceleration a:
  {accel:.2f} m/s²

━━━━━━━━━━━━━━━━━━━━━━━━
Feedforward: {np.rad2deg(math.atan2(L * self.ck[self.target_ind], 1)):.2f}°
Feedback:    {np.rad2deg(pi_2_pi(delta - math.atan2(L * self.ck[self.target_ind], 1))):.2f}°"""
            self.control_text.set_text(control_str)

            # Update gain display
            if K is not None:
                gain_str = f"""LQR GAIN MATRIX K
{'─'*25}
Steering gains (row 0):
  K[0,0]: {K[0,0]:.2f} (e)
  K[0,1]: {K[0,1]:.2f} (ė)
  K[0,2]: {K[0,2]:.2f} (θ_e)
  K[0,3]: {K[0,3]:.2f} (θ̇_e)
  K[0,4]: {K[0,4]:.2f} (Δv)

Speed gains (row 1):
  K[1,0]: {K[1,0]:.2f}
  K[1,4]: {K[1,4]:.2f}

Control Law: u = -Kx
  δ = -K[0,:]·x + ff
  a = -K[1,:]·x"""
                self.gain_text.set_text(gain_str)
        elif first_step:
            self.state_text.set_text("STATE VARIABLES\n" + "─" * 25 + "\nPress 'Step' to begin")
            self.control_text.set_text("CONTROL INPUTS\n" + "─" * 25 + "\nReady to start")
            self.gain_text.set_text("LQR GAIN MATRIX K\n" + "─" * 25 + "\nWill show on first step")

        self.fig.canvas.draw_idle()

    def toggle_auto_play(self, event=None):
        """Toggle auto-play mode"""
        if self.timer is None:
            self.start_auto_play()
        else:
            self.stop_auto_play()

    def start_auto_play(self):
        """Start auto-play mode"""
        if self.goal_reached:
            print("Goal already reached! Press Reset to start over.")
            return

        print("Auto-play started...")
        self.btn_auto.label.set_text('Stop')
        self.btn_auto.color = 'salmon'

        self.timer = self.fig.canvas.new_timer(interval=self.auto_play_speed)
        self.timer.add_callback(self.auto_step_callback)
        self.timer.start()

    def auto_step_callback(self):
        """Callback for auto-play timer"""
        if self.goal_reached or self.time >= 500.0:
            self.stop_auto_play()
            return

        # Stop timer if window was closed
        if not plt.fignum_exists(self.fig.number):
            self.stop_auto_play()
            return

        self.step_forward()

        # Continue timer if simulation not finished
        if not self.goal_reached and self.timer is not None:
            # Create new timer to avoid issues
            self.timer = self.fig.canvas.new_timer(interval=self.auto_play_speed)
            self.timer.add_callback(self.auto_step_callback)
            self.timer.start()

    def stop_auto_play(self):
        """Stop auto-play mode"""
        if self.timer is not None:
            self.timer.stop()
            self.timer = None

        self.btn_auto.label.set_text('Auto Play')
        self.btn_auto.color = 'lightgreen'
        print("Auto-play stopped.")

    def reset(self, event=None):
        """Reset simulation"""
        self.stop_auto_play()
        self.reset_simulation()
        print("Simulation reset. Ready to start!")

    def on_key_press(self, event):
        """Handle key press events"""
        if event.key == 'escape':
            self.stop_auto_play()
            plt.close('all')
            print("Simulation closed.")
        elif event.key == ' ':
            # Space bar for step
            self.step_forward()
        elif event.key == 'a':
            # 'a' for auto-play
            self.toggle_auto_play()
        elif event.key == 'r':
            # 'r' for reset
            self.reset()


def main():
    """Main function"""
    sim = LQRSimulation()
    plt.ioff()
    plt.show()


if __name__ == '__main__':
    main()