"""
Path tracking simulation with LQR steering control (Lookup Table version).

Based on PythonRobotics by Atsushi Sakai (@Atsushi_twi).
Modified to:
  1. Remove external dependencies (utils.angle, CubicSpline) - fully self-contained
  2. Pre-compute K lookup table keyed by discretized velocity for O(1) online query

Usage:
  python lqr_steer_control_lut.py
"""
import math
import numpy as np
import scipy.linalg as la
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# === Parameters ===
Kp = 1.0        # Speed P-controller gain

# LQR cost matrices
# State: [e, e_dot, th_e, th_e_dot]
Q = np.eye(4)
R = np.eye(1)

dt = 0.1         # Time step [s]
L = 0.5          # Wheelbase [m]
max_steer = np.deg2rad(45.0)  # Max steering angle [rad]

show_animation = True

# === Lookup Table Config ===
V_MIN = 0.05     # Min velocity for table [m/s]
V_MAX = 8.0      # Max velocity for table [m/s]
V_STEP = 0.05    # Velocity discretization step [m/s]


# =====================================================================
# Inline utilities (replacing external PythonRobotics dependencies)
# =====================================================================

def angle_mod(x):
    """Normalize angle to [-pi, pi]."""
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def pi_2_pi(angle):
    return angle_mod(angle)


def calc_spline_course(x_list, y_list, ds=0.1):
    """
    Generate a smooth course from waypoints using scipy CubicSpline.
    Returns: cx, cy, cyaw, ck, s  (positions, yaw, curvature, arc-length)
    """
    # Compute cumulative arc-length parameter
    dx = np.diff(x_list)
    dy = np.diff(y_list)
    ds_pts = np.hypot(dx, dy)
    s_pts = np.concatenate([[0], np.cumsum(ds_pts)])

    # Fit cubic splines
    cs_x = CubicSpline(s_pts, x_list)
    cs_y = CubicSpline(s_pts, y_list)

    # Resample at uniform arc-length intervals
    s_fine = np.arange(0, s_pts[-1], ds)

    cx = cs_x(s_fine)
    cy = cs_y(s_fine)

    # First derivatives
    dx_ds = cs_x(s_fine, 1)
    dy_ds = cs_y(s_fine, 1)

    # Yaw angle
    cyaw = np.arctan2(dy_ds, dx_ds)

    # Second derivatives
    ddx_ds = cs_x(s_fine, 2)
    ddy_ds = cs_y(s_fine, 2)

    # Curvature: k = (x'*y'' - y'*x'') / (x'^2 + y'^2)^(3/2)
    ck = (dx_ds * ddy_ds - dy_ds * ddx_ds) / (dx_ds**2 + dy_ds**2)**1.5

    return cx.tolist(), cy.tolist(), cyaw.tolist(), ck.tolist(), s_fine.tolist()


# =====================================================================
# LQR Core
# =====================================================================

def solve_DARE(A, B, Q, R, max_iter=150, eps=0.01):
    """Solve discrete-time algebraic Riccati equation by iteration."""
    X = Q.copy()
    for _ in range(max_iter):
        Xn = A.T @ X @ A - A.T @ X @ B @ la.inv(R + B.T @ X @ B) @ B.T @ X @ A + Q
        if (abs(Xn - X)).max() < eps:
            return Xn
        X = Xn
    return Xn


def dlqr(A, B, Q, R):
    """Solve discrete-time LQR: returns gain K, Riccati solution X, closed-loop eigenvalues."""
    X = solve_DARE(A, B, Q, R)
    K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)
    eigVals, _ = la.eig(A - B @ K)
    return K, X, eigVals


# =====================================================================
# Lookup Table
# =====================================================================

def build_state_space(v):
    """Build A(4x4), B(4x1) matrices for a given velocity v."""
    A = np.zeros((4, 4))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt

    B = np.zeros((4, 1))
    B[3, 0] = v / L
    return A, B


def generate_k_table():
    """
    Pre-compute K gain table for discretized velocities.
    Returns dict: { v_key_float -> K_matrix(1x4) }
    """
    table = {}
    v_values = np.arange(V_MIN, V_MAX + V_STEP / 2, V_STEP)
    print(f"Generating K lookup table: {len(v_values)} entries "
          f"(v = {V_MIN:.2f} ~ {V_MAX:.2f}, step = {V_STEP})...")

    for v in v_values:
        A, B = build_state_space(v)
        K, _, _ = dlqr(A, B, Q, R)
        key = round(v, 2)
        table[key] = K

    print(f"K table generated: {len(table)} entries.")
    return table


def lookup_K(table, v):
    """
    Look up K from table by snapping v to the nearest grid point.
    Falls back to the boundary entry if v is out of range.
    """
    v_clamped = max(V_MIN, min(v, V_MAX))
    key = round(round(v_clamped / V_STEP) * V_STEP, 2)
    # Safety: clamp key to valid range
    key = max(V_MIN, min(key, round(V_MAX, 2)))
    key = round(key, 2)
    if key in table:
        return table[key]
    # Fallback: find nearest key (should not normally happen)
    nearest_key = min(table.keys(), key=lambda k: abs(k - v_clamped))
    return table[nearest_key]


# =====================================================================
#  Vehicle model
# =====================================================================

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta):
    """Bicycle kinematic model update."""
    delta = np.clip(delta, -max_steer, max_steer)
    state.x += state.v * math.cos(state.yaw) * dt
    state.y += state.v * math.sin(state.yaw) * dt
    state.yaw += state.v / L * math.tan(delta) * dt
    state.v += a * dt
    return state


def pid_control(target, current):
    return Kp * (target - current)


# =====================================================================
# Nearest index search
# =====================================================================

def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [idx**2 + idy**2 for (idx, idy) in zip(dx, dy)]
    mind = min(d)
    ind = d.index(mind)
    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y
    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


# =====================================================================
# LQR Steering Control  (lookup-table version)
# =====================================================================

def lqr_steering_control(state, cx, cy, cyaw, ck, pe, pth_e, k_table):
    """
    LQR steering controller using pre-computed K lookup table.
    """
    ind, e = calc_nearest_index(state, cx, cy, cyaw)

    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    # --- Lookup K instead of solving DARE online ---
    v_abs = abs(v) if abs(v) > V_MIN else V_MIN
    K = lookup_K(k_table, v_abs)

    # State vector
    x = np.zeros((4, 1))
    x[0, 0] = e
    x[1, 0] = (e - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt

    # Feedforward + feedback
    ff = math.atan2(L * k, 1)
    fb = pi_2_pi((-K @ x)[0, 0])
    delta = ff + fb

    return delta, ind, e, th_e


# =====================================================================
# Speed profile
# =====================================================================

def calc_speed_profile(cx, cy, cyaw, target_speed):
    speed_profile = [target_speed] * len(cx)
    direction = 1.0

    for i in range(len(cx) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0
        if switch:
            direction *= -1
        if direction != 1.0:
            speed_profile[i] = -target_speed
        else:
            speed_profile[i] = target_speed
        if switch:
            speed_profile[i] = 0.0

    speed_profile[-1] = 0.0
    return speed_profile


# =====================================================================
# Closed-loop simulation
# =====================================================================

def closed_loop_prediction(cx, cy, cyaw, ck, speed_profile, goal, k_table):
    T = 500.0
    goal_dis = 0.3
    stop_speed = 0.05

    state = State(x=0.0, y=0.0, yaw=np.deg2rad(90), v=0.0)

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]

    e, e_th = 0.0, 0.0

    while T >= time:
        dl, target_ind, e, e_th = lqr_steering_control(
            state, cx, cy, cyaw, ck, e, e_th, k_table)

        ai = pid_control(speed_profile[target_ind], state.v)
        state = update(state, ai, dl)

        if abs(state.v) <= stop_speed:
            target_ind += 1

        time += dt

        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.hypot(dx, dy) <= goal_dis:
            print("Goal reached!")
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if show_animation:
            plt.cla()
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title(f"speed: {state.v * 3.6:.2f} km/h, target idx: {target_ind}")
            plt.pause(0.0001)

    return t, x, y, yaw, v


# =====================================================================
# Main
# =====================================================================

def main():
    print("LQR steering control (Lookup Table version) start!")

    # --- Step 1: Generate K lookup table (one-time) ---
    k_table = generate_k_table()

    # --- Step 2: Define waypoints and generate smooth trajectory ---
    ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
    goal = [ax[-1], ay[-1]]

    cx, cy, cyaw, ck, s = calc_spline_course(ax, ay, ds=0.1)
    target_speed = 20.0 / 3.6  # [m/s]

    sp = calc_speed_profile(cx, cy, cyaw, target_speed)

    # --- Step 3: Run closed-loop simulation ---
    t, x, y, yaw, v = closed_loop_prediction(cx, cy, cyaw, ck, sp, goal, k_table)

    # --- Step 4: Plot results ---
    if show_animation:
        plt.close()

        plt.subplots(1)
        plt.plot(ax, ay, "xb", label="waypoints")
        plt.plot(cx, cy, "-r", label="spline course")
        plt.plot(x, y, "-g", label="LQR tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.legend()

        plt.subplots(1)
        plt.plot(s, [np.rad2deg(iy) for iy in cyaw], "-r", label="yaw")
        plt.grid(True)
        plt.legend()
        plt.xlabel("arc length [m]")
        plt.ylabel("yaw [deg]")

        plt.subplots(1)
        plt.plot(s, ck, "-r", label="curvature")
        plt.grid(True)
        plt.legend()
        plt.xlabel("arc length [m]")
        plt.ylabel("curvature [1/m]")

        plt.show()


if __name__ == '__main__':
    main()
