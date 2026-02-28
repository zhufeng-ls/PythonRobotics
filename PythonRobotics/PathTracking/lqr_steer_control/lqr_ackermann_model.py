"""
LQR轨迹跟踪控制 - Ackermann转向模型

完整的Ackermann转向几何模型，考虑内外轮转向角差异。

Ackermann转向几何：
- 内轮转向角 δ_inner = arctan(L / (R - d/2))
- 外轮转向角 δ_outer = arctan(L / (R + d/2))
- 中心转向角 δ_center (用于控制)

对比Bicycle模型：
- Bicycle: 单一转向角δ，忽略左右轮差异
- Ackermann: 考虑内外轮转向角差异，更真实

车辆尺寸：
- 轴距 L = 0.5m (前后轮距离)
- 轮距 d = 0.35m (左右轮距离)

Author: AI Assistant
"""

import scipy.linalg as la
import matplotlib.pyplot as plt
import math
import numpy as np
import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
from utils.angle import angle_mod
from PathPlanning.CubicSpline import cubic_spline_planner

# LQR参数
Q = np.eye(4)  # 状态权重矩阵 [e, ė, θ_e, θ̇_e]
R = np.eye(1)  # 控制权重矩阵 [δ_center]

# 车辆参数（Ackermann几何）
dt = 0.1  # 时间步长 [s]
L = 0.5   # 轴距 [m] (前后轮距离)
d = 0.35  # 轮距 [m] (左右轮距离)
max_steer = np.deg2rad(45.0)  # 最大转向角 [rad]
target_speed = 30.0 / 3.6  # 目标速度 [m/s]

# PID速度控制参数
Kp = 1.0

show_animation = True


class State:
    """车辆状态类"""
    
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x      # x坐标 [m]
        self.y = y      # y坐标 [m] 
        self.yaw = yaw  # 航向角 [rad]
        self.v = v      # 速度 [m/s]


def ackermann_geometry(delta_center, v):
    """
    Ackermann转向几何计算
    
    给定中心转向角，计算内外轮转向角和瞬时转弯半径
    
    Parameters
    ----------
    delta_center : float
        中心等效转向角 [rad]
    v : float
        速度 [m/s]
        
    Returns
    -------
    delta_inner : float
        内轮转向角 [rad]
    delta_outer : float
        外轮转向角 [rad]
    R : float
        转弯半径 [m]
    omega : float
        角速度 [rad/s]
    """
    
    if abs(delta_center) < 1e-6:
        # 直行
        return 0.0, 0.0, float('inf'), 0.0
    
    # 从中心转向角计算转弯半径
    # 使用Bicycle模型的近似：R = L / tan(δ)
    R = L / math.tan(delta_center)
    
    # 计算内外轮转向角（Ackermann几何）
    # 内轮（转向侧）
    delta_inner = math.atan2(L, abs(R) - d/2)
    
    # 外轮
    delta_outer = math.atan2(L, abs(R) + d/2)
    
    # 保持转向方向
    if delta_center < 0:
        delta_inner = -delta_inner
        delta_outer = -delta_outer
    
    # 计算角速度
    # 使用车辆中心的转弯半径
    omega = v / R if abs(R) > 1e-6 else 0.0
    
    return delta_inner, delta_outer, R, omega


def update_ackermann(state, a, delta_center):
    """
    Ackermann运动模型更新函数
    
    考虑内外轮转向角差异的完整Ackermann模型
    
    Parameters
    ----------
    state : State
        当前车辆状态
    a : float
        纵向加速度 [m/s²]
    delta_center : float
        中心等效转向角 [rad]
        
    Returns
    -------
    state : State
        更新后的车辆状态
    """
    
    # 限制转向角
    delta_center = np.clip(delta_center, -max_steer, max_steer)
    
    # 计算Ackermann几何
    delta_inner, delta_outer, R, omega = ackermann_geometry(delta_center, state.v)
    
    # 位置更新（使用车辆中心）
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    
    # 航向角更新（使用计算出的角速度）
    state.yaw = state.yaw + omega * dt
    
    # 速度更新
    state.v = state.v + a * dt
    
    return state


def update_ackermann_simplified(state, a, delta_center):
    """
    Ackermann运动模型（简化计算）
    
    直接使用Bicycle公式，但会在后处理中显示内外轮转向角差异
    这样LQR控制器可以保持与Bicycle一致
    """
    
    delta_center = np.clip(delta_center, -max_steer, max_steer)
    
    # 位置更新
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    
    # 航向角更新（Bicycle公式）
    state.yaw = state.yaw + state.v * math.tan(delta_center) / L * dt
    
    # 速度更新
    state.v = state.v + a * dt
    
    return state


def pid_control(target, current):
    """PID速度控制"""
    a = Kp * (target - current)
    return a


def pi_2_pi(angle):
    """角度归一化到[-π, π]"""
    return angle_mod(angle)


def solve_DARE(A, B, Q, R):
    """
    求解离散时间代数黎卡提方程 (DARE)
    """
    X = Q
    Xn = Q
    max_iter = 150
    eps = 0.01

    for i in range(max_iter):
        Xn = A.T @ X @ A - A.T @ X @ B @ \
            la.inv(R + B.T @ X @ B) @ B.T @ X @ A + Q
        if (abs(Xn - X)).max() < eps:
            break
        X = Xn

    return Xn


def dlqr(A, B, Q, R):
    """求解离散时间LQR控制器"""
    X = solve_DARE(A, B, Q, R)
    K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)
    eigVals, eigVecs = la.eig(A - B @ K)
    return K, X, eigVals


def lqr_steering_control(state, cx, cy, cyaw, ck, pe, pth_e):
    """
    LQR转向控制（Ackermann模型）
    
    状态向量: [横向误差, 横向误差率, 航向误差, 航向误差率]
    控制输入: 中心转向角 δ_center
    
    注意：LQR计算与Bicycle模型相同，因为都是控制中心转向角
    区别在于update函数中如何应用转向角
    """
    # 找到最近路径点
    ind, e = calc_nearest_index(state, cx, cy, cyaw)
    
    # 获取当前曲率和速度
    k = ck[ind]
    v = max(state.v, 0.1)
    th_e = pi_2_pi(state.yaw - cyaw[ind])
    
    # 构建系统矩阵A
    A = np.zeros((4, 4))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt
    
    # 构建控制矩阵B
    # 对于Ackermann，仍然使用 v/L
    # 因为控制的是中心转向角，近似与Bicycle相同
    B = np.zeros((4, 1))
    B[3, 0] = v / L
    
    # 求解LQR增益
    K, _, _ = dlqr(A, B, Q, R)
    
    # 构建状态向量
    x = np.zeros((4, 1))
    x[0, 0] = e
    x[1, 0] = (e - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt
    
    # 前馈 + 反馈
    ff = math.atan2(L * k, 1.0)  # 参考转向角
    fb = (-K @ x)[0, 0]
    delta_center = ff + fb
    
    return delta_center, ind, e, th_e


def calc_nearest_index(state, cx, cy, cyaw):
    """
    计算到参考路径最近点的索引和横向误差
    """
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
    
    mind = min(d)
    ind = d.index(mind)
    mind = math.sqrt(mind)
    
    # 计算横向误差的符号
    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y
    
    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1
    
    return ind, mind


def closed_loop_prediction(cx, cy, cyaw, ck, speed_profile, goal, use_full_ackermann=True):
    """
    闭环轨迹跟踪仿真
    
    Parameters
    ----------
    use_full_ackermann : bool
        True: 使用完整Ackermann几何（考虑内外轮差异）
        False: 使用简化版本（Bicycle近似）
    """
    T = 500.0
    goal_dis = 0.3
    stop_speed = 0.05
    
    state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)
    
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    deltas_center = [0.0]
    deltas_inner = [0.0]
    deltas_outer = [0.0]
    errors = []
    
    e, e_th = 0.0, 0.0
    
    update_func = update_ackermann if use_full_ackermann else update_ackermann_simplified
    
    while T >= time:
        # LQR转向控制
        delta_center, target_ind, e, e_th = lqr_steering_control(
            state, cx, cy, cyaw, ck, e, e_th)
        
        # 计算内外轮转向角（用于记录和显示）
        delta_inner, delta_outer, R, omega = ackermann_geometry(delta_center, state.v)
        
        # PID速度控制
        ai = pid_control(speed_profile[target_ind], state.v)
        
        # 状态更新
        state = update_func(state, ai, delta_center)
        
        if abs(state.v) <= stop_speed:
            target_ind = min(target_ind + 1, len(cx) - 1)
        
        time = time + dt
        
        # 检查是否到达目标
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.hypot(dx, dy) <= goal_dis:
            print("Goal reached!")
            break
        
        # 记录数据
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        deltas_center.append(delta_center)
        deltas_inner.append(delta_inner)
        deltas_outer.append(delta_outer)
        errors.append(abs(e))
        
        # 实时动画
        if target_ind % 1 == 0 and show_animation:
            plt.cla()
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(cx, cy, "-r", label="reference path")
            plt.plot(x, y, "ob", label="vehicle trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title(f"Ackermann Model - Speed: {state.v*3.6:.1f} km/h\n"
                     f"δ_inner: {np.rad2deg(delta_inner):.1f}°, δ_outer: {np.rad2deg(delta_outer):.1f}°")
            plt.legend()
            plt.pause(0.0001)
    
    return t, x, y, yaw, v, deltas_center, deltas_inner, deltas_outer, errors


def calc_speed_profile(cx, cy, cyaw, target_speed):
    """计算速度规划"""
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


def main():
    """主函数"""
    print("=" * 70)
    print("LQR轨迹跟踪控制 - Ackermann转向模型")
    print("=" * 70)
    print(f"车辆参数:")
    print(f"  轴距 L = {L} m (前后轮距离)")
    print(f"  轮距 d = {d} m (左右轮距离)")
    print(f"  车辆尺寸: {L} × {d} m")
    print(f"  最大转向角: {np.rad2deg(max_steer):.1f}°")
    print(f"  目标速度: {target_speed*3.6:.1f} km/h")
    print("")
    print("Ackermann转向特点:")
    print("  ✓ 考虑内外轮转向角差异")
    print("  ✓ 内轮转向角 > 中心转向角 > 外轮转向角")
    print("  ✓ 更真实的车辆转向模型")
    print("=" * 70)
    
    # 定义路径点
    ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
    goal = [ax[-1], ay[-1]]
    
    # 生成样条路径
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
    
    # 计算速度规划
    sp = calc_speed_profile(cx, cy, cyaw, target_speed)
    
    print("\n选择运行模式:")
    print("1. 完整Ackermann模型（考虑内外轮差异）")
    print("2. 简化Ackermann模型（Bicycle近似）")
    
    import sys
    if sys.stdin.isatty():
        choice = input("请选择 (1/2，默认1): ").strip()
    else:
        choice = "1"
        print("自动选择: 1 (完整Ackermann模型)")
    
    use_full = (choice != "2")
    model_name = "Full Ackermann" if use_full else "Simplified (Bicycle)"
    
    print(f"\n使用模型: {model_name}")
    print("开始仿真...")
    
    # 运行仿真
    t, x, y, yaw, v, d_center, d_inner, d_outer, errors = closed_loop_prediction(
        cx, cy, cyaw, ck, sp, goal, use_full_ackermann=use_full)
    
    print(f"\n性能统计:")
    print(f"  平均横向误差: {np.mean(errors):.4f} m")
    print(f"  最大横向误差: {np.max(errors):.4f} m")
    print(f"  RMS横向误差: {np.sqrt(np.mean(np.array(errors)**2)):.4f} m")
    
    # 结果可视化
    if show_animation:
        plt.close("all")
        
        fig = plt.figure(figsize=(16, 10))
        
        # 创建子图
        ax1 = plt.subplot(2, 3, 1)
        ax1.plot(ax, ay, "xb", label="waypoints", markersize=10)
        ax1.plot(cx, cy, "-r", label="reference path", linewidth=2)
        ax1.plot(x, y, "-g", label="ackermann tracking", linewidth=2)
        ax1.grid(True)
        ax1.axis("equal")
        ax1.set_xlabel("x [m]")
        ax1.set_ylabel("y [m]")
        ax1.set_title(f"Path Tracking - {model_name}")
        ax1.legend()
        
        # 速度曲线
        ax2 = plt.subplot(2, 3, 2)
        ax2.plot(t, [iv * 3.6 for iv in v], "-b", linewidth=2)
        ax2.axhline(y=target_speed*3.6, color='r', linestyle='--')
        ax2.grid(True)
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Speed [km/h]")
        ax2.set_title("Speed Profile")
        
        # 转向角对比
        ax3 = plt.subplot(2, 3, 3)
        ax3.plot(t, [np.rad2deg(d) for d in d_inner], "-r", linewidth=2, label="Inner wheel")
        ax3.plot(t, [np.rad2deg(d) for d in d_center], "-g", linewidth=2, label="Center (control)")
        ax3.plot(t, [np.rad2deg(d) for d in d_outer], "-b", linewidth=2, label="Outer wheel")
        ax3.axhline(y=np.rad2deg(max_steer), color='k', linestyle='--', alpha=0.3)
        ax3.axhline(y=-np.rad2deg(max_steer), color='k', linestyle='--', alpha=0.3)
        ax3.grid(True)
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("Steering Angle [deg]")
        ax3.set_title("Ackermann Steering Angles")
        ax3.legend()
        
        # 转向角差异
        ax4 = plt.subplot(2, 3, 4)
        diff_inner = [np.rad2deg(d_inner[i] - d_center[i]) for i in range(len(t))]
        diff_outer = [np.rad2deg(d_center[i] - d_outer[i]) for i in range(len(t))]
        ax4.plot(t, diff_inner, "-r", linewidth=2, label="Inner - Center")
        ax4.plot(t, diff_outer, "-b", linewidth=2, label="Center - Outer")
        ax4.grid(True)
        ax4.set_xlabel("Time [s]")
        ax4.set_ylabel("Angle Difference [deg]")
        ax4.set_title("Ackermann Angle Difference")
        ax4.legend()
        ax4.text(0.5, 0.95, f"轮距d={d}m越大，差异越明显", 
                transform=ax4.transAxes, ha='center', va='top',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.3))
        
        # 横向误差
        ax5 = plt.subplot(2, 3, 5)
        ax5.plot(t[1:], errors, "-m", linewidth=2)
        ax5.axhline(y=np.mean(errors), color='r', linestyle='--', 
                    label=f'Mean: {np.mean(errors):.3f}m')
        ax5.grid(True)
        ax5.set_xlabel("Time [s]")
        ax5.set_ylabel("Lateral Error [m]")
        ax5.set_title("Tracking Error")
        ax5.legend()
        
        # 曲率
        ax6 = plt.subplot(2, 3, 6)
        ax6.plot(s, ck, "-r", linewidth=2)
        ax6.grid(True)
        ax6.set_xlabel("Path length [m]")
        ax6.set_ylabel("Curvature [1/m]")
        ax6.set_title("Path Curvature")
        
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    main()

