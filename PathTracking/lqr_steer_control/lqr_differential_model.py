"""
LQR轨迹跟踪控制 - 差速运动模型

基于差速驱动的车辆运动学模型，适用于机器人、AGV等双轮差速驱动系统。
差速模型通过控制左右轮的速度差来实现转向，是移动机器人常用的运动模型。

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

# PID速度控制参数
Kp = 1.0  # 速度比例增益

# LQR参数
Q = np.eye(4)  # 状态权重矩阵
R = np.eye(1)  # 控制权重矩阵

# 车辆参数
dt = 0.1  # 时间步长 [s]
L = 0.5   # 轮距 [m] (左右轮距离)
max_angular_vel = 0.5  # 最大角速度 [rad/s]

show_animation = True


class State:
    """车辆状态类"""
    
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x      # x坐标 [m]
        self.y = y      # y坐标 [m] 
        self.yaw = yaw  # 航向角 [rad]
        self.v = v      # 线速度 [m/s]


def update_differential(state, a, omega):
    """
    差速运动模型更新函数
    
    差速驱动模型：
    - 通过控制左右轮速度差实现转向
    - 线速度 v = (v_left + v_right) / 2
    - 角速度 ω = (v_right - v_left) / L
    
    Parameters
    ----------
    state : State
        当前车辆状态
    a : float
        线加速度 [m/s²]
    omega : float
        角速度 [rad/s]
        
    Returns
    -------
    state : State
        更新后的车辆状态
    """
    
    # 限制角速度
    if omega >= max_angular_vel:
        omega = max_angular_vel
    if omega <= -max_angular_vel:
        omega = -max_angular_vel
    
    # 位置更新
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    
    # 航向角更新 (直接使用角速度)
    state.yaw = state.yaw + omega * dt
    
    # 线速度更新
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
    """求解离散时间代数黎卡提方程 (DARE)"""
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
    # 求解黎卡提方程
    X = solve_DARE(A, B, Q, R)
    
    # 计算LQR增益
    K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)
    
    # 计算闭环系统特征值
    eigVals, eigVecs = la.eig(A - B @ K)
    
    return K, X, eigVals


def lqr_steering_control(state, cx, cy, cyaw, ck, pe, pth_e):
    """
    LQR转向控制 (差速模型)
    
    状态向量: [横向误差, 横向误差率, 航向误差, 航向误差率]
    控制量: 角速度 ω
    """
    # 找到最近路径点
    ind, e = calc_nearest_index(state, cx, cy, cyaw)
    
    # 获取当前曲率和速度
    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])
    
    # 构建系统矩阵A (差速模型)
    A = np.zeros((4, 4))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt
    
    # 构建控制矩阵B (角速度直接控制航向角变化率)
    B = np.zeros((4, 1))
    B[3, 0] = 1.0  # 角速度直接控制航向角变化率
    
    # 求解LQR增益
    K, _, _ = dlqr(A, B, Q, R)
    
    # 构建状态向量
    x = np.zeros((4, 1))
    x[0, 0] = e                    # 横向误差
    x[1, 0] = (e - pe) / dt        # 横向误差率
    x[2, 0] = th_e                 # 航向误差
    x[3, 0] = (th_e - pth_e) / dt  # 航向误差率
    
    # 前馈控制 (基于曲率)
    ff = v * k  # 角速度 = 线速度 × 曲率
    
    # 反馈控制 (LQR)
    fb = (-K @ x)[0, 0]
    
    # 总角速度
    omega = ff + fb
    
    return omega, ind, e, th_e


def calc_nearest_index(state, cx, cy, cyaw):
    """计算到参考路径最近点的索引和横向误差"""
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


def closed_loop_prediction(cx, cy, cyaw, ck, speed_profile, goal):
    """闭环轨迹跟踪仿真"""
    T = 500.0  # 最大仿真时间
    goal_dis = 0.3
    stop_speed = 0.05
    
    # 初始状态
    state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)
    
    # 记录历史数据
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    
    # 误差历史
    e, e_th = 0.0, 0.0
    
    while T >= time:
        # LQR转向控制
        omega, target_ind, e, e_th = lqr_steering_control(
            state, cx, cy, cyaw, ck, e, e_th)
        
        # PID速度控制
        ai = pid_control(speed_profile[target_ind], state.v)
        
        # 状态更新 (差速模型)
        state = update_differential(state, ai, omega)
        
        # 低速时推进目标点
        if abs(state.v) <= stop_speed:
            target_ind += 1
        
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
            plt.title(f"Differential Model - Speed: {state.v*3.6:.1f} km/h, Target: {target_ind}")
            plt.legend()
            plt.pause(0.0001)
    
    return t, x, y, yaw, v


def calc_speed_profile(cx, cy, cyaw, target_speed):
    """计算速度规划"""
    speed_profile = [target_speed] * len(cx)
    direction = 1.0
    
    # 根据航向变化设置停车点
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
    print("LQR轨迹跟踪控制 - 差速运动模型")
    
    # 定义路径点
    ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
    goal = [ax[-1], ay[-1]]
    
    # 生成样条路径
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)
    target_speed = 0.4  # km/h -> m/s
    
    # 计算速度规划
    sp = calc_speed_profile(cx, cy, cyaw, target_speed)
    
    # 运行仿真
    t, x, y, yaw, v = closed_loop_prediction(cx, cy, cyaw, ck, sp, goal)
    
    # 结果可视化
    if show_animation:
        plt.close()
        
        # 路径跟踪结果
        plt.subplots(1)
        plt.plot(ax, ay, "xb", label="waypoints")
        plt.plot(cx, cy, "-r", label="reference path")
        plt.plot(x, y, "-g", label="differential tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title("Differential Model - Path Tracking")
        plt.legend()
        
        # 航向角变化
        plt.subplots(1)
        plt.plot(s, [np.rad2deg(iyaw) for iyaw in cyaw], "-r", label="reference yaw")
        plt.grid(True)
        plt.legend()
        plt.xlabel("path length [m]")
        plt.ylabel("yaw angle [deg]")
        plt.title("Reference Yaw Angle")
        
        # 曲率变化
        plt.subplots(1)
        plt.plot(s, ck, "-r", label="curvature")
        plt.grid(True)
        plt.legend()
        plt.xlabel("path length [m]")
        plt.ylabel("curvature [1/m]")
        plt.title("Path Curvature")
        
        plt.show()


if __name__ == '__main__':
    main()
