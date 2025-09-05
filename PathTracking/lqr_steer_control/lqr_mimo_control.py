"""

Path tracking simulation with LQR steering control and PID speed control.

author Atsushi Sakai (@Atsushi_twi)

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

Kp = 1.0  # speed proportional gain

# LQR参数 - 双输入控制（速度和角速度）
Q = np.diag([1.0, 1.0, 1.0, 1.0])  # 状态权重矩阵 [e, e_dot, th_e, th_e_dot]
R = np.diag([1.0, 0.5])  # 控制权重矩阵 [v, omega] - 速度和角速度

# parameters
dt = 0.1  # time tick[s]
L = 0.5  # Wheelbase of the vehicle [m]
max_steer = np.deg2rad(45.0)  # maximum steering angle[rad]
max_angular_vel = 5.0 # maximum angular velocity [rad/s]
g_target_speed = 10.0 / 3.6 # target speed [m/s]

show_animation = True
#  show_animation = False


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update_velocity_angular(state, v_cmd, omega_cmd):
    """
    车辆运动学模型更新函数（速度和角速度控制）
    
    基于当前状态、速度命令和角速度命令，计算下一时刻的车辆状态
    
    Parameters
    ----------
    state : State
        当前车辆状态（位置x,y、航向角yaw、速度v）
    v_cmd : float
        速度命令 [m/s]
    omega_cmd : float
        角速度命令 [rad/s]
        
    Returns
    -------
    state : State
        更新后的车辆状态
    """
    
    # 限制角速度在物理范围内
    omega_cmd = np.clip(omega_cmd, -max_angular_vel, max_angular_vel)
    v_cmd = max(0, v_cmd)  # 速度不能为负

    # 位置更新：基于当前速度和航向角
    # x_{k+1} = x_k + v_k * cos(yaw_k) * dt
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    # y_{k+1} = y_k + v_k * sin(yaw_k) * dt  
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    
    # 航向角更新：基于角速度命令
    # yaw_{k+1} = yaw_k + omega_cmd * dt
    state.yaw = state.yaw + omega_cmd * dt
    
    # 速度更新：基于速度命令
    # v_{k+1} = v_cmd
    state.v = v_cmd

    return state


def pid_control(target, current):
    a = Kp * (target - current)

    return a


def pi_2_pi(angle):
    return angle_mod(angle)


def solve_DARE(A, B, Q, R):
    """
    求解离散时间代数黎卡提方程 (Discrete-time Algebraic Riccati Equation, DARE)
    
    该方程是LQR控制理论的核心，用于求解最优控制问题的价值函数矩阵P。
    DARE的形式为：
    P = A^T * P * A - A^T * P * B * (R + B^T * P * B)^(-1) * B^T * P * A + Q
    
    其中P是价值函数矩阵，满足Bellman最优性原理。
    
    Parameters
    ----------
    A : numpy.ndarray
        系统状态转移矩阵 (n×n)
    B : numpy.ndarray  
        控制输入矩阵 (n×m)
    Q : numpy.ndarray
        状态权重矩阵 (n×n)，通常为半正定矩阵
    R : numpy.ndarray
        控制权重矩阵 (m×m)，通常为正定矩阵
        
    Returns
    -------
    P : numpy.ndarray
        价值函数矩阵 (n×n)，满足DARE方程
        用于计算LQR最优反馈增益 K = (R + B^T*P*B)^(-1) * B^T*P*A
        
    Notes
    -----
    1. 算法原理：使用迭代法求解，从初始猜测P=Q开始
    2. 收敛条件：当连续两次迭代的差值小于阈值时停止
    3. 数值稳定性：需要系统(A,B)可控且Q≥0, R>0
    4. 时间复杂度：O(n³) per iteration，通常收敛很快
        
    Mathematical Background
    ----------------------
    DARE来源于离散时间LQR问题的最优性条件：
    min J = Σ(x^T*Q*x + u^T*R*u)
    s.t. x[k+1] = A*x[k] + B*u[k]
    
    最优控制律：u* = -K*x，其中 K = (R + B^T*P*B)^(-1) * B^T*P*A
    价值函数：V(x) = x^T*P*x
    """
    
    # 初始化：使用Q作为价值函数矩阵P的初始猜测
    # 这是一个合理的初始值，因为Q通常反映了对状态的重视程度
    X = Q.copy()  # 当前迭代的P矩阵
    Xn = Q.copy() # 下一次迭代的P矩阵
    
    # 迭代参数设置
    max_iter = 150  # 最大迭代次数，防止无限循环
    eps = 0.01      # 收敛阈值，当||P_new - P_old||_∞ < eps时停止
    
    # 迭代求解DARE方程
    for i in range(max_iter):
        # DARE方程的迭代形式：
        # P_{k+1} = A^T * P_k * A - A^T * P_k * B * (R + B^T * P_k * B)^(-1) * B^T * P_k * A + Q
        
        # 计算中间项 (R + B^T * P * B)^(-1)
        # 这个矩阵的逆存在，因为R是正定的，B^T*P*B是半正定的
        intermediate = la.inv(R + B.T @ X @ B)
        
        # 计算完整的DARE方程
        # 第一项：A^T * P * A (状态转移的二次型)
        # 第二项：A^T * P * B * (R + B^T * P * B)^(-1) * B^T * P * A (控制项)
        # 第三项：Q (状态权重)
        Xn = A.T @ X @ A - A.T @ X @ B @ intermediate @ B.T @ X @ A + Q
        
        # 检查收敛性：计算两次迭代之间的最大绝对差值
        # 使用无穷范数 ||P_new - P_old||_∞ = max|P_new[i,j] - P_old[i,j]|
        if (abs(Xn - X)).max() < eps:
            print(f"DARE收敛于第 {i+1} 次迭代")
            break
            
        # 更新P矩阵，准备下一次迭代
        X = Xn.copy()
    
    # 检查是否达到最大迭代次数
    if i == max_iter - 1:
        print(f"警告：DARE在{max_iter}次迭代后未收敛")
        print(f"最终误差：{(abs(Xn - X)).max():.6f}")
    
    return Xn


def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = solve_DARE(A, B, Q, R)

    # compute the LQR gain
    K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

    eigVals, eigVecs = la.eig(A - B @ K)

    return K, X, eigVals


def lqr_dual_control(state, cx, cy, cyaw, ck, target_speed, pe, pth_e, pv):
    """
    LQR双输入控制（速度和角速度）
    
    状态向量: [e, e_dot, th_e, th_e_dot]
    控制向量: [v, omega] - 速度和角速度
    """
    ind, e = calc_nearest_index(state, cx, cy, cyaw)

    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    # 系统矩阵A (4x4)
    A = np.zeros((4, 4))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v  # 航向误差对横向误差率的影响
    A[2, 2] = 1.0
    A[2, 3] = dt

    # 控制矩阵B (4x2) - 双输入控制（速度和角速度）
    B = np.zeros((4, 2))
    #B[1, 0] = 1.0    # 速度对横向误差率的影响
    #B[3, 0] = k      # 速度对航向误差变化率的影响（通过曲率）
    #B[1, 1] = 1.0    # 角速度对横向误差率的影响（间接）
    B[3, 1] = 1.0    # 角速度对航向误差变化率的影响（直接）

    K, _, _ = dlqr(A, B, Q, R)

    # 状态向量
    x = np.zeros((4, 1))
    x[0, 0] = e                    # 横向误差
    x[1, 0] = (e - pe) / dt       # 横向误差变化率
    x[2, 0] = th_e                # 航向误差
    x[3, 0] = (th_e - pth_e) / dt # 航向误差变化率

    # LQR控制律
    u = -K @ x
    
    # 前馈控制
    ff_v = target_speed              # 目标速度
    ff_omega = target_speed * k      # 基于曲率的前馈角速度
    
    # 总控制量
    v_cmd = u[0, 0] + ff_v
    omega_cmd = u[1, 0] + ff_omega
    
    # 限制控制量
    v_cmd = max(0, v_cmd)  # 速度不能为负
    omega_cmd = np.clip(omega_cmd, -max_angular_vel, max_angular_vel)

    return v_cmd, omega_cmd, ind, e, th_e


# 计算到参考路径最近点的索引和横向误差
def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    # 最小距离的平方
    mind = min(d)

    # 最小距离的索引
    ind = d.index(mind)

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    # >0 表示路径在车辆左侧，<0 表示路径在车辆右侧
    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def closed_loop_prediction(cx, cy, cyaw, ck, target_speed, goal):
    """
    闭环轨迹跟踪仿真 - 双输入LQR控制
    """
    T = 500.0  # max simulation time
    goal_dis = 0.3
    stop_speed = 0.05

    state = State(x=0.0, y=0.0, yaw=0.0, v=10.0)

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]

    # 误差历史
    e, e_th, e_v = 0.0, 0.0, 0.0

    while T >= time:
        # 双输入LQR控制（速度和角速度）
        v_cmd, omega_cmd, target_ind, e, e_th = lqr_dual_control(
            state, cx, cy, cyaw, ck, target_speed, e, e_th, e_v)

        # 状态更新（直接使用速度和角速度命令）
        state = update_velocity_angular(state, v_cmd, omega_cmd)

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
            plt.title(f"Velocity-Angular LQR - Speed: {state.v*3.6:.1f} km/h, Target: {target_ind}")
            plt.legend()
            plt.pause(0.0001)

    return t, x, y, yaw, v


def calc_speed_profile(cx, cy, cyaw, target_speed):
    """
    计算速度规划
    
    根据路径的曲率和方向，设置速度规划，包括停车点和速度方向变化
    
    Parameters
    ----------
    cx : list
        路径点的x坐标序列
    cy : list
        路径点的y坐标序列
    cyaw : list
        路径点的航向角序列
    target_speed : float
        目标速度 [m/s]
        
    Returns
    -------
    speed_profile : list
        速度规划列表
    """
    
    # 初始化速度规划，全部设置为目标速度
    speed_profile = [target_speed] * len(cx)

    direction = 1.0

    # Set stop point
    for i in range(len(cx) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])
        switch = math.pi / 4. <= dyaw < math.pi

        if switch:
            direction *= -1

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

        if switch:
            speed_profile[i] = 0.0

    speed_profile[-1] = 0.0

    return speed_profile


def main():
    print("LQR速度和角速度控制轨迹跟踪开始!!")
    ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
    goal = [ax[-1], ay[-1]]

    # 计算三次样条路径
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)
    target_speed = g_target_speed  # simulation parameter km/h -> m/s

    # 运行速度和角速度LQR控制仿真
    t, x, y, yaw, v = closed_loop_prediction(cx, cy, cyaw, ck, target_speed, goal)

    if show_animation:  # pragma: no cover
        plt.close()
        plt.subplots(1)
        plt.plot(ax, ay, "xb", label="input")
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="velocity-angular LQR tracking")
        # sc = plt.scatter(cx, cy, c=ck, cmap="viridis", s=20)
        # cbar = plt.colorbar(sc)
        # cbar.set_label("curvature [1/m]")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots(1)
        plt.plot(s, [np.rad2deg(iyaw) for iyaw in cyaw], "-r", label="yaw")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("yaw angle[deg]")

        plt.subplots(1)
        plt.plot(s, ck, "-r", label="curvature")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("curvature [1/m]")

        plt.show()


if __name__ == '__main__':
    main()
