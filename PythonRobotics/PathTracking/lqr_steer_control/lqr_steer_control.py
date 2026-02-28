"""
Path tracking simulation with LQR steering control and PID speed control.

author Atsushi Sakai (@Atsushi_twi)

本脚本是一个路径跟踪仿真程序，其中：
- 转向控制：使用LQR（线性二次调节器）
- 速度控制：使用PID控制器（实际为P控制器）
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

# --- 参数定义 ---

Kp = 1.0  # 速度P控制器增益

# LQR 控制器参数
# Q 矩阵: 状态代价矩阵, 4x4的单位矩阵。
# 分别对应状态量 [e, e_dot, th_e, th_e_dot] 的权重
# e: 横向误差 (lateral error)
# e_dot: 横向误差的变化率
# th_e: 航向误差 (heading error)
# th_e_dot: 航向误差的变化率
Q = np.eye(4)
# R 矩阵: 控制输入代价矩阵, 1x1的单位矩阵。
# 对应控制量 [delta] (转向角) 的权重
R = np.eye(1)

# 系统参数
dt = 0.1  # 时间步长 [s]
L = 0.5  # 车辆轴距 [m]
max_steer = np.deg2rad(45.0)  # 最大转向角 [rad]

show_animation = True  # 是否显示动画

#  show_animation = False


class State:
    """
    车辆状态类
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """
        初始化车辆状态
        :param x: x坐标 [m]
        :param y: y坐标 [m]
        :param yaw: 航向角 [rad]
        :param v: 速度 [m/s]
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta):
    """
    根据自行车运动学模型更新车辆状态
    :param state: 当前状态
    :param a: 加速度 [m/s^2]
    :param delta: 转向角 [rad]
    :return: 更新后的状态
    """
    # 输入限制
    if delta >= max_steer:
        delta = max_steer
    if delta <= - max_steer:
        delta = - max_steer

    # 状态更新方程
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt

    return state


def pid_control(target, current):
    """
    速度P控制器
    :param target: 目标速度
    :param current: 当前速度
    :return: 加速度指令
    """
    a = Kp * (target - current)

    return a


def pi_2_pi(angle):
    """
    将角度归一化到 [-pi, pi] 范围
    """
    return angle_mod(angle)


def solve_DARE(A, B, Q, R):
    """
    通过迭代法求解离散时间代数黎卡提方程 (DARE)
    方程形式: X = A.T @ X @ A - (A.T @ X @ B) @ inv(R + B.T @ X @ B) @ (B.T @ X @ A) + Q
    """
    X = Q
    Xn = Q
    max_iter = 150  # 最大迭代次数
    eps = 0.01      # 收敛阈值

    for i in range(max_iter):
        Xn = A.T @ X @ A - A.T @ X @ B @ \
            la.inv(R + B.T @ X @ B) @ B.T @ X @ A + Q
        # 判断是否收敛
        if (abs(Xn - X)).max() < eps:
            break
        X = Xn

    return Xn


def dlqr(A, B, Q, R):
    """
    求解离散时间LQR控制器
    系统模型: x[k+1] = A x[k] + B u[k]
    代价函数: sum(x[k].T*Q*x[k] + u[k].T*R*u[k])
    """

    # 1. 求解DARE方程得到解X
    X = solve_DARE(A, B, Q, R)

    # 2. 计算LQR增益K
    # K = (B.T * X * B + R)^-1 * (B.T * X * A)
    K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

    # 计算闭环系统特征值，用于分析稳定性
    eigVals, eigVecs = la.eig(A - B @ K)

    return K, X, eigVals


def lqr_steering_control(state, cx, cy, cyaw, ck, pe, pth_e):
    """
    LQR转向控制器
    :param state: 当前车辆状态
    :param cx, cy, cyaw, ck: 参考轨迹的x, y, 航向角, 曲率
    :param pe: 上一时刻的横向误差
    :param pth_e: 上一时刻的航向误差
    :return:
        delta: 期望转向角 [rad]
        ind: 最近的路点索引
        e: 当前横向误差
        th_e: 当前航向误差
    """
    # 查找最近的路点
    ind, e = calc_nearest_index(state, cx, cy, cyaw)

    k = ck[ind]  # 参考点曲率
    v = state.v  # 当前速度
    th_e = pi_2_pi(state.yaw - cyaw[ind])  # 航向误差

    # 状态空间模型 A, B 矩阵
    # 状态量 x = [e, e_dot, th_e, th_e_dot].T
    # e_dot = v * sin(th_e)
    # th_e_dot = v * tan(delta) / L - v * k
    # 线性化后:
    # e_dot ≈ v * th_e
    # th_e_dot ≈ v * delta / L - v * k
    A = np.zeros((4, 4))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt
    # print(A)

    B = np.zeros((4, 1))
    B[3, 0] = v / L

    # 使用dlqr求解器计算最优反馈增益K
    K, _, _ = dlqr(A, B, Q, R)

    # 定义状态向量x
    x = np.zeros((4, 1))

    x[0, 0] = e
    x[1, 0] = (e - pe) / dt          # 横向误差的微分
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt    # 航向误差的微分

    # 控制量 u = -Kx
    # 包含前馈(feedforward)和反馈(feedback)两部分
    # 前馈控制: 基于路径曲率，用于补偿稳态误差
    ff = math.atan2(L * k, 1)
    # 反馈控制: 基于LQR计算的误差反馈
    fb = pi_2_pi((-K @ x)[0, 0])

    # 最终控制量 = 前馈 + 反馈
    delta = ff + fb

    return delta, ind, e, th_e


def calc_nearest_index(state, cx, cy, cyaw):
    """
    计算车辆到参考轨迹最近点的索引和横向误差
    """
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    # 计算车辆到所有轨迹点的距离平方
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    # 找到最小距离的点的索引
    mind = min(d)
    ind = d.index(mind)

    # 距离开方得到实际距离
    mind = math.sqrt(mind)

    # 计算误差的正负号
    # 通过车辆位置和最近点航向的矢量关系判断车辆在轨迹左侧还是右侧
    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y
    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1  # 车辆在轨迹右侧为负

    return ind, mind


def closed_loop_prediction(cx, cy, cyaw, ck, speed_profile, goal):
    """
    闭环仿真主函数
    :param cx, cy, cyaw, ck: 参考轨迹
    :param speed_profile: 目标速度曲线
    :param goal: 目标点 [x, y]
    :return: 仿真结果
    """
    T = 500.0  # 最大仿真时间
    goal_dis = 0.3  # 到达目标的距离阈值
    stop_speed = 0.05  # 停止速度阈值

    # 初始化车辆状态
    state = State(x=-0.0, y=-0.0, yaw=np.deg2rad(90), v=0.0)

    time = 0.0
    # 用于记录仿真过程数据的列表
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]

    e, e_th = 0.0, 0.0  # 初始化误差

    # 仿真主循环
    while T >= time:
        # 1. 计算LQR转向控制指令
        dl, target_ind, e, e_th = lqr_steering_control(
            state, cx, cy, cyaw, ck, e, e_th)

        # 2. 计算PID速度控制指令
        ai = pid_control(speed_profile[target_ind], state.v)
        # 3. 更新车辆状态
        state = update(state, ai, dl)

        # 如果速度过低，强制前进到下一个目标点，防止卡住
        if abs(state.v) <= stop_speed:
            target_ind += 1

        time = time + dt

        # 检查是否到达终点
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.hypot(dx, dy) <= goal_dis:
            print("Goal")
            break

        # 记录数据
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        # 动画显示
        if target_ind % 1 == 0 and show_animation:
            plt.cla()
            # 监听ESC键，用于退出仿真
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed[km/h]:" + str(round(state.v * 3.6, 2))
                      + ",target index:" + str(target_ind))
            plt.pause(0.0001)

    return t, x, y, yaw, v


def calc_speed_profile(cx, cy, cyaw, target_speed):
    """
    计算目标速度曲线，处理转弯和倒车情况
    """
    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # 1.0: 前进, -1.0: 后退

    # 遍历路径点，设置停车点和倒车点
    for i in range(len(cx) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])
        # 判断是否是急转弯（航向角变化较大）
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1  # 切换行驶方向

        if direction != 1.0:
            speed_profile[i] = - target_speed  # 设置为后退速度
        else:
            speed_profile[i] = target_speed   # 设置为前进速度

        if switch:
            speed_profile[i] = 0.0  # 在急转弯点停车

    speed_profile[-1] = 0.0  # 最后一个点速度为0

    return speed_profile


def main():
    """
    主函数
    """
    print("LQR steering control tracking start!!")
    # 定义路径点
    ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
    goal = [ax[-1], ay[-1]]

    # 使用三次样条插值生成平滑轨迹
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)
    target_speed = 20.0 / 3.6  # 目标速度 [m/s]

    # 计算速度曲线
    sp = calc_speed_profile(cx, cy, cyaw, target_speed)

    # 运行闭环仿真
    t, x, y, yaw, v = closed_loop_prediction(cx, cy, cyaw, ck, sp, goal)

    # 绘制最终结果
    if show_animation:  # pragma: no cover
        plt.close()
        # 绘制路径和轨迹
        plt.subplots(1)
        plt.plot(ax, ay, "xb", label="input")
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
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