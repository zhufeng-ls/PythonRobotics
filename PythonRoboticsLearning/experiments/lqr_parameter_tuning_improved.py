"""
LQR 参数调整实验 - 改进版本
通过修改 LQR 函数来支持可变的 Q、R 矩阵

实验目的：
1. 理解 Q 矩阵各元素对状态误差的影响
2. 分析 R 矩阵对控制输入的影响
3. 比较不同参数组合的控制效果

修改方法：
- 复制原始 LQR 代码并修改为接受 Q、R 参数
- 保持其他功能不变
"""

import sys
sys.path.insert(0, '/home/zhufeng/code/PythonRobotics/PythonRobotics')

import numpy as np
import matplotlib.pyplot as plt
import math
import scipy.linalg as la
from utils.angle import angle_mod
from PathPlanning.CubicSpline import cubic_spline_planner

# 系统参数（复制自原始代码）
dt = 0.1  # 时间步长 [s]
L = 0.5   # 车辆轴距 [m]
max_steer = np.deg2rad(45.0)  # 最大转向角 [rad]
Kp = 1.0  # 速度P控制器增益

class State:
    """车辆状态类"""
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

def pi_2_pi(angle):
    """将角度归一化到 [-pi, pi] 范围"""
    return angle_mod(angle)

def update(state, a, delta):
    """根据自行车运动学模型更新车辆状态"""
    # 输入限制
    if delta >= max_steer:
        delta = max_steer
    if delta <= -max_steer:
        delta = -max_steer

    # 状态更新方程
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt

    return state

def pid_control(target, current):
    """速度P控制器"""
    a = Kp * (target - current)
    return a

def solve_DARE(A, B, Q, R):
    """通过迭代法求解离散时间代数黎卡提方程 (DARE)"""
    X = Q
    Xn = Q
    max_iter = 150
    eps = 0.01

    for i in range(max_iter):
        Xn = A.T @ X @ A - A.T @ X @ B @ la.inv(R + B.T @ X @ B) @ B.T @ X @ A + Q
        # 判断是否收敛
        if (abs(Xn - X)).max() < eps:
            break
        X = Xn

    return Xn

def dlqr(A, B, Q, R):
    """求解离散时间LQR控制器"""
    # 1. 求解DARE方程得到解X
    X = solve_DARE(A, B, Q, R)

    # 2. 计算LQR增益K
    K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

    # 计算闭环系统特征值，用于分析稳定性
    eigVals = la.eigvals(A - B @ K)

    return K, X, eigVals

def calc_nearest_index(state, cx, cy, cyaw):
    """计算车辆到参考轨迹最近点的索引和横向误差"""
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
    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y
    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1  # 车辆在轨迹右侧为负

    return ind, mind

def lqr_steering_control_with_params(state, cx, cy, cyaw, ck, pe, pth_e, Q, R):
    """
    修改版 LQR 转向控制器 - 接受 Q、R 参数

    Args:
        state: 当前车辆状态
        cx, cy, cyaw, ck: 参考轨迹的x, y, 航向角, 曲率
        pe: 上一时刻的横向误差
        pth_e: 上一时刻的航向误差
        Q: 状态权重矩阵 (4x4)
        R: 控制权重矩阵 (1x1)

    Returns:
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
    A = np.zeros((4, 4))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt

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
    # 前馈控制: 基于路径曲率
    ff = math.atan2(L * k, 1)
    # 反馈控制: 基于LQR计算的误差反馈
    fb = pi_2_pi((-K @ x)[0, 0])

    # 最终控制量 = 前馈 + 反馈
    delta = ff + fb

    return delta, ind, e, th_e

def calc_speed_profile(cx, cy, cyaw, target_speed):
    """计算目标速度曲线，处理转弯和倒车情况"""
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

def closed_loop_prediction_with_params(cx, cy, cyaw, ck, speed_profile, goal, Q, R):
    """
    修改版闭环仿真主函数 - 接受 Q、R 参数
    """
    T = 500.0
    goal_dis = 0.3
    stop_speed = 0.05

    # 初始化车辆状态
    state = State(x=-0.0, y=-0.0, yaw=np.deg2rad(90), v=0.0)

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]

    e, e_th = 0.0, 0.0

    # 仿真主循环
    while T >= time:
        # 1. 计算LQR转向控制指令（使用传入的Q、R参数）
        dl, target_ind, e, e_th = lqr_steering_control_with_params(
            state, cx, cy, cyaw, ck, e, e_th, Q, R)

        # 2. 计算PID速度控制指令
        ai = pid_control(speed_profile[target_ind], state.v)

        # 3. 更新车辆状态
        state = update(state, ai, dl)

        # 如果速度过低，强制前进到下一个目标点
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

    return t, x, y, yaw, v

def run_lqr_experiment(Q_matrix, R_matrix, experiment_name):
    """运行单次 LQR 实验"""

    print(f"\\n=== {experiment_name} ===")
    print(f"Q matrix:\\n{Q_matrix}")
    print(f"R matrix: {R_matrix}")

    # 定义相同的测试路径
    ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
    goal = [ax[-1], ay[-1]]

    # 生成参考轨迹
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
    target_speed = 20.0 / 3.6
    sp = calc_speed_profile(cx, cy, cyaw, target_speed)

    # 运行仿真（传入 Q、R 参数）
    t, x, y, yaw, v = closed_loop_prediction_with_params(cx, cy, cyaw, ck, sp, goal, Q_matrix, R_matrix)

    # 计算性能指标
    lateral_errors = []
    heading_errors = []

    for i in range(len(x)):
        state = State(x[i], y[i], yaw[i], v[i])
        ind, e = calc_nearest_index(state, cx, cy, cyaw)
        th_e = pi_2_pi(yaw[i] - cyaw[ind])

        lateral_errors.append(float(abs(e)))
        heading_errors.append(float(abs(th_e)))

    # 性能统计
    max_lateral_error = max(lateral_errors)
    mean_lateral_error = np.mean(lateral_errors)
    max_heading_error = max(heading_errors)
    mean_heading_error = np.mean(heading_errors)
    completion_time = t[-1]

    results = {
        'name': experiment_name,
        'trajectory_x': x,
        'trajectory_y': y,
        'reference_x': cx,
        'reference_y': cy,
        'lateral_errors': lateral_errors,
        'heading_errors': heading_errors,
        'max_lateral_error': max_lateral_error,
        'mean_lateral_error': mean_lateral_error,
        'max_heading_error': max_heading_error,
        'mean_heading_error': mean_heading_error,
        'completion_time': completion_time,
        'Q': Q_matrix.copy(),
        'R': R_matrix.copy()
    }

    print(f"Max lateral error: {max_lateral_error:.4f} m")
    print(f"Mean lateral error: {mean_lateral_error:.4f} m")
    print(f"Max heading error: {math.degrees(max_heading_error):.2f} deg")
    print(f"Mean heading error: {math.degrees(mean_heading_error):.2f} deg")
    print(f"Completion time: {completion_time:.2f} s")

    return results

def compare_q_matrix_effects():
    """比较 Q 矩阵不同元素的影响"""
    experiments = []

    # 基准实验：所有权重相等
    Q_base = np.eye(4)
    R_base = np.eye(1)
    experiments.append(run_lqr_experiment(Q_base, R_base, "Baseline (Q=I, R=I)"))

    # 实验1：强调横向误差 (增大 Q[0,0])
    Q1 = np.diag([10.0, 1.0, 1.0, 1.0])
    experiments.append(run_lqr_experiment(Q1, R_base, "High Lateral Error Weight"))

    # 实验2：强调航向误差 (增大 Q[2,2])
    Q2 = np.diag([1.0, 1.0, 10.0, 1.0])
    experiments.append(run_lqr_experiment(Q2, R_base, "High Heading Error Weight"))

    # 实验3：强调误差变化率 (增大 Q[1,1] 和 Q[3,3])
    Q3 = np.diag([1.0, 10.0, 1.0, 10.0])
    experiments.append(run_lqr_experiment(Q3, R_base, "High Error Rate Weight"))

    return experiments

def compare_r_matrix_effects():
    """比较 R 矩阵不同值的影响"""
    experiments = []
    Q_base = np.eye(4)

    # 小的控制权重 (允许大转向角)
    R1 = np.array([[0.1]])
    experiments.append(run_lqr_experiment(Q_base, R1, "Low Control Weight (R=0.1)"))

    # 基准控制权重
    R2 = np.array([[1.0]])
    experiments.append(run_lqr_experiment(Q_base, R2, "Baseline Control Weight (R=1.0)"))

    # 大的控制权重 (限制转向角)
    R3 = np.array([[10.0]])
    experiments.append(run_lqr_experiment(Q_base, R3, "High Control Weight (R=10.0)"))

    return experiments

def plot_comparison_results(experiments, title):
    """绘制实验结果对比图"""
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle(f'LQR Parameter Tuning Results - {title}', fontsize=16)

    # 轨迹对比
    ax1 = axes[0, 0]
    for i, exp in enumerate(experiments):
        if i == 0:  # 只为第一个实验绘制参考轨迹
            ax1.plot(exp['reference_x'], exp['reference_y'], 'k--', alpha=0.7, label='Reference')
        ax1.plot(exp['trajectory_x'], exp['trajectory_y'], label=exp['name'])
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title('Trajectory Comparison')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')

    # 横向误差对比
    ax2 = axes[0, 1]
    for exp in experiments:
        ax2.plot(exp['lateral_errors'], label=exp['name'])
    ax2.set_xlabel('Time Steps')
    ax2.set_ylabel('Lateral Error (m)')
    ax2.set_title('Lateral Error Over Time')
    ax2.legend()
    ax2.grid(True)

    # 航向误差对比
    ax3 = axes[1, 0]
    for exp in experiments:
        heading_errors_deg = [math.degrees(e) for e in exp['heading_errors']]
        ax3.plot(heading_errors_deg, label=exp['name'])
    ax3.set_xlabel('Time Steps')
    ax3.set_ylabel('Heading Error (degrees)')
    ax3.set_title('Heading Error Over Time')
    ax3.legend()
    ax3.grid(True)

    # 性能指标对比
    ax4 = axes[1, 1]
    names = [exp['name'] for exp in experiments]
    max_lat_errors = [exp['max_lateral_error'] for exp in experiments]
    mean_lat_errors = [exp['mean_lateral_error'] for exp in experiments]

    x_pos = np.arange(len(names))
    width = 0.35

    ax4.bar(x_pos - width/2, max_lat_errors, width, label='Max Lateral Error', alpha=0.7)
    ax4.bar(x_pos + width/2, mean_lat_errors, width, label='Mean Lateral Error', alpha=0.7)

    ax4.set_xlabel('Experiments')
    ax4.set_ylabel('Lateral Error (m)')
    ax4.set_title('Performance Metrics Comparison')
    ax4.set_xticks(x_pos)
    ax4.set_xticklabels(names, rotation=45, ha='right')
    ax4.legend()
    ax4.grid(True)

    plt.tight_layout()
    return fig

def main():
    """主函数 - 运行所有实验"""
    print("Starting LQR Parameter Tuning Experiments...")
    print("=" * 60)

    # Q 矩阵影响实验
    print("\\n🔬 Running Q Matrix Effect Experiments...")
    q_experiments = compare_q_matrix_effects()

    # R 矩阵影响实验
    print("\\n🔬 Running R Matrix Effect Experiments...")
    r_experiments = compare_r_matrix_effects()

    # 绘制结果
    print("\\n📊 Generating comparison plots...")

    fig1 = plot_comparison_results(q_experiments, "Q Matrix Effects")
    plt.savefig('/home/zhufeng/code/PythonRobotics/PythonRoboticsLearning/experiments/lqr_q_matrix_comparison.png',
                dpi=300, bbox_inches='tight')

    fig2 = plot_comparison_results(r_experiments, "R Matrix Effects")
    plt.savefig('/home/zhufeng/code/PythonRobotics/PythonRoboticsLearning/experiments/lqr_r_matrix_comparison.png',
                dpi=300, bbox_inches='tight')

    # 生成性能报告
    print("\\n📋 Performance Summary:")
    print("\\n--- Q Matrix Effects ---")
    for exp in q_experiments:
        print(f"{exp['name']:25} | Lat: {exp['mean_lateral_error']:.4f}m | "
              f"Head: {math.degrees(exp['mean_heading_error']):.2f}° | "
              f"Time: {exp['completion_time']:.1f}s")

    print("\\n--- R Matrix Effects ---")
    for exp in r_experiments:
        print(f"{exp['name']:25} | Lat: {exp['mean_lateral_error']:.4f}m | "
              f"Head: {math.degrees(exp['mean_heading_error']):.2f}° | "
              f"Time: {exp['completion_time']:.1f}s")

    # 实验结论
    print("\\n" + "=" * 60)
    print("🎯 Key Findings:")
    print("1. Higher Q[0,0] (lateral error weight) → Better path following accuracy")
    print("2. Higher Q[2,2] (heading error weight) → Smoother heading control")
    print("3. Higher Q[1,1], Q[3,3] (rate weights) → Reduced oscillations")
    print("4. Lower R (control weight) → More aggressive steering, faster response")
    print("5. Higher R (control weight) → Smoother control, may increase tracking error")
    print("6. Trade-off: Accuracy vs. Control Effort vs. Smoothness")

    plt.show()

if __name__ == '__main__':
    main()