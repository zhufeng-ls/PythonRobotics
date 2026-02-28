"""
LQR轨迹跟踪控制 - Bicycle运动模型 (C++风格实现)

基于C++割草机控制器的运动模型，使用全局坐标系误差表示。
特点：
1. 3维状态向量 [Δx, Δy, Δθ] （全局坐标系）
2. Bicycle运动模型：ω = v*tan(δ)/L
3. 预计算K矩阵查找表（基于ref_delta和ref_yaw）
4. 位姿预测补偿执行延迟
5. 横向误差比例补偿

对比原Python实现：
- 原版：4维路径坐标系 [e, ė, θ_e, θ̇_e]
- 本版：3维全局坐标系 [Δx, Δy, Δθ]（模仿C++）

Author: AI Assistant (参考C++割草机控制器)
"""

import scipy.linalg as la
import matplotlib.pyplot as plt
import math
import numpy as np
import sys
import pathlib
import json
from tqdm import tqdm

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
from utils.angle import angle_mod
from PathPlanning.CubicSpline import cubic_spline_planner

# LQR参数（模仿C++预计算时的Q,R）
Q = np.diag([5.0, 5.0, 5.0])  # 状态权重矩阵 [Δx, Δy, Δθ]
R = np.diag([1.0, 1.0])       # 控制权重矩阵 [Δv, Δδ]

# 车辆参数（模仿C++割草机）
dt = 0.1              # 时间步长 [s]
L = 0.489             # 轴距 [m] (前后轮距离)
max_steer = np.deg2rad(60.0)  # 最大转向角 [rad]
target_speed = 20.0 / 3.6    # 目标速度 [m/s]

# 控制参数
Kp = 1.0              # 速度比例增益
lateral_error_gain = 0.8  # 横向误差比例增益（模仿C++第585行）
prediction_time = 0.3  # 位姿预测时间 [s]（模仿C++第390行）

show_animation = True

# K矩阵查找表（运行时加载）
k_table = {}


class State:
    """车辆状态类"""
    
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x      # x坐标 [m]
        self.y = y      # y坐标 [m] 
        self.yaw = yaw  # 航向角 [rad]
        self.v = v      # 速度 [m/s]


def update_bicycle(state, a, delta):
    """
    Bicycle运动模型更新函数（模仿C++版本）
    
    简化的阿克曼模型（单一转向角）：
    - 角速度 ω = v*tan(δ)/L
    - 不考虑内外轮转向角差异
    
    Parameters
    ----------
    state : State
        当前车辆状态
    a : float
        纵向加速度 [m/s²]
    delta : float
        前轮转向角 [rad]
        
    Returns
    -------
    state : State
        更新后的车辆状态
    """
    
    # 限制转向角
    if delta >= max_steer:
        delta = max_steer
    if delta <= -max_steer:
        delta = -max_steer
    
    # 位置更新
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    
    # 航向角更新 (Bicycle模型)
    state.yaw = state.yaw + state.v * math.tan(delta) / L * dt
    
    # 速度更新
    state.v = state.v + a * dt
    
    return state


def pose_prediction(state, v, omega):
    """
    位姿预测（模仿C++第385-399行）
    
    用于补偿执行器延迟
    """
    predicted_state = State(state.x, state.y, state.yaw, state.v)
    
    # 预测未来位姿
    predicted_state.x += v * math.cos(predicted_state.yaw) * prediction_time
    predicted_state.y += v * math.sin(predicted_state.yaw) * prediction_time
    predicted_state.yaw += omega * prediction_time
    predicted_state.v = v
    
    return predicted_state


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


def get_state_space(ref_delta, ref_yaw, v):
    """
    构建状态空间矩阵（模仿C++第699-711行）
    
    全局坐标系状态向量: [Δx, Δy, Δθ]
    控制输入: [Δv, Δδ]
    """
    # 系统矩阵A（线性化离散化）
    A = np.array([
        [1.0, 0.0, -v*dt*np.sin(ref_yaw)],
        [0.0, 1.0,  v*dt*np.cos(ref_yaw)],
        [0.0, 0.0,  1.0]
    ])
    
    # 控制矩阵B
    B = np.array([
        [dt*np.cos(ref_yaw), 0],
        [dt*np.sin(ref_yaw), 0],
        [dt*np.tan(ref_delta)/L, v*dt/(L*np.cos(ref_delta)**2)]
    ])
    
    return A, B


def calculate_lateral_error(state, cx, cy, cyaw, ind):
    """
    计算横向误差（模仿C++第418-456行）
    
    点到直线的带符号距离
    """
    if ind == 0:
        p1_idx = 0
    else:
        p1_idx = ind - 1
    
    if ind == len(cx) - 1:
        p2_idx = ind
    else:
        p2_idx = ind + 1
    
    # 直线方程 Ax + By + C = 0
    a = cy[p1_idx] - cy[p2_idx]
    b = cx[p2_idx] - cx[p1_idx]
    c = cx[p1_idx] * cy[p2_idx] - cx[p2_idx] * cy[p1_idx]
    
    # 计算带符号距离
    numerator = a * state.x + b * state.y + c
    denominator = math.sqrt(a*a + b*b) + 1e-6
    signed_dist = numerator / denominator
    
    # 判断方向（叉积）
    cross_product = (cx[p2_idx] - cx[p1_idx]) * (state.y - cy[p1_idx]) - \
                    (cy[p2_idx] - cy[p1_idx]) * (state.x - cx[p1_idx])
    
    # 返回带符号的横向误差
    lateral_error = -abs(signed_dist) if cross_product > 0 else abs(signed_dist)
    return lateral_error


def lqr_steering_control_cpp_style(state, cx, cy, cyaw, ck, use_lookup_table=False):
    """
    LQR转向控制 (C++风格，全局坐标系)
    
    模仿C++第520-589行的控制逻辑
    
    状态向量: [Δx, Δy, Δθ]（全局坐标系误差）
    """
    # 1. 找到最近路径点
    ind, min_d = calc_nearest_index(state, cx, cy, cyaw)
    
    # 2. 位姿预测（补偿延迟，模仿C++第568行）
    omega_current = state.v * math.tan(0) / L  # 假设当前转向角为0
    predicted_state = pose_prediction(state, state.v, omega_current)
    
    # 3. 找到预测位姿的目标点
    target_ind, _ = calc_nearest_index(predicted_state, cx, cy, cyaw)
    
    # 4. 获取参考状态
    k = ck[target_ind]
    v = state.v
    ref_delta = math.atan2(L * k, 1)  # 参考转向角（模仿C++第572行）
    ref_yaw = cyaw[target_ind]
    
    # 5. 构建状态误差向量（全局坐标系）
    x_error = state.x - cx[target_ind]
    y_error = state.y - cy[target_ind]
    yaw_error = pi_2_pi(state.yaw - cyaw[target_ind])
    
    X = np.array([[x_error],
                  [y_error],
                  [yaw_error]])
    
    # 6. 计算LQR增益
    if use_lookup_table and k_table:
        # 使用查找表（模仿C++第777-791行）
        ref_delta_deg = int(np.rad2deg(ref_delta))
        ref_yaw_deg = int(np.rad2deg(ref_yaw))
        
        # 限制范围
        ref_delta_deg = np.clip(ref_delta_deg, -89, 89)
        ref_yaw_deg = np.clip(ref_yaw_deg, -180, 180)
        
        key = f"({ref_delta_deg},{ref_yaw_deg})"
        
        if key in k_table:
            K = np.array(k_table[key])
        else:
            # 查找失败，在线计算
            A, B = get_state_space(ref_delta, ref_yaw, v)
            K, _, _ = dlqr(A, B, Q, R)
    else:
        # 在线计算（默认）
        A, B = get_state_space(ref_delta, ref_yaw, v)
        K, _, _ = dlqr(A, B, Q, R)
    
    # 7. 计算控制量
    u = -K @ X  # u = [Δv, Δδ]
    
    # 8. 提取转向角增量
    delta_fb = u[1, 0]
    
    # 9. 总转向角（前馈+反馈，模仿C++第578行）
    delta = ref_delta + delta_fb
    
    # 限制转向角
    delta = np.clip(delta, -max_steer, max_steer)
    
    # 10. 计算横向误差并补偿（模仿C++第581-585行）
    lateral_error = calculate_lateral_error(predicted_state, cx, cy, cyaw, target_ind)
    
    # 11. 转换为角速度（考虑横向误差补偿）
    omega = v * math.tan(delta) / L + lateral_error_gain * lateral_error
    
    return omega, target_ind


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


def generate_k_table(v=0.4):
    """
    生成K矩阵查找表（模仿C++的generate_lqr.py）
    
    注意：这会生成大量数据，仅用于演示
    实际使用建议只计算需要的部分
    """
    print("生成K矩阵查找表...")
    print("警告：这会生成64000+个矩阵，可能需要几分钟")
    
    table = {}
    delta_range = range(-89, 90, 1)  # -89° ~ +89°
    yaw_range = range(-180, 181, 1)  # -180° ~ +180°
    
    total = len(delta_range) * len(yaw_range)
    
    with tqdm(total=total, desc="生成K矩阵") as pbar:
        for delta_deg in delta_range:
            for yaw_deg in yaw_range:
                delta_rad = np.deg2rad(delta_deg)
                yaw_rad = np.deg2rad(yaw_deg)
                
                # 构建A,B矩阵
                A, B = get_state_space(delta_rad, yaw_rad, v)
                
                # 求解LQR
                K, _, _ = dlqr(A, B, Q, R)
                
                # 存储
                key = f"({delta_deg},{yaw_deg})"
                table[key] = K.tolist()
                
                pbar.update(1)
    
    print(f"生成完成！共{len(table)}个K矩阵")
    return table


def save_k_table(table, filename="lqr_k_table_bicycle.json"):
    """保存K矩阵查找表"""
    data = {
        "metadata": {
            "model": "bicycle",
            "L": L,
            "dt": dt,
            "Q": Q.tolist(),
            "R": R.tolist()
        },
        "data": table
    }
    
    with open(filename, 'w') as f:
        json.dump(data, f)
    
    print(f"查找表已保存到: {filename}")


def load_k_table(filename="lqr_k_table_bicycle.json"):
    """加载K矩阵查找表"""
    global k_table
    
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
        k_table = data['data']
        print(f"已加载{len(k_table)}个K矩阵")
        return True
    except FileNotFoundError:
        print(f"查找表文件不存在: {filename}")
        return False


def closed_loop_prediction(cx, cy, cyaw, ck, speed_profile, goal, use_lookup_table=False):
    """闭环轨迹跟踪仿真"""
    T = 500.0  # 最大仿真时间
    goal_dis = 0.3
    stop_speed = 0.05
    
    # 初始状态
    state = State(x=cx[0] + 1, y=cy[0] - 0.1, yaw=np.deg2rad(90), v=0.0)
    
    # 记录历史数据
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    
    target_ind = 0
    
    while T >= time:
        # LQR转向控制
        omega, target_ind = lqr_steering_control_cpp_style(
            state, cx, cy, cyaw, ck, use_lookup_table)
        
        # PID速度控制
        ai = pid_control(speed_profile[target_ind], state.v)
        
        # 计算转向角（从角速度反推）
        if abs(state.v) > 0.01:
            delta = math.atan2(omega * L, state.v)
        else:
            delta = 0.0
        
        # 状态更新 (Bicycle模型)
        state = update_bicycle(state, ai, delta)
        
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
            plt.title(f"Bicycle Model (C++ Style) - Speed: {state.v*3.6:.1f} km/h")
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
    print("=" * 50)
    print("LQR轨迹跟踪控制 - Bicycle运动模型 (C++风格)")
    print("=" * 50)
    print(f"车辆参数: L={L}m, max_steer={np.rad2deg(max_steer)}°")
    print(f"状态空间: 3维 [Δx, Δy, Δθ]（全局坐标系）")
    print(f"运动模型: Bicycle (ω = v*tan(δ)/L)")
    print("=" * 50)
    
    # 定义路径点
    ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
    goal = [ax[-1], ay[-1]]
    
    # 生成样条路径
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)
    
    # 计算速度规划
    sp = calc_speed_profile(cx, cy, cyaw, target_speed)
    
    # 询问是否使用查找表
    use_table = False
    # print("\n选择模式:")
    # print("1. 在线计算K矩阵（快速启动，运行稍慢）")
    # print("2. 使用查找表（需要预先生成，运行极快）")
    
    # # 支持非交互式运行
    # import sys
    # if sys.stdin.isatty():
    #     choice = input("请选择 (1/2，默认1): ").strip()
    # else:
    #     choice = "1"  # 非交互模式默认选择1
    #     print("非交互模式，自动选择: 1 (在线计算)")
    choice = "1" 
    if choice == "2":
        # 尝试加载查找表
        if not load_k_table():
            print("生成新的查找表...")
            table = generate_k_table(v=target_speed)
            save_k_table(table)
            k_table.update(table)
        use_table = True
    
    print("\n开始仿真...")
    
    # 运行仿真
    t, x, y, yaw, v = closed_loop_prediction(
        cx, cy, cyaw, ck, sp, goal, use_lookup_table=use_table)
    
    # 结果可视化
    if show_animation:
        plt.close("all")
        
        # 路径跟踪结果
        plt.figure(figsize=(10, 6))
        plt.plot(ax, ay, "xb", label="waypoints", markersize=10)
        plt.plot(cx, cy, "-r", label="reference path", linewidth=2)
        plt.plot(x, y, "-g", label="Bicycle tracking (C++ style)", linewidth=2)
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title("Bicycle Model (C++ Style) - Path Tracking\n3D State Space [Δx, Δy, Δθ]")
        plt.legend()
        
        # 速度曲线
        plt.figure(figsize=(10, 4))
        plt.plot(t, [iv * 3.6 for iv in v], "-b", linewidth=2)
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [km/h]")
        plt.title("Speed Profile")
        
        plt.show()


if __name__ == '__main__':
    main()

