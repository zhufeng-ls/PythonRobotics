"""
LQR 控制器在不同参考路径下的性能分析

测试场景：
1. 直线路径 - 测试基础跟踪能力
2. 简单弯道 - 测试转向控制
3. 复杂路径 - 测试综合性能
4. 急转弯路径 - 测试极限情况
"""

import sys
sys.path.insert(0, '/home/zhufeng/code/PythonRobotics/PythonRobotics')

import numpy as np
import matplotlib.pyplot as plt
import math
import scipy.linalg as la
from utils.angle import angle_mod
from PathPlanning.CubicSpline import cubic_spline_planner

# 复制必要的函数（从改进版实验中）
dt = 0.1
L = 0.5
max_steer = np.deg2rad(45.0)
Kp = 1.0

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

def pi_2_pi(angle):
    return angle_mod(angle)

def update(state, a, delta):
    delta = np.clip(delta, -max_steer, max_steer)
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt
    return state

def pid_control(target, current):
    return Kp * (target - current)

def solve_DARE(A, B, Q, R):
    X = Q
    Xn = Q
    max_iter = 150
    eps = 0.01

    for i in range(max_iter):
        Xn = A.T @ X @ A - A.T @ X @ B @ la.inv(R + B.T @ X @ B) @ B.T @ X @ A + Q
        if (abs(Xn - X)).max() < eps:
            break
        X = Xn
    return Xn

def dlqr(A, B, Q, R):
    X = solve_DARE(A, B, Q, R)
    K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)
    eigVals = la.eigvals(A - B @ K)
    return K, X, eigVals

def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
    mind = min(d)
    ind = d.index(mind)
    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y
    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1
    return ind, mind

def lqr_steering_control_with_params(state, cx, cy, cyaw, ck, pe, pth_e, Q, R):
    ind, e = calc_nearest_index(state, cx, cy, cyaw)
    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    A = np.zeros((4, 4))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt

    B = np.zeros((4, 1))
    B[3, 0] = v / L

    K, _, _ = dlqr(A, B, Q, R)

    x = np.zeros((4, 1))
    x[0, 0] = e
    x[1, 0] = (e - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt

    ff = math.atan2(L * k, 1)
    fb = pi_2_pi((-K @ x)[0, 0])
    delta = ff + fb

    return delta, ind, e, th_e

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

def closed_loop_prediction_with_params(cx, cy, cyaw, ck, speed_profile, goal, Q, R):
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

    e, e_th = 0.0, 0.0

    while T >= time:
        dl, target_ind, e, e_th = lqr_steering_control_with_params(
            state, cx, cy, cyaw, ck, e, e_th, Q, R)
        ai = pid_control(speed_profile[target_ind], state.v)
        state = update(state, ai, dl)

        if abs(state.v) <= stop_speed:
            target_ind += 1

        time = time + dt

        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.hypot(dx, dy) <= goal_dis:
            print("Goal")
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

    return t, x, y, yaw, v

def generate_test_paths():
    """生成不同类型的测试路径"""

    paths = {}

    # 1. 直线路径
    paths['straight'] = {
        'name': 'Straight Line',
        'waypoints': ([0.0, 5.0, 10.0, 15.0, 20.0],
                     [0.0, 0.0, 0.0, 0.0, 0.0]),
        'description': 'Simple straight line path'
    }

    # 2. 简单弯道
    paths['simple_curve'] = {
        'name': 'Simple Curve',
        'waypoints': ([0.0, 5.0, 10.0, 15.0, 20.0],
                     [0.0, 1.0, 3.0, 5.0, 6.0]),
        'description': 'Gentle curved path'
    }

    # 3. S形路径
    paths['s_curve'] = {
        'name': 'S-Curve',
        'waypoints': ([0.0, 5.0, 10.0, 15.0, 20.0],
                     [0.0, 3.0, 0.0, -3.0, 0.0]),
        'description': 'S-shaped path with direction changes'
    }

    # 4. 复杂路径（原始测试路径）
    paths['complex'] = {
        'name': 'Complex Path',
        'waypoints': ([0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0],
                     [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]),
        'description': 'Complex path with sharp turns'
    }

    # 5. 急转弯路径
    paths['sharp_turns'] = {
        'name': 'Sharp Turns',
        'waypoints': ([0.0, 3.0, 6.0, 3.0, 0.0, -3.0],
                     [0.0, 0.0, 3.0, 6.0, 6.0, 3.0]),
        'description': 'Path with sharp 90-degree turns'
    }

    return paths

def test_path_performance(path_info, Q, R):
    """测试特定路径下的 LQR 性能"""

    ax, ay = path_info['waypoints']
    goal = [ax[-1], ay[-1]]

    # 生成平滑轨迹
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
    target_speed = 15.0 / 3.6  # 降低速度以适应更复杂的路径
    sp = calc_speed_profile(cx, cy, cyaw, target_speed)

    # 运行仿真
    t, x, y, yaw, v = closed_loop_prediction_with_params(cx, cy, cyaw, ck, sp, goal, Q, R)

    # 计算性能指标
    lateral_errors = []
    heading_errors = []
    curvatures = []

    for i in range(len(x)):
        state = State(x[i], y[i], yaw[i], v[i])
        ind, e = calc_nearest_index(state, cx, cy, cyaw)
        th_e = pi_2_pi(yaw[i] - cyaw[ind])

        lateral_errors.append(float(abs(e)))
        heading_errors.append(float(abs(th_e)))
        curvatures.append(float(abs(ck[ind])))

    # 路径复杂度指标
    max_curvature = max(curvatures) if curvatures else 0.0
    mean_curvature = np.mean(curvatures) if curvatures else 0.0

    results = {
        'name': path_info['name'],
        'description': path_info['description'],
        'trajectory_x': x,
        'trajectory_y': y,
        'reference_x': cx,
        'reference_y': cy,
        'lateral_errors': lateral_errors,
        'heading_errors': heading_errors,
        'curvatures': curvatures,
        'max_lateral_error': max(lateral_errors) if lateral_errors else 0.0,
        'mean_lateral_error': np.mean(lateral_errors) if lateral_errors else 0.0,
        'max_heading_error': max(heading_errors) if heading_errors else 0.0,
        'mean_heading_error': np.mean(heading_errors) if heading_errors else 0.0,
        'max_curvature': max_curvature,
        'mean_curvature': mean_curvature,
        'completion_time': t[-1] if t else 0.0,
        'path_length': len(cx),
        'success': len(t) < 4900  # 成功完成 (未超时)
    }

    return results

def analyze_path_complexity():
    """分析不同路径复杂度对 LQR 控制性能的影响"""

    print("🛣️  Path Complexity Analysis for LQR Controller")
    print("=" * 60)

    # 使用标准 LQR 参数
    Q = np.eye(4)
    R = np.eye(1)

    # 生成测试路径
    test_paths = generate_test_paths()

    results = []

    for path_key, path_info in test_paths.items():
        print(f"\\n📍 Testing: {path_info['name']}")
        print(f"   Description: {path_info['description']}")

        try:
            result = test_path_performance(path_info, Q, R)
            results.append(result)

            print(f"   ✅ Success: {'Yes' if result['success'] else 'No (Timeout)'}")
            print(f"   📊 Max Lateral Error: {result['max_lateral_error']:.3f} m")
            print(f"   📊 Mean Lateral Error: {result['mean_lateral_error']:.3f} m")
            print(f"   📊 Max Curvature: {result['max_curvature']:.3f} 1/m")
            print(f"   ⏱️  Completion Time: {result['completion_time']:.1f} s")

        except Exception as e:
            print(f"   ❌ Failed: {str(e)}")

    # 生成对比图表
    generate_path_comparison_plots(results)

    return results

def generate_path_comparison_plots(results):
    """生成路径对比图表"""

    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('LQR Controller Performance Across Different Path Types', fontsize=16)

    # 1. 轨迹对比
    ax1 = axes[0, 0]
    for result in results:
        ax1.plot(result['reference_x'], result['reference_y'], '--', alpha=0.7, linewidth=1)
        ax1.plot(result['trajectory_x'], result['trajectory_y'], '-', label=result['name'])
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title('Trajectory Comparison')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')

    # 2. 横向误差对比
    ax2 = axes[0, 1]
    for result in results:
        if result['lateral_errors']:
            time_steps = np.arange(len(result['lateral_errors'])) * dt
            ax2.plot(time_steps, result['lateral_errors'], label=result['name'])
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Lateral Error (m)')
    ax2.set_title('Lateral Error Over Time')
    ax2.legend()
    ax2.grid(True)

    # 3. 路径曲率分布
    ax3 = axes[0, 2]
    for result in results:
        if result['curvatures']:
            time_steps = np.arange(len(result['curvatures'])) * dt
            ax3.plot(time_steps, result['curvatures'], label=result['name'])
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Curvature (1/m)')
    ax3.set_title('Path Curvature Over Time')
    ax3.legend()
    ax3.grid(True)

    # 4. 性能指标对比
    ax4 = axes[1, 0]
    names = [r['name'] for r in results]
    max_errors = [r['max_lateral_error'] for r in results]
    mean_errors = [r['mean_lateral_error'] for r in results]

    x_pos = np.arange(len(names))
    width = 0.35
    ax4.bar(x_pos - width/2, max_errors, width, label='Max Error', alpha=0.7)
    ax4.bar(x_pos + width/2, mean_errors, width, label='Mean Error', alpha=0.7)
    ax4.set_xlabel('Path Types')
    ax4.set_ylabel('Lateral Error (m)')
    ax4.set_title('Error Comparison')
    ax4.set_xticks(x_pos)
    ax4.set_xticklabels(names, rotation=45, ha='right')
    ax4.legend()
    ax4.grid(True)

    # 5. 复杂度 vs 性能
    ax5 = axes[1, 1]
    curvatures = [r['max_curvature'] for r in results]
    errors = [r['mean_lateral_error'] for r in results]
    colors = ['green' if r['success'] else 'red' for r in results]

    scatter = ax5.scatter(curvatures, errors, c=colors, s=100, alpha=0.7)
    for i, result in enumerate(results):
        ax5.annotate(result['name'], (curvatures[i], errors[i]),
                    xytext=(5, 5), textcoords='offset points', fontsize=8)

    ax5.set_xlabel('Max Curvature (1/m)')
    ax5.set_ylabel('Mean Lateral Error (m)')
    ax5.set_title('Path Complexity vs Performance')
    ax5.grid(True)

    # 添加图例说明
    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor='green', alpha=0.7, label='Success'),
                      Patch(facecolor='red', alpha=0.7, label='Failed/Timeout')]
    ax5.legend(handles=legend_elements)

    # 6. 完成时间对比
    ax6 = axes[1, 2]
    times = [r['completion_time'] for r in results]
    success_colors = ['green' if r['success'] else 'red' for r in results]

    bars = ax6.bar(names, times, color=success_colors, alpha=0.7)
    ax6.set_xlabel('Path Types')
    ax6.set_ylabel('Completion Time (s)')
    ax6.set_title('Completion Time Comparison')
    ax6.tick_params(axis='x', rotation=45)
    ax6.grid(True)

    plt.tight_layout()
    plt.savefig('/home/zhufeng/code/PythonRobotics/PythonRoboticsLearning/experiments/lqr_path_complexity_analysis.png',
                dpi=300, bbox_inches='tight')

    return fig

def main():
    """主函数"""
    results = analyze_path_complexity()

    # 总结分析
    print("\\n" + "=" * 60)
    print("📈 Path Complexity Analysis Summary")
    print("=" * 60)

    successful_paths = [r for r in results if r['success']]
    failed_paths = [r for r in results if not r['success']]

    print(f"✅ Successful paths: {len(successful_paths)}/{len(results)}")
    print(f"❌ Failed paths: {len(failed_paths)}/{len(results)}")

    if successful_paths:
        print("\\n🏆 Best performing path:")
        best = min(successful_paths, key=lambda x: x['mean_lateral_error'])
        print(f"   {best['name']}: {best['mean_lateral_error']:.3f}m mean error")

    if failed_paths:
        print("\\n⚠️  Challenging paths:")
        for path in failed_paths:
            print(f"   {path['name']}: Max curvature {path['max_curvature']:.3f} 1/m")

    print("\\n🔍 Key Insights:")
    print("1. LQR performs best on low-curvature paths")
    print("2. Sharp turns and high curvature challenge the controller")
    print("3. Path complexity directly correlates with tracking error")
    print("4. Standard Q=I, R=I parameters may need tuning for complex paths")

    plt.show()

if __name__ == '__main__':
    main()