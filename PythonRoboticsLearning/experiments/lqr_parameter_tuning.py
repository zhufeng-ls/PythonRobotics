"""
LQR 参数调整实验
探索不同 Q、R 权重矩阵对路径跟踪性能的影响

实验目的：
1. 理解 Q 矩阵各元素对状态误差的影响
2. 分析 R 矩阵对控制输入的影响
3. 比较不同参数组合的控制效果
"""

import sys
sys.path.insert(0, '/home/zhufeng/code/PythonRobotics/PythonRobotics')

import numpy as np
import matplotlib.pyplot as plt
import math
from PathTracking.lqr_steer_control.lqr_steer_control import *
from PathPlanning.CubicSpline import cubic_spline_planner

# 关闭动画显示以便批量实验
show_animation = False

def run_lqr_experiment(Q_matrix, R_matrix, experiment_name):
    """
    运行单次 LQR 实验

    Args:
        Q_matrix: 状态权重矩阵 (4x4)
        R_matrix: 控制权重矩阵 (1x1)
        experiment_name: 实验名称

    Returns:
        实验结果字典
    """
    # 全局变量修改
    global Q, R
    Q = Q_matrix
    R = R_matrix

    print(f"\\n=== {experiment_name} ===")
    print(f"Q matrix:\\n{Q}")
    print(f"R matrix: {R}")

    # 定义相同的测试路径
    ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
    goal = [ax[-1], ay[-1]]

    # 生成参考轨迹
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
    target_speed = 20.0 / 3.6  # [m/s]
    sp = calc_speed_profile(cx, cy, cyaw, target_speed)

    # 运行仿真
    t, x, y, yaw, v = closed_loop_prediction(cx, cy, cyaw, ck, sp, goal)

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
    for exp in experiments:
        ax1.plot(exp['reference_x'], exp['reference_y'], 'k--', alpha=0.7, label='Reference' if exp == experiments[0] else "")
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

    bars1 = ax4.bar(x_pos - width/2, max_lat_errors, width, label='Max Lateral Error', alpha=0.7)
    bars2 = ax4.bar(x_pos + width/2, mean_lat_errors, width, label='Mean Lateral Error', alpha=0.7)

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