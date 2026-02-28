"""
实验: RRT 算法参数影响研究
日期: 2026-01-20
目标: 研究不同参数对 RRT 性能的影响

路径: PythonRoboticsLearning/experiments/rrt_comparison/rrt_parameter_study.py
"""

import sys
import time
import matplotlib.pyplot as plt
import numpy as np

# 添加原始项目到路径
sys.path.append('/home/zhufeng/code/PythonRobotics/PythonRobotics')

from PathPlanning.RRT import rrt


def run_experiment(params, show_animation=False):
    """
    运行单次 RRT 实验

    参数:
        params: 包含 RRT 参数的字典
        show_animation: 是否显示动画

    返回:
        dict: 包含路径长度、节点数、运行时间的字典
    """
    # 创建 RRT 对象
    rrt_planner = rrt.RRT(
        start=params['start'],
        goal=params['goal'],
        rand_area=params['rand_area'],
        obstacle_list=params['obstacle_list'],
        expand_dis=params['expand_dis'],
        path_resolution=params['path_resolution'],
        goal_sample_rate=params['goal_sample_rate'],
        max_iter=params['max_iter'],
        robot_radius=params.get('robot_radius', 0.8)
    )

    # 运行规划
    start_time = time.time()
    path = rrt_planner.planning(animation=show_animation)
    elapsed_time = time.time() - start_time

    if path is None:
        return {
            'success': False,
            'path_length': None,
            'node_count': len(rrt_planner.node_list),
            'time': elapsed_time
        }

    # 计算路径长度
    path_length = 0.0
    for i in range(len(path) - 1):
        dx = path[i+1][0] - path[i][0]
        dy = path[i+1][1] - path[i][1]
        path_length += np.hypot(dx, dy)

    return {
        'success': True,
        'path_length': path_length,
        'node_count': len(rrt_planner.node_list),
        'time': elapsed_time,
        'path': path
    }


def experiment_1_expand_dis():
    """
    实验 1: 研究 expand_dis 参数的影响

    扩展距离决定了树每次迭代的生长步长
    """
    print("\n" + "="*60)
    print("实验 1: expand_dis 参数影响")
    print("="*60)

    # 基础参数
    base_params = {
        'start': [0, 0],
        'goal': [6, 10],
        'rand_area': [-2, 15],
        'obstacle_list': [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2),
                         (7, 5, 2), (9, 5, 2), (8, 10, 1)],
        'path_resolution': 0.5,
        'goal_sample_rate': 5,
        'max_iter': 500,
        'robot_radius': 0.8
    }

    # 测试不同的 expand_dis 值
    expand_dis_values = [1.0, 2.0, 3.0, 5.0, 8.0]
    results = []

    for expand_dis in expand_dis_values:
        print(f"\n测试 expand_dis = {expand_dis}")

        # 运行 10 次取平均值
        success_count = 0
        total_length = 0
        total_nodes = 0
        total_time = 0

        for trial in range(10):
            params = base_params.copy()
            params['expand_dis'] = expand_dis

            result = run_experiment(params, show_animation=False)

            if result['success']:
                success_count += 1
                total_length += result['path_length']
                total_nodes += result['node_count']
                total_time += result['time']

        if success_count > 0:
            avg_length = total_length / success_count
            avg_nodes = total_nodes / success_count
            avg_time = total_time / 10  # 包括失败的尝试
            success_rate = success_count / 10 * 100

            results.append({
                'expand_dis': expand_dis,
                'success_rate': success_rate,
                'avg_length': avg_length,
                'avg_nodes': avg_nodes,
                'avg_time': avg_time
            })

            print(f"  成功率: {success_rate:.1f}%")
            print(f"  平均路径长度: {avg_length:.2f}")
            print(f"  平均节点数: {avg_nodes:.1f}")
            print(f"  平均时间: {avg_time:.3f}s")
        else:
            print(f"  ❌ 所有尝试均失败")

    # 可视化结果
    if results:
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))

        # Success Rate
        ax1 = axes[0, 0]
        x = [r['expand_dis'] for r in results]
        y = [r['success_rate'] for r in results]
        ax1.plot(x, y, 'o-', linewidth=2, markersize=8)
        ax1.set_xlabel('expand_dis')
        ax1.set_ylabel('Success Rate (%)')
        ax1.set_title('Success Rate vs Expansion Distance')
        ax1.grid(True)

        # Path Length
        ax2 = axes[0, 1]
        y = [r['avg_length'] for r in results]
        ax2.plot(x, y, 's-', linewidth=2, markersize=8, color='green')
        ax2.set_xlabel('expand_dis')
        ax2.set_ylabel('Average Path Length')
        ax2.set_title('Path Length vs Expansion Distance')
        ax2.grid(True)

        # Node Count
        ax3 = axes[1, 0]
        y = [r['avg_nodes'] for r in results]
        ax3.plot(x, y, '^-', linewidth=2, markersize=8, color='orange')
        ax3.set_xlabel('expand_dis')
        ax3.set_ylabel('Average Node Count')
        ax3.set_title('Node Count vs Expansion Distance')
        ax3.grid(True)

        # Computation Time
        ax4 = axes[1, 1]
        y = [r['avg_time'] for r in results]
        ax4.plot(x, y, 'd-', linewidth=2, markersize=8, color='red')
        ax4.set_xlabel('expand_dis')
        ax4.set_ylabel('Average Time (s)')
        ax4.set_title('Computation Time vs Expansion Distance')
        ax4.grid(True)

        plt.tight_layout()
        plt.savefig('/home/zhufeng/code/PythonRobotics/PythonRoboticsLearning/experiments/rrt_comparison/expand_dis_results.png')
        print(f"\n📊 结果已保存到: experiments/rrt_comparison/expand_dis_results.png")
        plt.close()


def experiment_2_goal_sample_rate():
    """
    实验 2: 研究 goal_sample_rate 参数的影响

    目标采样率决定了以目标点作为随机采样点的概率
    """
    print("\n" + "="*60)
    print("实验 2: goal_sample_rate 参数影响")
    print("="*60)

    # 基础参数
    base_params = {
        'start': [0, 0],
        'goal': [6, 10],
        'rand_area': [-2, 15],
        'obstacle_list': [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2),
                         (7, 5, 2), (9, 5, 2), (8, 10, 1)],
        'expand_dis': 3.0,
        'path_resolution': 0.5,
        'max_iter': 500,
        'robot_radius': 0.8
    }

    # 测试不同的 goal_sample_rate 值
    goal_rate_values = [0, 5, 10, 20, 50]
    results = []

    for goal_rate in goal_rate_values:
        print(f"\n测试 goal_sample_rate = {goal_rate}%")

        # 运行 10 次取平均值
        success_count = 0
        total_length = 0
        total_nodes = 0
        total_time = 0

        for trial in range(10):
            params = base_params.copy()
            params['goal_sample_rate'] = goal_rate

            result = run_experiment(params, show_animation=False)

            if result['success']:
                success_count += 1
                total_length += result['path_length']
                total_nodes += result['node_count']
                total_time += result['time']

        if success_count > 0:
            avg_length = total_length / success_count
            avg_nodes = total_nodes / success_count
            avg_time = total_time / 10
            success_rate = success_count / 10 * 100

            results.append({
                'goal_rate': goal_rate,
                'success_rate': success_rate,
                'avg_length': avg_length,
                'avg_nodes': avg_nodes,
                'avg_time': avg_time
            })

            print(f"  成功率: {success_rate:.1f}%")
            print(f"  平均路径长度: {avg_length:.2f}")
            print(f"  平均节点数: {avg_nodes:.1f}")
            print(f"  平均时间: {avg_time:.3f}s")

    # 可视化结果
    if results:
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))

        x = [r['goal_rate'] for r in results]

        # Success Rate
        ax1 = axes[0, 0]
        y = [r['success_rate'] for r in results]
        ax1.plot(x, y, 'o-', linewidth=2, markersize=8)
        ax1.set_xlabel('goal_sample_rate (%)')
        ax1.set_ylabel('Success Rate (%)')
        ax1.set_title('Success Rate vs Goal Sample Rate')
        ax1.grid(True)

        # Path Length
        ax2 = axes[0, 1]
        y = [r['avg_length'] for r in results]
        ax2.plot(x, y, 's-', linewidth=2, markersize=8, color='green')
        ax2.set_xlabel('goal_sample_rate (%)')
        ax2.set_ylabel('Average Path Length')
        ax2.set_title('Path Length vs Goal Sample Rate')
        ax2.grid(True)

        # Node Count
        ax3 = axes[1, 0]
        y = [r['avg_nodes'] for r in results]
        ax3.plot(x, y, '^-', linewidth=2, markersize=8, color='orange')
        ax3.set_xlabel('goal_sample_rate (%)')
        ax3.set_ylabel('Average Node Count')
        ax3.set_title('Node Count vs Goal Sample Rate')
        ax3.grid(True)

        # Computation Time
        ax4 = axes[1, 1]
        y = [r['avg_time'] for r in results]
        ax4.plot(x, y, 'd-', linewidth=2, markersize=8, color='red')
        ax4.set_xlabel('goal_sample_rate (%)')
        ax4.set_ylabel('Average Time (s)')
        ax4.set_title('Computation Time vs Goal Sample Rate')
        ax4.grid(True)

        plt.tight_layout()
        plt.savefig('/home/zhufeng/code/PythonRobotics/PythonRoboticsLearning/experiments/rrt_comparison/goal_sample_rate_results.png')
        print(f"\n📊 结果已保存到: experiments/rrt_comparison/goal_sample_rate_results.png")
        plt.close()


def main():
    """主函数: 运行所有实验"""
    print("="*60)
    print("RRT 算法参数影响研究")
    print("="*60)

    # 运行实验 1
    experiment_1_expand_dis()

    # 运行实验 2
    experiment_2_goal_sample_rate()

    print("\n" + "="*60)
    print("所有实验完成！")
    print("="*60)


if __name__ == '__main__':
    main()
