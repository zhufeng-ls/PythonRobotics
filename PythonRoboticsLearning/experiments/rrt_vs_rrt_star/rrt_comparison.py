"""
Experiment: RRT vs RRT* Comparison
Date: 2026-01-21
Objective: Compare path quality and performance between RRT and RRT*

Path: PythonRoboticsLearning/experiments/rrt_vs_rrt_star/rrt_comparison.py
"""

import sys
import time
import matplotlib.pyplot as plt
import numpy as np

# Add original project to path
sys.path.append('/home/zhufeng/code/PythonRobotics/PythonRobotics')

from PathPlanning.RRT import rrt
from PathPlanning.RRTStar import rrt_star


def calculate_path_length(path):
    """Calculate total path length"""
    if path is None:
        return None

    length = 0.0
    for i in range(len(path) - 1):
        dx = path[i+1][0] - path[i][0]
        dy = path[i+1][1] - path[i][1]
        length += np.hypot(dx, dy)

    return length


def run_rrt_experiment(params, show_animation=False):
    """Run RRT experiment"""
    rrt_planner = rrt.RRT(
        start=params['start'],
        goal=params['goal'],
        rand_area=params['rand_area'],
        obstacle_list=params['obstacle_list'],
        expand_dis=params.get('expand_dis', 3.0),
        path_resolution=params.get('path_resolution', 0.5),
        goal_sample_rate=params.get('goal_sample_rate', 5),
        max_iter=params['max_iter'],
        robot_radius=params.get('robot_radius', 0.8)
    )

    start_time = time.time()
    path = rrt_planner.planning(animation=show_animation)
    elapsed_time = time.time() - start_time

    result = {
        'algorithm': 'RRT',
        'success': path is not None,
        'time': elapsed_time,
        'node_count': len(rrt_planner.node_list),
        'path': path
    }

    if path is not None:
        result['path_length'] = calculate_path_length(path)

    return result


def run_rrt_star_experiment(params, show_animation=False):
    """Run RRT* experiment"""
    rrt_star_planner = rrt_star.RRTStar(
        start=params['start'],
        goal=params['goal'],
        rand_area=params['rand_area'],
        obstacle_list=params['obstacle_list'],
        expand_dis=params.get('expand_dis', 3.0),
        path_resolution=params.get('path_resolution', 0.5),
        goal_sample_rate=params.get('goal_sample_rate', 20),
        max_iter=params['max_iter'],
        connect_circle_dist=params.get('connect_circle_dist', 50.0),
        search_until_max_iter=params.get('search_until_max_iter', True),
        robot_radius=params.get('robot_radius', 0.8)
    )

    start_time = time.time()
    path = rrt_star_planner.planning(animation=show_animation)
    elapsed_time = time.time() - start_time

    result = {
        'algorithm': 'RRT*',
        'success': path is not None,
        'time': elapsed_time,
        'node_count': len(rrt_star_planner.node_list),
        'path': path
    }

    if path is not None:
        result['path_length'] = calculate_path_length(path)

    return result


def run_comparison_experiment():
    """Run RRT vs RRT* comparison experiment"""
    print("\n" + "="*60)
    print("RRT vs RRT* Comparison Experiment")
    print("="*60)

    # Common parameters
    base_params = {
        'start': [0, 0],
        'goal': [6, 10],
        'rand_area': [-2, 15],
        'obstacle_list': [
            (5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2),
            (7, 5, 2), (9, 5, 2), (8, 10, 1)
        ],
        'max_iter': 300,
        'robot_radius': 0.8
    }

    # Test different iterations
    max_iter_values = [50, 100, 200, 300]
    results = []

    for max_iter in max_iter_values:
        print(f"\n{'='*60}")
        print(f"Testing with max_iter = {max_iter}")
        print(f"{'='*60}")

        # Update parameters
        params = base_params.copy()
        params['max_iter'] = max_iter
        params['expand_dis'] = 1.0  # Same as RRT* demo

        # Run RRT (5 trials)
        print("\nRunning RRT...")
        rrt_success_count = 0
        rrt_total_length = 0
        rrt_total_time = 0
        rrt_total_nodes = 0

        for trial in range(5):
            result = run_rrt_experiment(params, show_animation=False)
            if result['success']:
                rrt_success_count += 1
                rrt_total_length += result['path_length']
                rrt_total_nodes += result['node_count']
            rrt_total_time += result['time']

        if rrt_success_count > 0:
            rrt_avg_length = rrt_total_length / rrt_success_count
            rrt_avg_nodes = rrt_total_nodes / rrt_success_count
            rrt_avg_time = rrt_total_time / 5
            rrt_success_rate = rrt_success_count / 5 * 100
        else:
            rrt_avg_length = None
            rrt_avg_nodes = None
            rrt_avg_time = None
            rrt_success_rate = 0

        print(f"  Success Rate: {rrt_success_rate:.1f}%")
        if rrt_avg_length:
            print(f"  Avg Path Length: {rrt_avg_length:.2f}")
            print(f"  Avg Node Count: {rrt_avg_nodes:.1f}")
            print(f"  Avg Time: {rrt_avg_time:.3f}s")

        # Run RRT* (5 trials)
        print("\nRunning RRT*...")
        rrt_star_success_count = 0
        rrt_star_total_length = 0
        rrt_star_total_time = 0
        rrt_star_total_nodes = 0

        for trial in range(5):
            result = run_rrt_star_experiment(params, show_animation=False)
            if result['success']:
                rrt_star_success_count += 1
                rrt_star_total_length += result['path_length']
                rrt_star_total_nodes += result['node_count']
            rrt_star_total_time += result['time']

        if rrt_star_success_count > 0:
            rrt_star_avg_length = rrt_star_total_length / rrt_star_success_count
            rrt_star_avg_nodes = rrt_star_total_nodes / rrt_star_success_count
            rrt_star_avg_time = rrt_star_total_time / 5
            rrt_star_success_rate = rrt_star_success_count / 5 * 100
        else:
            rrt_star_avg_length = None
            rrt_star_avg_nodes = None
            rrt_star_avg_time = None
            rrt_star_success_rate = 0

        print(f"  Success Rate: {rrt_star_success_rate:.1f}%")
        if rrt_star_avg_length:
            print(f"  Avg Path Length: {rrt_star_avg_length:.2f}")
            print(f"  Avg Node Count: {rrt_star_avg_nodes:.1f}")
            print(f"  Avg Time: {rrt_star_avg_time:.3f}s")

        # Store results
        if rrt_avg_length and rrt_star_avg_length:
            results.append({
                'max_iter': max_iter,
                'rrt_success_rate': rrt_success_rate,
                'rrt_avg_length': rrt_avg_length,
                'rrt_avg_nodes': rrt_avg_nodes,
                'rrt_avg_time': rrt_avg_time,
                'rrt_star_success_rate': rrt_star_success_rate,
                'rrt_star_avg_length': rrt_star_avg_length,
                'rrt_star_avg_nodes': rrt_star_avg_nodes,
                'rrt_star_avg_time': rrt_star_avg_time,
                'improvement': (rrt_avg_length - rrt_star_avg_length) / rrt_avg_length * 100
            })

    # Visualization
    if results:
        visualize_results(results)

    return results


def visualize_results(results):
    """Visualize comparison results"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    x = [r['max_iter'] for r in results]

    # Success Rate
    ax1 = axes[0, 0]
    y_rrt = [r['rrt_success_rate'] for r in results]
    y_rrt_star = [r['rrt_star_success_rate'] for r in results]
    ax1.plot(x, y_rrt, 'o-', linewidth=2, markersize=8, label='RRT')
    ax1.plot(x, y_rrt_star, 's-', linewidth=2, markersize=8, label='RRT*')
    ax1.set_xlabel('Max Iterations')
    ax1.set_ylabel('Success Rate (%)')
    ax1.set_title('Success Rate Comparison')
    ax1.legend()
    ax1.grid(True)

    # Path Length
    ax2 = axes[0, 1]
    y_rrt = [r['rrt_avg_length'] for r in results]
    y_rrt_star = [r['rrt_star_avg_length'] for r in results]
    ax2.plot(x, y_rrt, 'o-', linewidth=2, markersize=8, label='RRT')
    ax2.plot(x, y_rrt_star, 's-', linewidth=2, markersize=8, label='RRT*')
    ax2.set_xlabel('Max Iterations')
    ax2.set_ylabel('Average Path Length')
    ax2.set_title('Path Quality Comparison (Lower is Better)')
    ax2.legend()
    ax2.grid(True)

    # Node Count
    ax3 = axes[1, 0]
    y_rrt = [r['rrt_avg_nodes'] for r in results]
    y_rrt_star = [r['rrt_star_avg_nodes'] for r in results]
    ax3.plot(x, y_rrt, 'o-', linewidth=2, markersize=8, label='RRT')
    ax3.plot(x, y_rrt_star, 's-', linewidth=2, markersize=8, label='RRT*')
    ax3.set_xlabel('Max Iterations')
    ax3.set_ylabel('Average Node Count')
    ax3.set_title('Node Count Comparison')
    ax3.legend()
    ax3.grid(True)

    # Computation Time
    ax4 = axes[1, 1]
    y_rrt = [r['rrt_avg_time'] for r in results]
    y_rrt_star = [r['rrt_star_avg_time'] for r in results]
    ax4.plot(x, y_rrt, 'o-', linewidth=2, markersize=8, label='RRT')
    ax4.plot(x, y_rrt_star, 's-', linewidth=2, markersize=8, label='RRT*')
    ax4.set_xlabel('Max Iterations')
    ax4.set_ylabel('Average Time (s)')
    ax4.set_title('Computation Time Comparison')
    ax4.legend()
    ax4.grid(True)

    plt.tight_layout()
    plt.savefig('/home/zhufeng/code/PythonRobotics/PythonRoboticsLearning/experiments/rrt_vs_rrt_star/comparison_results.png')
    print(f"\n📊 Results saved to: experiments/rrt_vs_rrt_star/comparison_results.png")
    plt.close()

    # Print summary
    print("\n" + "="*60)
    print("Summary")
    print("="*60)
    for r in results:
        print(f"\nMax Iterations: {r['max_iter']}")
        print(f"  RRT Path Length: {r['rrt_avg_length']:.2f}")
        print(f"  RRT* Path Length: {r['rrt_star_avg_length']:.2f}")
        print(f"  Improvement: {r['improvement']:.1f}%")


def main():
    """Main function"""
    results = run_comparison_experiment()

    print("\n" + "="*60)
    print("Experiment completed!")
    print("="*60)


if __name__ == '__main__':
    main()
