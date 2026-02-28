"""
实验 1: A* 启发式函数对比
日期: 2026-01-08
目标: 对比不同启发式权重对搜索效率的影响
"""

import sys
import time
import math
from typing import Any

# 添加项目路径
sys.path.insert(0, '/home/zhufeng/code/PythonRobotics/PythonRobotics')

# 导入 A* 规划器（显式导入，便于 IDE 跳转）
from PathPlanning.AStar.a_star import AStarPlanner as OriginalAStarPlanner


def run_experiment(weight: float, description: str) -> dict:
    """
    运行 A* 实验

    Args:
        weight: 启发式权重 w
        description: 实验描述

    Returns:
        实验结果字典
    """
    print(f"\n{'='*60}")
    print(f"实验: {description}")
    print(f"启发式权重 w = {weight}")
    print(f"{'='*60}")

    # 创建带自定义启发式权重的规划器子类
    class CustomAStarPlanner(OriginalAStarPlanner):
        @staticmethod
        def calc_heuristic(n1, n2):
            w = weight
            d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
            return d

    # 关闭原始模块的动画
    import PathPlanning.AStar.a_star as a_star_module
    a_star_module.show_animation = False

    # 场景参数
    sx, sy = 10.0, 10.0
    gx, gy = 50.0, 50.0
    grid_size = 2.0
    robot_radius = 1.0

    # 障碍物（复制自原代码）
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)

    # 计时运行
    start_time = time.time()
    planner = CustomAStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = planner.planning(sx, sy, gx, gy)
    elapsed_time = time.time() - start_time

    # 计算路径长度
    path_length = 0.0
    for i in range(len(rx) - 1):
        path_length += math.hypot(rx[i+1] - rx[i], ry[i+1] - ry[i])

    # 输出结果
    print(f"\n📊 实验结果:")
    print(f"  ✅ 找到路径: {'是' if len(rx) > 0 else '否'}")
    print(f"  📍 路径点数: {len(rx)}")
    print(f"  📏 路径长度: {path_length:.2f} m")
    print(f"  ⏱️  计算时间: {elapsed_time:.4f} s")

    return {
        'weight': weight,
        'path_length': path_length,
        'time': elapsed_time,
        'nodes': len(rx)
    }


def main():
    print("=" * 60)
    print("A* 启发式函数对比实验")
    print("=" * 60)

    # 实验配置
    experiments = [
        (0.5, "保守搜索 (w=0.5) - 更精确但更慢"),
        (1.0, "标准 A* (w=1.0) - 保证最优"),
        (1.5, "激进搜索 (w=1.5) - 更快但不保证最优"),
        (2.0, "贪婪搜索 (w=2.0) - 最快但质量低"),
    ]

    results = []

    # 运行所有实验
    for weight, desc in experiments:
        try:
            result = run_experiment(weight, desc)
            results.append(result)
        except Exception as e:
            print(f"❌ 实验失败: {e}")

    # 对比总结
    print(f"\n{'='*60}")
    print("📋 实验总结")
    print(f"{'='*60}")
    print(f"{'权重':<8} {'路径长度(m)':<12} {'计算时间(s)':<12} {'路径点数':<10}")
    print(f"{'-'*60}")

    for r in results:
        print(f"{r['weight']:<8.1f} {r['path_length']:<12.2f} "
              f"{r['time']:<12.4f} {r['nodes']:<10}")

    # 分析
    print(f"\n💡 结论:")
    if len(results) >= 2:
        # 对比 w=1.0 和 w=2.0
        standard = next(r for r in results if r['weight'] == 1.0)
        greedy = next(r for r in results if r['weight'] == 2.0)

        time_improvement = (standard['time'] - greedy['time']) / standard['time'] * 100
        path_degradation = (greedy['path_length'] - standard['path_length']) / standard['path_length'] * 100

        print(f"  - 增加权重从 1.0 → 2.0:")
        print(f"    ⚡ 计算时间减少: {time_improvement:.1f}%")
        print(f"    📏 路径长度增加: {path_degradation:.1f}%")

    print(f"\n{'='*60}\n")


if __name__ == '__main__':
    main()
