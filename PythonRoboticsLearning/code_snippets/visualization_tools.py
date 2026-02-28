"""
PythonRobotics 学习实用工具集
包含: 可视化工具、实验辅助函数、常用代码片段
"""

import matplotlib.pyplot as plt
import numpy as np
import time
from typing import List, Tuple


# ============================================================================
# 可视化工具
# ============================================================================

def plot_grid_map(ox: List[float], oy: List[float],
                  sx: float, sy: float,
                  gx: float, gy: float,
                  rx: List[float] = None, ry: List[float] = None,
                  title: str = "路径规划结果"):
    """
    绘制网格地图（障碍物、起点、终点、路径）

    Args:
        ox, oy: 障碍物坐标列表
        sx, sy: 起点
        gx, gy: 终点
        rx, ry: 路径坐标（可选）
        title: 图表标题
    """
    plt.figure(figsize=(10, 10))

    # 绘制障碍物
    plt.plot(ox, oy, ".k", label="障碍物", markersize=2)

    # 绘制起点和终点
    plt.plot(sx, sy, "og", label="起点", markersize=15)
    plt.plot(gx, gy, "xb", label="终点", markersize=15)

    # 绘制路径
    if rx is not None and ry is not None:
        plt.plot(rx, ry, "-r", label="规划路径", linewidth=2)

    plt.grid(True)
    plt.axis("equal")
    plt.title(title)
    plt.legend()
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")

    return plt.gcf()


def plot_algorithm_comparison(results: dict, metric: str = "time"):
    """
    绘制算法性能对比图

    Args:
        results: 字典 {算法名: {metric: value}}
        metric: 对比的指标 ("time", "path_length", "nodes")
    """
    algorithms = list(results.keys())
    values = [results[alg][metric] for alg in algorithms]

    plt.figure(figsize=(10, 6))
    bars = plt.bar(algorithms, values, color='steelblue', alpha=0.7)

    # 添加数值标签
    for bar in bars:
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.4f}',
                ha='center', va='bottom')

    plt.ylabel(f'{metric}')
    plt.title(f'算法对比 - {metric}')
    plt.xticks(rotation=45, ha='right')
    plt.tight_layout()

    return plt.gcf()


# ============================================================================
# 性能测试工具
# ============================================================================

def measure_performance(func, *args, **kwargs):
    """
    测量函数执行时间和内存使用

    Args:
        func: 要测量的函数
        *args, **kwargs: 函数参数

    Returns:
        (result, elapsed_time)
    """
    start_time = time.time()
    result = func(*args, **kwargs)
    elapsed_time = time.time() - start_time

    return result, elapsed_time


class PerformanceProfiler:
    """
    性能分析器 - 用于批量测试
    """

    def __init__(self):
        self.results = []

    def run_test(self, name: str, func, *args, **kwargs):
        """运行单个测试"""
        result, elapsed = measure_performance(func, *args, **kwargs)

        self.results.append({
            'name': name,
            'time': elapsed,
            'result': result
        })

        print(f"✅ {name}: {elapsed:.4f}s")
        return result

    def summary(self):
        """打印测试总结"""
        print("\n" + "="*60)
        print("性能测试总结")
        print("="*60)

        for r in self.results:
            print(f"{r['name']:<30}: {r['time']:.4f}s")

        total_time = sum(r['time'] for r in self.results)
        print(f"\n总时间: {total_time:.4f}s")


# ============================================================================
# 路径分析工具
# ============================================================================

def calculate_path_length(rx: List[float], ry: List[float]) -> float:
    """
    计算路径总长度

    Args:
        rx, ry: 路径坐标

    Returns:
        路径长度
    """
    length = 0.0
    for i in range(len(rx) - 1):
        length += np.hypot(rx[i+1] - rx[i], ry[i+1] - ry[i])
    return length


def calculate_path_smoothness(rx: List[float], ry: List[float]) -> float:
    """
    计算路径平滑度（角度变化总和）

    Args:
        rx, ry: 路径坐标

    Returns:
        平滑度指标（越小越平滑）
    """
    if len(rx) < 3:
        return 0.0

    total_angle_change = 0.0

    for i in range(1, len(rx) - 1):
        # 计算三个连续点的角度变化
        v1x, v1y = rx[i] - rx[i-1], ry[i] - ry[i-1]
        v2x, v2y = rx[i+1] - rx[i], ry[i+1] - ry[i]

        # 计算夹角
        angle1 = np.arctan2(v1y, v1x)
        angle2 = np.arctan2(v2y, v2x)
        angle_change = abs(angle2 - angle1)

        total_angle_change += angle_change

    return total_angle_change


def analyze_path(rx: List[float], ry: List[float]) -> dict:
    """
    综合分析路径质量

    Args:
        rx, ry: 路径坐标

    Returns:
        包含各项指标的字典
    """
    return {
        'length': calculate_path_length(rx, ry),
        'smoothness': calculate_path_smoothness(rx, ry),
        'num_waypoints': len(rx),
    }


# ============================================================================
# 地图生成工具
# ============================================================================

def create_empty_map(width: int, height: int, resolution: float = 1.0):
    """
    创建空地图

    Returns:
        (ox, oy): 障碍物坐标列表
    """
    return [], []


def create_rectangular_obstacle(center_x: float, center_y: float,
                                width: float, height: float,
                                resolution: float = 0.5):
    """
    创建矩形障碍物

    Args:
        center_x, center_y: 中心坐标
        width, height: 尺寸
        resolution: 采样分辨率

    Returns:
        (ox, oy): 障碍物坐标
    """
    ox, oy = [], []

    x_start = center_x - width / 2
    x_end = center_x + width / 2
    y_start = center_y - height / 2
    y_end = center_y + height / 2

    # 采样矩形边界
    x = np.arange(x_start, x_end + resolution, resolution)
    y = np.arange(y_start, y_end + resolution, resolution)

    # 上下边
    for xi in x:
        ox.append(xi)
        oy.append(y_start)
        ox.append(xi)
        oy.append(y_end)

    # 左右边
    for yi in y:
        ox.append(x_start)
        oy.append(yi)
        ox.append(x_end)
        oy.append(yi)

    return ox, oy


def create_maze_map():
    """
    创建简单的迷宫地图

    Returns:
        (ox, oy): 障碍物坐标
    """
    ox, oy = [], []

    # 外边界
    for i in range(-10, 61):
        ox.append(i)
        oy.append(-10.0)
        ox.append(60.0)
        oy.append(i)
        ox.append(i)
        oy.append(60.0)
        ox.append(-10.0)
        oy.append(i)

    # 内部障碍物
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)

    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)

    return ox, oy


# ============================================================================
# 实验辅助函数
# ============================================================================

def run_multiple_runs(planner_class, sx: float, sy: float,
                     gx: float, gy: float, ox: List[float], oy: List[float],
                     num_runs: int = 10, **planner_kwargs):
    """
    多次运行算法取平均值

    Args:
        planner_class: 规划器类
        sx, sy, gx, gy: 起点终点
        ox, oy: 障碍物
        num_runs: 运行次数
        **planner_kwargs: 规划器参数

    Returns:
        统计结果字典
    """
    times = []
    path_lengths = []
    success_count = 0

    for i in range(num_runs):
        planner = planner_class(ox, oy, **planner_kwargs)

        start_time = time.time()
        rx, ry = planner.planning(sx, sy, gx, gy)
        elapsed = time.time() - start_time

        times.append(elapsed)

        if len(rx) > 0:
            path_lengths.append(calculate_path_length(rx, ry))
            success_count += 1

    return {
        'success_rate': success_count / num_runs,
        'avg_time': np.mean(times),
        'std_time': np.std(times),
        'avg_path_length': np.mean(path_lengths) if path_lengths else 0,
        'std_path_length': np.std(path_lengths) if path_lengths else 0,
    }


# ============================================================================
# 调试工具
# ============================================================================

def print_search_progress(iteration: int, open_set_size: int,
                         closed_set_size: int, current_cost: float):
    """
    打印搜索进度（用于调试）

    Args:
        iteration: 迭代次数
        open_set_size: open_set 大小
        closed_set_size: closed_set 大小
        current_cost: 当前节点代价
    """
    print(f"[Iter {iteration:4d}] "
          f"Open: {open_set_size:4d} | "
          f"Closed: {closed_set_size:4d} | "
          f"Cost: {current_cost:.2f}")


# ============================================================================
# 示例使用
# ============================================================================

if __name__ == '__main__':
    print("可视化工具使用示例")
    print("="*60)

    # 1. 创建地图
    ox, oy = create_maze_map()
    sx, sy = 10.0, 10.0
    gx, gy = 50.0, 50.0

    # 2. 使用可视化工具绘制地图
    fig = plot_grid_map(ox, oy, sx, sy, gx, gy, title="测试地图")
    print("✅ 地图绘制完成")
    plt.close(fig)

    # 3. 路径分析示例
    sample_path_x = [0, 1, 2, 3, 4, 5]
    sample_path_y = [0, 1, 2, 3, 4, 5]

    analysis = analyze_path(sample_path_x, sample_path_y)
    print(f"\n路径分析结果:")
    print(f"  长度: {analysis['length']:.2f}m")
    print(f"  平滑度: {analysis['smoothness']:.2f}")
    print(f"  路径点数: {analysis['num_waypoints']}")

    print("\n" + "="*60)
    print("工具库加载完成！")
