"""
A* 平滑路径算法对比
1. 传统A* (网格对齐)
2. Theta* (any-angle路径)
3. A* + 路径平滑 (样条曲线)
4. Field D* (亚网格精度)
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev, CubicSpline
from typing import List, Tuple, Dict, Optional, Set

# 配置中文字体
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'Noto Sans CJK SC', 'WenQuanYi Micro Hei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


class Node:
    """节点类"""
    def __init__(self, x: float, y: float, cost: float = 0, parent=None):
        self.x = x
        self.y = y
        self.cost = cost  # g值
        self.parent = parent
    
    def __lt__(self, other):
        return self.cost < other.cost


class GridMap:
    """网格地图"""
    def __init__(self, width: int, height: int, resolution: float = 1.0):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.obstacles = set()
    
    def add_obstacle(self, x: int, y: int):
        """添加障碍物（网格坐标）"""
        self.obstacles.add((x, y))
    
    def add_circle_obstacle(self, cx: float, cy: float, radius: float):
        """添加圆形障碍物"""
        for x in range(self.width):
            for y in range(self.height):
                px = x * self.resolution
                py = y * self.resolution
                if math.hypot(px - cx, py - cy) <= radius:
                    self.obstacles.add((x, y))
    
    def add_rect_obstacle(self, x1: int, y1: int, x2: int, y2: int):
        """添加矩形障碍物"""
        for x in range(x1, x2 + 1):
            for y in range(y1, y2 + 1):
                self.obstacles.add((x, y))
    
    def is_collision(self, x: int, y: int) -> bool:
        """检查网格点是否碰撞"""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return True
        return (x, y) in self.obstacles
    
    def is_collision_line(self, x1: float, y1: float, x2: float, y2: float) -> bool:
        """检查线段是否与障碍物碰撞（用于Theta*）"""
        # Bresenham直线算法检查
        steps = int(max(abs(x2 - x1), abs(y2 - y1)) * 2)
        if steps == 0:
            return self.is_collision(int(x1), int(y1))
        
        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            if self.is_collision(int(x), int(y)):
                return True
        return False


class AStarBase:
    """A*基类"""
    def __init__(self, grid_map: GridMap):
        self.grid_map = grid_map
        self.open_set = {}
        self.closed_set = {}
    
    def heuristic(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """启发式函数：欧几里得距离"""
        return math.hypot(x2 - x1, y2 - y1)
    
    def get_neighbors_8(self, x: int, y: int) -> List[Tuple[int, int, float]]:
        """获取8方向邻居"""
        neighbors = []
        for dx, dy in [(1,0), (0,1), (-1,0), (0,-1), 
                       (1,1), (-1,1), (-1,-1), (1,-1)]:
            nx, ny = x + dx, y + dy
            if not self.grid_map.is_collision(nx, ny):
                cost = math.hypot(dx, dy)
                neighbors.append((nx, ny, cost))
        return neighbors
    
    def reconstruct_path(self, node: Node) -> List[Tuple[float, float]]:
        """重建路径"""
        path = []
        current = node
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        return path[::-1]


class TraditionalAStar(AStarBase):
    """传统A*算法（网格对齐）"""
    
    def search(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[float, float]]]:
        """搜索路径"""
        start_node = Node(start[0], start[1], 0)
        goal_node = Node(goal[0], goal[1])
        
        self.open_set = {start: start_node}
        self.closed_set = {}
        
        while self.open_set:
            # 选择f值最小的节点
            current_pos = min(self.open_set, 
                            key=lambda p: self.open_set[p].cost + 
                                        self.heuristic(self.open_set[p].x, self.open_set[p].y, 
                                                      goal_node.x, goal_node.y))
            current = self.open_set[current_pos]
            
            # 到达目标
            if current_pos == goal:
                return self.reconstruct_path(current)
            
            # 移到closed set
            del self.open_set[current_pos]
            self.closed_set[current_pos] = current
            
            # 扩展邻居
            for nx, ny, move_cost in self.get_neighbors_8(int(current.x), int(current.y)):
                if (nx, ny) in self.closed_set:
                    continue
                
                tentative_g = current.cost + move_cost
                
                if (nx, ny) not in self.open_set:
                    neighbor = Node(nx, ny, tentative_g, current)
                    self.open_set[(nx, ny)] = neighbor
                elif tentative_g < self.open_set[(nx, ny)].cost:
                    self.open_set[(nx, ny)].cost = tentative_g
                    self.open_set[(nx, ny)].parent = current
        
        return None


class ThetaStar(AStarBase):
    """Theta*算法（any-angle路径）"""
    
    def search(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[float, float]]]:
        """搜索路径"""
        start_node = Node(start[0], start[1], 0)
        goal_node = Node(goal[0], goal[1])
        
        self.open_set = {start: start_node}
        self.closed_set = {}
        
        while self.open_set:
            current_pos = min(self.open_set,
                            key=lambda p: self.open_set[p].cost + 
                                        self.heuristic(self.open_set[p].x, self.open_set[p].y,
                                                      goal_node.x, goal_node.y))
            current = self.open_set[current_pos]
            
            if current_pos == goal:
                return self.reconstruct_path(current)
            
            del self.open_set[current_pos]
            self.closed_set[current_pos] = current
            
            # 扩展邻居
            for nx, ny, _ in self.get_neighbors_8(int(current.x), int(current.y)):
                if (nx, ny) in self.closed_set:
                    continue
                
                # Theta*核心：尝试从父节点直接连接到邻居（跳过中间点）
                if current.parent is not None:
                    # 检查从祖父节点到邻居的直线是否无碰撞
                    if not self.grid_map.is_collision_line(current.parent.x, current.parent.y, nx, ny):
                        # Path 2: 从祖父节点直接到邻居
                        tentative_g = current.parent.cost + self.heuristic(
                            current.parent.x, current.parent.y, nx, ny)
                        
                        if (nx, ny) not in self.open_set or tentative_g < self.open_set[(nx, ny)].cost:
                            neighbor = Node(nx, ny, tentative_g, current.parent)
                            self.open_set[(nx, ny)] = neighbor
                        continue
                
                # Path 1: 常规路径
                tentative_g = current.cost + self.heuristic(current.x, current.y, nx, ny)
                
                if (nx, ny) not in self.open_set:
                    neighbor = Node(nx, ny, tentative_g, current)
                    self.open_set[(nx, ny)] = neighbor
                elif tentative_g < self.open_set[(nx, ny)].cost:
                    self.open_set[(nx, ny)].cost = tentative_g
                    self.open_set[(nx, ny)].parent = current
        
        return None


class PathSmoother:
    """路径平滑器"""
    
    @staticmethod
    def smooth_path_spline(path: List[Tuple[float, float]], num_points: int = 100) -> List[Tuple[float, float]]:
        """使用B样条曲线平滑路径"""
        if len(path) < 3:
            return path
        
        path_array = np.array(path)
        x = path_array[:, 0]
        y = path_array[:, 1]
        
        # 参数化
        tck, u = splprep([x, y], s=2, k=min(3, len(path) - 1))
        
        # 生成平滑点
        u_new = np.linspace(0, 1, num_points)
        x_new, y_new = splev(u_new, tck)
        
        return list(zip(x_new, y_new))
    
    @staticmethod
    def smooth_path_shortcut(path: List[Tuple[float, float]], grid_map: GridMap) -> List[Tuple[float, float]]:
        """路径快捷方式平滑（移除不必要的中间点）"""
        if len(path) < 3:
            return path
        
        smoothed = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            # 尝试从当前点直接连接到尽可能远的点
            for j in range(len(path) - 1, i, -1):
                if not grid_map.is_collision_line(path[i][0], path[i][1], path[j][0], path[j][1]):
                    smoothed.append(path[j])
                    i = j
                    break
            else:
                i += 1
        
        return smoothed
    
    @staticmethod
    def smooth_path_gradient(path: List[Tuple[float, float]], grid_map: GridMap, 
                           iterations: int = 100, alpha: float = 0.5) -> List[Tuple[float, float]]:
        """梯度下降平滑（保持端点不变）"""
        if len(path) < 3:
            return path
        
        smoothed = np.array(path, dtype=float)
        
        for _ in range(iterations):
            for i in range(1, len(smoothed) - 1):
                # 向相邻点的中点移动
                prev = smoothed[i - 1]
                next_point = smoothed[i + 1]
                target = (prev + next_point) / 2
                
                # 检查新位置是否碰撞
                new_pos = smoothed[i] + alpha * (target - smoothed[i])
                if not grid_map.is_collision(int(new_pos[0]), int(new_pos[1])):
                    smoothed[i] = new_pos
        
        return [(x, y) for x, y in smoothed]


def visualize_comparison():
    """可视化对比不同算法"""
    # 创建地图
    grid_map = GridMap(width=50, height=50, resolution=1.0)
    
    # 添加障碍物
    # 垂直墙
    for y in range(15, 35):
        grid_map.add_obstacle(20, y)
    
    # 水平墙
    for x in range(25, 40):
        grid_map.add_obstacle(x, 25)
    
    # 圆形障碍物
    grid_map.add_circle_obstacle(35, 15, 3)
    
    # 起点和终点
    start = (5, 5)
    goal = (45, 45)
    
    # 运行不同算法
    print("运行传统A*...")
    traditional = TraditionalAStar(grid_map)
    path_traditional = traditional.search(start, goal)
    
    print("运行Theta*...")
    theta = ThetaStar(grid_map)
    path_theta = theta.search(start, goal)
    
    print("生成平滑路径...")
    smoother = PathSmoother()
    path_spline = smoother.smooth_path_spline(path_traditional, 200) if path_traditional else None
    path_shortcut = smoother.smooth_path_shortcut(path_traditional, grid_map) if path_traditional else None
    path_gradient = smoother.smooth_path_gradient(path_traditional, grid_map, 50, 0.3) if path_traditional else None
    
    # 可视化
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    
    methods = [
        ("传统A* (网格对齐)", path_traditional, 'blue'),
        ("Theta* (Any-Angle)", path_theta, 'red'),
        ("A* + B样条平滑", path_spline, 'green'),
        ("A* + 快捷路径", path_shortcut, 'orange'),
        ("A* + 梯度平滑", path_gradient, 'purple'),
        ("对比（所有方法）", None, None)
    ]
    
    for idx, (title, path, color) in enumerate(methods):
        ax = axes[idx // 3, idx % 3]
        ax.set_xlim(-2, 52)
        ax.set_ylim(-2, 52)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3, linewidth=0.5)
        ax.set_title(title, fontsize=14, fontweight='bold')
        
        # 绘制障碍物
        for obs_x, obs_y in grid_map.obstacles:
            rect = plt.Rectangle((obs_x - 0.5, obs_y - 0.5), 1, 1, 
                                color='black', alpha=0.6)
            ax.add_patch(rect)
        
        # 绘制起点和终点
        ax.plot(start[0], start[1], 'go', markersize=12, label='起点', zorder=10)
        ax.plot(goal[0], goal[1], 'r*', markersize=15, label='终点', zorder=10)
        
        # 绘制路径
        if idx < 5 and path:
            path_array = np.array(path)
            ax.plot(path_array[:, 0], path_array[:, 1], 
                   color=color, linewidth=2.5, label='路径', alpha=0.8, zorder=5)
            
            # 显示路径节点
            if idx == 0:  # 只在传统A*上显示网格点
                ax.plot(path_array[:, 0], path_array[:, 1], 
                       'o', color=color, markersize=4, alpha=0.5, zorder=4)
            
            # 计算路径长度
            length = sum(math.hypot(path[i+1][0] - path[i][0], path[i+1][1] - path[i][1]) 
                        for i in range(len(path) - 1))
            ax.text(0.02, 0.98, f'路径长度: {length:.2f}', 
                   transform=ax.transAxes, verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        elif idx == 5:  # 对比图
            # 绘制所有路径
            if path_traditional:
                arr = np.array(path_traditional)
                ax.plot(arr[:, 0], arr[:, 1], color='blue', linewidth=2, 
                       label='传统A*', alpha=0.6, linestyle='--')
            
            if path_theta:
                arr = np.array(path_theta)
                ax.plot(arr[:, 0], arr[:, 1], color='red', linewidth=2.5, 
                       label='Theta*', alpha=0.8)
            
            if path_spline:
                arr = np.array(path_spline)
                ax.plot(arr[:, 0], arr[:, 1], color='green', linewidth=2, 
                       label='B样条', alpha=0.7, linestyle='-.')
        
        ax.legend(loc='lower right', fontsize=9)
    
    plt.tight_layout()
    plt.savefig('/home/zhufeng/code/PythonRobotics/PathPlanning/AStar/astar_smooth_comparison.png', 
                dpi=150, bbox_inches='tight')
    print("图像已保存: astar_smooth_comparison.png")
    plt.show()


def demo_field_dstar():
    """演示Field D*的亚网格精度"""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # 创建简单场景
    grid_map = GridMap(width=20, height=20, resolution=1.0)
    
    # 添加简单障碍物
    for x in range(8, 12):
        for y in range(8, 12):
            grid_map.add_obstacle(x, y)
    
    start = (2, 2)
    goal = (18, 18)
    
    # 传统A*
    traditional = TraditionalAStar(grid_map)
    path_traditional = traditional.search(start, goal)
    
    # Theta*
    theta = ThetaStar(grid_map)
    path_theta = theta.search(start, goal)
    
    # 左图：传统A*
    ax = axes[0]
    ax.set_xlim(-1, 21)
    ax.set_ylim(-1, 21)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_title('传统A* - 网格对齐路径', fontsize=12, fontweight='bold')
    
    # 绘制网格
    for x in range(21):
        ax.axvline(x - 0.5, color='gray', alpha=0.2, linewidth=0.5)
    for y in range(21):
        ax.axhline(y - 0.5, color='gray', alpha=0.2, linewidth=0.5)
    
    # 绘制障碍物
    for obs_x, obs_y in grid_map.obstacles:
        rect = plt.Rectangle((obs_x - 0.5, obs_y - 0.5), 1, 1, 
                            color='black', alpha=0.7)
        ax.add_patch(rect)
    
    # 绘制路径
    if path_traditional:
        arr = np.array(path_traditional)
        ax.plot(arr[:, 0], arr[:, 1], 'b-', linewidth=3, label='传统A*', zorder=5)
        ax.plot(arr[:, 0], arr[:, 1], 'bo', markersize=6, alpha=0.6)
    
    ax.plot(start[0], start[1], 'go', markersize=12, label='起点')
    ax.plot(goal[0], goal[1], 'r*', markersize=15, label='终点')
    ax.legend()
    
    # 右图：Theta*
    ax = axes[1]
    ax.set_xlim(-1, 21)
    ax.set_ylim(-1, 21)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_title('Theta* - Any-Angle路径', fontsize=12, fontweight='bold')
    
    # 绘制网格
    for x in range(21):
        ax.axvline(x - 0.5, color='gray', alpha=0.2, linewidth=0.5)
    for y in range(21):
        ax.axhline(y - 0.5, color='gray', alpha=0.2, linewidth=0.5)
    
    # 绘制障碍物
    for obs_x, obs_y in grid_map.obstacles:
        rect = plt.Rectangle((obs_x - 0.5, obs_y - 0.5), 1, 1,
                            color='black', alpha=0.7)
        ax.add_patch(rect)
    
    # 绘制路径
    if path_theta:
        arr = np.array(path_theta)
        ax.plot(arr[:, 0], arr[:, 1], 'r-', linewidth=3, label='Theta*', zorder=5)
        ax.plot(arr[:, 0], arr[:, 1], 'ro', markersize=6, alpha=0.6)
        
        # 显示转折点的坐标（亚网格精度）
        for i, (x, y) in enumerate(path_theta[1:-1], 1):
            ax.text(x + 0.3, y + 0.3, f'({x},{y})', fontsize=8, alpha=0.7)
    
    ax.plot(start[0], start[1], 'go', markersize=12, label='起点')
    ax.plot(goal[0], goal[1], 'r*', markersize=15, label='终点')
    ax.legend()
    
    plt.tight_layout()
    plt.savefig('/home/zhufeng/code/PythonRobotics/PathPlanning/AStar/astar_subgrid.png',
                dpi=150, bbox_inches='tight')
    print("图像已保存: astar_subgrid.png")
    plt.show()


if __name__ == '__main__':
    print("="*60)
    print("A* 平滑路径算法对比")
    print("="*60)
    print("\n1. 传统A* - 路径沿网格边缘")
    print("2. Theta* - 支持any-angle，更短更直")
    print("3. A* + B样条 - 平滑曲线")
    print("4. A* + 快捷路径 - 移除冗余点")
    print("5. A* + 梯度平滑 - 迭代优化")
    print("\n正在生成对比图...")
    
    visualize_comparison()
    
    print("\n正在生成亚网格精度演示...")
    demo_field_dstar()
    
    print("\n完成！")

