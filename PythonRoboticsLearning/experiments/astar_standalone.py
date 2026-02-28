"""
实验 1: A* 启发式函数对比（独立版本）
日期: 2026-01-08
目标: 对比不同启发式权重对搜索效率的影响

说明：这个版本直接包含 A* 算法代码，不依赖模块导入
"""

import math
import time
import matplotlib.pyplot as plt

show_animation = False


class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x  # grid index
        self.y = y  # grid index
        self.cost = cost
        self.parent_index = parent_index


class AStarPlanner:
    def __init__(self, ox, oy, resolution, rr, heuristic_weight=1.0):
        """
        Initialize grid map for A star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        heuristic_weight: weight for heuristic function (default: 1.0)
        """
        self.resolution = resolution
        self.rr = rr
        self.heuristic_weight = heuristic_weight
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    def calc_heuristic(self, n1, n2):
        d = self.heuristic_weight * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


def run_experiment(weight, description):
    """
    运行 A* 实验

    Args:
        weight: 启发式权重 w
        description: 实验描述
    """
    print(f"\n{'='*60}")
    print(f"实验: {description}")
    print(f"启发式权重 w = {weight}")
    print(f"{'='*60}")

    # 场景参数
    sx, sy = 10.0, 10.0
    gx, gy = 50.0, 50.0
    grid_size = 2.0
    robot_radius = 1.0

    # 障碍物
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
    planner = AStarPlanner(ox, oy, grid_size, robot_radius, heuristic_weight=weight)
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
