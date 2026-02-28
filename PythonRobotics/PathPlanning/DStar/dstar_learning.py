"""
简化版 D* 算法学习实现
通过填空的方式逐步理解 D* 核心逻辑

修复版本 - 包含以下改进：
✓ 修复地图可视化显示起点和终点
✓ 修复返回值一致性问题
✓ 改进动态障碍物检测逻辑
✓ 添加错误处理和边界检查
✓ 提供更好的教学演示场景

学习要点：
1. D* 采用反向搜索（从目标到起点）
2. parent 指针指向目标方向
3. 三分支 process_state 处理不同情况
4. 动态修复比完全重新规划更高效
"""

import math
from sys import maxsize

# ==================== 基础数据结构 ====================

class State:
    """表示网格中的一个状态节点"""

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None      # TODO: parent 的作用是什么？
        self.t = "new"          # 标签: "new", "open", "close"
        self.h = 0              # TODO: h 表示什么？从该状态到目标的代价
        self.k = 0              # TODO: k 的作用是什么？用于优先级排序

    def cost(self, state):
        """计算当前状态到另一个状态的代价"""
        # TODO: 实现代价计算（简单欧几里得距离）
        # 障碍物的代价在 Map 类中通过 state 标记处理
        return math.sqrt((self.x - state.x)**2 + (self.y - state.y)**2)



class SimpleMap:
    """简化的网格地图"""

    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.grid = [[State(x, y) for y in range(cols)] for x in range(rows)]
        self.obstacles = set()  # 存储障碍物坐标

    def get(self, x, y):
        """获取指定坐标的 State"""
        if 0 <= x < self.rows and 0 <= y < self.cols:
            return self.grid[x][y]
        return None

    def is_obstacle(self, x, y):
        """检查是否是障碍物"""
        return (x, y) in self.obstacles

    def add_obstacle(self, x, y):
        """添加障碍物"""
        if 0 <= x < self.rows and 0 <= y < self.cols:
            self.obstacles.add((x, y))

    def get_neighbors(self, state):
        """
        获取邻居节点（4-连通：上、下、左、右）
        TODO: 为什么用 4-连通？如果用 8-连通会有什么不同？
        """
        neighbors = []
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 上、下、左、右

        for dx, dy in directions:
            nx, ny = state.x + dx, state.y + dy
            neighbor = self.get(nx, ny)
            if neighbor and (nx, ny) not in self.obstacles:
                neighbors.append(neighbor)

        return neighbors


# ==================== D* 核心算法 ====================

class SimpleDstar:
    """简化版 D* 算法"""

    def __init__(self, map_obj):
        self.map = map_obj
        self.open_list = []  # 简单列表，实际应使用优先队列

    def get_min_state(self):
        """
        从 open_list 中获取 k 值最小的状态

        TODO: 实现这个函数
        提示：使用 min() 函数和 lambda 表达式
        """
        if not self.open_list:
            return None
        _min_state = min(self.open_list, key=lambda s: s.k)
        return _min_state

    def get_kmin(self):
        """
        获取 open_list 中最小的 k 值

        TODO: 实现这个函数
        """
        if not self.open_list:
            return -1
        return min(s.k for s in self.open_list)

    def insert(self, state, h_new):
        """
        将状态插入 open_list

        这是 D* 的关键操作！理解这三种情况至关重要：
        - new:      节点首次加入
        - open:     节点已在 open_list 中，取更小的 k
        - close:    节点之前处理过，需要重新评估

        TODO: 完成这个核心函数的实现
        """
        if state.t == "new":
            # 首次加入，k = h
            state.k = h_new
        elif state.t == "open":
            # 已在 open_list，取较小的 k
            state.k = min(state.k, h_new)
        elif state.t == "close":
            # 之前处理过，取 h 和 h_new 中较小的
            state.k = min(state.h, h_new)

        state.h = h_new
        state.t = "open"
        if state not in self.open_list:
            self.open_list.append(state)

    def remove(self, state):
        """从 open_list 中移除状态"""
        state.t = "close"
        if state in self.open_list:
            self.open_list.remove(state)

    def modify_cost(self, state):
        """
        当某个状态的代价发生变化时，重新将其加入 open_list

        TODO: 理解这个函数的逻辑
        为什么要用 state.parent.h + cost(state.parent)？
        """
        if state.t == "close":
            new_cost = state.parent.h + state.cost(state.parent)
            self.insert(state, new_cost)

    def process_state(self):
        """
        D* 的核心函数！处理 open_list 中的状态

        TODO: 这是整个算法最难理解的部分
        三个分支条件分别处理什么情况？

        分支1: if k_old < x.h      - h 值降低了
        分支2: elif k_old == x.h   - h 值未变，正常扩展
        分支3: else                - h 值升高了
        """
        if not self.open_list:
            return -1

        x = self.get_min_state()
        k_old = self.get_kmin()
        self.remove(x)

        # 分支 1: k_old < x.h
        # TODO: 这个分支的意义是什么？
        if k_old < x.h:
            for y in self.map.get_neighbors(x):
                if y.h <= k_old and x.h > y.h + x.cost(y):
                    x.parent = y
                    x.h = y.h + x.cost(y)

        # 分支 2: k_old == x.h
        # TODO: 这是正常的 D* 扩展逻辑，代码在做什么？
        elif k_old == x.h:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or (y.parent == x and y.h != x.h + x.cost(y)) \
                        or (y.parent != x and y.h > x.h + x.cost(y)):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))

        # 分支 3: k_old > x.h
        # TODO: h 值升高了需要怎么处理？
        else:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or (y.parent == x and y.h != x.h + x.cost(y)):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
                else:
                    if y.parent != x and y.h > x.h + x.cost(y):
                        self.insert(x, x.h)
                    else:
                        if y.parent != x and x.h > y.h + x.cost(y) \
                                and y.t == "close" and y.h > k_old:
                            self.insert(y, y.h)

        return self.get_kmin()

    def run(self, start, goal):
        """
        执行 D* 搜索

        TODO: 理解这个完整的流程
        1. 为什么是从 goal 反向搜索到 start？
        2. 最后如何构建路径？
        """
        # 将目标点加入 open_list，h = 0
        self.insert(goal, 0.0)

        print(f"开始反向搜索: goal({goal.x}, {goal.y}) -> start...")
        steps = 0

        # 反向搜索直到找到起点
        while True:
            k_min = self.process_state()
            steps += 1
            if start.t == "close" or k_min == -1:
                break

        print(f"反向搜索完成，共处理 {steps} 个状态")

        # 检查是否找到路径
        if start.t != "close":
            print("❌ 无法找到路径！")
            return []

        # 构建路径（从 start 跟随 parent 到 goal）
        path = []
        current = start
        while current != goal:
            path.append((current.x, current.y))
            current = current.parent

        path.append((goal.x, goal.y))

        print(f"✓ 找到路径，长度: {len(path)} 步")
        return path

    def modify(self, state):
        """
        当检测到障碍物影响路径时，修复路径

        TODO: 这个函数如何与 process_state 配合？
        """
        self.modify_cost(state)

        while True:
            k_min = self.process_state()
            if k_min == -1 or k_min >= state.h:
                break


# ==================== 演示函数 ====================

def print_map(map_obj, path=None, start=None, goal=None):
    """打印地图可视化"""
    print("\n地图可视化 (s=起点, e=终点, #=障碍, *=路径, X=路径上的障碍物):")
    for x in range(map_obj.rows):
        row = ""
        for y in range(map_obj.cols):
            # 显示优先级：起点 > 终点 > 路径上的障碍物 > 路径 > 障碍物 > 空地
            if start and (x, y) == (start.x, start.y):
                row += "s "
            elif goal and (x, y) == (goal.x, goal.y):
                row += "e "
            elif path and (x, y) in path and map_obj.is_obstacle(x, y):
                row += "X "  # 路径上的障碍物 - 这表明有问题！
            elif path and (x, y) in path:
                row += "* "
            elif map_obj.is_obstacle(x, y):
                row += "# "
            else:
                row += ". "
        print(row)
    print()


def demo_static():
    """演示 1: 静态环境下的路径规划"""
    print("=" * 60)
    print("演示 1: 静态环境搜索")
    print("=" * 60)

    # 创建 10x10 的地图
    m = SimpleMap(10, 10)

    # 添加一些障碍物
    obstacles = [(3, 3), (3, 4), (3, 5), (4, 3)]
    for obs in obstacles:
        m.add_obstacle(*obs)

    # 设置起点和终点
    start = m.get(0, 0)
    goal = m.get(9, 9)

    print(f"起点: ({start.x}, {start.y})")
    print(f"终点: ({goal.x}, {goal.y})")
    print(f"障碍物: {obstacles}")

    # 运行 D* 算法
    dstar = SimpleDstar(m)
    path = dstar.run(start, goal)

    print_map(m, path if path else None, start, goal)


def demo_dynamic():
    """演示 2: 动态障碍物环境"""
    print("\n" + "=" * 60)
    print("演示 2: 动态障碍物修复")
    print("=" * 60)

    # 创建 10x10 的地图
    m = SimpleMap(10, 10)

    # 初始障碍物
    m.add_obstacle(5, 2)
    m.add_obstacle(5, 3)
    m.add_obstacle(5, 4)

    print("初始状态:")
    start = m.get(0, 5)
    goal = m.get(9, 5)

    dstar = SimpleDstar(m)
    path1 = dstar.run(start, goal)
    print_map(m, path1, start, goal)

    # 添加新的障碍物，阻断原有路径
    print("\n添加新障碍物 (6, 5)，阻断路径中的一个连接...")
    m.add_obstacle(6, 5)

    # 找到受影响的节点并修复
    if path1:
        # 检查新障碍物是否直接在路径上，或者阻断了路径
        affected_node = None
        for x, y in path1:
            if m.is_obstacle(x, y):  # 新障碍物直接在路径上
                affected_node = m.get(x, y)
                print(f"受影响的节点: 路径节点 ({x}, {y}) 被新障碍物占据")
                break

        # 如果没有直接冲突，检查路径连接是否被阻断
        if not affected_node:
            for i in range(len(path1) - 1):
                x1, y1 = path1[i]
                x2, y2 = path1[i + 1]
                # 检查相邻路径点之间是否有新障碍物
                if m.is_obstacle(x2, y2):
                    affected_node = m.get(x1, y1)
                    print(f"受影响的节点: ({x1}, {y1}) -> ({x2}, {y2}) 连接被阻断")
                    break

        if affected_node:
            dstar.modify(affected_node)

        # 重新构建路径
        current = start
        path2 = []
        max_steps = m.rows * m.cols  # 防止无限循环
        step_count = 0

        while current != goal and step_count < max_steps:
            path2.append((current.x, current.y))
            if current.parent is None:
                print("❌ 路径修复失败：parent 指针断裂")
                break
            current = current.parent
            step_count += 1

        if current == goal:
            path2.append((goal.x, goal.y))
            print("修复后的路径:")
            print_map(m, path2, start, goal)

            # 对比分析
            print(f"\n路径对比:")
            print(f"原路径: {path1}")
            print(f"修复后: {path2}")

            if path1 == path2:
                print("⚠️  注意：路径没有改变！")
                print("   这表明 D* 修复算法在这个简化实现中可能有问题。")
                print("   在实际的 D* 中，应该能找到绕过障碍物的新路径。")
        else:
            print("❌ 路径修复失败：无法到达目标点")
def demo_better_dynamic():
    """演示 3: 更明显的动态路径修复"""
    print("\n" + "=" * 60)
    print("演示 3: D* 动态路径修复能力")
    print("=" * 60)

    # 创建一个更小的地图便于观察
    m = SimpleMap(6, 6)

    # 创建一个狭窄通道
    for y in range(6):
        if y != 2:  # 在 y=2 处留一个通道
            m.add_obstacle(2, y)

    start = m.get(0, 2)
    goal = m.get(5, 2)

    print("步骤1: 初始路径规划")
    print("地图有一个狭窄通道，机器人应该直接通过")

    dstar = SimpleDstar(m)
    path1 = dstar.run(start, goal)
    print_map(m, path1, start, goal)

    # 现在阻断通道
    print("步骤2: 阻断通道")
    print("在通道中添加障碍物，强制 D* 重新规划")
    m.add_obstacle(2, 2)  # 阻断唯一通道

    # 手动触发路径修复
    blocked_state = m.get(2, 2)
    if blocked_state in [m.get(x, y) for x, y in path1]:
        print("检测到路径被阻断，开始 D* 修复...")

        # 找到被阻断的路径段
        blocked_index = None
        for i, (x, y) in enumerate(path1):
            if (x, y) == (2, 2):
                blocked_index = i
                break

        if blocked_index and blocked_index > 0:
            # 从被阻断节点的前一个节点开始修复
            prev_x, prev_y = path1[blocked_index - 1]
            affected_node = m.get(prev_x, prev_y)

            # 重置被阻断的节点
            blocked_node = m.get(2, 2)
            blocked_node.h = float('inf')
            blocked_node.t = "new"
            blocked_node.parent = None

            print(f"从节点 ({prev_x}, {prev_y}) 开始修复路径")
            dstar.modify(affected_node)

            # 重新构建路径
            current = start
            path2 = []
            max_steps = 36

            while current != goal and len(path2) < max_steps:
                path2.append((current.x, current.y))
                if current.parent is None:
                    print("无法找到有效路径")
                    break
                current = current.parent

            if current == goal:
                path2.append((goal.x, goal.y))
                print("步骤3: 修复后的路径")
                print_map(m, path2, start, goal)

                print(f"原路径长度: {len(path1)} (直线)")
                print(f"修复后路径长度: {len(path2)} (绕行)")
            else:
                print("❌ 无法找到替代路径")

    return True


def verify_parent_direction():
    """
    验证 D* 中的 parent 指针方向
    这个函数将帮助我们理解 parent 是否指向 goal
    """
    print("\n" + "="*60)
    print("验证: Parent 指针方向")
    print("="*60)

    m = SimpleMap(5, 5)  # 小地图便于观察
    start = m.get(0, 0)
    goal = m.get(4, 4)

    dstar = SimpleDstar(m)
    path = dstar.run(start, goal)

    # TODO: 遍历路径，观察 parent 是否指向 goal 方向
    # 提示：打印每个节点及其 parent 的坐标
    for i, (x, y) in enumerate(path):
        state = m.get(x, y)
        # 打印当前节点信息
        if state.parent:
            print(f"节点 {i}: ({x},{y}) -> parent: ({state.parent.x},{state.parent.y})")
        else:
            print(f"节点 {i}: ({x},{y}) -> parent: None")

    # 最后一个问题：从 start 跟随 parent，能否到达 goal？
    print("\n结论分析:")
    print("从 (0,0) 跟随 parent 指针的路径：")
    current = start
    step = 0
    while current and step < 10:
        if current.parent:
            print(f"  步骤 {step}: ({current.x},{current.y}) -> ({current.parent.x},{current.parent.y})")
            current = current.parent
        else:
            print(f"  步骤 {step}: ({current.x},{current.y}) -> 终点！")
            break
        step += 1

    print("\n✓ D* 重要理解：")
    print("  - D* 反向搜索：从终点搜索到起点")
    print("  - parent 指针：从起点指向终点方向")
    print("  - 路径构建：沿着 parent 指针从起点走向终点")
    print("  - 这种设计使得动态修复更高效！")


if __name__ == "__main__":
    demo_dynamic()
    demo_better_dynamic()
    # TODO: 取消注释下面这行来测试：
    verify_parent_direction()