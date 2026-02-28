"""
D* Interactive Visualization Demo

Demonstrates the D* algorithm with step-by-step controls:
- Initial backward path planning from goal to start
- Dynamic obstacle handling with path repair
- The three branches of process_state(): k < h, k == h, k > h

Author: Educational Demo
"""

import math
from sys import maxsize
from enum import Enum

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button, Slider
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation


class DStarBranch(Enum):
    """Represents the branch of process_state() being executed"""
    K_LESS_THAN_H = "k < h: Path cost increased, propagate cost changes"
    K_EQUAL_H = "k == h: Normal expansion, update neighbors"
    K_GREATER_THAN_H = "k > h: Path cost decreased, propagate improvements"


class State:
    """
    状态类，表示网格中的一个节点

    Attributes:
        x, y: 坐标位置
        parent: 父节点（用于路径回溯）
        state: 节点状态标记 (.:new, #:obstacle, e:edge, *:closed, s:start)
        t: 标记 (new, open, close)
        h: 估计代价 (从当前点到起点的最小代价)
        k: 键值 (用于优先级队列)
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.state = "."
        self.t = "new"
        self.h = 0
        self.k = 0

    def cost(self, state):
        """计算两个状态之间的移动代价"""
        if self.state == "#" or state.state == "#":
            return maxsize
        return math.sqrt(math.pow((self.x - state.x), 2) +
                         math.pow((self.y - state.y), 2))

    def set_state(self, state):
        """
        设置节点状态标记
        .: new (new)
        #: obstacle (#)
        e: parent of current state (edge)
        *: closed state
        s: start state
        """
        if state not in ["s", ".", "#", "e", "*"]:
            return
        self.state = state

    def __lt__(self, other):
        return self.k < other.k


class Map:
    """
    地图类，管理网格和状态

    支持八连通移动 (包括对角线)
    """
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.map = self.init_map()

    def init_map(self):
        """初始化空地图"""
        map_list = []
        for i in range(self.row):
            tmp = []
            for j in range(self.col):
                tmp.append(State(i, j))
            map_list.append(tmp)
        return map_list

    def get_neighbors(self, state):
        """获取八连通邻居节点"""
        state_list = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if state.x + i < 0 or state.x + i >= self.row:
                    continue
                if state.y + j < 0 or state.y + j >= self.col:
                    continue
                state_list.append(self.map[state.x + i][state.y + j])
        return state_list

    def set_obstacle(self, point_list):
        """设置障碍物"""
        for x, y in point_list:
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                continue
            self.map[x][y].set_state("#")


class DStarInteractive:
    """
    交互式 D* 算法可视化

    关键概念:
    - h: 从该点到起点的估计代价 (g-values in A*)
    - k: 键值，用于开放列表排序 (优先级队列中的排序键)
    - 开放列表 (open): 待处理的节点集合
    - 关闭列表 (closed): 已处理的节点

    三个处理分支:
    1. k < h: 该节点的代价增加了，需要传播代价增加
    2. k == h: 正常的节点扩展，更新邻居代价
    3. k > h: 该节点的代价减少了，需要传播代价减少
    """
    def __init__(self, maps):
        self.map = maps
        self.open_list = set()
        self.current_branch = None  # 当前执行的分支
        self.current_state = None  # 当前处理的状态
        self.step_info = ""  # 当前步骤的说明文本

    def process_state(self):
        """
        处理开放列表中的最小 k 值节点

        Returns:
            最小的 k 值，如果开放列表为空返回 -1
        """
        x = self.min_state()

        if x is None:
            return -1

        k_old = self.get_kmin()
        self.remove(x)
        self.current_state = x

        # 分支 1: k < h - 代价增加，传播变化
        if k_old < x.h:
            self.current_branch = DStarBranch.K_LESS_THAN_H
            self.step_info = f"Processing node ({x.x}, {x.y}): k={k_old:.1f} < h={x.h:.1f}\n" \
                           f"Cost increased for this path, propagating changes..."
            for y in self.map.get_neighbors(x):
                if y.h <= k_old and x.h > y.h + x.cost(y):
                    x.parent = y
                    x.h = y.h + x.cost(y)

        # 分支 2: k == h - 正常扩展
        elif k_old == x.h:
            self.current_branch = DStarBranch.K_EQUAL_H
            self.step_info = f"Processing node ({x.x}, {x.y}): k={k_old:.1f} == h={x.h:.1f}\n" \
                           f"Normal expansion, updating neighbor costs..."
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y) \
                        or y.parent != x and y.h > x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))

        # 分支 3: k > h - 代价减少，传播改进
        else:
            self.current_branch = DStarBranch.K_GREATER_THAN_H
            self.step_info = f"Processing node ({x.x}, {x.y}): k={k_old:.1f} > h={x.h:.1f}\n" \
                           f"Cost decreased for this path, propagating improvements..."
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y):
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

    def min_state(self):
        """获取开放列表中 k 值最小的状态"""
        if not self.open_list:
            return None
        min_state = min(self.open_list, key=lambda x: x.k)
        return min_state

    def get_kmin(self):
        """获取开放列表中最小的 k 值"""
        if not self.open_list:
            return -1
        k_min = min([x.k for x in self.open_list])
        return k_min

    def insert(self, state, h_new):
        """
        将状态插入开放列表

        根据状态标签设置合适的 k 值:
        - new: k = h_new
        - open: k = min(k, h_new)
        - close: k = min(h, h_new)
        """
        if state.t == "new":
            state.k = h_new
        elif state.t == "open":
            state.k = min(state.k, h_new)
        elif state.t == "close":
            state.k = min(state.h, h_new)
        state.h = h_new
        state.t = "open"
        self.open_list.add(state)

    def remove(self, state):
        """从开放列表中移除状态并标记为关闭"""
        if state.t == "open":
            state.t = "close"
        self.open_list.remove(state)

    def modify_cost(self, x):
        """
        修改节点代价 - 当检测到新的障碍物时调用

        这是 D* 算法的核心：只重新计算受影响的节点
        """
        if x.t == "close":
            self.insert(x, x.parent.h + x.cost(x.parent))

    def run_interactive(self, start, end):
        """
        交互式运行 D* 算法
        返回生成器，每次 yield 一步算法状态
        """
        rx, ry = [], []

        # 步骤 1: 初始搜索，从 goal 向后搜索到 start
        self.insert(end, 0.0)
        yield "initial_planning", rx, ry

        phase = "initial_planning"

        while phase == "initial_planning":
            if not self.open_list:
                yield "finished", rx, ry
                break

            k_min = self.process_state()
            yield "processing", rx, ry

            if start.t == "close":
                phase = "building_path"
                self.step_info = "Initial planning complete! Starting node reached.\n" \
                               "Now extracting path..."

        # 步骤 2: 构建初始路径
        if phase == "building_path":
            start.set_state("s")
            s = start
            s = s.parent
            s.set_state("e")
            tmp = start

            yield "building_path", rx, ry

            # 添加新的障碍物
            yield "adding_obstacle", rx, ry

            # 添加动态障碍物
            self.add_new_obstacle()
            phase = "path_repair"
            self.step_info = "New obstacle detected! Current path is blocked.\n" \
                           "Starting incremental path repair..."

            while tmp != end:
                tmp.set_state("*")
                rx.append(tmp.x)
                ry.append(tmp.y)
                yield "path_tracking", rx, ry

                if tmp.parent.state == "#":
                    # 障碍物在路径上，需要修复
                    phase = "repairing"
                    self.step_info = f"Obstacle encountered at ({tmp.x}, {tmp.y})!\n" \
                                   f"Triggering D* incremental repair..."
                    yield "repair_started", rx, ry

                    # 修复路径
                    self.modify_cost(tmp)
                    yield "modify_cost", rx, ry

                    while True:
                        k_min = self.process_state()
                        yield "repair_step", rx, ry

                        if k_min >= tmp.h:
                            break

                    phase = "path_tracking"
                    self.step_info = "Path repair complete! Resuming navigation..."
                    yield "repair_complete", rx, ry
                    continue

                tmp = tmp.parent

            tmp.set_state("e")
            yield "finished", rx, ry

    def add_new_obstacle(self):
        """添加新的障碍物来演示动态规划"""
        # 在原有路径上添加障碍物
        for i in range(5, 21):
            if 0 <= i < self.map.row:
                self.map[i][40].set_state("#")


class DStarVisualizer:
    """
    D* 算法交互式可视化器

    使用 matplotlib 提供逐步控制、动画和信息面板
    """

    def __init__(self, row=40, col=60):
        self.row = row
        self.col = col
        self.setup_map()
        self.setup_visualization()

    def setup_map(self):
        """设置测试地图"""
        self.map = Map(self.row, self.col)

        # 边界障碍物
        for i in range(10, 35):
            self.map.set_obstacle([(i, 10)])  # 上边界
            self.map.set_obstacle([(i, 50)])  # 下边界
        for i in range(10, 51):
            self.map.set_obstacle([(35, i)])  # 右边界
            self.map.set_obstacle([(10, i)])  # 左边界

        # 第一个障碍物
        for i in range(10, 30):
            self.map.set_obstacle([(20, i + 10)])

        # 起点和目标
        self.start = self.map.map[15][15]
        self.end = self.map.map[30][40]

        self.dstar = DStarInteractive(self.map)
        self.generator = None
        self.current_phase = None
        self.path_x = []
        self.path_y = []

    def setup_visualization(self):
        """设置 matplotlib 可视化界面"""
        self.fig = plt.figure(figsize=(16, 9))
        self.fig.canvas.manager.set_window_title('D* Algorithm Interactive Demo')

        # 主绘图区域
        gs = self.fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)

        # 地图绘制区域
        self.ax = self.fig.add_subplot(gs[0:2, 0:2])
        self.ax.set_title('D* Path Planning Visualization', fontsize=14, fontweight='bold')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_xlim(0, self.col)
        self.ax.set_ylim(0, self.row)
        self.ax.grid(True, alpha=0.3, linestyle='--')
        self.ax.set_aspect('equal')

        # 信息面板 1: 统计信息
        self.ax_info1 = self.fig.add_subplot(gs[0, 2])
        self.ax_info1.axis('off')
        self.info1_text = self.ax_info1.text(0, 1, '', transform=self.ax_info1.transAxes,
                                            fontsize=10, verticalalignment='top',
                                            family='monospace')

        # 信息面板 2: 当前步骤详情
        self.ax_info2 = self.fig.add_subplot(gs[1, 2])
        self.ax_info2.axis('off')
        self.info2_text = self.ax_info2.text(0, 1, '', transform=self.ax_info2.transAxes,
                                            fontsize=9, verticalalignment='top',
                                            family='monospace')

        # 图例
        self.ax_legend = self.fig.add_subplot(gs[2, 0:2])
        self.ax_legend.axis('off')
        self.setup_legend()

        # 控制按钮区域
        self.ax_controls = self.fig.add_subplot(gs[:, 2])
        self.ax_controls.axis('off')
        self.setup_controls()

        # 初始化状态
        self.auto_play = False
        self.auto_play_speed = 0.1
        self.finished = False
        self.auto_play_timer = None

        plt.figure(self.fig.number)

    def setup_legend(self):
        """设置图例"""
        legend_items = [
            ('START', 's', 'green'),
            ('GOAL', 'e', 'blue'),
            ('OBSTACLE', '#', 'black'),
            ('CLOSED (*)', '*', 'gray'),
            ('OPEN LIST', 'open', 'cyan'),
            ('CURRENT PATH', 'path', 'red'),
            ('NEW OBSTACLE', 'new_obstacle', 'yellow'),
        ]

        legend_text = "LEGEND:\n\n"
        for i, (name, marker, color) in enumerate(legend_items):
            legend_text += f"{color.upper()} {marker}: {name}\n"

        self.ax_legend.text(0.05, 0.9, legend_text,
                           fontsize=9, fontweight='bold',
                           verticalalignment='top')

    def setup_controls(self):
        """设置控制按钮"""
        btn_y_start = 0.85
        btn_height = 0.08
        btn_spacing = 0.10

        # Step Forward 按钮
        ax_step = plt.axes([0.75, btn_y_start, 0.15, btn_height])
        self.btn_step = Button(ax_step, 'Step Forward', color='lightblue', hovercolor='blue')
        self.btn_step.on_clicked(self.step_forward)

        # Auto-play 按钮
        ax_play = plt.axes([0.75, btn_y_start - btn_spacing, 0.15, btn_height])
        self.btn_play = Button(ax_play, 'Auto-play', color='lightgreen', hovercolor='green')
        self.btn_play.on_clicked(self.toggle_auto_play)

        # Reset 按钮
        ax_reset = plt.axes([0.75, btn_y_start - 2*btn_spacing, 0.15, btn_height])
        self.btn_reset = Button(ax_reset, 'Reset', color='lightcoral', hovercolor='red')
        self.btn_reset.on_clicked(self.reset)

        # 速度滑块
        ax_speed = plt.axes([0.75, btn_y_start - 3*btn_spacing, 0.15, 0.05])
        self.slider_speed = Slider(ax_speed, 'Speed', 0.01, 1.0, valinit=0.1)
        self.slider_speed.on_changed(self.update_speed)

        # 控制说明
        controls_text = "CONTROLS:\n\n" \
                       "Step Forward: Advance one step\n" \
                       "Auto-play: Run automatically\n" \
                       "Reset: Start over\n" \
                       "Speed: Animation speed\n\n" \
                       "PHASES:\n\n" \
                       "1. Initial Planning\n" \
                       "   Backward search from\n" \
                       "   goal to start\n\n" \
                       "2. Add Obstacle\n" \
                       "   Dynamic obstacle\n" \
                       "   appears\n\n" \
                       "3. Path Repair\n" \
                       "   Incremental replanning\n" \
                       "   (D* advantage!)"

        self.ax_controls.text(0.05, 0.05, controls_text,
                            fontsize=9, verticalalignment='bottom')

    def update_speed(self, val):
        """更新动画速度"""
        self.auto_play_speed = val
        # 如果正在自动播放，重新启动计时器以应用新速度
        if self.auto_play:
            self.run_auto_play()

    def step_forward(self, event):
        """前进一步"""
        if self.finished:
            return

        if self.generator is None:
            self.generator = self.dstar.run_interactive(self.start, self.end)

        try:
            phase, rx, ry = next(self.generator)
            self.current_phase = phase
            self.path_x = rx
            self.path_y = ry

            if phase == "finished":
                self.finished = True
                self.dstar.step_info = "Algorithm complete! Path found."
            elif phase == "add_new_obstacle":
                self.dstar.step_info = "Adding new obstacle blocking the path..."
            elif phase == "modify_cost":
                self.dstar.step_info = "Modifying costs for affected nodes..."

            self.update_plot()

        except StopIteration:
            self.finished = True
            self.update_plot()

    def toggle_auto_play(self, event):
        """切换自动播放"""
        self.auto_play = not self.auto_play
        if self.auto_play:
            self.btn_play.label.set_text('Pause')
            self.run_auto_play()
        else:
            self.btn_play.label.set_text('Auto-play')

    def run_auto_play(self):
        """自动播放"""
        if self.finished:
            return

        # Stop existing timer if any
        if self.auto_play_timer is not None:
            try:
                # Stop the timer if it has a stop method (works with most backends)
                if hasattr(self.auto_play_timer, 'stop'):
                    self.auto_play_timer.stop()
                elif hasattr(self.auto_play_timer, 'event_source'):
                    # Try the alternative method
                    self.auto_play_timer.event_source.stop()
            except (AttributeError, Exception):
                # If stopping fails, just create a new timer
                pass

        if self.auto_play:
            # Start timer for auto-play
            def auto_play_step():
                if self.auto_play and not self.finished:
                    self.step_forward(None)
                    # Continue auto-play if still running
                    if self.auto_play and not self.finished:
                        self.run_auto_play()

            self.auto_play_timer = self.fig.canvas.new_timer(
                interval=int(self.auto_play_speed * 1000)
            )
            self.auto_play_timer.add_callback(auto_play_step)
            self.auto_play_timer.start()

    def reset(self, event):
        """重置算法"""
        # 停止自动播放
        self.auto_play = False
        # 停止并清理计时器
        if self.auto_play_timer is not None:
            try:
                if hasattr(self.auto_play_timer, 'stop'):
                    self.auto_play_timer.stop()
            except (AttributeError, Exception):
                pass
            self.auto_play_timer = None

        self.btn_play.label.set_text('Auto-play')
        self.finished = False
        self.generator = None
        self.current_phase = None
        self.path_x = []
        self.path_y = []
        self.setup_map()
        self.update_plot()

    def update_plot(self):
        """更新可视化显示"""
        self.ax.clear()

        # 重新设置标题和标签
        self.ax.set_title('D* Path Planning Visualization', fontsize=14, fontweight='bold')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_xlim(0, self.col)
        self.ax.set_ylim(0, self.row)
        self.ax.grid(True, alpha=0.3, linestyle='--')
        self.ax.invert_yaxis()  # 翻转 Y 轴以匹配网格坐标

        # 绘制障碍物
        ox, oy = [], []
        for i in range(self.row):
            for j in range(self.col):
                state = self.map.map[i][j]
                if state.state == "#":
                    ox.append(state.y)
                    oy.append(state.x)

        if ox:
            self.ax.scatter(ox, oy, c='black', s=40, marker='s', label='Obstacle', zorder=2)

        # 绘制开放列表
        open_list_x = [s.y for s in self.dstar.open_list]
        open_list_y = [s.x for s in self.dstar.open_list]
        if open_list_x:
            self.ax.scatter(open_list_x, open_list_y, c='cyan', s=80,
                          marker='o', edgecolors='blue', linewidth=2,
                          label='Open List', zorder=3)

        # 绘制关闭列表
        closed_x, closed_y = [], []
        for i in range(self.row):
            for j in range(self.col):
                state = self.map.map[i][j]
                if state.state == "*" or state.t == "close":
                    closed_x.append(state.y)
                    closed_y.append(state.x)

        if closed_x:
            self.ax.scatter(closed_x, closed_y, c='lightgray', s=40,
                          marker='.', label='Closed', zorder=1)

        # 高亮当前处理的状态
        if self.dstar.current_state:
            s = self.dstar.current_state
            self.ax.scatter([s.y], [s.x], c='orange', s=120, marker='*',
                          edgecolors='red', linewidth=3, label='Current', zorder=5)

        # 绘制起点和终点
        self.ax.scatter(self.start.y, self.start.x, c='green', s=150,
                       marker='s', edgecolors='black', linewidth=2,
                       label=f"Start ({self.start.x},{self.start.y})", zorder=4)
        self.ax.scatter(self.end.y, self.end.x, c='blue', s=150,
                       marker='x', linewidth=4,
                       label=f"Goal ({self.end.x},{self.end.y})", zorder=4)

        # 绘制当前路径
        if self.path_x and self.path_y:
            self.ax.plot(self.path_y, self.path_x, 'r-', linewidth=2,
                        label='Current Path', zorder=3)

        # 如果有新障碍物，用黄色高亮
        new_obst_x, new_obst_y = [], []
        for i in range(5, 21):
            if i < self.row and 40 < self.col:
                if self.map.map[i][40].state == "#":
                    new_obst_x.append(40)
                    new_obst_y.append(i)

        if new_obst_x:
            self.ax.scatter(new_obst_x, new_obst_y, c='yellow', s=60,
                          marker='s', edgecolors='red', linewidth=2,
                          label='New Obstacle', zorder=4)

        self.ax.legend(loc='upper left', fontsize=8)

        # 更新信息面板 1: 统计信息
        open_count = len(self.dstar.open_list)
        closed_count = len(closed_x)

        stats_text = f"STATISTICS:\n\n" \
                    f"Open List Size: {open_count}\n" \
                    f"Closed Nodes: {closed_count}\n" \
                    f"Current Phase: {self.current_phase or 'Not Started'}\n\n" \
                    f"Start: ({self.start.x}, {self.start.y})\n" \
                    f"Goal: ({self.end.x}, {self.end.y})"

        self.info1_text.set_text(stats_text)

        # 更新信息面板 2: 当前步骤详情
        step_text = f"CURRENT STEP:\n\n"

        if self.dstar.current_branch:
            branch_name = self.dstar.current_branch.value
            step_text += f"Branch: {branch_name}\n\n"

        if self.dstar.current_state:
            s = self.dstar.current_state
            step_text += f"Processing Node: ({s.x}, {s.y})\n" \
                        f"h = {s.h:.2f}\n" \
                        f"k = {s.k:.2f}\n\n"

            if s.parent:
                step_text += f"Parent: ({s.parent.x}, {s.parent.y})\n"

        step_text += "\n" + self.dstar.step_info

        self.info2_text.set_text(step_text)

        # 高亮当前分支的颜色说明
        if self.dstar.current_branch == DStarBranch.K_LESS_THAN_H:
            branch_color = 'red'
        elif self.dstar.current_branch == DStarBranch.K_EQUAL_H:
            branch_color = 'green'
        elif self.dstar.current_branch == DStarBranch.K_GREATER_THAN_H:
            branch_color = 'blue'
        else:
            branch_color = 'black'

        # 添加分支说明
        branch_explanation = "\n\nBRANCH EXPLANATION:\n"
        if self.dstar.current_branch == DStarBranch.K_LESS_THAN_H:
            branch_explanation += "k < h: Node cost increased\n" \
                                 "This path became more expensive.\n" \
                                 "Propagating the cost increase\n" \
                                 "to affected neighbors."
        elif self.dstar.current_branch == DStarBranch.K_EQUAL_H:
            branch_explanation += "k == h: Normal expansion\n" \
                                 "Standard D* node processing.\n" \
                                 "Updating neighbor costs\n" \
                                 "and expanding search."
        elif self.dstar.current_branch == DStarBranch.K_GREATER_THAN_H:
            branch_explanation += "k > h: Node cost decreased\n" \
                                 "Found a better path!\n" \
                                 "Propagating the improvement\n" \
                                 "through changed costs."

        self.info2_text.set_text(step_text + branch_explanation)
        self.info2_text.set_color(branch_color)

        self.fig.canvas.draw()


def main():
    """主函数：启动交互式 D* 演示"""
    print("D* Interactive Algorithm Demo")
    print("==============================")
    print("This demo demonstrates:")
    print("1. Initial backward planning from goal to start")
    print("2. Dynamic obstacle handling with incremental repair")
    print("3. The three branches of process_state()")
    print("\nUse on-screen controls to step through the algorithm.")
    print("\nPress 'Step Forward' to advance one step at a time.")
    print("Press 'Auto-play' to run the demonstration automatically.")
    print("Press 'Reset' to start over.")
    print("\nKey D* Concepts:")
    print("- h: Estimated cost to start (like g-value)")
    print("- k: Key for priority queue sorting")
    print("- Incremental repair: Only recompute affected nodes")

    visualizer = DStarVisualizer(row=40, col=60)
    visualizer.update_plot()
    plt.show()


if __name__ == '__main__':
    main()
