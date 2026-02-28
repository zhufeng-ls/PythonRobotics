"""
Hybrid A* 交互式可视化测试 Demo (增强版)
=============================================

功能:
1. 可调参数: SB_COST, BACK_COST, STEER_CHANGE_COST
2. 真正的步进式运行 - 使用生成器模式
3. 实时可视化搜索过程 (open list, closed list, current node)
4. 自动播放模式 - 使用 timer 实现非阻塞动画
5. 显示详细的搜索统计信息

作者: PythonRoboticsLearning
日期: 2026-01-23
版本: 2.0 (Enhanced with step-by-step visualization)
"""

import sys
sys.path.insert(0, '/home/zhufeng/code/PythonRobotics/PythonRobotics')

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from matplotlib.gridspec import GridSpec
import heapq
import time
from scipy.spatial import cKDTree

# 导入 Hybrid A* 相关模块
from PathPlanning.HybridAStar import hybrid_a_star
from PathPlanning.HybridAStar.hybrid_a_star import (
    Node, Config, Path,
    calc_index, calc_cost, calc_distance_heuristic,
    update_node_with_analytic_expansion, get_neighbors,
    BUBBLE_R, SB_COST, BACK_COST, STEER_CHANGE_COST
)
# 修改导入路径 - reeds_shepp 在 ReedsSheppPath 目录下
sys.path.append('/home/zhufeng/code/PythonRobotics/PythonRobotics/PathPlanning')
from ReedsSheppPath import reeds_shepp_path_planning as rs


class HybridAStarWrapper:
    """
    Hybrid A* 算法包装器 - 实现步进式执行
    使用生成器模式,每次迭代 yield 当前状态供可视化使用
    """
    def __init__(self, sb_cost=100.0, back_cost=5.0, steer_change_cost=5.0):
        self.sb_cost = sb_cost
        self.back_cost = back_cost
        self.steer_change_cost = steer_change_cost

    def run_step_by_step(self, start, goal, ox, oy, xy_resolution, yaw_resolution):
        """
        步进式运行 Hybrid A* 算法
        每次迭代 yield 当前状态字典

        Yields:
            dict: {
                'phase': 'searching' | 'analytic_expansion' | 'finished' | 'failed',
                'current': Node,
                'open_list': dict,
                'closed_list': dict,
                'pq_size': int,
                'final_path': Path (仅 finished 时)
            }
        """
        # 设置参数
        hybrid_a_star.SB_COST = self.sb_cost
        hybrid_a_star.BACK_COST = self.back_cost
        hybrid_a_star.STEER_CHANGE_COST = self.steer_change_cost

        # 初始化 (复制自 hybrid_a_star_planning 函数 254-280 行)
        start[2], goal[2] = rs.pi_2_pi(start[2]), rs.pi_2_pi(goal[2])
        tox, toy = ox[:], oy[:]

        obstacle_kd_tree = cKDTree(np.vstack((tox, toy)).T)

        config = Config(tox, toy, xy_resolution, yaw_resolution)

        start_node = Node(round(start[0] / xy_resolution),
                          round(start[1] / xy_resolution),
                          round(start[2] / yaw_resolution), True,
                          [start[0]], [start[1]], [start[2]], [True], cost=0)
        goal_node = Node(round(goal[0] / xy_resolution),
                         round(goal[1] / xy_resolution),
                         round(goal[2] / yaw_resolution), True,
                         [goal[0]], [goal[1]], [goal[2]], [True])

        openList, closedList = {}, {}

        h_dp = calc_distance_heuristic(
            goal_node.x_list[-1], goal_node.y_list[-1],
            ox, oy, xy_resolution, BUBBLE_R)

        pq = []
        openList[calc_index(start_node, config)] = start_node
        heapq.heappush(pq, (calc_cost(start_node, h_dp, config),
                            calc_index(start_node, config)))
        final_path = None

        # Yield 初始状态
        yield {
            'phase': 'initialized',
            'current': start_node,
            'open_list': dict(openList),
            'closed_list': dict(closedList),
            'pq_size': len(pq),
            'final_path': None,
            'goal_node': goal_node
        }

        # 主搜索循环 (复制自 282-321 行,转换为生成器)
        iteration_count = 0
        while True:
            iteration_count += 1

            if not openList:
                yield {
                    'phase': 'failed',
                    'current': None,
                    'open_list': dict(openList),
                    'closed_list': dict(closedList),
                    'pq_size': 0,
                    'final_path': None,
                    'iteration': iteration_count
                }
                break

            cost, c_id = heapq.heappop(pq)
            if c_id in openList:
                current = openList.pop(c_id)
                closedList[c_id] = current
            else:
                continue

            # **关键: 每次迭代 yield 当前状态**
            yield {
                'phase': 'searching',
                'current': current,
                'open_list': dict(openList),
                'closed_list': dict(closedList),
                'pq_size': len(pq),
                'final_path': None,
                'goal_node': goal_node,
                'iteration': iteration_count
            }

            # 尝试解析扩展 (Reeds-Shepp 曲线)
            is_updated, final_path = update_node_with_analytic_expansion(
                current, goal_node, config, ox, oy, obstacle_kd_tree)

            if is_updated:
                # 找到路径!
                path = hybrid_a_star.get_final_path(closedList, final_path)
                yield {
                    'phase': 'finished',
                    'current': current,
                    'open_list': dict(openList),
                    'closed_list': dict(closedList),
                    'pq_size': len(pq),
                    'final_path': path,
                    'goal_node': goal_node,
                    'iteration': iteration_count
                }
                break

            # 扩展邻居节点
            for neighbor in get_neighbors(current, config, ox, oy,
                                          obstacle_kd_tree):
                neighbor_index = calc_index(neighbor, config)
                if neighbor_index in closedList:
                    continue
                if neighbor_index not in openList \
                        or openList[neighbor_index].cost > neighbor.cost:
                    heapq.heappush(
                        pq, (calc_cost(neighbor, h_dp, config),
                             neighbor_index))
                    openList[neighbor_index] = neighbor


class HybridAStarInteractiveDemo:
    """·
    Hybrid A* 交互式演示 (增强版)
    支持真正的步进式运行和实时可视化
    """
    def __init__(self):
        # 默认参数
        self.sb_cost = 100.0
        self.back_cost = 5.0
        self.steer_change_cost = 5.0
        self.xy_resolution = 2.0  # Grid resolution (m)
        self.yaw_resolution_deg = 15.0  # Yaw resolution (degrees)

        # 算法状态
        self.wrapper = None
        self.generator = None
        self.current_state = None
        self.finished = False
        self.failed = False

        # 自动播放状态
        self.auto_play = False
        self.auto_play_timer = None

        # 场景设置
        self.setup_scene()

        # 创建图形界面
        self.create_widgets()

    def setup_scene(self):
        """设置测试场景 - 复杂障碍物布局展示参数影响"""
        self.ox, self.oy = [], []

        # 边界
        for i in range(80):
            self.ox.append(i)
            self.oy.append(0.0)
        for i in range(80):
            self.ox.append(80.0)
            self.oy.append(i)
        for i in range(81):
            self.ox.append(i)
            self.oy.append(80.0)
        for i in range(81):
            self.ox.append(0.0)
            self.oy.append(i)

        # 障碍物 1: 左上角L形墙 - 需要绕行
        for i in range(30, 50):
            self.ox.append(20.0)
            self.oy.append(i)
        for i in range(20, 31):
            self.ox.append(i)
            self.oy.append(50.0)

        # 障碍物 2: 右侧狭窄通道 - 测试倒车能力
        for i in range(20, 60):
            self.ox.append(60.0)
            self.oy.append(i)
        for i in range(60, 71):
            self.ox.append(i)
            self.oy.append(20.0)

        # 障碍物 3: 中间小障碍物群 - 测试精细避障
        for i in range(35, 45):
            self.ox.append(40.0)
            self.oy.append(i)
        for i in range(38, 43):
            self.ox.append(i + 5)
            self.oy.append(35.0)

        # 障碍物 4: 底部T形墙 - 需要切换方向
        for i in range(25, 55):
            self.ox.append(i)
            self.oy.append(15.0)
        for i in range(15, 25):
            self.ox.append(40.0)
            self.oy.append(i)

        # 障碍物 5: 目标区域附近的小障碍物 - 挑战性停车
        for i in range(65, 75):
            self.ox.append(i)
            self.oy.append(65.0)
        for i in range(60, 71):
            self.ox.append(65.0)
            self.oy.append(i)

        # 起点和终点 - 设计为需要切换方向和倒车的场景
        # 起点：左下角，朝右
        self.start = [10.0, 10.0, np.deg2rad(0.0)]
        # 终点：右上角狭窄空间，朝左（需要倒车入库）
        self.goal = [70.0, 70.0, np.deg2rad(180.0)]

    def create_widgets(self):
        """创建交互式控件 (使用 GridSpec 布局)"""
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('Hybrid A* Interactive Demo - Step-by-Step Visualization',
                         fontsize=16, fontweight='bold')

        # 使用 GridSpec 布局 (5行以容纳更多滑块)
        gs = GridSpec(5, 3, figure=self.fig,
                      height_ratios=[3, 0.3, 0.3, 0.3, 0.3],
                      hspace=0.4, wspace=0.3)

        # 主可视化区域 (左侧大块)
        self.ax_main = self.fig.add_subplot(gs[0:2, 0:2])

        # 信息面板 (右侧)
        self.ax_info = self.fig.add_subplot(gs[0:2, 2])

        # 控制按钮区域 (底部第一行)
        ax_step = self.fig.add_subplot(gs[2, 0])
        self.btn_step = Button(ax_step, 'Step Forward',
                               color='lightblue', hovercolor='blue')
        self.btn_step.on_clicked(self.step_forward)

        ax_play = self.fig.add_subplot(gs[2, 1])
        self.btn_auto_play = Button(ax_play, 'Auto Play',
                                    color='lightgreen', hovercolor='green')
        self.btn_auto_play.on_clicked(self.toggle_auto_play)

        ax_reset = self.fig.add_subplot(gs[2, 2])
        self.btn_reset = Button(ax_reset, 'Reset',
                               color='lightyellow', hovercolor='orange')
        self.btn_reset.on_clicked(self.reset)

        # 参数滑块 (底部第二行)
        ax_sb = self.fig.add_subplot(gs[3, 0])
        self.slider_sb = Slider(ax_sb, 'SB_COST', 10.0, 300.0,
                               valinit=100.0, valstep=10)
        self.slider_sb.on_changed(self.update_params)

        ax_back = self.fig.add_subplot(gs[3, 1])
        self.slider_back = Slider(ax_back, 'BACK_COST', 1.0, 20.0,
                                 valinit=5.0, valstep=0.5)
        self.slider_back.on_changed(self.update_params)

        ax_speed = self.fig.add_subplot(gs[3, 2])
        self.slider_speed = Slider(ax_speed, 'Speed(s)', 0.01, 1.0,
                                  valinit=0.05, valstep=0.01)

        # 分辨率滑块 (底部第三行)
        ax_xy_res = self.fig.add_subplot(gs[4, 0])
        self.slider_xy_res = Slider(ax_xy_res, 'XY Res(m)', 0.5, 5.0,
                                   valinit=2.0, valstep=0.5)
        self.slider_xy_res.on_changed(self.update_params)

        ax_yaw_res = self.fig.add_subplot(gs[4, 1])
        self.slider_yaw_res = Slider(ax_yaw_res, 'Yaw Res(deg)', 5.0, 45.0,
                                    valinit=15.0, valstep=5.0)
        self.slider_yaw_res.on_changed(self.update_params)

        # 初始化显示
        self.draw_initial_scene()
        self.update_info_panel()

    def draw_initial_scene(self):
        """绘制初始场景"""
        self.ax_main.clear()

        # 障碍物
        self.ax_main.plot(self.ox, self.oy, ".k", markersize=3, label='Obstacles')

        # 起点和终点
        self.ax_main.plot(self.start[0], self.start[1], "xg",
                         markersize=15, linewidth=4, label='Start')
        self.ax_main.plot(self.goal[0], self.goal[1], "xr",
                         markersize=15, linewidth=4, label='Goal')

        self.ax_main.legend(loc='upper right', fontsize=9)
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.axis('equal')
        self.ax_main.set_title('Hybrid A* Step-by-Step Visualization')

        plt.draw()

    def update_params(self, val):
        """更新参数"""
        self.sb_cost = self.slider_sb.val
        self.back_cost = self.slider_back.val
        self.xy_resolution = self.slider_xy_res.val
        self.yaw_resolution_deg = self.slider_yaw_res.val

    def step_forward(self, event):
        """单步前进一次迭代"""
        if self.finished or self.failed:
            return

        # 首次运行: 创建生成器
        if self.generator is None:
            self.wrapper = HybridAStarWrapper(
                self.sb_cost, self.back_cost, self.steer_change_cost
            )
            self.generator = self.wrapper.run_step_by_step(
                self.start.copy(), self.goal.copy(),
                self.ox, self.oy, self.xy_resolution, np.deg2rad(self.yaw_resolution_deg)
            )

        # 获取下一步状态
        try:
            self.current_state = next(self.generator)
            self.update_visualization()

            if self.current_state['phase'] == 'finished':
                self.finished = True
                print("\n✓ Path found!")
                if self.auto_play:
                    self.toggle_auto_play(None)
            elif self.current_state['phase'] == 'failed':
                self.failed = True
                print("\n✗ Failed to find path!")
                if self.auto_play:
                    self.toggle_auto_play(None)
        except StopIteration:
            self.finished = True
            if self.auto_play:
                self.toggle_auto_play(None)

    def toggle_auto_play(self, event):
        """切换自动播放模式"""
        self.auto_play = not self.auto_play

        if self.auto_play:
            self.btn_auto_play.label.set_text('Pause')
            self.run_auto_play()
        else:
            self.btn_auto_play.label.set_text('Auto Play')
            if self.auto_play_timer:
                try:
                    self.auto_play_timer.stop()
                except:
                    pass

    def run_auto_play(self):
        """使用 timer 实现非阻塞自动播放"""
        # 停止旧 timer
        if self.auto_play_timer:
            try:
                self.auto_play_timer.stop()
            except:
                pass

        if self.auto_play and not self.finished and not self.failed:
            def auto_step():
                if self.auto_play and not self.finished and not self.failed:
                    self.step_forward(None)
                    if self.auto_play and not self.finished and not self.failed:
                        self.run_auto_play()  # 递归调度

            # 使用 speed slider 的值作为延迟
            interval = int(self.slider_speed.val * 1000)  # 转为毫秒
            self.auto_play_timer = self.fig.canvas.new_timer(interval=interval)
            self.auto_play_timer.add_callback(auto_step)
            self.auto_play_timer.start()

    def update_visualization(self):
        """更新所有可视化面板"""
        if self.current_state is None:
            return

        self.ax_main.clear()

        # 绘制基础场景
        self.ax_main.plot(self.ox, self.oy, ".k", markersize=3, label='Obstacles')
        self.ax_main.plot(self.start[0], self.start[1], "xg",
                         markersize=15, linewidth=4, label='Start')
        self.ax_main.plot(self.goal[0], self.goal[1], "xr",
                         markersize=15, linewidth=4, label='Goal')

        # 绘制 closed list (已访问节点) - 限制数量避免卡顿
        closed_list = self.current_state.get('closed_list', {})
        if len(closed_list) < 5000:  # 限制绘制数量
            for node in list(closed_list.values())[::2]:  # 每隔一个绘制
                self.ax_main.plot(node.x_list[-1], node.y_list[-1],
                                 ".", color='lightgray', markersize=4)

        # 绘制 open list (待访问节点)
        open_list = self.current_state.get('open_list', {})
        if len(open_list) < 2000:  # 限制绘制数量
            for node in open_list.values():
                self.ax_main.plot(node.x_list[-1], node.y_list[-1],
                                 "o", color='cyan', markersize=6)

        # 绘制当前节点 (高亮)
        current = self.current_state.get('current')
        if current:
            self.ax_main.plot(current.x_list[-1], current.y_list[-1],
                             "*", color='orange', markersize=20,
                             markeredgecolor='red', markeredgewidth=2,
                             label='Current')

            # 绘制当前节点的轨迹
            self.ax_main.plot(current.x_list, current.y_list,
                             '-', color='blue', linewidth=2, alpha=0.7)

        # 如果有最终路径,绘制路径
        if self.current_state.get('phase') == 'finished':
            path = self.current_state.get('final_path')
            if path and path.x_list:
                for i in range(len(path.x_list) - 1):
                    color = 'red' if path.direction_list[i] else 'blue'
                    self.ax_main.plot([path.x_list[i], path.x_list[i+1]],
                                    [path.y_list[i], path.y_list[i+1]],
                                    c=color, linewidth=3, alpha=0.9)
                self.ax_main.plot([], [], 'r-', linewidth=3, label='Forward')
                self.ax_main.plot([], [], 'b-', linewidth=3, label='Backward')

        self.ax_main.legend(loc='upper right', fontsize=9)
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.axis('equal')

        # 设置标题显示迭代次数
        iteration = self.current_state.get('iteration', 0)
        phase = self.current_state.get('phase', 'unknown')
        self.ax_main.set_title(f'Hybrid A* Visualization - Iteration: {iteration} - Phase: {phase}')

        # 更新信息面板
        self.update_info_panel()

        plt.draw()

    def update_info_panel(self):
        """更新右侧信息面板"""
        self.ax_info.clear()
        self.ax_info.axis('off')

        if self.current_state is None:
            info_text = "READY TO START\n"
            info_text += "=" * 30 + "\n\n"
            info_text += "Click 'Step Forward' to begin\n"
            info_text += "or 'Auto Play' for continuous\n\n"
            info_text += "PARAMETERS\n"
            info_text += "-" * 30 + "\n"
            info_text += f"SB_COST: {self.sb_cost}\n"
            info_text += f"BACK_COST: {self.back_cost}\n"
            info_text += f"STEER_COST: {self.steer_change_cost}\n"
            info_text += f"XY_RES: {self.xy_resolution:.1f} m\n"
            info_text += f"YAW_RES: {self.yaw_resolution_deg:.0f}°\n"
        else:
            open_size = len(self.current_state.get('open_list', {}))
            closed_size = len(self.current_state.get('closed_list', {}))
            phase = self.current_state.get('phase', 'unknown')
            iteration = self.current_state.get('iteration', 0)

            current = self.current_state.get('current')

            info_text = "SEARCH STATUS\n"
            info_text += "=" * 30 + "\n\n"
            info_text += f"Phase: {phase}\n"
            info_text += f"Iteration: {iteration}\n\n"
            info_text += f"Open List: {open_size}\n"
            info_text += f"Closed Nodes: {closed_size}\n"
            info_text += f"Total Explored: {closed_size + open_size}\n\n"

            if current:
                info_text += "CURRENT NODE\n"
                info_text += "-" * 30 + "\n"
                info_text += f"Pos: ({current.x_list[-1]:.2f}, "
                info_text += f"{current.y_list[-1]:.2f})\n"
                info_text += f"Heading: {np.rad2deg(current.yaw_list[-1]):.1f}°\n"
                info_text += f"Cost: {current.cost:.2f}\n"
                info_text += f"Direction: {'FWD' if current.direction else 'BWD'}\n\n"

            # 如果完成,显示路径信息
            if phase == 'finished':
                path = self.current_state.get('final_path')
                if path and path.x_list:
                    path_len = sum(np.hypot(np.diff(path.x_list), np.diff(path.y_list)))
                    switches = sum(1 for i in range(len(path.direction_list)-1)
                                  if path.direction_list[i] != path.direction_list[i+1])

                    info_text += "PATH FOUND!\n"
                    info_text += "-" * 30 + "\n"
                    info_text += f"Length: {path_len:.2f} m\n"
                    info_text += f"Switches: {switches}\n"
                    info_text += f"Points: {len(path.x_list)}\n\n"

            info_text += "PARAMETERS\n"
            info_text += "-" * 30 + "\n"
            info_text += f"SB_COST: {self.sb_cost}\n"
            info_text += f"BACK_COST: {self.back_cost}\n"
            info_text += f"STEER_COST: {self.steer_change_cost}\n"
            info_text += f"XY_RES: {self.xy_resolution:.1f} m\n"
            info_text += f"YAW_RES: {self.yaw_resolution_deg:.0f}°\n"

        self.ax_info.text(0.05, 0.95, info_text,
                         transform=self.ax_info.transAxes,
                         fontsize=9, verticalalignment='top',
                         fontfamily='monospace')

        plt.draw()

    def reset(self, event):
        """重置所有状态"""
        # 停止自动播放
        self.auto_play = False
        if self.auto_play_timer:
            try:
                self.auto_play_timer.stop()
            except:
                pass
            self.auto_play_timer = None

        # 重置按钮状态
        self.btn_auto_play.label.set_text('Auto Play')

        # 清空算法状态
        self.wrapper = None
        self.generator = None
        self.current_state = None
        self.finished = False
        self.failed = False

        # 重绘初始场景
        self.draw_initial_scene()
        self.update_info_panel()

        print("\n" + "="*60)
        print("Reset complete. Ready to plan again.")
        print("="*60)

    def show(self):
        """显示界面"""
        plt.show()


def main():
    """主函数"""
    print("="*60)
    print("Hybrid A* Interactive Demo - Enhanced Version")
    print("="*60)
    print("\nFeatures:")
    print("  ✓ True step-by-step execution")
    print("  ✓ Real-time search visualization")
    print("  ✓ Auto-play mode with adjustable speed")
    print("  ✓ Detailed search statistics")
    print("\nComplex Test Scene:")
    print("  • L-shape wall (top-left) - requires detour")
    print("  • Narrow channel (right) - tests reversing capability")
    print("  • T-shape wall (bottom) - requires direction changes")
    print("  • Parking zone (top-right) - challenging reverse parking")
    print("  • Start: (10, 10) heading East →")
    print("  • Goal: (70, 70) heading West ← (requires reverse)")
    print("\nInstructions:")
    print("  1. Adjust parameters using sliders:")
    print("     - SB_COST, BACK_COST (cost penalties)")
    print("     - XY Res, Yaw Res (search resolution)")
    print("  2. Click 'Step Forward' to execute one search iteration")
    print("  3. Click 'Auto Play' for continuous execution")
    print("  4. Adjust 'Speed' slider to control auto-play speed")
    print("  5. Click 'Reset' to restart with different parameters")
    print("\nParameter Effects:")
    print("  • High SB_COST    → Fewer direction switches, longer path")
    print("  • High BACK_COST  → Less reversing, may take longer route")
    print("  • Low both        → More reversing/switching, shorter path")
    print("  • Low XY_RES      → Finer grid, slower but more accurate")
    print("  • Low YAW_RES     → More angles, smoother but slower")
    print("  • High resolutions → Faster search, coarser path")
    print("\nVisualization Legend:")
    print("  Gray dots    = Closed nodes (explored)")
    print("  Cyan circles = Open nodes (frontier)")
    print("  Orange star  = Current node")
    print("  Red line     = Forward motion")
    print("  Blue line    = Backward motion")
    print("\n" + "="*60)

    demo = HybridAStarInteractiveDemo()
    demo.show()


if __name__ == "__main__":
    main()
