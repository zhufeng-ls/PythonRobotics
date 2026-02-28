"""
Interactive A* Demo for Hybrid A* Phase 1 Understanding
Date: 2026-01-22
Objective: Step-by-step visualization of 3D A* search used in Hybrid A*

This demo shows:
1. 3D grid search (x, y, yaw)
2. Open set (frontier) and closed set (visited)
3. Node expansion process
4. Cost calculation (g + h)
5. Path reconstruction

Controls:
- Step Forward: Execute one iteration
- Step Backward: Go back one step
- Reset: Restart the demo
- Auto-play: Run automatically
"""

import heapq
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button
import numpy as np


class Node:
    """
    3D Node for A* search
    State: (x, y, yaw) - position and heading
    """

    def __init__(self, x, y, yaw, g_cost=0, h_cost=0, parent=None):
        self.x = x
        self.y = y
        self.yaw = yaw  # Heading angle in degrees
        self.g_cost = g_cost  # Cost from start
        self.h_cost = h_cost  # Heuristic to goal
        self.f_cost = g_cost + h_cost  # Total cost
        self.parent = parent

    def __lt__(self, other):
        """For priority queue comparison"""
        return self.f_cost < other.f_cost

    def get_position(self):
        """Get (x, y) position"""
        return (self.x, self.y)

    def get_state(self):
        """Get full state (x, y, yaw)"""
        return (self.x, self.y, self.yaw)


class InteractiveAStar:
    """
    Interactive A* algorithm with 3D grid search (x, y, yaw)

    This demonstrates the core search mechanism used in Hybrid A* Phase 1
    """

    def __init__(self, grid_size=20, obstacle_list=None):
        """
        Initialize A* search

        Args:
            grid_size: Size of the grid (grid_size x grid_size)
            obstacle_list: List of obstacles [(x, y, radius), ...]
        """
        self.grid_size = grid_size
        self.obstacle_list = obstacle_list if obstacle_list else []

        # Grid resolution
        self.xy_resolution = 1.0  # 1 meter per grid
        self.yaw_resolution = 45  # 45 degrees per yaw level

        # Initialize search state
        self.start = None
        self.goal = None
        self.open_set = []  # Priority queue: (f_cost, node)
        self.open_dict = {}  # Quick lookup: state -> node
        self.closed_set = {}  # Visited nodes: state -> node
        self.current_node = None

        # Neighbor generation state (for step-by-step visualization)
        self.neighbors = []  # List of generated neighbors
        self.current_neighbor_index = 0  # Which neighbor we're evaluating
        self.evaluating_neighbors = False  # Are we in neighbor evaluation phase?

        # History for step backward
        self.history = []  # List of (open_set, open_dict, closed_set, current_node, info)

        # Statistics
        self.iteration = 0
        self.nodes_expanded = 0
        self.path_found = False
        self.final_path = None

    def heuristic(self, node):
        """
        Calculate heuristic cost (h_cost) from node to goal
        Using Euclidean distance

        Args:
            node: Current node

        Returns:
            float: Heuristic cost
        """
        if self.goal is None:
            return 0

        dx = node.x - self.goal.x
        dy = node.y - self.goal.y
        return np.sqrt(dx**2 + dy**2)

    def is_collision(self, x, y):
        """
        Check if position (x, y) is in collision

        Args:
            x, y: Position to check

        Returns:
            bool: True if collision
        """
        # Check grid boundaries
        if x < 0 or x >= self.grid_size or y < 0 or y >= self.grid_size:
            return True

        # Check obstacles
        for ox, oy, radius in self.obstacle_list:
            d = np.sqrt((x - ox)**2 + (y - oy)**2)
            if d <= radius:
                return True

        return False

    def get_neighbors(self, node):
        """
        Generate neighbor nodes considering vehicle motion primitives

        This is where Hybrid A* differs from standard 2D A*:
        - Instead of 8 directions (up, down, left, right, diagonals)
        - We use motion primitives (forward, backward, left turn, right turn)

        Args:
            node: Current node

        Returns:
            list: Valid neighbor nodes
        """
        neighbors = []

        # Motion primitives: (dx, dy, dyaw, motion_cost)
        # These simulate vehicle movement with steering
        motions = [
            # Forward movements
            (1, 0, 0, 1.0),        # Move forward (heading 0°)
            (0, 1, 90, 1.0),       # Move forward (heading 90°)
            (-1, 0, 180, 1.0),     # Move forward (heading 180°)
            (0, -1, 270, 1.0),     # Move forward (heading 270°)

            # Diagonal movements (simplified, real Hybrid A* uses curves)
            (1, 1, 45, 1.414),     # Diagonal NE
            (1, -1, 315, 1.414),   # Diagonal SE
            (-1, 1, 135, 1.414),   # Diagonal NW
            (-1, -1, 225, 1.414),  # Diagonal SW
        ]

        for dx, dy, dyaw, move_cost in motions:
            # Calculate new position
            new_x = node.x + dx
            new_y = node.y + dy
            new_yaw = dyaw  # In real Hybrid A*, this would be node.yaw + dyaw

            # Check collision
            if self.is_collision(new_x, new_y):
                continue

            # Calculate new g_cost (cost from start)
            new_g_cost = node.g_cost + move_cost

            # Create neighbor node
            neighbor = Node(
                x=new_x,
                y=new_y,
                yaw=new_yaw,
                g_cost=new_g_cost,
                h_cost=0,  # Will be calculated later
                parent=node
            )

            # Calculate heuristic
            neighbor.h_cost = self.heuristic(neighbor)
            neighbor.f_cost = neighbor.g_cost + neighbor.h_cost

            neighbors.append(neighbor)

        return neighbors

    def get_state_index(self, node):
        """
        Get discrete state index for 3D grid (x, y, yaw)
        This is used for hashing visited nodes

        Args:
            node: Node to get index for

        Returns:
            tuple: (ix, iy, iyaw) discrete indices
        """
        ix = int(node.x / self.xy_resolution)
        iy = int(node.y / self.xy_resolution)
        iyaw = int(node.yaw / self.yaw_resolution) % (360 // self.yaw_resolution)
        return (ix, iy, iyaw)

    def plan(self, start, goal, max_iter=1000):
        """
        Run A* algorithm (for auto-play mode)

        Args:
            start: Start position (x, y)
            goal: Goal position (x, y)
            max_iter: Maximum iterations

        Returns:
            list: Final path if found, None otherwise
        """
        self.reset()
        self.set_start_goal(start, goal)

        for _ in range(max_iter):
            if self.step():
                # Found goal
                return self.get_final_path()

            if len(self.open_set) == 0:
                # No path found
                return None

        return None

    def set_start_goal(self, start, goal):
        """
        Set start and goal positions

        Args:
            start: Start position (x, y)
            goal: Goal position (x, y)
        """
        self.start = Node(x=start[0], y=start[1], yaw=0)
        self.goal = Node(x=goal[0], y=goal[1], yaw=0)

        # Initialize open set with start node
        self.start.h_cost = self.heuristic(self.start)
        self.start.f_cost = self.start.g_cost + self.start.h_cost

        heapq.heappush(self.open_set, (self.start.f_cost, id(self.start), self.start))
        start_state = self.get_state_index(self.start)
        self.open_dict[start_state] = self.start

        # Save initial state
        self._save_history("Initialization: Start node added to open set")

    def step(self):
        """
        Execute one A* iteration (or one neighbor evaluation)

        This is the core A* algorithm step:
        1. Pop node with lowest f_cost from open set
        2. Move it to closed set
        3. Check if goal reached
        4. Generate neighbors (one by one now!)
        5. Add neighbors to open set if better

        Returns:
            bool: True if goal reached, False otherwise
        """
        # If we're in the middle of evaluating neighbors, continue with next neighbor
        if self.evaluating_neighbors:
            return self._evaluate_next_neighbor()

        if len(self.open_set) == 0:
            self._save_history("No path found - open set is empty!")
            return False

        # Step 1: Pop node with lowest f_cost
        f_cost, _, node = heapq.heappop(self.open_set)
        state = self.get_state_index(node)

        # Remove from open_dict if it's the same node
        if state in self.open_dict and self.open_dict[state].f_cost == node.f_cost:
            del self.open_dict[state]

        # Step 2: Add to closed set
        self.closed_set[state] = node
        self.current_node = node
        self.nodes_expanded += 1
        self.iteration += 1

        info = f"Iteration {self.iteration}: Expanding node ({node.x:.1f}, {node.y:.1f}, yaw={node.yaw}°) with f_cost={node.f_cost:.2f}"

        # Step 3: Check if goal reached
        dist_to_goal = np.sqrt((node.x - self.goal.x)**2 + (node.y - self.goal.y)**2)
        if dist_to_goal < 1.0:  # Within 1 meter
            self.path_found = True
            self.final_path = self._reconstruct_path(node)
            self._save_history(f"Goal reached! Path length: {len(self.final_path)} nodes")
            return True

        # Step 4: Generate all neighbors at once (but evaluate one by one)
        self.neighbors = self.get_neighbors(node)
        self.current_neighbor_index = 0
        self.evaluating_neighbors = True

        info += f"\n  Generated {len(self.neighbors)} neighbors, evaluating one by one..."
        self._save_history(info)

        # Immediately evaluate the first neighbor
        return self._evaluate_next_neighbor()

    def _evaluate_next_neighbor(self):
        """
        Evaluate the next neighbor in the list

        Returns:
            bool: True if goal reached, False otherwise
        """
        if self.current_neighbor_index >= len(self.neighbors):
            # All neighbors evaluated
            self.evaluating_neighbors = False
            self.neighbors = []
            self.current_neighbor_index = 0
            return False

        neighbor = self.neighbors[self.current_neighbor_index]
        neighbor_state = self.get_state_index(neighbor)

        info = f"Evaluating neighbor {self.current_neighbor_index + 1}/{len(self.neighbors)}: ({neighbor.x:.1f}, {neighbor.y:.1f}, yaw={neighbor.yaw}°)"

        # Check if in closed set
        if neighbor_state in self.closed_set:
            info += f"\n  ❌ Already visited (in closed set)"
        # Check if in open set
        elif neighbor_state in self.open_dict:
            existing_node = self.open_dict[neighbor_state]
            if neighbor.g_cost < existing_node.g_cost:
                # Found better path to this node
                info += f"\n  ✓ Better path found! (g: {existing_node.g_cost:.2f} -> {neighbor.g_cost:.2f})"
                heapq.heappush(self.open_set, (neighbor.f_cost, id(neighbor), neighbor))
                self.open_dict[neighbor_state] = neighbor
            else:
                info += f"\n  ⚠ Already in open set with better cost (g={existing_node.g_cost:.2f})"
        else:
            # New node, add to open set
            info += f"\n  ✓ Added to open set (g={neighbor.g_cost:.2f}, h={neighbor.h_cost:.2f}, f={neighbor.f_cost:.2f})"
            heapq.heappush(self.open_set, (neighbor.f_cost, id(neighbor), neighbor))
            self.open_dict[neighbor_state] = neighbor

        self.current_neighbor_index += 1
        self._save_history(info)
        return False

    def step_backward(self):
        """
        Go back one step in history

        Returns:
            bool: True if successful, False if at beginning
        """
        if len(self.history) <= 1:
            return False

        # Remove current state
        self.history.pop()

        # Restore previous state
        state = self.history[-1]
        self.open_set = state['open_set'].copy()
        self.open_dict = state['open_dict'].copy()
        self.closed_set = state['closed_set'].copy()
        self.current_node = state['current_node']
        self.iteration = state['iteration']
        self.nodes_expanded = state['nodes_expanded']
        self.path_found = state['path_found']
        self.final_path = state['final_path']

        return True

    def reset(self):
        """Reset the search state"""
        self.open_set = []
        self.open_dict = {}
        self.closed_set = {}
        self.current_node = None
        self.neighbors = []
        self.current_neighbor_index = 0
        self.evaluating_neighbors = False
        self.history = []
        self.iteration = 0
        self.nodes_expanded = 0
        self.path_found = False
        self.final_path = None

    def _save_history(self, info):
        """Save current state to history"""
        # Deep copy open_set (priority queue)
        open_set_copy = [(f, nid, n) for f, nid, n in self.open_set]

        self.history.append({
            'open_set': open_set_copy,
            'open_dict': self.open_dict.copy(),
            'closed_set': self.closed_set.copy(),
            'current_node': self.current_node,
            'iteration': self.iteration,
            'nodes_expanded': self.nodes_expanded,
            'path_found': self.path_found,
            'final_path': self.final_path,
            'info': info
        })

    def _reconstruct_path(self, node):
        """
        Reconstruct path from start to goal

        Args:
            node: Goal node

        Returns:
            list: Path from start to goal
        """
        path = []
        current = node
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        return path[::-1]  # Reverse to get start -> goal

    def get_final_path(self):
        """Get the final path"""
        return self.final_path


class AStarVisualizer:
    """
    Interactive visualization for A* algorithm
    """

    def __init__(self, grid_size=20):
        self.grid_size = grid_size
        self.planner = InteractiveAStar(grid_size=grid_size)
        self.is_playing = False
        self.play_timer = None

        # Create figure
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        plt.subplots_adjust(bottom=0.2)

        # Setup plot elements
        self._setup_plot()

        # Setup buttons
        self._setup_buttons()

        # Setup text displays
        self._setup_text_displays()

        # Initialize with default start and goal
        self.reset_demo()

    def _setup_plot(self):
        """Setup plot elements"""
        self.ax.set_xlim(-1, self.grid_size)
        self.ax.set_ylim(-1, self.grid_size)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Interactive A* Search Demo (3D Grid: x, y, yaw)')

        # Plot elements
        self.obstacles_patches = []
        self.open_set_scatter = self.ax.scatter([], [], c='lightgreen', s=100, marker='o', alpha=0.6, label='Open Set (Frontier)', edgecolors='green')
        self.closed_set_scatter = self.ax.scatter([], [], c='lightcoral', s=100, marker='o', alpha=0.6, label='Closed Set (Visited)', edgecolors='red')
        self.current_node_scatter = self.ax.scatter([], [], c='blue', s=200, marker='o', label='Current Node', zorder=10)
        self.evaluating_neighbor_scatter = self.ax.scatter([], [], c='yellow', s=150, marker='D', label='Evaluating Neighbor', zorder=9, edgecolors='orange', linewidth=2)
        self.start_scatter = self.ax.scatter([], [], c='green', s=200, marker='s', label='Start', zorder=10)
        self.goal_scatter = self.ax.scatter([], [], c='red', s=200, marker='*', label='Goal', zorder=10)
        self.path_line, = self.ax.plot([], [], 'b--', linewidth=2, label='Path')

        # Arrow for current node heading
        self.quiver = None

        self.ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1))

    def _setup_buttons(self):
        """Setup interactive buttons"""
        # Button positions
        ax_step_forward = plt.axes([0.15, 0.05, 0.15, 0.075])
        ax_step_backward = plt.axes([0.31, 0.05, 0.15, 0.075])
        ax_reset = plt.axes([0.47, 0.05, 0.15, 0.075])
        ax_play = plt.axes([0.63, 0.05, 0.15, 0.075])

        self.btn_step_forward = Button(ax_step_forward, 'Step Forward')
        self.btn_step_backward = Button(ax_step_backward, 'Step Backward')
        self.btn_reset = Button(ax_reset, 'Reset')
        self.btn_play = Button(ax_play, 'Auto Play')

        # Connect callbacks
        self.btn_step_forward.on_clicked(self.on_step_forward)
        self.btn_step_backward.on_clicked(self.on_step_backward)
        self.btn_reset.on_clicked(self.on_reset)
        self.btn_play.on_clicked(self.on_play)

    def _setup_text_displays(self):
        """Setup text information displays"""
        # Info text (top right)
        self.info_text = self.fig.text(0.02, 0.95, '', fontsize=9, verticalalignment='top',
                                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        # Statistics text (bottom right)
        self.stats_text = self.fig.text(0.02, 0.15, '', fontsize=9, verticalalignment='top',
                                        bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))

    def set_obstacles(self, obstacle_list):
        """Set obstacles for the demo"""
        self.planner.obstacle_list = obstacle_list

        # Clear old obstacles
        for patch in self.obstacles_patches:
            patch.remove()
        self.obstacles_patches.clear()

        # Draw new obstacles
        for ox, oy, radius in obstacle_list:
            circle = patches.Circle((ox, oy), radius, facecolor='gray', alpha=0.5, edgecolor='black')
            self.ax.add_patch(circle)
            self.obstacles_patches.append(circle)

        self.fig.canvas.draw()

    def reset_demo(self):
        """Reset the demo with default scenario"""
        # Define obstacles
        obstacles = [
            (5, 5, 1.5),
            (8, 8, 1.5),
            (12, 5, 1.5),
            (5, 12, 1.5),
            (15, 10, 1.5),
        ]
        self.set_obstacles(obstacles)

        # Reset planner
        self.planner.reset()
        self.planner.set_start_goal(start=(2, 2), goal=(17, 17))

        # Reset visualization
        self.is_playing = False
        self.btn_play.label.set_text('Auto Play')

        # Update display
        self.update_display()

    def on_step_forward(self, event):
        """Step forward button callback"""
        if self.planner.path_found:
            return

        self.planner.step()
        self.update_display()

    def on_step_backward(self, event):
        """Step backward button callback"""
        if len(self.planner.history) <= 1:
            return

        self.planner.step_backward()
        self.update_display()

    def on_reset(self, event):
        """Reset button callback"""
        self.reset_demo()

    def on_play(self, event):
        """Auto play button callback"""
        if self.is_playing:
            # Stop playing
            self.is_playing = False
            self.btn_play.label.set_text('Auto Play')
        else:
            # Start playing
            if self.planner.path_found:
                self.reset_demo()

            self.is_playing = True
            self.btn_play.label.set_text('Stop')
            self.auto_play()

    def auto_play(self):
        """Auto play mode"""
        if not self.is_playing:
            return

        if self.planner.path_found or len(self.planner.open_set) == 0:
            self.is_playing = False
            self.btn_play.label.set_text('Auto Play')
            return

        self.planner.step()
        self.update_display()

        # Schedule next step (500ms delay)
        self.fig.canvas.draw_idle()
        self.play_timer = self.fig.canvas.new_timer(interval=500)
        self.play_timer.single_shot = True
        self.play_timer.add_callback(self.auto_play)
        self.play_timer.start()

    def update_display(self):
        """Update the visualization"""
        # Update open set
        open_nodes = [(n.x, n.y) for n in self.planner.open_dict.values()]
        if open_nodes:
            self.open_set_scatter.set_offsets(open_nodes)
        else:
            self.open_set_scatter.set_offsets(np.zeros((0, 2)))

        # Update closed set
        closed_nodes = [(n.x, n.y) for n in self.planner.closed_set.values()]
        if closed_nodes:
            self.closed_set_scatter.set_offsets(closed_nodes)
        else:
            self.closed_set_scatter.set_offsets(np.zeros((0, 2)))

        # Update current node
        if self.planner.current_node:
            self.current_node_scatter.set_offsets([[self.planner.current_node.x, self.planner.current_node.y]])

            # Update quiver (heading arrow)
            if self.quiver:
                self.quiver.remove()

            angle_rad = np.radians(self.planner.current_node.yaw)
            dx = np.cos(angle_rad) * 1.5  # Make arrow more visible (1.5 units long)
            dy = np.sin(angle_rad) * 1.5
            self.quiver = self.ax.quiver(
                self.planner.current_node.x, self.planner.current_node.y,
                dx, dy, angles='xy', scale_units='xy', scale=1,
                color='blue', width=0.008, zorder=11,
                headwidth=4, headlength=5, headaxislength=4
            )
        else:
            self.current_node_scatter.set_offsets(np.zeros((0, 2)))
            # Remove quiver when no current node
            if self.quiver:
                self.quiver.remove()
                self.quiver = None

        # Update evaluating neighbor (YELLOW diamond)
        if self.planner.evaluating_neighbors and self.planner.current_neighbor_index < len(self.planner.neighbors):
            neighbor = self.planner.neighbors[self.planner.current_neighbor_index]
            self.evaluating_neighbor_scatter.set_offsets([[neighbor.x, neighbor.y]])
        else:
            self.evaluating_neighbor_scatter.set_offsets(np.zeros((0, 2)))

        # Update start and goal
        if self.planner.start:
            self.start_scatter.set_offsets([[self.planner.start.x, self.planner.start.y]])
        if self.planner.goal:
            self.goal_scatter.set_offsets([[self.planner.goal.x, self.planner.goal.y]])

        # Update path
        if self.planner.final_path:
            path_x = [p[0] for p in self.planner.final_path]
            path_y = [p[1] for p in self.planner.final_path]
            self.path_line.set_data(path_x, path_y)
        else:
            self.path_line.set_data([], [])

        # Update info text
        if self.planner.history:
            info = self.planner.history[-1]['info']
            self.info_text.set_text(info)

        # Update statistics
        current_node_info = ""
        if self.planner.current_node:
            current_node_info = f"""
Current Node:
  Position: ({self.planner.current_node.x:.1f}, {self.planner.current_node.y:.1f})
  Heading: {self.planner.current_node.yaw}°
  g_cost: {self.planner.current_node.g_cost:.2f}
  h_cost: {self.planner.current_node.h_cost:.2f}
  f_cost: {self.planner.current_node.f_cost:.2f}"""

        neighbor_info = ""
        if self.planner.evaluating_neighbors and self.planner.current_neighbor_index < len(self.planner.neighbors):
            neighbor = self.planner.neighbors[self.planner.current_neighbor_index]
            neighbor_info = f"""
Evaluating Neighbor: {self.planner.current_neighbor_index + 1}/{len(self.planner.neighbors)}
  Position: ({neighbor.x:.1f}, {neighbor.y:.1f})
  Heading: {neighbor.yaw}°
  g_cost: {neighbor.g_cost:.2f}
  h_cost: {neighbor.h_cost:.2f}
  f_cost: {neighbor.f_cost:.2f}"""

        stats = f"""Statistics:
Iteration: {self.planner.iteration}
Nodes Expanded: {self.planner.nodes_expanded}
Open Set Size: {len(self.planner.open_dict)}
Closed Set Size: {len(self.planner.closed_set)}
Path Found: {self.planner.path_found}{current_node_info}{neighbor_info}"""
        self.stats_text.set_text(stats)

        self.fig.canvas.draw()


def main():
    """Main function"""
    print("="*60)
    print("Interactive A* Demo for Hybrid A* Phase 1")
    print("="*60)
    print("\nControls:")
    print("  - Step Forward: Execute one A* iteration (or evaluate one neighbor)")
    print("  - Step Backward: Go back one step")
    print("  - Reset: Restart the demo")
    print("  - Auto Play: Run automatically")
    print("\nVisual Elements:")
    print("  - Green Square: Start position")
    print("  - Red Star: Goal position")
    print("  - Green Circles: Open Set (Frontier - nodes to explore)")
    print("  - Red Circles: Closed Set (Already visited nodes)")
    print("  - Blue Circle: Current node being expanded")
    print("  - Blue Arrow: Heading direction of current node")
    print("  - Yellow Diamond: Neighbor currently being evaluated")
    print("  - Gray Circles: Obstacles (NOT brown - these are obstacles!)")
    print("  - Dashed Line: Final path")
    print("\nKey Concepts:")
    print("  - 3D Grid: Each node has (x, y, yaw) - position AND heading")
    print("  - Neighbors are generated ONE BY ONE for clear visualization")
    print("  - Open Set: Frontier nodes (priority queue sorted by f_cost)")
    print("  - Closed Set: Already explored nodes")
    print("  - f_cost = g_cost + h_cost")
    print("    - g_cost: Distance from start")
    print("    - h_cost: Estimated distance to goal (heuristic)")
    print("\nAlgorithm Flow:")
    print("  1. Pop node with lowest f_cost from open set")
    print("  2. Move it to closed set")
    print("  3. Generate neighbors (8 directions)")
    print("  4. Evaluate each neighbor ONE BY ONE:")
    print("     - If in closed set: Skip (already visited)")
    print("     - If in open set: Update if better path found")
    print("     - If new: Add to open set")
    print("  5. Repeat until goal reached")
    print("="*60)

    # Create visualizer
    AStarVisualizer(grid_size=20)

    # Show plot
    plt.show()


if __name__ == '__main__':
    main()
