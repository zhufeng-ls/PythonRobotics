# A* (A-Star) 路径规划算法学习笔记

> **学习日期**: 2026-01-08
> **算法类别**: 网格搜索算法
> **难度等级**: ⭐⭐
> **文件路径**: `PathPlanning/AStar/a_star.py`

---

## 📖 算法概述

### 核心思想
A* 算法是一种启发式搜索算法，结合了 Dijkstra 算法（保证最短路径）和贪心最佳优先搜索（使用启发式加速）的优点。

### 数学原理
A* 使用以下评价函数选择下一个探索节点：

```
f(n) = g(n) + h(n)
```

其中：
- **g(n)**: 从起点到节点 n 的实际代价（已经走过的距离）
- **h(n)**: 从节点 n 到终点的启发式估计代价（预估距离）
- **f(n)**: 节点 n 的总估计代价

### 启发式函数
本代码使用 **欧几里得距离** 作为启发式：
```python
h(n) = w * sqrt((n1.x - n2.x)² + (n1.y - n2.y)²)
```

- `w = 1.0`: 启发式权重
- 当 `w = 1` 时，A* 保证找到最短路径
- 当 `w > 1` 时，搜索更快但不保证最优
- 当 `w < 1` 时，搜索更慢但更精确

---

## 🏗️ 代码结构分析

### 类设计

#### 1. `AStarPlanner` 主规划器类

**初始化参数**:
```python
def __init__(self, ox, oy, resolution, rr):
```

- `ox, oy`: 障碍物的 x, y 坐标列表
- `resolution`: 网格分辨率（单位：米）
- `rr`: 机器人半径（用于碰撞检测）

**核心方法**:
- `planning(sx, sy, gx, gy)`: 主规划函数
- `calc_obstacle_map(ox, oy)`: 构建障碍物栅格地图
- `verify_node(node)`: 检查节点是否有效（边界 + 碰撞检测）
- `calc_final_path(goal_node, closed_set)`: 回溯生成最终路径

#### 2. `Node` 内部类（节点表示）

```python
class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x              # 网格 x 索引
        self.y = y              # 网格 y 索引
        self.cost = cost        # 从起点到该节点的代价 g(n)
        self.parent_index = parent_index  # 父节点索引
```

**关键点**:
- 使用网格索引而非实际坐标
- `parent_index` 用于路径回溯
- `cost` 累积从起点到当前节点的实际代价

---

## 🔄 算法流程详解

### 主循环逻辑（第 73-127 行）

```
1. 初始化
   ├── 创建起点节点 start_node
   ├── 创建终点节点 goal_node
   ├── open_set = {start_node}  # 待探索节点
   └── closed_set = {}           # 已探索节点

2. 循环搜索（while True）
   ├── 从 open_set 选择 f(n) 最小的节点
   ├── 如果当前节点是目标 → 成功！
   ├── 将当前节点从 open_set 移到 closed_set
   ├── 扩展邻居节点（8个方向）
   │   ├── 检查有效性（边界、碰撞）
   │   ├── 如果在 closed_set → 跳过
   │   ├── 如果不在 open_set → 加入
   │   └── 如果已在 open_set → 更新更优路径
   └── 重复直到找到目标或 open_set 为空

3. 路径回溯
   └── 从 goal_node 通过 parent_index 链回溯到起点
```

### 节点选择策略（第 78-82 行）

```python
c_id = min(open_set,
           key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
```

这是 A* 的核心：**选择 `cost + h` 最小的节点**，其中：
- `cost` = g(n) = 已走距离
- `calc_heuristic()` = h(n) = 预估距离

---

## 🎨 运动模型（第 218-229 行）

### 8 方向移动

```python
motion = [
    [dx, dy, cost],     # 直线移动代价 = 1
    [1, 0, 1],          # 右
    [0, 1, 1],          # 上
    [-1, 0, 1],         # 左
    [0, -1, 1],         # 下
    [-1, -1, √2],       # 左下（对角线）
    [-1, 1, √2],        # 左上
    [1, -1, √2],        # 右下
    [1, 1, √2],         # 右上
]
```

**特点**：
- 4 个直线方向：代价 = 1
- 4 个对角线方向：代价 = √2 ≈ 1.414
- 允许对角线移动使路径更平滑

---

## 🗺️ 障碍物地图构建（第 188-216 行）

### 栅格化流程

```python
1. 计算地图边界
   ├── min_x = min(ox), min_y = min(oy)
   ├── max_x = max(ox), max_y = max(oy)
   └── 扩展到机器人半径

2. 创建二维网格
   ├── x_width = (max_x - min_x) / resolution
   └── y_width = (max_y - min_y) / resolution

3. 标记障碍物
   for 每个网格 (ix, iy):
       for 每个障碍物点 (iox, ioy):
           if distance(网格中心, 障碍物) <= robot_radius:
               obstacle_map[ix][iy] = True
```

**关键设计**：
- 使用 `robot_radius` 扩展障碍物，确保机器人不会碰撞
- 每个网格检查与所有障碍物的距离

---

## 🧪 实验场景（main 函数）

### 默认参数

```python
sx, sy = 10.0, 10.0    # 起点 (10, 10)
gx, gy = 50.0, 50.0    # 终点 (50, 50)
grid_size = 2.0        # 网格大小 2 米
robot_radius = 1.0     # 机器人半径 1 米
```

### 障碍物布局

```
60 ┼──────────────┐
   │              │
40 │              │
   │              │
20 │    █         █
   │    █         █
 0 │    █         █
   └──────────────┴
   0   20  40    60
```

- 外边界墙（-10 到 60）
- 两道内部障碍墙：
  - 垂直墙：x=20, y=-10~40
  - 水平墙：y=60, x=0~40（从顶部向下）

---

## 📊 可视化与动画

### 动画控制
- `show_animation = True/False`: 全局开关
- 每 10 个节点暂停 0.5 秒（第 93-94 行）
- 按 `ESC` 键退出（第 90-92 行）

### 绘图元素
- `.k`: 黑色点 = 障碍物
- `og`: 绿色圆圈 = 起点
- `xb`: 蓝色叉 = 终点
- `xc`: 青色叉 = 正在探索的节点
- `-r`: 红色线 = 最终路径

---

## 🔑 关键函数解析

### 1. `calc_heuristic(n1, n2)` - 启发式计算

```python
@staticmethod
def calc_heuristic(n1, n2):
    w = 1.0  # 可调整的启发式权重
    d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
    return d
```

**数学基础**：欧几里得距离
- `hypot(dx, dy) = sqrt(dx² + dy²)`
- 满足启发式可采纳性（不过高估计）
- 保证找到最短路径

### 启发式函数的关键性质

#### 1. 可采纳性 (Admissibility)

**定义**: 启发式函数 h(n) 永远不会高估从节点 n 到目标的实际代价

```
h(n) ≤ h*(n)
```

其中 h*(n) 是从 n 到目标的真实最优代价。

**重要性**:
- ✅ 满足可采纳性 → A* 保证找到最优路径
- ❌ 高估 (h(n) > h*(n)) → 可能找到次优路径

#### 2. 代价高估的影响

**关键理解: 高估指的是对 h(n) 的高估,不影响 g(n)**

```
f(n) = g(n) + h(n)
       ↑         ↑
       |         └─ 启发式估计 (可能高估)
       └─ 实际累积代价 (始终准确)
```

**三个组件的关系**:

1. **g(n) - 实际代价 (不会被影响)**
   - 从起点到当前节点 n 的**真实代价**
   - 是已经走过的路径,是确定的历史
   - 无论 h(n) 如何计算,g(n) 的值永远准确

2. **h(n) - 启发式估计 (可能被高估)**
   - 从节点 n 到目标的**预估代价**
   - 当 `h(n) > h*(n)` (真实最优代价) 时,就是高估
   - 例如: 真实剩余距离是 10,但 h(n) 算出来是 15

3. **高估的影响机制**
   - 高估的 h(n) 让 f(n) **虚高**
   - 算法误以为这条路径"很贵"
   - 可能跳过实际最优的路径
   - 导致最终找到的解不是最优

**为什么高估会破坏最优性?**

当启发式高估时:
- 算法可能过早地"放弃"某些路径
- 这些路径实际上可能通向更优的解
- 因为 f(n) = g(n) + h(n) 被高估了
- 算法认为这些路径不值得探索

**实际意义**:
```
保守估计 (h ≤ h*): 算法谨慎探索,保证最优
高估估计 (h > h*):   算法激进剪枝,可能错过最优解
```

#### 3. 代价低估的影响

**关键理解: 算法终止的条件是从 `openset` 中取出 `goal`, 而不是邻居节点中到达 `goal`**

这样会导致虽然 h 被低估偏小，但是 goal 的 g 过大，算法不会结束，会继续从 `openset` 中寻找 g 偏小，h 正常的节点，最后会多次更新终点的 g, 直至从 `openset` 取出 g

#### 4. 常见启发式函数的可采纳性

| 启发式 | 移动方式 | 可采纳性 | 适用场景 |
|:------|:---------|:---------|:---------|
| **曼哈顿距离** | 4方向 | ✅ | 只能上下左右移动 |
| **欧几里得距离** | 任意方向 | ✅ | 任意角度移动 |
| **切比雪夫距离** | 8方向 | ✅ | 允许对角线移动 |
| **对角线距离** | 8方向 | ✅ | 对角线代价 = √2 |

**注意事项**:
- 启发式必须匹配实际移动方式
- 允许对角线移动时,曼哈顿距离会高估 → 不可采纳
- 只能直线移动时,欧几里得距离会高估 → 不可采纳

### 2. `verify_node(node)` - 节点验证

```python
def verify_node(self, node):
    # 1. 边界检查
    if px < self.min_x or px >= self.max_x: return False
    if py < self.min_y or py >= self.max_y: return False

    # 2. 碰撞检查
    if self.obstacle_map[node.x][node.y]:
        return False

    return True
```

### 3. `calc_final_path()` - 路径回溯

```python
def calc_final_path(self, goal_node, closed_set):
    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index

    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index  # 回溯

    return rx, ry
```

**时间复杂度**：O(路径长度)

---

## 🎯 算法复杂度分析

### 时间复杂度
- **最坏情况**: O(b^d)
  - b = 分支因子（本代码为 8）
  - d = 最短路径深度
- **使用二叉堆优化**: O(n log n)

### 空间复杂度
- O(b^d) - 需要存储所有探索的节点
- `open_set` + `closed_set` 的总大小

### 与 Dijkstra 对比
| 算法 | 时间复杂度 | 空间复杂度 | 启发式 | 最优性 |
|------|----------|----------|--------|--------|
| Dijkstra | O(b^d) | O(b^d) | 无 | ✅ |
| A* | O(b^d) | O(b^d) | 有 | ✅ |
| 贪心最佳优先 | O(b^m) | O(b^m) | 有 | ❌ |

**实际性能**: A* 通常比 Dijkstra 快很多，因为启发式引导搜索方向。

---

## 🧩 实验记录

### 实验 1: 默认参数运行

**运行命令**:
```bash
cd PathPlanning/AStar
python3 a_star.py
```

**观察结果**:
- [x] 算法成功找到路径
- [x] 路径绕过了两道障碍墙
- [x] 最终路径是最短路径（使用对角线）

**参数设置**:
- 起点: (10, 10)
- 终点: (50, 50)
- 网格大小: 2.0m
- 机器人半径: 1.0m

### 实验 2: 调整启发式权重

**修改代码**:
```python
# 在 calc_heuristic 函数中修改
w = 2.0  # 增加权重，加速搜索
```

**预期行为**:
- ⚡ 搜索速度更快
- ⚠️ 可能不保证最短路径
- 📊 探索的节点数量减少

### 实验 3: 改变网格分辨率

**测试不同分辨率**:
| 分辨率 | 节点数 | 路径质量 | 计算时间 |
|--------|--------|---------|---------|
| 0.5m   | 多     | 高      | 慢      |
| 2.0m   | 中     | 中      | 中      |
| 5.0m   | 少     | 低      | 快      |

---

## 💡 学习心得

### 1. A* 的核心优势
- ✅ **结合最优性和效率**: 既保证最短路径，又比 Dijkstra 快
- ✅ **启发式引导**: 智能地朝目标方向搜索
- ✅ **灵活性**: 可通过调整权重 `w` 平衡速度和精度

### 2. 实现技巧
- 📌 **使用字典存储节点**: `open_set[node_id] = node`
- 📌 **节点索引设计**: `index = y * width + x`（一维化二维）
- 📌 **路径回溯**: 通过 `parent_index` 链
- 📌 **障碍物扩展**: 考虑机器人半径

### 3. 适用场景
- ✅ 2D 网格地图
- ✅ 游戏寻路
- ✅ 机器人路径规划
- ✅ 导航系统

### 4. 局限性
- ❌ 需要精确的启发式函数
- ❌ 内存消耗较大（存储所有节点）
- ❌ 动态环境需要重新规划

---

## 📚 扩展知识

### 1. 其他启发式函数
- **曼哈顿距离**: `h(n) = |x1-x2| + |y1-y2|`（仅 4 方向移动）
- **切比雪夫距离**: `h(n) = max(|x1-x2|, |y1-y2|)`（8 方向移动）
- **平方距离**: `h(n) = (x1-x2)² + (y1-y2)²`（更快但不保证最优）

### 2. A* 变体
- ** weighted A*`: 增加 w > 1 加速
- **Dynamic A* (D*)**: 动态环境重规划
- **Anytime A*`: 快速找到初始路径，逐步优化
- **Theta***: 允许任意角度移动（不仅限网格）

### 3. 优化方向
- **使用优先队列**: 将 open_set 改为 `heapq`
- **双向 A***: 从起点和终点同时搜索
- **跳点搜索 (JPS)**: 减少对称路径的探索
- **层次 A***: 先粗规划再细规划

---

## 🔗 相关资源

### 参考资料
- 📖 [Wikipedia: A* search algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)
- 📖 [Amit's A* Pages](https://www.redblobgames.com/pathfinding/a-star/introduction.html)
- 📹 [Red Blob Games - Pathfinding](https://www.redblobgames.com/pathfinding/a-star/)

### 代码位置
- 主文件: `PathPlanning/AStar/a_star.py:18`
- 核心算法: `a_star.py:50-131` (planning 方法)

### 相关算法
- **Dijkstra**: `PathPlanning/Dijkstra/dijkstra.py`
- **Greedy Best First**: `PathPlanning/GreedyBestFirstSearch/greedy_best_first_search.py`
- **Bidirectional A***: `PathPlanning/BidirectionalAStar/`

---

## ✅ 检查清单

### 理解程度自评
- [x] 理解 A* 的核心思想 (f = g + h)
- [x] 能够解释启发式函数的作用
- [x] 掌握 open_set 和 closed_set 的维护
- [x] 理解路径回溯机制
- [x] 能够修改参数并预测结果

### 实践能力
- [ ] 能独立实现简化版 A*
- [ ] 能修改代码适配新场景
- [ ] 能对比不同启发式函数的效果
- [ ] 能解释时间/空间复杂度

---

## 📝 下一步学习

1. **对比实验**: 运行 Dijkstra，对比搜索效率
2. **代码改进**: 使用优先队列优化 open_set
3. **实际应用**: 在自己设计的地图上测试
4. **进阶算法**: 学习 RRT（处理高维空间）

---

## 🗂️ 元数据

```
学习日期: 2026-01-08
最后更新: 2026-01-08
学习状态: ✅ 已完成初步理解
下次复习: 2026-01-15
相关标签: #路径规划 #网格搜索 #启发式搜索 #Astar
```
