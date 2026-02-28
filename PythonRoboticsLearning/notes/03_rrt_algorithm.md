# RRT (Rapidly-exploring Random Tree) 算法学习笔记

> **学习日期**: 2026-01-20
> **算法类别**: 采样-based 路径规划
> **原始代码**: [PythonRobotics/PathPlanning/RRT/rrt.py](../PythonRobotics/PathPlanning/RRT/rrt.py)
> **难度**: ⭐⭐⭐

---

## 📖 算法原理

### 什么是 RRT？

**RRT (Rapidly-exploring Random Tree)** 是一种基于采样的路径规划算法，由 LaValle 在 1998 年提出。

**核心思想**：
- 从起点开始，通过**随机采样**构建一棵树
- 树在空间中快速扩展，最终到达目标点
- 适合**高维空间**和**复杂障碍物**环境

### 算法流程

```
1. 初始化: 树只包含起点节点
2. 循环（直到找到路径或达到最大迭代次数）:
   a. 随机采样一个点 (rand)
   b. 找到树中距离 rand 最近的节点 (nearest)
   c. 从 nearest 向 rand 方向延伸固定距离，得到新节点 (new)
   d. 检查 new 是否与障碍物碰撞
      - 无碰撞: 将 new 加入树
      - 有碰撞: 舍弃
   e. 如果 new 距离目标足够近，连接目标并返回路径
3. 回溯路径: 从目标点沿着父节点指针回到起点
```

### 关键参数

| 参数 | 代码变量 | 作用 | 典型值 |
|------|---------|------|--------|
| 扩展距离 | `expand_dis` | 每次迭代树生长的距离 | 3.0 |
| 路径分辨率 | `path_resolution` | 路径离散化步长 | 0.5 |
| 目标采样率 | `goal_sample_rate` | 以目标点作为随机点的概率(%) | 5% |
| 最大迭代 | `max_iter` | 最大采样次数 | 500 |

---

## 💻 代码结构分析

### 核心类: `RRT`

文件位置: [rrt.py:18-257](../PythonRobotics/PathPlanning/RRT/rrt.py#L18-L257)

### 1. 数据结构

#### Node 类 ([rrt.py:23-33](../PythonRobotics/PathPlanning/RRT/rrt.py#L23-L33))

```python
class Node:
    def __init__(self, x, y):
        self.x = x              # 节点 x 坐标
        self.y = y              # 节点 y 坐标
        self.path_x = []        # 从父节点到此点的路径 x 坐标序列
        self.path_y = []        # 从父节点到此点的路径 y 坐标序列
        self.parent = None      # 父节点指针（用于回溯路径）
```

**设计亮点**：
- `path_x/y` 存储完整路径段，用于碰撞检测
- `parent` 指针形成树结构，便于路径回溯

#### AreaBounds 类 ([rrt.py:35-41](../PythonRobotics/PathPlanning/RRT/rrt.py#L35-L41))

```python
class AreaBounds:
    def __init__(self, area):
        self.xmin = float(area[0])
        self.xmax = float(area[1])
        self.ymin = float(area[2])
        self.ymax = float(area[3])
```

用于限制探索区域边界。

### 2. 初始化函数

`__init__` ([rrt.py:44-81](../PythonRobotics/PathPlanning/RRT/rrt.py#L44-L81))

**关键参数**:
- `start/goal`: 起点和目标坐标
- `obstacle_list`: 障碍物列表 `[(x, y, radius), ...]`
- `rand_area`: 随机采样区域 `[min, max]`
- `play_area`: 可选的活动区域限制
- `robot_radius`: 机器人半径（用于碰撞检测）

### 3. 核心规划函数

#### `planning()` ([rrt.py:83-117](../PythonRobotics/PathPlanning/RRT/rrt.py#L83-L117))

主循环逻辑：

```python
def planning(self, animation=True):
    self.node_list = [self.start]  # 初始化树

    for i in range(self.max_iter):
        # 1. 随机采样
        rnd_node = self.get_random_node()

        # 2. 找最近节点
        nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
        nearest_node = self.node_list[nearest_ind]

        # 3. 扩展新节点
        new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

        # 4. 碰撞检测
        if self.check_if_outside_play_area(new_node, self.play_area) and \
           self.check_collision(new_node, self.obstacle_list, self.robot_radius):
            self.node_list.append(new_node)

        # 5. 检查是否到达目标
        if self.calc_dist_to_goal(self.node_list[-1].x,
                                  self.node_list[-1].y) <= self.expand_dis:
            final_node = self.steer(self.node_list[-1], self.end,
                                    self.expand_dis)
            if self.check_collision(final_node, ...):
                return self.generate_final_course(len(self.node_list) - 1)

    return None  # 未找到路径
```

**关键点**：
- 每次迭代只添加一个节点
- 目标检测：当最新节点距离目标 ≤ `expand_dis` 时尝试连接

#### `steer()` ([rrt.py:119-147](../PythonRobotics/PathPlanning/RRT/rrt.py#L119-L147))

**树扩展的核心逻辑**：

```python
def steer(self, from_node, to_node, extend_length=float("inf")):
    new_node = self.Node(from_node.x, from_node.y)
    d, theta = self.calc_distance_and_angle(new_node, to_node)

    # 限制扩展长度，扩展点永远在两点线段之间
    # 使其均匀探索，控制密度
    if extend_length > d:
        extend_length = d

    # 沿方向逐步扩展
    n_expand = math.floor(extend_length / self.path_resolution)
    for _ in range(n_expand):
        new_node.x += self.path_resolution * math.cos(theta)
        new_node.y += self.path_resolution * math.sin(theta)
        new_node.path_x.append(new_node.x)
        new_node.path_y.append(new_node.y)

    # 确保到达目标点
    d, _ = self.calc_distance_and_angle(new_node, to_node)
    if d <= self.path_resolution:
        new_node.path_x.append(to_node.x)
        new_node.path_y.append(to_node.y)
        new_node.x = to_node.x
        new_node.y = to_node.y

    new_node.parent = from_node
    return new_node
```

**设计亮点**：
- 使用 `path_resolution` 离散化路径，便于碰撞检测
- 记录完整路径段，而不是只记录端点

#### `get_random_node()` ([rrt.py:164-171](../PythonRobotics/PathPlanning/RRT/rrt.py#L164-L171))

**随机采样策略**：

```python
def get_random_node(self):
    if random.randint(0, 100) > self.goal_sample_rate:
        # 随机采样
        rnd = self.Node(
            random.uniform(self.min_rand, self.max_rand),
            random.uniform(self.min_rand, self.max_rand))
    else:
        # 以目标点作为采样点（偏向贪婪）
        rnd = self.Node(self.end.x, self.end.y)
    return rnd
```

**技巧**：
- 以 `goal_sample_rate` 概率直接采样目标点
- 加速收敛，避免盲目探索

#### `check_collision()` ([rrt.py:235-248](../PythonRobotics/PathPlanning/RRT/rrt.py#L235-L248))

**碰撞检测逻辑**：

```python
def check_collision(node, obstacleList, robot_radius):
    for (ox, oy, size) in obstacleList:
        # 检查路径上每个点到障碍物的距离
        dx_list = [ox - x for x in node.path_x]
        dy_list = [oy - y for y in node.path_y]
        d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

        # 如果任意点距离障碍物太近，则碰撞
        if min(d_list) <= (size + robot_radius)**2:
            return False  # 碰撞

    return True  # 安全
```

**安全边界**：`size + robot_radius` 确保机器人不会碰到障碍物

#### `generate_final_course()` ([rrt.py:149-157](../PythonRobotics/PathPlanning/RRT/rrt.py#L149-L157))

**路径回溯**：

```python
def generate_final_course(self, goal_ind):
    path = [[self.end.x, self.end.y]]
    node = self.node_list[goal_ind]

    # 沿着父节点指针回溯
    while node.parent is not None:
        path.append([node.x, node.y])
        node = node.parent

    path.append([node.x, node.y])  # 添加起点
    return path
```

---

## 🎯 算法特点

### 优点

✅ **适用于高维空间**: 不需要网格，计算复杂度与维度关系较小
✅ **概率完备**: 只要有路径，随着迭代增加必然能找到
✅ **实现简单**: 代码逻辑清晰，易于理解和修改
✅ **处理复杂障碍物**: 不需要对障碍物进行特殊处理

### 缺点

❌ **路径不最优**: 生成的路径通常不是最短的
❌ **非确定性**: 每次运行结果不同
❌ **收敛速度慢**: 在狭窄通道中难以通过
❌ **路径抖动**: 生成的路径通常不够平滑

---

## 📊 可视化分析

### RRT 树的生长过程

运行演示时的视觉元素：
- **绿色线 (-g)**: 树的边（[rrt.py:185](../PythonRobotics/PathPlanning/RRT/rrt.py#L185)）
- **黑色三角 (^k)**: 随机采样点（[rrt.py:180](../PythonRobotics/PathPlanning/RRT/rrt.py#L180)）
- **蓝色圆圈**: 障碍物（[rrt.py:207](../PythonRobotics/PathPlanning/RRT/rrt.py#L207)）
- **红色叉号 (xr)**: 起点和目标（[rrt.py:199-200](../PythonRobotics/PathPlanning/RRT/rrt.py#L199-L200)）
- **红色线 (-r)**: 最终路径（[rrt.py:284](../PythonRobotics/PathPlanning/RRT/rrt.py#L284)）

---

## 🔬 实验记录

### 实验 1: 基本运行

**命令**:
```bash
cd PythonRobotics
python3 PathPlanning/RRT/rrt.py
```

**参数**:
- 起点: (0, 0)
- 目标: (6, 10)
- 障碍物: 7 个圆形障碍物
- 机器人半径: 0.8

**观察**:
- ✅ 能够找到路径
- 树在空间中快速探索
- 路径较为曲折（非最优）

### 实验 2: 参数影响分析

| 参数 | 值 | 效果 |
|------|-----|------|
| `expand_dis` | 1.0 | 树生长慢，路径更细致 |
| `expand_dis` | 5.0 | 树生长快，路径更粗糙 |
| `goal_sample_rate` | 0% | 纯随机探索，收敛慢 |
| `goal_sample_rate` | 20% | 偏向目标，收敛快 |

---

## 🔄 RRT 变体

PythonRobotics 中提供了多个 RRT 变体：

1. **RRT\*** ([PathPlanning/RRTStar/](../PythonRobotics/PathPlanning/RRTStar/))
   - 渐近最优版本
   - 通过重连父节点优化路径

2. **RRT with Path Smoothing** ([PathPlanning/RRT/rrt_with_pathsmoothing.py](../PythonRobotics/PathPlanning/RRT/rrt_with_pathsmoothing.py))
   - 使用样条曲线平滑路径

3. **RRT Dubins** ([PathPlanning/RRTDubins/](../PythonRobotics/PathPlanning/RRTDubins/))
   - 考虑车辆运动学约束（最小转弯半径）

4. **RRT* Dubins** ([PathPlanning/RRTStarDubins/](../PythonRobotics/PathPlanning/RRTStarDubins/))
   - 结合最优性和运动学约束

---

## 💡 理解要点

### 1. 为什么是"快速探索" (Rapidly-exploring)？

- 每次迭代从现有树中**最近节点**扩展
- 树会向**未探索区域**快速生长
- 相比纯随机采样，更快覆盖空间

### 2. 为什么路径不最优？

- 只考虑可达性，不考虑路径质量
- 扩展顺序受随机性影响
- 没有路径优化机制

### 3. 如何改进？

- **RRT\***: 重连接机制优化路径
- **目标偏向**: 提高目标采样率
- **路径平滑**: 后处理优化路径

---

## 📚 参考资源

### 论文
- LaValle, S. M. (1998). "Rapidly-exploring random trees: A new tool for path planning"

### 书籍
- "Planning Algorithms" by LaValle (Chapter 5)
- "Probabilistic Robotics" by Thrun (Chapter 5)

### 在线资源
- [RRT 算法可视化](https://motionplanning.ri.cmu.edu/)
- [OMPL (Open Motion Planning Library)](http://ompl.kavrakilab.org/)

---

## 🎓 学习心得

### 核心理解

RRT 的本质是：
1. **随机采样** 探索状态空间
2. **最近邻** 连接保证树的连续性
3. **碰撞检测** 确保路径安全性
4. **贪婪目标偏向** 加速收敛

### 与 A* 的对比

| 特性 | A* | RRT |
|------|----|-----|
| 搜索空间 | 离散网格 | 连续空间 |
| 完备性 | 完备 | 概率完备 |
| 路径质量 | 最优 | 非最优 |
| 维度扩展性 | 差 | 好 |
| 障碍物处理 | 需要网格化 | 直接处理 |

### 适用场景

✅ RRT 适合：
- 高维空间（机械臂、无人机）
- 复杂动态障碍物
- 非完整约束系统

❌ A* 适合：
- 低维网格地图
- 需要最优路径
- 实时性要求高

---

## 🚀 下一步学习

- [ ] RRT* 算法（渐近最优）
- [ ] Informed RRT*（基于椭圆采样）
- [ ] 路径平滑技术
- [ ] 其他采样算法（PRM, EST）

---

**学习完成度**: ⭐⭐⭐⭐ (4/5)
**掌握程度**: 理解原理和实现，能够修改参数和场景
**日期**: 2026-01-20
