# RRT* (RRT Star) 算法学习笔记

> **学习日期**: 2026-01-21
> **算法类别**: 采样-based 路径规划（最优版本）
> **原始代码**: [PythonRobotics/PathPlanning/RRTStar/rrt_star.py](../../PythonRobotics/PathPlanning/RRTStar/rrt_star.py)
> **难度**: ⭐⭐⭐⭐

---

## 📖 算法原理

### 什么是 RRT*？

**RRT\*** (RRT Star) 是 RRT 的**渐近最优**版本，由 S. Karaman 和 E. Frazzoli 在 2011 年提出。

**核心改进**：
- ✅ **渐近最优**: 随着迭代增加，路径趋向最优
- ✅ **路径优化**: 通过重连接机制不断改进已找到的路径
- ✅ **成本感知**: 每个节点记录从起点到该点的成本

### RRT vs RRT* 的核心区别

| 特性 | RRT | RRT* |
|------|-----|------|
| **路径质量** | 非最优 | 渐近最优 |
| **节点结构** | 位置 + 父节点 | 位置 + 父节点 + **成本** |
| **新节点处理** | 连接最近节点 | 选择最优父节点 + **重连接** |
| **计算复杂度** | O(n) | O(n log n) |
| **收敛性** | 概率完备 | 渐近最优 |

### RRT* 的核心创新

```
RRT 的基本流程:
  1. 随机采样
  2. 找最近节点
  3. 扩展新节点
  4. 碰撞检测
  5. 加入树

RRT* 的改进流程:
  1. 随机采样
  2. 找最近节点
  3. 扩展新节点
  4. 碰撞检测
  5. 【选择最优父节点】 ← 新增
  6. 【重连接邻近节点】 ← 新增
  7. 加入树
```

---

## 💻 代码结构分析

### 核心类: `RRTStar`

文件位置: [rrt_star.py:20-247](../../PythonRobotics/PathPlanning/RRTStar/rrt_star.py#L20-L247)

**继承关系**:
```python
class RRTStar(RRT):
    """
    RRT* 继承自 RRT，复用基本结构
    """
```

### 1. 节点结构增强

#### Node 类 ([rrt_star.py:25-28](../../PythonRobotics/PathPlanning/RRTStar/rrt_star.py#L25-L28))

```python
class Node(RRT.Node):
    def __init__(self, x, y):
        super().__init__(x, y)
        self.cost = 0.0  # ← 新增：从起点到此节点的成本
```

**成本定义**：
```python
node.cost = 从起点到此节点的路径总长度
```

**示例**：
```
起点 S: cost = 0.0
节点 A: cost = 3.0 (S → A, 距离 3.0)
节点 B: cost = 5.0 (S → A → B, A.cost + 2.0)
节点 C: cost = 4.0 (S → C, 距离 4.0, 比 S → A → C 更优)
```

### 2. 初始化参数

#### `__init__` ([rrt_star.py:30-57](../../PythonRobotics/PathPlanning/RRTStar/rrt_star.py#L30-L57))

**新增关键参数**:

```python
def __init__(self,
             # ... 继承 RRT 的参数 ...
             connect_circle_dist=50.0,     # ← 新增：重连接搜索半径
             search_until_max_iter=False,  # ← 新增：是否运行到最大迭代
             ):
    self.connect_circle_dist = connect_circle_dist
    self.search_until_max_iter = search_until_max_iter
```

**参数说明**:

| 参数 | 代码变量 | 作用 | 典型值 |
|------|---------|------|--------|
| connect_circle_dist | 搜索半径 | 重连接时考虑的邻近节点范围 | 50.0 |
| search_until_max_iter | 是否继续优化 | 找到路径后是否继续优化 | False |

**动态搜索半径** ([rrt_star.py:193](../../PythonRobotics/PathPlanning/RRTStar/rrt_star.py#L193)):
```python
r = self.connect_circle_dist * math.sqrt(math.log(nnode) / nnode)
# ↑
# 随着节点数增加，搜索半径逐渐减小
# 理论保证：O(n log n) 复杂度
```

### 3. 主规划循环

#### `planning()` ([rrt_star.py:59-104](../../PythonRobotics/PathPlanning/RRTStar/rrt_star.py#L59-L104))

**RRT* 的核心流程**：

```python
def planning(self, animation=True):
    self.node_list = [self.start]

    for i in range(self.max_iter):
        # 1. 随机采样
        rnd = self.get_random_node()

        # 2. 找最近节点
        nearest_ind = self.get_nearest_node_index(self.node_list, rnd)

        # 3. 扩展新节点
        new_node = self.steer(self.node_list[nearest_ind], rnd,
                              self.expand_dis)

        # 4. 计算成本（从起点到新节点）
        near_node = self.node_list[nearest_ind]
        new_node.cost = near_node.cost + \
            math.hypot(new_node.x - near_node.x,
                       new_node.y - near_node.y)

        # 5. 碰撞检测
        if self.check_collision(new_node, ...):
            # 6. 【核心改进 1】选择最优父节点
            near_inds = self.find_near_nodes(new_node)
            node_with_updated_parent = self.choose_parent(
                new_node, near_inds)

            if node_with_updated_parent:
                # 7. 【核心改进 2】重连接邻近节点
                self.rewire(node_with_updated_parent, near_inds)
                self.node_list.append(node_with_updated_parent)
            else:
                self.node_list.append(new_node)

        # 8. 检查是否到达目标
        if ((not self.search_until_max_iter) and new_node):
            last_index = self.search_best_goal_node()
            if last_index is not None:
                return self.generate_final_course(last_index)

    # 9. 运行完所有迭代，返回最优路径
    last_index = self.search_best_goal_node()
    if last_index is not None:
        return self.generate_final_course(last_index)

    return None
```

---

## 🎯 核心改进详解

### 改进 1: 选择最优父节点 (`choose_parent`)

**位置**: [rrt_star.py:106-145](../../PythonRobotics/PathPlanning/RRTStar/rrt_star.py#L106-L145)

**目的**: 不仅考虑最近的节点，而是在邻近节点中选择成本最低的作为父节点

```python
def choose_parent(self, new_node, near_inds):
    """
    在 near_inds 中找到成本最低的节点作为父节点
    """
    if not near_inds:
        return None

    # 计算从每个邻近节点到 new_node 的成本
    costs = []
    for i in near_inds:
        near_node = self.node_list[i]
        t_node = self.steer(near_node, new_node)

        if self.check_collision(t_node, ...):
            # 成本 = near_node.cost + 距离(near_node → new_node)
            costs.append(self.calc_new_cost(near_node, new_node))
        else:
            costs.append(float("inf"))  # 碰撞，成本无穷大

    # 选择成本最低的节点
    min_cost = min(costs)

    if min_cost == float("inf"):
        return None  # 所有可能的父节点都碰撞

    min_ind = near_inds[costs.index(min_cost)]
    new_node = self.steer(self.node_list[min_ind], new_node)
    new_node.cost = min_cost

    return new_node
```

**可视化**：

```
RRT 的选择方式:
        S ────→ A (cost = 5.0)
        │
        └──────→ B (cost = 8.0)
                  │
        new_node ←┘ (最近节点是 B)

        结果: new_node.parent = B
              new_node.cost = 8.0 + 2.0 = 10.0

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

RRT* 的选择方式:
        S ────→ A (cost = 5.0)
        │        │
        │        └────→ new_node (距离 3.0)
        │              成本 = 5.0 + 3.0 = 8.0 ✓
        │
        └──────→ B (cost = 8.0)
                  │
                  └──→ new_node (距离 2.0)
                        成本 = 8.0 + 2.0 = 10.0

        结果: new_node.parent = A (成本更低!)
              new_node.cost = 8.0
```

---

### 改进 2: 重连接邻近节点 (`rewire`)

**位置**: [rrt_star.py:203-236](../../PythonRobotics/PathPlanning/RRTStar/rrt_star.py#L203-L236)

**目的**: 检查是否可以通过新节点降低邻近节点的成本

```python
def rewire(self, new_node, near_inds):
    """
    对于每个邻近节点，检查从新节点到达是否更便宜
    如果是，则重新连接
    """
    for i in near_inds:
        near_node = self.node_list[i]

        # 尝试从 new_node 到 near_node
        edge_node = self.steer(new_node, near_node)
        if not edge_node:
            continue

        # 计算通过 new_node 到达 near_node 的成本
        edge_node.cost = self.calc_new_cost(new_node, near_node)

        # 检查是否改进
        no_collision = self.check_collision(edge_node, ...)
        improved_cost = near_node.cost > edge_node.cost

        if no_collision and improved_cost:
            # 更新父节点
            for node in self.node_list:
                if node.parent == self.node_list[i]:
                    node.parent = edge_node

            # 更新节点及其所有子节点的成本
            self.node_list[i] = edge_node
            self.propagate_cost_to_leaves(self.node_list[i])
```

**可视化**：

```
重连接前:
        S ────→ A (cost = 5.0)
        │
        └──────→ B (cost = 8.0)

        new_node (cost = 6.0, 从 S 直接到达)
                    │
                    └──→ 可以到达 B (距离 1.0)

        当前 B 的成本: 8.0 (S → ... → B)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

重连接后:
        S ────→ A (cost = 5.0)
        │
        └─────→ new_node (cost = 6.0)
                  │
                  └──→ B (cost = 7.0) ← 重连接!

        新 B 的成本: 7.0 (S → new_node → B)
        改进: 8.0 → 7.0 ✓

        同时更新 B 的所有子节点的成本
```

---

## 🎯 RRT* 核心流程总结（完全理解版）

### 完整的 5 步流程

基于对代码的深入理解，RRT* 的每次迭代可以总结为：

```
步骤 1: 生成随机节点 A
  └─ 在整个空间均匀随机采样

步骤 2: 寻找最近节点 B，从 B 到 A 生成扩展节点 C
  └─ C 与 A 可能是同一个位置（如果 A 很近）
  └─ 这是 RRT 的基础扩展机制

步骤 3: 为 C 在附近，寻找 cost 最小的父节点 D
  └─ 不只连接最近的节点 B
  └─ 在 C 附近的所有节点中选择成本最低的作为父节点
  └─ 这确保 C 以最优方式加入树
  └─ 代码: choose_parent()

步骤 4: 为 C 在附近，寻找附近 cost 比较大的节点 E，进行重连接
  └─ 检查：如果通过 C 到达 E，成本是否更低？
  └─ 如果是：
      a. 重新创建一个节点 F（从 C 到 E 的路径）
      b. 让 C 做 F 的父节点
      c. E 的所有子节点都"过继"给 F（更新 parent 指针）
      d. 递归传递 F 的子节点、孙子节点的 cost
  └─ 这是 RRT* 实现渐近最优的关键
  └─ 代码: rewire()

步骤 5: 重复步骤 1
  └─ 持续迭代，不断优化整个树的结构
```

### 两个方向相反的优化

**Choose Parent（为新节点找父节点）**：
- **方向**: 邻近节点 → 新节点
- **目的**: 让新节点以最优方式加入树
- **效果**: 新节点获得最低成本

**Rewire（新节点优化邻近节点）**：
- **方向**: 新节点 → 邻近节点
- **目的**: 让邻近节点找到更好的父节点
- **效果**: 现有节点的成本降低

### "过继"机制的精确含义

在重连接过程中，"E 的子节点都过继给 F"指的是：

```python
# 步骤 1: 更新子节点的 parent 指针（"过继"）
for node in self.node_list:
    if node.parent == self.node_list[i]:  # 原来指向 E
        node.parent = edge_node            # 现在指向 F

# 步骤 2: 替换节点
self.node_list[i] = edge_node  # F 替换 E

# 步骤 3: 递归更新子树成本
self.propagate_cost_to_leaves(self.node_list[i])
```

**关键点**：
- 必须先"过继"子节点，再替换父节点
- 否则子节点会指向已失效的旧节点
- 这保持了树的完整性

### 渐近最优的实现机制

**为什么 RRT* 是渐近最优？**

1. **每次迭代都是优化机会**：
   - 新节点通过 choose_parent 优化自己
   - 新节点通过 rewire 优化其他节点

2. **局部优化的累积**：
   - 每次重连接可能只改进一点点
   - 但成千上万次迭代后累积成大优化

3. **最终收敛**：
   ```
   迭代 100: 找到路径，成本 30
   迭代 200: 重连接优化到 27
   迭代 300: 继续优化到 26.5
   迭代 ∞:   收敛到最优成本
   ```

### 核心要点总结

✅ **理解了 RRT* 的本质**：
- 不只找到路径，而是持续优化路径
- 每个新节点都是改进现有树的机会
- 局部优化的累积带来全局最优

✅ **掌握了两大核心机制**：
1. **Choose Parent**: 为新节点找最优父节点
2. **Rewire**: 新节点帮助邻近节点优化

✅ **理解了关键技术细节**：
- 成本传播必须递归
- "过继"子节点要保持树的完整性
- 动态搜索半径平衡效率和效果

---

### 改进 3: 寻找邻近节点 (`find_near_nodes`)

**位置**: [rrt_star.py:177-201](../../PythonRobotics/PathPlanning/RRTStar/rrt_star.py#L177-L201)

**目的**: 找到新节点周围指定半径内的所有节点

```python
def find_near_nodes(self, new_node):
    """
    1) 定义以 new_node 为中心的球
    2) 返回树中所有在球内的节点
    """
    nnode = len(self.node_list) + 1

    # 动态搜索半径
    r = self.connect_circle_dist * math.sqrt(math.log(nnode) / nnode)

    # 限制不超过扩展距离
    if hasattr(self, 'expand_dis'):
        r = min(r, self.expand_dis)

    # 找到所有距离 ≤ r 的节点
    dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2
                 for node in self.node_list]
    near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]

    return near_inds
```

**动态搜索半径**：

```
节点数少时:
  nnode = 10
  r ≈ 50.0 * sqrt(log(10)/10) ≈ 50.0 * 0.48 ≈ 24.0
  搜索范围较大，促进探索

节点数多时:
  nnode = 1000
  r ≈ 50.0 * sqrt(log(1000)/1000) ≈ 50.0 * 0.08 ≈ 4.0
  搜索范围较小，提高效率

理论保证:
  平均邻近节点数 = O(log n)
  总复杂度 = O(n log n)
```

---

### 改进 4: 搜索最优目标节点 (`search_best_goal_node`)

**位置**: [rrt_star.py:147-175](../../PythonRobotics/PathPlanning/RRTStar/rrt_star.py#L147-L175)

**目的**: 在所有可以到达目标的节点中，选择成本最低的

```python
def search_best_goal_node(self):
    # 1. 找到所有距离目标 ≤ expand_dis 的节点
    dist_to_goal_list = [
        self.calc_dist_to_goal(n.x, n.y) for n in self.node_list
    ]
    goal_inds = [
        dist_to_goal_list.index(i) for i in dist_to_goal_list
        if i <= self.expand_dis
    ]

    # 2. 筛选可以安全到达目标的节点
    safe_goal_inds = []
    for goal_ind in goal_inds:
        t_node = self.ster(self.node_list[goal_ind], self.goal_node)
        if self.check_collision(t_node, ...):
            safe_goal_inds.append(goal_ind)

    if not safe_goal_inds:
        return None

    # 3. 选择成本最低的节点
    safe_goal_costs = [
        self.node_list[i].cost +
        self.calc_dist_to_goal(self.node_list[i].x,
                               self.node_list[i].y)
        for i in safe_goal_inds
    ]

    min_cost = min(safe_goal_costs)
    for i, cost in zip(safe_goal_inds, safe_goal_costs):
        if cost == min_cost:
            return i  # 返回最优目标节点的索引

    return None
```

**与 RRT 的区别**：

```
RRT:
  找到第一个可以到达目标的节点
  → 立即返回路径
  → 路径质量依赖运气

RRT*:
  找到所有可以到达目标的节点
  → 选择成本最低的
  → 路径质量最优 ✓
```

---

## 📊 算法特点

### 优点

✅ **渐近最优**: 随着迭代增加，路径趋向最优
✅ **路径优化**: 持续改进已找到的路径
✅ **成本感知**: 每个节点都有成本信息
✅ **适应性强**: 在各种环境中都能找到较优路径

### 缺点

❌ **计算复杂度高**: O(n log n) vs O(n)
❌ **内存消耗大**: 需要存储成本信息和邻近节点
❌ **收敛速度慢**: 需要更多迭代才能达到最优
❌ **参数敏感**: search_until_max_iter 影响性能

---

## 🔬 RRT vs RRT* 对比

### 代码层面对比

| 特性 | RRT | RRT* |
|------|-----|------|
| **Node 类** | `x, y, path_x, path_y, parent` | `x, y, path_x, path_y, parent, cost` |
| **新节点** | `self.node_list.append(new_node)` | `choose_parent + rewire + append` |
| **目标检测** | 第一个可到达的节点 | 成本最低的可到达节点 |
| **继续优化** | 无 | `search_until_max_iter` |

### 性能对比

**场景**: 起点 (0,0) → 目标 (6,10)，7个障碍物

| 指标 | RRT | RRT* |
|------|-----|------|
| **路径长度** | ~30-35 | ~25-30 |
| **迭代次数** | ~50-100 | ~200-300 |
| **计算时间** | 快 | 慢 2-3 倍 |
| **路径质量** | 一般 | 优秀 |

---

## 💡 理解要点

### 1. 为什么 RRT* 是渐近最优？

**数学保证**：
```
设 c* 为最优路径成本

RRT: lim(n→∞) P(cost ≤ c*) = 概率完备（可能找到）
RRT*: lim(n→∞) E[cost] = c*         （期望收敛到最优）
```

**直观理解**：
- 每次迭代都有机会改进现有路径
- 随着节点增多，找到更好路径的概率增加
- 最终路径会趋向最优

### 2. 选择父节点 + 重连接的作用

**选择父节点**：
- 确保新节点以最优方式连接到树
- 类似 Dijkstra 的松弛操作

**重连接**：
- 确保现有节点有机会找到更好的父节点
- 持续优化整个树的结构

**类比**：
```
RRT: 像贪心算法，只看眼前最近
RRT*: 像 Dijkstra，全局考虑成本
```

### 3. 动态搜索半径的智慧

```python
r ∝ sqrt(log(n) / n)
```

**为什么这样设计**：
- n 小时：r 大，促进快速探索
- n 大时：r 小，减少计算量
- 平衡探索和效率

---

## 🎓 学习心得

### 核心理解

RRT* 的本质是：
1. **成本优化**: 不只考虑可达性，还考虑路径质量
2. **持续改进**: 每次迭代都可能优化现有路径
3. **局部优化**: 选择父节点和重连接都是局部操作
4. **全局最优**: 局部优化的累积带来全局最优

### 与其他算法的对比

| 特性 | Dijkstra | A* | RRT | RRT* |
|------|----------|----|-----|------|
| 最优性 | ✓ | ✓ | ✗ | ✓ (渐近) |
| 速度 | 慢 | 快 | 中 | 慢 |
| 高维 | 差 | 差 | 好 | 好 |

---

## 📚 参考资源

### 论文
- Karaman, S., & Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning"

### 书籍
- "Planning Algorithms" by LaValle (Chapter 5)
- "Probabilistic Robotics" by Thrun (Chapter 5)

### 在线资源
- [OMPL (Open Motion Planning Library)](http://ompl.kavrakilab.org/)
- [RRT* 可视化](https://www.youtube.com/watch?v=313aTiHwSes)

---

## 🚀 下一步学习

- [ ] Informed RRT* - 基于椭圆采样的加速版本
- [ ] RRT* Smart - 智能采样策略
- [ ] Anytime RRT* - 时间约束的最优规划
- [ ] 多目标 RRT* - 考虑多个优化目标

---

**学习完成度**: ⭐⭐⭐⭐⭐ (5/5) - 完全掌握！
**掌握程度**: 深入理解原理、代码实现和核心机制
**日期**: 2026-01-21

---

## 🏆 学习成果总结

### ✅ 已掌握的核心内容

1. **RRT* vs RRT 的本质区别**
   - RRT: 快速找到路径（概率完备）
   - RRT*: 持续优化路径（渐近最优）

2. **两大核心机制**（完全理解）
   - **Choose Parent**: 为新节点选择最优父节点
   - **Rewire**: 新节点优化邻近节点的连接

3. **关键技术细节**
   - 成本传播的递归机制
   - "过继"子节点保持树的完整性
   - 动态搜索半径的数学原理

4. **完整的 5 步流程**
   ```
   1. 生成随机节点 A
   2. 找最近节点 B，扩展到 C
   3. 为 C 选最优父节点 D
   4. 重连接：C 帮助邻近节点 E 优化
   5. 重复迭代
   ```

### 🎯 深刻理解的关键点

1. **为什么需要 rewire？**
   - 每个新节点都是优化现有树的机会
   - 局部优化的累积 → 全局最优
   - 这是 RRT* 实现渐近最优的关键

2. **为什么是"过继"？**
   - 必须先更新子节点的 parent 指针
   - 再替换父节点本身
   - 否则会破坏树的完整性

3. **为什么渐近最优？**
   - 每次迭代都有机会改进路径
   - 随着节点增多，找到更好路径的概率增加
   - 最终收敛到最优解

### 📊 实验验证

**RRT vs RRT* 对比实验结果**：
- RRT 路径长度: 27.89
- RRT* 路径长度: 26.66
- 改进: 4.4%

✅ 实验验证了理论：RRT* 确实能找到更优路径

### 🚀 可以进一步探索的方向

- Informed RRT*（椭圆采样加速）
- RRT* Smart（智能采样）
- 动态环境中的 RRT*
- 高维空间的 RRT* 优化

---

**恭喜！你已经完全掌握了 RRT* 算法的核心原理和实现细节！** 🎓🎉
