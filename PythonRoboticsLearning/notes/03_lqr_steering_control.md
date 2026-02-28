# LQR 转向控制学习笔记

## 1. LQR 控制理论基础

### 1.1 线性二次调节器 (Linear Quadratic Regulator)

LQR 是一种最优控制方法，用于设计线性系统的反馈控制器。

**系统模型**：
```
x[k+1] = A*x[k] + B*u[k]
```

**代价函数**：
```
J = Σ(x[k]ᵀ*Q*x[k] + u[k]ᵀ*R*u[k])
```
- `Q`: 状态权重矩阵 (状态偏差的惩罚)
- `R`: 控制权重矩阵 (控制输入的惩罚)

**最优控制律**：
```
u[k] = -K*x[k]
```

### 1.2 离散时间代数黎卡提方程 (DARE)

```
X = AᵀXA - AᵀXB(R + BᵀXB)⁻¹BᵀXA + Q
```
*(注：代码中的 `X` 即理论中的 `P` 矩阵)*

**最优增益矩阵**：
```
K = (BᵀXB + R)⁻¹(BᵀXA)
```

### 1.3 【核心概念】P、K、u* 与 J 的关系 (物理直觉)
LQR 的核心不仅仅是求一个控制量，而是建立一个因果链条：

1.  **P 矩阵 (X)**：代表**“当前处境的糟糕程度”**（最小代价地形图）。$x^T P x$ 预示了从当前状态恢复到平衡点所需的理论最小代价。
2.  **K 矩阵**：代表**“最陡峭的下降路径”**（策略）。它是权衡了“处境糟糕程度(P)”和“能量成本(R)”后算出的最佳梯度方向。
3.  **u* (控制量)**：代表**“当下的执行动作”**。$u = -Kx$，即根据当前状态和最佳策略计算出的具体推力。

### 1.4 【误区辨析】开环序列 vs 闭环策略
- **误解**：认为 LQR 是一次性算出未来所有的控制序列 `u(0), u(1), ...`。
- **正解**：LQR 算出的是**反馈增益 K (锦囊妙计)**。
    - LQR 是**闭环控制**：每一步根据当前的实际状态 $x(t)$，通过 $u(t) = -K \cdot x(t)$ 实时计算控制量。
    - 即使受到扰动，由于 $x(t)$ 变了，计算出的 $u(t)$ 也会自动调整，确保持续最优。

## 2. 路径跟踪中的 LQR 应用

### 2.1 状态空间建模

**状态量定义** (`lqr_steer_control.py:170-175`)：
- `e`: 横向误差 (lateral error) - 车辆到参考轨迹的垂直距离
- `e_dot`: 横向误差变化率 ≈ v * sin(θ_e)
- `θ_e`: 航向误差 (heading error) - 车辆航向与参考航向的差值
- `θ_e_dot`: 航向误差变化率 ≈ v*tan(δ)/L - v*k

**控制量**：
- `δ`: 转向角 (steering angle)

### 2.2 线性化状态空间模型

基于车辆运动学和误差动力学的线性化：

```python
# 状态转移矩阵 A (4x4)
A = [[1, dt,  0,  0],
     [0,  1, v,   0],  # 注意：包含变量 v
     [0,  0,  1, dt],
     [0,  0,  0,  1]]

# 输入矩阵 B (4x1)
B = [[0],
     [0],
     [0],
     [v/L]]            # 注意：包含变量 v      
```

**物理含义**：
- `A[0,1] = dt`: 横向误差的积分关系
- `A[1,2] = v`: 航向误差影响横向误差变化率
- `A[2,3] = dt`: 航向误差的积分关系
- `B[3,0] = v/L`: 转向角对航向误差变化率的影响

### 2.3 LQR 控制器设计

**权重矩阵设计** (`lqr_steer_control.py:32-35`)：
```python
Q = np.eye(4)  # 状态权重矩阵 - 各状态量等权重
R = np.eye(1)  # 控制权重矩阵 - 转向角的权重
```

**控制律实现** (`lqr_steer_control.py:198-206`)：
```python
# 前馈控制 (feedforward)
ff = math.atan2(L * k, 1)  # 基于路径曲率的补偿

# 反馈控制 (feedback)
fb = pi_2_pi((-K @ x)[0, 0])  # 基于状态误差的修正

# 最终控制量
delta = ff + fb
```

### 2.4 【关键实现细节】为什么在 While 循环中反复计算 K？
这是本代码与标准 LQR 的最大区别：**LPV (Linear Parameter-Varying) 设计**。

1.  **现象**：代码在 `lqr_steering_control` 函数内，每次都重新构建 A、B 矩阵并调用 `dlqr` 求解 K。
2.  **原因**：
    - 车辆运动学模型是非线性的，线性化后的矩阵 **A 和 B 含有当前速度 $v$**。
    - 速度 $v$ 是变化的。低速时方向盘对车体影响小，高速时影响大。
    - 如果用固定的 $K$（例如基于 $v=0$ 设计），在高速时会导致系统不稳定（画龙）；反之则反应迟钝。
3.  **结论**：这是一个**增益调度 (Gain Scheduling)** 的实时实现。
    - **标准 LQR**：针对定常系统 ($A, B$ 不变)，$K$ 是常数，只需离线算一次。
    - **本代码 LQR**：针对变参数系统 ($v$ 变化)，$K$ 随速度变化，因此需要实时重算（或查表）。

## 3. 核心算法分析

### 3.1 DARE 求解器 (`solve_DARE` 函数)

```python
def solve_DARE(A, B, Q, R):
    """迭代法求解离散时间代数黎卡提方程"""
    X = Q
    max_iter = 150
    eps = 0.01

    for i in range(max_iter):
        Xn = A.T @ X @ A - A.T @ X @ B @ \
            la.inv(R + B.T @ X @ B) @ B.T @ X @ A + Q

        if (abs(Xn - X)).max() < eps:
            break
        X = Xn
    return Xn
```

**算法特点**：
- 使用迭代法求解而非直接求解
- 收敛判断基于矩阵元素的最大变化量
- 最大迭代次数限制防止无限循环

### 3.2 最近点搜索 (`calc_nearest_index` 函数)

```python
def calc_nearest_index(state, cx, cy, cyaw):
    """计算最近点索引和横向误差"""
    # 计算到所有路径点的距离
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    # 找最小距离点
    mind = min(d)
    ind = d.index(mind)
    mind = math.sqrt(mind)

    # 计算横向误差的正负号
    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y
    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1  # 右侧为负

    return ind, mind
```

**关键点**：
- 横向误差有正负之分：左侧为正，右侧为负
- 通过矢量夹角判断车辆相对于路径的位置

## 4. 仿真实现分析

### 4.1 主仿真循环 (`closed_loop_prediction`)

```python
while T >= time:
    # 1. LQR转向控制
    dl, target_ind, e, e_th = lqr_steering_control(
        state, cx, cy, cyaw, ck, e, e_th)

    # 2. PID速度控制
    ai = pid_control(speed_profile[target_ind], state.v)

    # 3. 状态更新
    state = update(state, ai, dl)

    time = time + dt
```

### 4.2 车辆运动学模型 (`update` 函数)

```python
def update(state, a, delta):
    """自行车运动学模型"""
    # 转向角限制
    delta = np.clip(delta, -max_steer, max_steer)

    # 状态更新方程
    state.x += state.v * math.cos(state.yaw) * dt
    state.y += state.v * math.sin(state.yaw) * dt
    state.yaw += state.v / L * math.tan(delta) * dt
    state.v += a * dt

    return state
```

## 5. 参数调节指南

### 5.1 权重矩阵调节

**Q矩阵调节**：
- `Q[0,0]` ↑: 更严格控制横向误差
- `Q[1,1]` ↑: 更平滑的横向误差变化
- `Q[2,2]` ↑: 更严格控制航向误差
- `Q[3,3]` ↑: 更平滑的航向误差变化

**R矩阵调节**：
- `R[0,0]` ↑: 减少转向动作，更平滑但响应慢
- `R[0,0]` ↓: 增加转向动作，响应快但可能振荡

### 5.2 系统参数影响

- `dt`: 时间步长，影响离散化精度
- `L`: 轴距，影响转向响应特性
- `max_steer`: 最大转向角，限制控制输入

## 6. 优缺点分析

### 6.1 优点
- **最优性**: 在给定代价函数下保证最优控制。
- **稳定性**: 只要系统可控，LQR 保证闭环稳定。
- **多变量处理**: 能够自动处理多个状态量之间的耦合（如同时优化横向误差和航向误差）。

### 6.2 缺点
- **算力消耗 (特指本实现)**：由于 A、B 矩阵随速度变化，需要在每个控制周期求解 DARE 方程（$O(N^3)$），对嵌入式硬件压力大。（*工业界通常采用离线计算+查表法解决此问题*）。
- **全状态反馈需求**: 需要知道所有状态量（如 $e, \dot{e}$），实际中若无法直接测量，需配合状态观测器（如卡尔曼滤波）。
- **模型依赖**: 极其依赖 $A, B$ 矩阵的准确性。

## 7. 与其他控制方法比较

| 特性 | LQR | PID | Pure Pursuit |
|------|-----|-----|--------------|
| 理论基础 | 最优控制理论 | 经典控制 | 几何学 |
| 稳定性 | 理论保证 | 需要调节 | 一般稳定 |
| 参数调节 | 权重矩阵 | 三个增益 | 前瞻距离 |
| 计算复杂度 | 高 | 低 | 低 |
| 适应性 | 中等 | 强 | 弱 |

## 8. 扩展思考

1. **自适应LQR**: 根据车辆状态和路径特性自动调节Q、R矩阵
2. **鲁棒LQR**: 考虑模型不确定性和外界干扰
3. **非线性MPC**: 处理强非线性和约束条件
4. **学习增强**: 结合机器学习优化参数选择

## 9. 参考文献

- 原始实现: `PythonRobotics/PathTracking/lqr_steer_control/lqr_steer_control.py`
- LQR理论: Modern Control Engineering (Ogata)
- 路径跟踪: Vehicle Dynamics and Control (Rajesh Rajamani)