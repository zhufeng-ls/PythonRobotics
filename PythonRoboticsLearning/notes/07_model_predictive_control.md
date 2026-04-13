# MPC算法原理与数学推导

> **作者**：学习笔记
> **日期**：2026-03-09
> **主题**：Model Predictive Control (MPC) 数学推导与关键问题解析

---

## 📚 目录

1. [问题的提出](#1-问题的提出)
2. [车辆运动学模型](#2-车辆运动学模型)
3. [线性化方法](#3-线性化方法)
4. [MPC优化问题构建](#4-mpc优化问题构建)
5. [迭代求解算法](#5-迭代求解算法)
6. [核心疑问解析](#6-核心疑问解析)

---

## 1. 问题的提出

### 1.1 MPC问题描述

考虑离散非线性系统：
```
x[k+1] = f(x[k], u[k])
```

**状态变量**：x = [x_pos, y_pos, v, yaw]^T
**控制输入**：u = [a, δ]^T (加速度，转向角)

**MPC目标**：在每个时刻k，求解未来T步的最优控制序列：
```
U* = [u*[k], u*[k+1], ..., u*[k+T-1]]
```

使得预测状态轨迹尽可能接近参考轨迹，同时满足各种约束条件。

### 1.2 MPC核心思想

**滚动优化机制**：
```
时刻 k:   [k] → [k+1] → [k+2] → [k+3] → [k+4]
          ↑    ╰─────── 优化窗口 T=4 ────────╯
        执行u*[k]

时刻 k+1:     [k+1] → [k+2] → [k+3] → [k+4] → [k+5]
              ↑      ╰─────── 优化窗口 T=4 ────────╯
            执行u*[k+1]
```

**为什么只执行第一步？**
- 模型误差会随预测步数累积
- 滚动优化提供反馈校正机制
- 平衡预测价值与计算负担

---

## 2. 车辆运动学模型

### 2.1 非线性车辆模型

MPC基于经典的自行车模型（Bicycle Model）：

```
        前轮
         │δ (转向角)
         │
    ─────┼─────  ← v (速度)
         │
    ─────┼─────
         │
        后轮
         │
        L (轴距)
```

**连续时间动力学方程**：
```
ẋ = v cos(yaw)
ẏ = v sin(yaw)
v̇ = a
yaẇ = (v tan(δ))/L
```

**离散化模型**（前向欧拉法，时间步长dt）：
```
x[k+1] = x[k] + v[k] cos(yaw[k]) · dt
y[k+1] = y[k] + v[k] sin(yaw[k]) · dt
v[k+1] = v[k] + a[k] · dt
yaw[k+1] = yaw[k] + (v[k] tan(δ[k]))/L · dt
```

**向量形式**：
```
x[k+1] = f(x[k], u[k])
```

其中：
- 状态: `x = [x_pos, y_pos, v, yaw]^T ∈ R^4`
- 控制: `u = [a, δ]^T ∈ R^2` (加速度，转向角)

### 2.2 非线性特性分析

**为什么是非线性的？**

1. **三角函数项**：cos(yaw), sin(yaw)
2. **乘积项**：v · cos(yaw), v · sin(yaw)
3. **非线性函数**：tan(δ)

**线性化的必要性**：
- 非线性优化(NLP)：计算复杂，可能陷入局部最优
- 线性化后：转化为凸二次规划(QP)，全局最优且快速

---

## 3. 线性化方法

### 3.1 线性化基本原理

**目标**：将非线性模型 `f(x, u)` 在工作点 `(x₀, u₀)` 处线性化

**泰勒展开（一阶近似）**：
```
f(x, u) ≈ f(x₀, u₀) + ∂f/∂x|_(x₀,u₀) (x - x₀) + ∂f/∂u|_(x₀,u₀) (u - u₀)
```

**线性化形式**：
```
x[k+1] = A x[k] + B u[k] + C
```

其中：
- A = ∂f/∂x|_(x₀,u₀)：状态雅可比矩阵 (4×4)
- B = ∂f/∂u|_(x₀,u₀)：控制雅可比矩阵 (4×2)
- C = f(x₀, u₀) - A x₀ - B u₀：线性化余项 (4×1)

### 3.2 雅可比矩阵推导

**A矩阵计算**：

对于 f = [f₁, f₂, f₃, f₄]^T，其中：
```
f₁ = x + v cos(yaw) · dt
f₂ = y + v sin(yaw) · dt
f₃ = v + a · dt
f₄ = yaw + (v tan(δ))/L · dt
```

**A矩阵各元素**：
```
A[0,0] = ∂f₁/∂x = 1
A[0,1] = ∂f₁/∂y = 0
A[0,2] = ∂f₁/∂v = dt cos(yaw₀)
A[0,3] = ∂f₁/∂yaw = -dt v₀ sin(yaw₀)

A[1,0] = ∂f₂/∂x = 0
A[1,1] = ∂f₂/∂y = 1
A[1,2] = ∂f₂/∂v = dt sin(yaw₀)
A[1,3] = ∂f₂/∂yaw = dt v₀ cos(yaw₀)

A[2,0] = A[2,1] = A[2,2] = A[2,3] = 0
A[2,2] = 1

A[3,0] = A[3,1] = 0
A[3,2] = ∂f₄/∂v = dt tan(δ₀)/L
A[3,3] = 1
```

**最终A矩阵**：
```
A = [1   0   dt·cos(yaw₀)   -dt·v₀·sin(yaw₀)]
    [0   1   dt·sin(yaw₀)    dt·v₀·cos(yaw₀)]
    [0   0   1               0              ]
    [0   0   dt·tan(δ₀)/L    1              ]
```

**B矩阵计算**：

```
B[0,0] = ∂f₁/∂a = 0
B[0,1] = ∂f₁/∂δ = 0
B[1,0] = ∂f₂/∂a = 0
B[1,1] = ∂f₂/∂δ = 0
B[2,0] = ∂f₃/∂a = dt
B[2,1] = ∂f₃/∂δ = 0
B[3,0] = ∂f₄/∂a = 0
B[3,1] = ∂f₄/∂δ = dt v₀/(L cos²(δ₀))
```

**最终B矩阵**：
```
B = [0    0                    ]
    [0    0                    ]
    [dt   0                    ]
    [0    dt·v₀/(L·cos²(δ₀))  ]
```

**关键推导 B[3,1]**：
```
∂/∂δ[v tan(δ)/L] = v/(L cos²(δ))
```

### 3.3 线性化余项C

**C向量的作用**：补偿线性化误差，确保在工作点处：
```
f(x₀, u₀) = A x₀ + B u₀ + C
```

**计算公式**：
```
C = f(x₀, u₀) - A x₀ - B u₀
```

**各分量**：
```
C[0] = dt v₀ sin(yaw₀) yaw₀
C[1] = -dt v₀ cos(yaw₀) yaw₀
C[2] = 0
C[3] = -dt v₀ δ₀/(L cos²(δ₀))
```

**重要性验证**：在工作点处，线性模型与非线性模型完全匹配（误差为0）。

---

## 4. MPC优化问题构建

### 4.1 标准QP形式

MPC的核心是将轨迹跟踪问题转化为凸二次规划：

```
minimize:   (1/2) z^T P z + q^T z
subject to: l ≤ A_eq z ≤ u
            x[0] = x_init
```

其中：
- z = [x[0], u[0], x[1], u[1], ..., x[T], u[T]]^T：所有优化变量
- P, q：二次代价函数参数
- A_eq：约束矩阵（动力学+边界约束）

### 4.2 代价函数设计

**目标函数**：
```
J = Σ(t=0→T-1) [||x_ref[t] - x[t]||²_Q + ||u[t]||²_R + ||Δu[t]||²_R_d] + ||x_ref[T] - x[T]||²_Q_f
```

**各项含义**：

1. **跟踪误差项**：`||x_ref - x||²_Q`
   - 惩罚状态偏离参考轨迹
   - Q为状态权重矩阵，通常 Q = diag([q_x, q_y, q_v, q_yaw])

2. **控制代价项**：`||u||²_R`
   - 惩罚控制输入幅值，节约能耗
   - R为控制权重矩阵，通常 R = diag([r_a, r_δ])

3. **控制平滑项**：`||Δu||²_R_d`
   - 惩罚控制变化率，保证平滑性
   - R_d为控制变化权重矩阵

4. **终端代价项**：`||x_ref[T] - x[T]||²_Q_f`
   - 强化终端状态精度

### 4.3 约束条件

**1. 动力学约束**（每个时间步）：
```
x[t+1] = A[t] x[t] + B[t] u[t] + C[t],  t = 0,1,...,T-1
```

**2. 初值约束**：
```
x[0] = x_init
```

**3. 状态约束**：
```
v_min ≤ v[t] ≤ v_max,  t = 0,1,...,T
```

**4. 控制约束**：
```
|a[t]| ≤ a_max,   t = 0,1,...,T-1
|δ[t]| ≤ δ_max,   t = 0,1,...,T-1
```

**5. 控制变化率约束**：
```
|δ[t+1] - δ[t]| ≤ δ̇_max · dt,  t = 0,1,...,T-2
```

## 5. 迭代求解算法

### 5.1 为什么需要迭代？

**问题根源**：线性化只在工作点附近准确

**核心矛盾**：
- A, B, C 是基于预测轨迹 x̄ 计算的线性化系数
- 但最优控制量 u* 会让系统状态偏离预测轨迹
- 当真实状态偏离时，原来的 A, B, C 就不准确了

**迭代改进思路**：
```
初始猜测 → 线性化 → 求解QP → 更新猜测 → 收敛检查
```

### 5.2 两种迭代方式

#### 方式1：全轨迹迭代（PythonRobotics代码采用）

```python
def mpc_trajectory_iteration():
    """
    全轨迹迭代：整个预测轨迹作为一个整体进行迭代
    """
    for iter in range(MAX_ITER):  # 对整个轨迹迭代
        # 1. 用当前控制序列预测整条轨迹
        x_trajectory = predict_full_trajectory(u_sequence)

        # 2. 在整条轨迹上进行线性化
        for t in range(N):
            A[t], B[t], C[t] = linearize_at_point(x_trajectory[t], u_sequence[t])

        # 3. 求解整个QP问题（所有时刻一起优化）
        u_new_sequence = solve_full_qp(A_all, B_all, C_all)

        # 4. 检查整个序列的收敛
        if converged_trajectory(u_new_sequence, u_sequence):
            break

        u_sequence = u_new_sequence
```

#### 方式2：逐点迭代

```python
def mpc_point_by_point_iteration():
    """
    逐点迭代：在每个预测步骤内部进行迭代
    """
    for t in range(N):  # 对预测时域的每个时刻
        for iter in range(max_iter):  # 在每个点上迭代
            # 在当前点 (x[t], u[t]) 线性化
            A[t], B[t], C[t] = linearize_at_point(x_guess[t], u_guess[t])

            # 求解当前时刻的最优控制
            u_new[t] = solve_single_step_optimization(...)

            # 检查当前点收敛
            if converged_at_point(u_new[t], u_guess[t]):
                break

            u_guess[t] = u_new[t]
```

**关键区别**：
- **全轨迹迭代**：整个预测时域作为一个统一的优化问题来解决
- **逐点迭代**：在每个时间点上独立达到一致性

### 5.3 PythonRobotics的实现方式

**核心流程**：

```python
def iterative_linear_mpc_control(xref, x0, dref, oa, od):
    for i in range(MAX_ITER):  # 轨迹级别的迭代
        # 第1步：用当前控制序列预测完整轨迹
        xbar = predict_motion(x0, oa, od, xref)  # 关键！整条轨迹

        # 第2步：在完整轨迹上线性化所有点
        poa, pod = oa[:], od[:]
        oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)

        # 第3步：检查整个控制序列的收敛
        du = sum(abs(oa - poa)) + sum(abs(pod - od))  # 整个序列的变化
        if du <= DU_TH:
            break
```

**轨迹预测函数**：
```python
def predict_motion(x0, oa, od, xref):
    """
    用整个控制序列预测整条状态轨迹
    这是全轨迹方法的核心
    """
    xbar = xref * 0.0
    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])

    # 用非线性模型逐步积分整条轨迹
    for (ai, di, i) in zip(oa, od, range(1, T + 1)):
        state = update_state(state, ai, di)  # 非线性状态转移
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar  # 返回完整的预测轨迹
```

---

## 6. 核心疑问解析

### 6.1 预测轨迹与线性化的矛盾

**疑问**：A, B, C 是根据预测的 yaw 与 v 求得的，但实际的 u 控制量控制后，yaw, v 会发生变化，这不是矛盾吗？

**回答**：这确实是一个**根本性矛盾**！

**矛盾点**：
1. A, B, C 是基于预测轨迹 (x̄, ū) 计算的线性化系数
2. 但最优控制量 u* 会让系统状态偏离预测轨迹
3. 当真实状态偏离时，原来的 A, B, C 就不准确了

这是一个**"鸡生蛋、蛋生鸡"**的问题！

### 6.2 工程解决方案

**方法1：单次求解 + 滚动时域（最常用）**

```
关键思想：接受不一致性，依靠闭环反馈修正

时刻k:   基于真实x[k]重新线性化 → 求解u*[k:k+N-1] → 只用u*[k]
时刻k+1: 基于真实x[k+1]重新线性化 → 求解u*[k+1:k+N] → 只用u*[k+1]
```

**为什么这样可行？**
1. **滚动时域策略**：只应用第一个控制量，下一步重新计算
2. **闭环反馈**：即使预测不准确，下一步会基于真实状态重新规划
3. **工程实用性**：单次求解速度快，满足实时性要求

**方法2：迭代线性化（更精确）**

通过多次迭代减少线性化误差：
1. 在当前轨迹上线性化
2. 求解QP得到新控制序列
3. 用新控制量预测新轨迹
4. 检查收敛，未收敛则继续迭代

### 6.3 数学本质：约束到最优解的转换

**核心等式**：
$$\underbrace{x[k+1] = A[k] x[k] + B[k] u[k] + C[k]}_{\text{线性约束}} \Rightarrow \underbrace{X = \Phi x_0 + \Gamma U + \Psi}_{\text{只含决策变量 U}}$$

**从约束到决策变量的转换**：
- **原来**：约束涉及 $x[k]$ 和 $u[k]$（多个变量）
- **现在**：约束只涉及 $U$（单一决策变量向量）

**关键步骤**：通过递推展开，将所有未来状态用控制序列表达：

```
x[1] = A[0]·x[0] + B[0]·u[0] + C[0]
x[2] = A[1]·x[1] + B[1]·u[1] + C[1]
     = A[1]·(A[0]·x[0] + B[0]·u[0] + C[0]) + B[1]·u[1] + C[1]
     = A[1]A[0]·x[0] + A[1]B[0]·u[0] + B[1]·u[1] + A[1]C[0] + C[1]
```

**变量消除的意义**：
- QP求解器看到的是：`minimize U^T H U + f^T U subject to A_ineq U <= b_ineq`
- 所有状态变量被消除，只优化控制序列U
- 未来约束会影响当前决策（这就是MPC预测能力的数学体现）

### 6.4 迭代方式的本质区别

**用户理解完全正确**：

- **我之前举例的迭代方式**：在一次迭代中，对每个点进行迭代
- **PythonRobotics代码**：一次对预测轨迹上的所有点进行迭代，使用生成的轨迹进行下次迭代

**具体区别**：
- **全轨迹迭代**：将整个预测时域作为一个统一的优化问题
- **逐点迭代**：每个时间点独立达到一致性

**为什么选择全轨迹迭代？**
1. **符合MPC本质**：MPC本身就是"全时域优化"问题
2. **更好的收敛性**：整条轨迹联合优化比逐点独立优化更容易收敛
3. **实现简单**：只需要一个外层循环
4. **计算效率**：虽然每次迭代计算量大，但总迭代次数少（通常2-3次）

### 6.5 关键洞察总结

1. **线性化不一致性是客观存在的**：工程上通过滚动优化和迭代求解来处理

2. **MPC的成功不在于单次预测的完美**：而在于**持续的闭环反馈和重新规划**能力

3. **变量消除是QP求解的核心**：将多变量约束问题转化为单一变量向量的优化问题

4. **全轨迹迭代符合MPC本质**：体现了"整体优化"的思想

5. **预测能力的数学本质**：未来约束通过变量耦合影响当前决策

---

**核心理解**：MPC通过巧妙的**滚动优化**策略，将一个复杂的全局非线性问题转化为一系列相对简单的局部线性化QP问题。虽然每次线性化都有误差，但通过频繁的重新规划和闭环反馈，整体上实现了优秀的控制性能。

### 5.1 为什么需要迭代？

**问题根源**：线性化只在工作点附近准确

**迭代改进思路**：
```
初始猜测 → 线性化 → 求解QP → 更新猜测 → 收敛检查
```

**核心机制**：
1. 用非线性模型预测轨迹（工作点）
2. 在预测轨迹上逐步线性化
3. 求解QP得到新控制序列
4. 检查收敛，未收敛则迭代

### 5.2 完整算法流程

```python
def iterative_linear_mpc_control(x_ref, x0, u_prev):
    """
    迭代线性化MPC算法

    参数:
        x_ref: 参考轨迹 [4 × T+1]
        x0: 当前状态 [4 × 1]
        u_prev: 上次控制序列 [2 × T] (热启动)

    返回:
        u_opt: 最优控制序列 [2 × T]
        x_pred: 预测状态轨迹 [4 × T+1]
    """

    # 步骤1: 初始化控制序列
    if u_prev is None:
        u = zeros(2, T)  # 冷启动
    else:
        u = warm_start(u_prev)  # 热启动

    # 步骤2: 迭代求解
    for iter in range(MAX_ITER):  # MAX_ITER = 3

        # 步骤2a: 非线性模型预测轨迹
        x_bar = predict_motion_nonlinear(x0, u)

        # 步骤2b: 在预测轨迹上线性化
        A, B, C = [], [], []
        for t in range(T):
            A_t, B_t, C_t = get_linear_model_matrix(
                x_bar[2,t], x_bar[3,t], u[1,t]
            )
            A.append(A_t)
            B.append(B_t)
            C.append(C_t)

        # 步骤2c: 求解QP优化问题
        u_prev = u.copy()
        u, x_opt = solve_qp(x_ref, x_bar, x0, A, B, C)

        # 步骤2d: 收敛检查
        du = sum(abs(u - u_prev))
        if du <= DU_TH:  # DU_TH = 0.1
            print(f"迭代收敛，共{iter+1}步")
            break

    return u, x_opt
```

### 5.3 关键步骤详解

**步骤2a：非线性预测**

```python
def predict_motion_nonlinear(x0, u):
    """用非线性模型预测轨迹，作为线性化工作点"""
    x = zeros(4, T+1)
    x[:, 0] = x0

    for t in range(T):
        # 非线性车辆模型
        x[0, t+1] = x[0, t] + x[2, t] * cos(x[3, t]) * dt
        x[1, t+1] = x[1, t] + x[2, t] * sin(x[3, t]) * dt
        x[2, t+1] = x[2, t] + u[0, t] * dt
        x[3, t+1] = x[3, t] + x[2, t] * tan(u[1, t]) / L * dt

    return x
```

**步骤2c：QP求解**

```python
def solve_qp(x_ref, x_bar, x0, A, B, C):
    """构建并求解QP问题"""
    import cvxpy

    # 定义优化变量
    x = cvxpy.Variable((4, T+1))
    u = cvxpy.Variable((2, T))

    # 构建代价函数
    cost = 0.0
    for t in range(T):
        # 跟踪误差 + 控制代价
        if t > 0:  # 跳过初始状态
            cost += cvxpy.quad_form(x_ref[:, t] - x[:, t], Q)
        cost += cvxpy.quad_form(u[:, t], R)

        # 控制变化代价
        if t > 0:
            cost += cvxpy.quad_form(u[:, t] - u[:, t-1], R_d)

    # 终端代价
    cost += cvxpy.quad_form(x_ref[:, T] - x[:, T], Q_f)

    # 构建约束
    constraints = []

    # 初值约束
    constraints += [x[:, 0] == x0]

    # 动力学约束
    for t in range(T):
        constraints += [x[:, t+1] == A[t] @ x[:, t] + B[t] @ u[:, t] + C[t]]

    # 状态约束
    constraints += [x[2, :] <= V_MAX]  # 速度上界
    constraints += [x[2, :] >= V_MIN]  # 速度下界

    # 控制约束
    constraints += [cvxpy.abs(u[0, :]) <= A_MAX]  # 加速度
    constraints += [cvxpy.abs(u[1, :]) <= STEER_MAX]  # 转向角

    # 控制变化率约束
    for t in range(T-1):
        constraints += [
            cvxpy.abs(u[1, t+1] - u[1, t]) <= STEER_RATE_MAX * dt
        ]

    # 求解
    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.CLARABEL, verbose=False)

    if prob.status != cvxpy.OPTIMAL:
        raise RuntimeError(f"QP求解失败: {prob.status}")

    return u.value, x.value
```

### 5.4 热启动机制

**定义**：使用上一时刻的优化结果初始化当前优化

```python
def warm_start(u_prev):
    """热启动：时间平移 + 复制最后控制"""
    u_init = zeros(2, T)
    u_init[:, :-1] = u_prev[:, 1:]  # 时间平移
    u_init[:, -1] = u_prev[:, -1]   # 复制最后一个
    return u_init
```

**效果对比**：
- 冷启动（u=0）：通常需要3-5次迭代
- 热启动：通常1-2次迭代即收敛

**收敛判断**：
```
du = Σ|u_new[i] - u_old[i]| < 0.1
```

---

## 💻 代码实现分析

### 核心函数结构

#### 1. 主控制函数

```python
def iterative_linear_mpc_control(xref, x0, dref, oa, od):
    """
    参数:
        xref: 参考轨迹 [4 x T+1]
        x0: 当前状态 [x, y, v, yaw]
        dref: 参考转向角 [1 x T+1]
        oa: 上次加速度序列 [T]
        od: 上次转向角序列 [T]
    返回:
        oa, od: 优化的控制序列
        ox, oy, oyaw, ov: 预测轨迹
    """
```

#### 2. 线性MPC求解器

```python
def linear_mpc_control(xref, xbar, x0, dref):
    """
    参数:
        xref: 参考轨迹
        xbar: 线性化工作点轨迹
        x0: 初始状态
        dref: 参考转向角

    核心步骤:
    1. 定义优化变量 x[4,T+1], u[2,T]
    2. 构建代价函数
    3. 添加动力学约束
    4. 添加控制约束
    5. 求解优化问题
    """
```

### CVXPY 优化实现

```python
import cvxpy

# 1. 定义优化变量
x = cvxpy.Variable((NX, T + 1))  # 状态轨迹
u = cvxpy.Variable((NU, T))      # 控制输入

# 2. 构建代价函数
cost = 0.0
for t in range(T):
    cost += cvxpy.quad_form(u[:, t], R)                    # 控制代价
    if t != 0:
        cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)   # 跟踪代价
    if t < (T - 1):
        cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd) # 平滑代价

cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)  # 终端代价

# 3. 添加约束
constraints = []
constraints += [x[:, 0] == x0]  # 初值约束

for t in range(T):
    A, B, C = get_linear_model_matrix(xbar[2, t], xbar[3, t], dref[0, t])
    constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]  # 动力学约束

# 控制约束
constraints += [x[2, :] <= MAX_SPEED]                    # 最大速度
constraints += [x[2, :] >= MIN_SPEED]                    # 最小速度
constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]        # 最大加速度
constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]        # 最大转向角
constraints += [cvxpy.abs(u[1, t+1] - u[1, t]) <= MAX_DSTEER * DT]  # 转向变化率

# 4. 求解
prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
prob.solve(solver=cvxpy.CLARABEL, verbose=False)
```

### 关键参数设置

**MPC 参数**：
$$T = 5 \text{（预测时域长度）}$$
$$\Delta t = 0.2 \text{ s（时间步长）}$$
$$\text{MAX\_ITER} = 3 \text{（最大迭代次数）}$$
$$\epsilon = 0.1 \text{（收敛判断阈值）}$$

**车辆参数**：
$$L = 2.5 \text{ m（轴距）}$$
$$\delta_{\max} = 45° \text{（最大转向角）}$$
$$\dot{\delta}_{\max} = 30°/\text{s} \text{（最大转向角速度）}$$
$$v_{\max} = 15.28 \text{ m/s（最大速度）}$$
$$v_{\min} = -5.56 \text{ m/s（最小速度）}$$
$$a_{\max} = 1.0 \text{ m/s}^2\text{（最大加速度）}$$

**权重矩阵**：
$$\mathbf{R} = \text{diag}([0.01, 0.01]) \text{（控制权重）}$$
$$\mathbf{R_d} = \text{diag}([0.01, 1.0]) \text{（控制变化权重）}$$
$$\mathbf{Q} = \text{diag}([1.0, 1.0, 0.5, 0.5]) \text{（状态权重）}$$
$$\mathbf{Q_f} = \mathbf{Q} \text{（终端权重）}$$

---

## 📊 算法特点分析

### 优点

- ✅ **预测能力强**: 考虑未来T步的系统行为
- ✅ **约束处理**: 天然支持各种物理约束
- ✅ **最优性**: 在给定模型下理论最优
- ✅ **鲁棒性**: 滚动优化提供反馈校正
- ✅ **通用性**: 可处理MIMO系统

### 缺点

- ❌ **计算复杂**: 每步需求解QP问题，实时性挑战
- ❌ **模型依赖**: 性能极度依赖模型准确性
- ❌ **参数调节**: Q、R权重矩阵需要仔细调节
- ❌ **局部最优**: 非线性系统可能陷入局部最优
- ❌ **实现复杂**: 比PID、LQR复杂得多

---

## 🎯 与其他控制方法对比

### 路径跟踪算法横向对比

| 算法           | Pure Pursuit | LQR      | MPC           |
| -------------- | ------------ | -------- | ------------- |
| **控制理论**   | 几何学       | 最优控制 | 预测控制      |
| **预测能力**   | 单点前瞻     | 无       | 多步预测 ✓    |
| **约束处理**   | 难           | 难       | 天然支持 ✓    |
| **参数调节**   | 前瞻距离     | Q、R矩阵 | Q、R、T多参数 |
| **计算复杂度** | O(1)         | O(1)     | O(T³)         |
| **实时性**     | 优秀         | 良好     | 一般          |
| **最优性**     | 无保证       | 线性最优 | 受限最优      |
| **鲁棒性**     | 一般         | 良好     | 优秀 ✓        |

### 适用场景

**Pure Pursuit**:
- ✅ 低速机器人
- ✅ 简单环境
- ❌ 动态约束严格

**LQR**:
- ✅ 线性系统
- ✅ 无约束场景
- ❌ 强非线性系统

**MPC**:
- ✅ 复杂约束场景 ✓
- ✅ 高精度要求 ✓
- ✅ 多变量耦合系统 ✓
- ❌ 强实时性要求

---

## 🛠️ 参数调节指南

### 1. 权重矩阵调节

**$\mathbf{Q}$ 矩阵调节**（状态跟踪）:
$$\mathbf{Q} = \text{diag}([q_x, q_y, q_v, q_{\psi}])$$
- $q_x, q_y \uparrow$: 位置跟踪更精确，但控制更激烈
- $q_v \uparrow$: 速度跟踪更严格
- $q_{\psi} \uparrow$: 航向跟踪更精确

**$\mathbf{R}$ 矩阵调节**（控制代价）:
$$\mathbf{R} = \text{diag}([r_a, r_{\delta}])$$
- $r_a \uparrow$: 加速更温和，但速度响应变慢
- $r_{\delta} \uparrow$: 转向更温和，但横向响应变慢

**$\mathbf{R_d}$ 矩阵调节**（控制平滑性）:
$$\mathbf{R_d} = \text{diag}([r_{d,a}, r_{d,\delta}])$$
- $r_{d,a} \uparrow$: 加速度变化更平滑
- $r_{d,\delta} \uparrow$: 转向变化更平滑，避免震荡

### 2. MPC 参数调节

**预测时域 $T$**:
- $T$ 太小: 预测不够，性能下降
- $T$ 太大: 计算负担重，实时性差
- 经验值: $T = 5 \sim 10$

**时间步长 $\Delta t$**:
- $\Delta t$ 太小: 计算量大，线性化误差小
- $\Delta t$ 太大: 计算量小，线性化误差大
- 经验值: $\Delta t = 0.1 \sim 0.3$ s

**迭代次数 $N_{\text{iter}}$**:
- 迭代少: 线性化不够准确
- 迭代多: 计算时间长
- 经验值: $N_{\text{iter}} = 3 \sim 5$

### 3. 收敛判断调节

$$\epsilon = 0.1 \text{（控制变化阈值）}$$
- $\epsilon$ 太大: 可能未充分收敛
- $\epsilon$ 太小: 可能过度迭代
- 需要根据具体应用调节

---

## 🔍 深入理解要点

### 1. 为什么MPC需要滚动优化？

**关键理解**: MPC不是一次性规划整条轨迹

```
传统轨迹规划:
t=0: 规划整条路径 → 执行完毕

MPC滚动优化:
t=0: 规划未来5步 → 只执行第1步
t=1: 重新规划未来5步 → 只执行第1步
t=2: 重新规划未来5步 → 只执行第1步
...
```

**优势**:
- 能够适应环境变化
- 能够纠正模型误差
- 能够处理未建模干扰

### 2. 线性化 vs 非线性MPC

**当前实现**: 每步线性化的MPC
- 计算相对简单（求解QP）
- 需要迭代提高精度
- 适合实时应用

**非线性MPC**: 直接优化非线性模型
- 理论上更准确
- 需要求解NLP，计算更复杂
- 难以保证实时性

### 3. 热启动的重要性

**无热启动**：每次从零开始
$$\mathbf{u}^0 = [0, 0, 0, 0, 0] \text{（可能需要更多迭代）}$$

**热启动**：利用上次结果
$$\mathbf{u}_k = [u_0, u_1, u_2, u_3, u_4] \rightarrow \mathbf{u}_{k+1} = [u_1, u_2, u_3, u_4, u_4]$$

**经验数据**:
- 无热启动: 平均需要 3-5 次迭代
- 有热启动: 平均需要 1-2 次迭代

---

## 🧪 实际调试经验

### 常见问题与解决方案

**问题1**: 系统震荡
$$\text{原因：}\mathbf{R_d}\text{ 权重太小} \quad \mathbf{R_d} = \text{diag}([0.01, 0.01])$$
$$\text{解决：增加控制变化惩罚} \quad \mathbf{R_d} = \text{diag}([0.01, 1.0])$$

**问题2**: 跟踪精度不够
$$\text{原因：}\mathbf{Q}\text{ 相对于 }\mathbf{R}\text{ 太小}$$
$$\mathbf{Q} = \text{diag}([0.1, 0.1, 0.1, 0.1]), \quad \mathbf{R} = \text{diag}([1.0, 1.0])$$
$$\text{解决：提高状态权重，降低控制权重}$$
$$\mathbf{Q} = \text{diag}([1.0, 1.0, 0.5, 0.5]), \quad \mathbf{R} = \text{diag}([0.01, 0.01])$$

**问题3**: 求解失败
$$\text{检查约束冲突：}v_{\min} \leq v[t] \leq v_{\max}$$
$$\text{确保初值满足约束：}v_0 \in [v_{\min}, v_{\max}]$$

### 性能优化建议

**1. 求解器选择**:
```python
# 快速但精度一般
prob.solve(solver=cvxpy.SCS)

# 精度高但速度慢
prob.solve(solver=cvxpy.CLARABEL)

# 商业求解器（需要许可证）
prob.solve(solver=cvxpy.GUROBI)
```

**2. 预编译优化**:
```python
# 第一次编译比较慢，后续调用很快
prob.solve(verbose=False, warm_start=True)
```

**3. 矩阵预分配**:
```python
# 预分配矩阵避免重复分配内存
A = np.zeros((NX, NX))  # 预分配
B = np.zeros((NX, NU))
```

---

## 🎓 学习心得总结

### 核心理解

**MPC的本质**：
1. **预测**: 利用模型预测未来行为
2. **优化**: 在约束下寻找最优控制
3. **反馈**: 滚动优化提供闭环纠错

**关键技术点**：
1. **迭代线性化**: 提高非线性系统的控制精度
2. **热启动机制**: 保证实时性和控制连续性
3. **约束处理**: MPC相比传统方法的最大优势
4. **权重调节**: 平衡跟踪精度与控制代价

### 深度实践体会

**通过亲手编写MPC代码，我们获得了以下关键洞察**：

#### 1. 线性化的数学魔法
- **工作点的概念**：每个预测步都有不同的线性化工作点(v_t, φ_t, δ_t)
- **三角函数变常数**：sin(φ)和cos(φ)在特定φ值下变成常数，这是线性化的核心
- **C向量的必要性**：不仅仅是数学技巧，而是确保线性模型在工作点处与非线性模型完全匹配

#### 2. 优化变量的耦合性
- **整体优化思想**：u[0], u[1], ..., u[T-1]不是独立求解，而是作为一个整体同时优化
- **未来影响现在**：如果未来某步会违反约束，当前的决策就要提前调整
- **预测的价值**：这就是MPC相比反应式控制的核心优势

#### 3. 收敛与精度的权衡
- **迭代的必要性**：线性化轨迹与实际优化轨迹的差异需要通过迭代消除
- **热启动的威力**：从上次结果开始迭代，大幅减少收敛时间
- **实时性考量**：3次迭代通常足够平衡精度与计算时间

#### 4. 约束的物理意义
每个约束都对应现实世界的物理限制：
- 动力学约束：车辆运动规律不可违背
- 输入约束：执行器的物理限制
- 状态约束：安全和性能要求
- 变化率约束：平滑性和舒适性要求

### 与已学算法的联系

**MPC vs LQR**:
- LQR: 无限时域最优控制 + 线性系统
- MPC: 有限时域优化 + 可处理非线性和约束

**MPC vs Pure Pursuit**:
- Pure Pursuit: 基于几何的简单跟踪
- MPC: 基于优化的多约束跟踪

**能力递进**:
```
PID → LQR → MPC
↑      ↑      ↑
经典   现代   智能
```

### 实现中的关键发现

1. **矩阵维度的意义**：
   - `x.shape = (4, T+1)`：4维状态在T+1个时间点的轨迹
   - `u.shape = (2, T)`：2维控制在T个时间间隔的序列

2. **线性化的精度验证**：
   - 通过数值验证确保`A@x₀ + B@u₀ + C = f(x₀,u₀)`
   - 误差应该在机器精度范围内（< 1e-10）

3. **求解器的选择**：
   - CLARABEL：精度高但速度相对慢
   - SCS：速度快但精度一般
   - 实际应用需要根据实时性要求选择

4. **调试经验**：
   - 系统震荡 → 增加Rd权重（控制变化惩罚）
   - 跟踪精度不够 → 提高Q相对于R的权重
   - 求解失败 → 检查约束冲突和初值设置

### 🚨 最容易混淆的概念总结

**❌ 常见误解汇总**：

1. **"A, B矩阵是固定的"**
   - ✅ 正确：每个预测步都重新计算A, B矩阵
   - 原因：线性化工作点 $(v_t, \varphi_t, \delta_t)$ 在每个预测时刻都不同

2. **"线性化工作点是上一步线性化的结果"**
   - ✅ 正确：线性化工作点来自非线性模型的预测轨迹
   - 流程：非线性预测 → 在预测点线性化 → QP求解 → 迭代

3. **"MPC分别优化每个时刻的控制"**
   - ✅ 正确：MPC同时优化整个控制序列 u[0:T-1]
   - 意义：未来约束会影响当前决策

4. **"只执行第一个控制的原因是计算限制"**
   - ✅ 正确：是为了平衡预测价值与模型误差累积
   - 机制：滚动优化提供反馈校正

---

## 📚 进一步学习方向

### 1. 高级MPC算法

- **Nonlinear MPC**: 直接处理非线性模型
- **Robust MPC**: 考虑模型不确定性
- **Adaptive MPC**: 在线参数辨识
- **Distributed MPC**: 多智能体协调控制

### 2. 实际工程应用

- **实时实现**: 嵌入式MPC优化
- **模型辨识**: 数据驱动建模
- **状态估计**: 结合卡尔曼滤波
- **多目标优化**: 安全性与舒适性平衡

### 3. 相关理论

- **凸优化理论**: QP求解算法
- **最优控制**: 变分法与动态规划
- **鲁棒控制**: H∞与μ综合