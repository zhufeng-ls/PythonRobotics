# Pure Pursuit 路径跟踪算法学习笔记

Pure Pursuit（纯追踪）是一种经典的几何路径跟踪算法。它的核心思想是基于车辆的几何模型，寻找路径上的一个前瞻点（Look-ahead Point），并控制车辆通过一条圆弧到达该点。

## 1. 核心原理

Pure Pursuit 建立在自行车模型（Bicycle Model）的基础上。假设车辆后轴中心为控制参考点。

### 1.1 几何推导（直观方法）
最直观的推导方法是将坐标系建立在**当前车辆**上：
- 设**后轴中心**为原点 $(0, 0)$。
- 车辆航向（Heading）指向 **$x$ 轴**正方向。
- 目标点（前瞻点） $G$ 的坐标为 $(x, y)$。

由于 $G$ 距离后轴中心的距离为 $L_f$，根据三角函数：
- $x = L_f \cos \alpha$
- $y = L_f \sin \alpha$
（其中 $\alpha$ 是航向与目标点的夹角）

我们希望车辆走一个半径为 $R$ 的圆弧经过点 $G$。这个圆的圆心一定在 $y$ 轴上（因为圆弧在原点处切线必须与车辆航向 $x$ 轴平行）。
设圆心坐标为 $(0, R)$。圆的方程为：
$$x^2 + (y - R)^2 = R^2$$

将 $G$ 点坐标代入方程：
$$(L_f \cos \alpha)^2 + (L_f \sin \alpha - R)^2 = R^2$$
展开并化简：
$$L_f^2 \cos^2 \alpha + L_f^2 \sin^2 \alpha - 2 R L_f \sin \alpha + R^2 = R^2$$
$$L_f^2 - 2 R L_f \sin \alpha = 0$$
解得曲率 $\kappa$（即 $1/R$）：
$$\kappa = \frac{1}{R} = \frac{2 \sin \alpha}{L_f}$$

根据自行车模型，转向角 $\delta$ 与半径 $R$ 的关系为 $\tan \delta = L / R$，所以：
$$\delta = \arctan\left(\frac{2L \sin \alpha}{L_f}\right)$$

## 2. 关键参数

Pure Pursuit 的表现很大程度上取决于**前瞻距离 $L_f$**：

1.  **$L_f$ 过小**：车辆对路径变化非常敏感，容易产生震荡（Oscillation）。
2.  **$L_f$ 过大**：跟踪平滑，但在弯道处会有较大的横向误差（Cutting corners）。
3.  **动态前瞻距离**：通常将 $L_f$ 设置为车速的线性函数：
    $$L_f = k \cdot v + L_{fc}$$
    - $k$：前瞻增益。
    - $L_{fc}$：最小前瞻距离。

## 3. 算法步骤

1.  获取当前车辆状态（位置 $x, y$，航向角 $yaw$，速度 $v$）。
2.  在预设路径中寻找距离当前后轴中心最近的点。
3.  从最近点开始，向路径前方搜索，找到第一个距离大于 $L_f$ 的点作为**目标前瞻点**。
4.  利用公式 $\delta = \arctan\left(\frac{2L \sin\alpha}{L_f}\right)$ 计算转向角。
5.  更新车辆状态，循环执行。

## 4. 代码实现分析 (PythonRobotics)

在 `PathTracking/pure_pursuit/pure_pursuit.py` 中，核心控制器逻辑如下：

```python
def pure_pursuit_steer_control(state, trajectory, pind):
    # 1. 搜索目标点索引
    ind, Lf = trajectory.search_target_index(state)

    # 考虑到搜索效率，pind 是上一步的索引
    if pind >= ind:
        ind = pind

    # 2. 获取目标点坐标
    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    # 3. 计算 alpha (目标点相对于当前航向的角度)
    # state.yaw 是车辆当前航向
    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    # 4. 计算转向角 delta
    # WB 是轴距 (Wheel Base)
    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind
```

## 5. 总结

| 特性 | 说明 |
| :--- | :--- |
| **优点** | 实现简单，计算量小，鲁棒性好，适合中低速场景。 |
| **缺点** | 依赖前瞻距离的选取，高速时过弯能力受限，不考虑车辆动力学模型。 |
| **适用场景** | 移动机器人、物流小车、自动导航的基础跟踪算法。 |
