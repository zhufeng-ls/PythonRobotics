
# B样条路径规划详解

## 概述

B样条（B-Spline）是一种强大的曲线拟合和路径规划方法，广泛应用于机器人路径规划、计算机图形学和CAD系统中。

## 文件说明

- [`bspline_path.py`](bspline_path.py:1) - 原始B样条实现
- [`bspline_path_demo_detailed.py`](bspline_path_demo_detailed.py:1) - **交互式详细演示（推荐）**
- `README_BSPLINE.md` - 本文档

## 运行演示

```bash
cd PathPlanning/BSplinePath
python3 bspline_path_demo_detailed.py
```

## B样条基础

### 什么是B样条？

B样条是一种**分段多项式曲线**，具有以下特点：

1. **局部控制**：移动一个控制点只影响局部曲线
2. **平滑性**：具有高阶连续性（C^k连续）
3. **灵活性**：可以通过调整参数控制曲线形状
4. **数值稳定**：计算稳定，不会出现龙格现象

### 数学定义

B样条曲线定义为：

```
C(u) = Σ P_i * N_{i,k}(u)
```

其中：
- `P_i` 是控制点（路径点）
- `N_{i,k}(u)` 是k阶B样条基函数
- `u` 是参数（通常归一化到[0,1]）

## 核心算法解析

### 1. 距离向量计算

[`_calc_distance_vector()`](bspline_path.py:98-103)函数计算路径点之间的累积距离：

```python
def _calc_distance_vector(x, y):
    dx, dy = np.diff(x), np.diff(y)
    distances = np.cumsum([np.hypot(idx, idy) 
                          for idx, idy in zip(dx, dy)])
    distances = np.concatenate(([0.0], distances))
    distances /= distances[-1]  # 归一化到[0,1]
    return distances
```

**作用**：
- 计算相邻路径点之间的欧氏距离
- 累积距离作为参数化的基础
- 归一化使参数范围统一

### 2. 样条创建

使用scipy的[`UnivariateSpline`](bspline_path.py:59-60)创建B样条：

```python
spl_i_x = interpolate.UnivariateSpline(distances, x, k=degree, s=s)
spl_i_y = interpolate.UnivariateSpline(distances, y, k=degree, s=s)
```

**参数说明**：
- `distances`: 参数向量（归一化距离）
- `x, y`: 控制点坐标
- `k`: B样条阶数（degree）
- `s`: 平滑参数

### 3. 样条评估

[`_evaluate_spline()`](bspline_path.py:106-115)函数计算路径属性：

```python
def _evaluate_spline(sampled, spl_i_x, spl_i_y):
    # 位置
    x = spl_i_x(sampled)
    y = spl_i_y(sampled)
    
    # 一阶导数（速度/切向量）
    dx = spl_i_x.derivative(1)(sampled)
    dy = spl_i_y.derivative(1)(sampled)
    heading = np.arctan2(dy, dx)
    
    # 二阶导数（加速度）
    ddx = spl_i_x.derivative(2)(sampled)
    ddy = spl_i_y.derivative(2)(sampled)
    
    # 曲率计算
    curvature = (ddy * dx - ddx * dy) / (dx² + dy²)^(3/2)
    
    return x, y, heading, curvature
```

## 两种模式对比

### 1. 近似模式（Approximation）

[`approximate_b_spline_path()`](bspline_path.py:19-63)函数实现：

```python
def approximate_b_spline_path(x, y, n_path_points, degree=3, s=None):
    # s > 0: 允许曲线偏离控制点
    spl_i_x = interpolate.UnivariateSpline(distances, x, k=degree, s=s)
    spl_i_y = interpolate.UnivariateSpline(distances, y, k=degree, s=s)
```

**特点**：
- ✅ 曲线**接近**但不一定通过控制点
- ✅ 更加**平滑**
- ✅ 对噪声数据**鲁棒**
- ✅ 适合**轨迹优化**
- ❌ 可能偏离原始路径点

**适用场景**：
- 路径点有测量噪声
- 需要非常平滑的轨迹
- 可以容忍小的位置偏差

### 2. 插值模式（Interpolation）

[`interpolate_b_spline_path()`](bspline_path.py:66-95)函数实现：

```python
def interpolate_b_spline_path(x, y, n_path_points, degree=3):
    # s = 0: 强制曲线通过所有控制点
    return approximate_b_spline_path(x, y, n_path_points, degree, s=0.0)
```

**特点**：
- ✅ 曲线**精确通过**所有控制点
- ✅ 路径**准确**
- ✅ 适合**精确导航**
- ❌ 可能不够平滑
- ❌ 对噪声敏感

**适用场景**：
- 必须经过特定位置
- 路径点是精确的
- 对平滑性要求不高

## 关键参数详解

### 1. 阶数（Degree, k）

B样条的多项式阶数，范围：2 ≤ k ≤ 5

| 阶数 | 名称 | 连续性 | 特点 |
|------|------|--------|------|
| k=2 | 二次 | C¹ | 简单，不够平滑 |
| k=3 | 三次 | C² | **默认**，平滑加速度 |
| k=4 | 四次 | C³ | 很平滑，计算稍慢 |
| k=5 | 五次 | C⁴ | 极度平滑，计算较慢 |

**连续性说明**：
- C⁰：位置连续
- C¹：速度连续（切线连续）
- C²：加速度连续（曲率连续）
- C³：加加速度连续

**推荐**：
- 一般使用 **k=3**（三次B样条）
- 需要极度平滑时使用 k=4 或 k=5

### 2. 平滑参数（Smoothing, s）

控制曲线对控制点的拟合程度：

| s值 | 效果 | 应用 |
|-----|------|------|
| s=0 | 插值（精确通过点） | 精确导航 |
| 0<s<1 | 轻微平滑 | 一般路径规划 |
| s≥1 | 强平滑 | 噪声数据处理 |

**选择建议**：
```python
# 精确路径
s = 0.0

# 一般平滑
s = 0.5

# 强平滑（噪声数据）
s = 1.0 - 2.0
```

### 3. 采样点数（n_path_points）

生成路径的点数：

- **太少**（<20）：路径不够精细，可能丢失细节
- **适中**（50-100）：平衡精度和计算量
- **太多**（>200）：计算量大，但精度提升有限

**推荐**：
- 短路径：50点
- 中等路径：100点
- 长路径：150-200点

## 曲率分析

### 曲率公式

曲率κ衡量曲线的弯曲程度：

```
κ = (x'y'' - y'x'') / (x'² + y'²)^(3/2)
```

其中：
- x', y' 是一阶导数（速度）
- x'', y'' 是二阶导数（加速度）

### 曲率的意义

- **κ = 0**：直线
- **κ > 0**：左转
- **κ < 0**：右转
- **|κ| 大**：急转弯
- **|κ| 小**：缓转弯

### 最小转弯半径

```
R_min = 1 / |κ_max|
```

这对于机器人路径规划很重要，因为：
- 车辆有最小转弯半径限制
- 高速运动时需要更大的转弯半径

## 交互式演示功能

### 界面布局

1. **主绘图区域**（左侧）
   - 显示B样条路径
   - 路径点可拖动
   - 显示方向箭头

2. **曲率图**（底部）
   - 显示沿路径的曲率变化
   - 高亮高曲率区域

3. **信息面板**（右上）
   - 路径统计信息
   - 曲率分析
   - 操作说明

4. **参数面板**（右下）
   - 算法原理说明
   - 参数解释

### 交互操作

1. **拖动路径点**
   - 点击并拖动绿色圆点
   - 实时更新路径

2. **添加路径点**
   - 在空白处点击
   - 自动添加新路径点

3. **调整参数**
   - 阶数滑块：改变多项式阶数
   - 平滑滑块：调整平滑程度
   - 点数滑块：改变采样密度

4. **切换模式**
   - 近似模式：红色路径
   - 插值模式：蓝色路径

5. **重置**
   - 点击"Reset Waypoints"恢复默认

## 实际应用

### 1. 机器人路径规划

```python
# 给定路径点
waypoints_x = [0, 5, 10, 15, 20]
waypoints_y = [0, 3, 2, 5, 4]

# 生成平滑路径
path_x, path_y, heading, curvature = approximate_b_spline_path(
    waypoints_x, waypoints_y, 
    n_path_points=100,
    degree=3,
    s=0.5  # 适度平滑
)

# 检查曲率约束
max_curvature = np.max(np.abs(curvature))
min_radius = 1.0 / max_curvature
print(f"Minimum turning radius: {min_radius:.2f} m")
```

### 2. 无人机航迹规划

```python
# 精确通过检查点
checkpoint_x = [0, 10, 20, 30]
checkpoint_y = [0, 5, 10, 5]

# 使用插值确保通过所有检查点
path_x, path_y, heading, curvature = interpolate_b_spline_path(
    checkpoint_x, checkpoint_y,
    n_path_points=150,
    degree=4  # 更高阶以获得更平滑的轨迹
)
```

### 3. 轨迹优化

```python
# 有噪声的测量数据
noisy_x = measured_x + np.random.normal(0, 0.1, len(measured_x))
noisy_y = measured_y + np.random.normal(0, 0.1, len(measured_y))

# 使用高平滑参数过滤噪声
smooth_x, smooth_y, _, _ = approximate_b_spline_path(
    noisy_x, noisy_y,
    n_path_points=100,
    degree=3,
    s=2.0  # 强平滑以去除噪声
)
```

## 优缺点分析

### 优点

1. **平滑性好**
   - 高阶连续性（C^k）
   - 无尖角和突变

2. **局部控制**
   - 修改一个控制点只影响局部
   - 便于交互式编辑

3. **灵活性高**
   - 可调整阶数和平滑度
   - 适应不同应用需求

4. **数值稳定**
   - 不会出现龙格现象
   - 计算稳定可靠

5. **易于计算导数**
   - 可直接计算速度、加速度
   - 便于动力学分析

### 缺点

1. **不保证避障**
   - 可能穿过障碍物
   - 需要额外的碰撞检测

2. **全局最优性**
   - 不保证路径最短
   - 可能需要后处理优化

3. **参数选择**
   - 需要调整多个参数
   - 不同场景需要不同设置

4. **计算复杂度**
   - 高阶B样条计算较慢
   - 实时性可能受影响

## 与其他方法对比

| 方法 | 平滑性 | 精确性 | 计算速度 | 适用场景 |
|------|--------|--------|----------|----------|
| B样条 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ | 轨迹优化 |
|