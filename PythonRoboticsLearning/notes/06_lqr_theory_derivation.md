# LQR算法原理与推导：为什么核心是求解代数Riccati方程

> **作者**：学习笔记
> **日期**：2026-03-05
> **主题**：LQR (Linear Quadratic Regulator) 完整数学推导

---

## 📚 目录

1. [问题的提出](#1-问题的提出)
2. [矩阵运算基础](#2-矩阵运算基础)
3. [动态规划推导](#3-动态规划推导)
4. [代数Riccati方程](#4-代数riccati方程)
5. [核心要点总结](#5-核心要点总结)

---

## 1. 问题的提出

### 1.1 LQR问题描述

考虑线性离散系统：
$$\mathbf{x}[k+1] = \mathbf{A}\mathbf{x}[k] + \mathbf{B}\mathbf{u}[k]$$

代价函数（无限时域）：
$$J = \sum_{k=0}^{\infty} \left( \mathbf{x}[k]^T \mathbf{Q} \mathbf{x}[k] + \mathbf{u}[k]^T \mathbf{R} \mathbf{u}[k] \right)$$

**目标**：找到最优控制律 $\mathbf{u}^*[k]$ 使 $J$ 最小。

**目标层次1：找到最优控制序列**

给定初始状态 x[0]，找到控制序列：
{u*[0], u*[1], u*[2], ...} 使得 J = Σ(x^T Q x + u^T R u) 最小

**目标层次2：找到通用控制律（更重要！）**

找到一个反馈增益矩阵 K，使得对于任意初始状态 x[0]：
- 控制律：u*[k] = -K x[k]
- 都能实现最小代价



### 1.2 关键假设

1. **系统矩阵**：$(\mathbf{A}, \mathbf{B})$ 可控
2. **权重矩阵**：$\mathbf{Q} \succeq 0$（半正定），$\mathbf{R} \succ 0$（正定）
3. **时域**：无限时域（$k \to \infty$）
4. **线性**：系统动力学线性，代价函数二次

---

## 2. 矩阵运算基础

### 2.1 标量转置与矩阵等式

**核心问题**：为什么 `x^T A^T Q_f B u = u^T B^T Q_f A x`？

**解答步骤**：

#### 步骤1：维度分析证明结果为标量

设矩阵维度：
- x: n×1, A: m×n, B: m×p, Q_f: m×m, u: p×1

计算 `x^T A^T Q_f B u` 的维度：
```
x^T:           1×n
x^T A^T:       1×n × n×m = 1×m
x^T A^T Q_f:   1×m × m×m = 1×m
x^T A^T Q_f B: 1×m × m×p = 1×p
x^T A^T Q_f B u: 1×p × p×1 = 1×1  (标量)
```

#### 步骤2：标量转置性质

**关键性质**：任何标量 s 都满足 `s^T = s`

因此：
```
x^T A^T Q_f B u = (x^T A^T Q_f B u)^T  (标量转置等于自身)
```

#### 步骤3：转置链式法则

应用转置规则 `(ABC)^T = C^T B^T A^T`：
```
(x^T A^T Q_f B u)^T = u^T B^T Q_f^T (A^T)^T (x^T)^T
                     = u^T B^T Q_f^T A x    (因为 (A^T)^T = A, (x^T)^T = x)
```

#### 步骤4：对称矩阵性质

由于 Q_f 是对称矩阵：`Q_f^T = Q_f`

因此：
```
u^T B^T Q_f^T A x = u^T B^T Q_f A x
```

**结论**：
```
x^T A^T Q_f B u = u^T B^T Q_f A x  ✓
```

### 2.2 矩阵方程求解（矩阵"除法"）

#### 标量类比
- 标量方程：5x = 10
- 解法：x = 10/5 = 10 × (1/5) = 2

#### 矩阵方程
- 矩阵方程：A x = b
- **注意**：矩阵没有"除法"，只有逆矩阵
- 解法：x = A^(-1) b

#### 逆矩阵概念

**定义**：如果矩阵A有逆矩阵A^(-1)，那么：
```
A × A^(-1) = I  (单位矩阵)
A^(-1) × A = I
```

**单位矩阵**：矩阵世界的"1"
```
2×2单位矩阵：I = [1  0]
                [0  1]

3×3单位矩阵：I = [1  0  0]
                [0  1  0]
                [0  0  1]
```

#### 求解矩阵方程的关键规则

**重要**：必须**左乘**逆矩阵，因为矩阵乘法不满足交换律！

对于方程 `A x = b`：

```
原方程：         A x = b
两边左乘A^(-1)：  A^(-1) A x = A^(-1) b
化简：          I x = A^(-1) b         (因为 A^(-1) A = I)
最终结果：       x = A^(-1) b
```

#### LQR中的应用

LQR优化中的偏导方程：
```
2B^T P A x + 2(R + B^T P B) u = 0
```

整理为标准形式：
```
(R + B^T P B) u = -B^T P A x
```

应用矩阵方程求解：
```
u = -(R + B^T P B)^(-1) B^T P A x
```

---

## 3. 动态规划推导

### 3.1 值函数定义

定义值函数（从时刻 $k$ 到无穷大的最小代价）：
$$V(\mathbf{x}[k]) = \min_{\{\mathbf{u}[j]\}_{j=k}^{\infty}} \sum_{j=k}^{\infty} \left( \mathbf{x}[j]^T \mathbf{Q} \mathbf{x}[j] + \mathbf{u}[j]^T \mathbf{R} \mathbf{u}[j] \right)$$

### 3.2 Bellman最优性原理

**核心思想**：最优策略的任何子策略也是最优的。

数学表达：
$$V(\mathbf{x}[k]) = \min_{\mathbf{u}[k]} \left\{ \mathbf{x}[k]^T \mathbf{Q} \mathbf{x}[k] + \mathbf{u}[k]^T \mathbf{R} \mathbf{u}[k] + V(\mathbf{x}[k+1]) \right\}$$

其中：$\mathbf{x}[k+1] = \mathbf{A}\mathbf{x}[k] + \mathbf{B}\mathbf{u}[k]$

### 3.3 值函数的假设形式

**关键洞察**：对于线性系统和二次代价，值函数应该是二次的！

**这不是假设，而是数学推导的必然结果！**

假设：
$$V(\mathbf{x}) = \mathbf{x}^T \mathbf{P} \mathbf{x}$$

其中 $\mathbf{P}$ 是待求的正定矩阵。

### 3.4 为什么值函数必须是二次的：归纳法证明

**定理**：对于线性系统和二次代价，最优值函数必然具有二次形式。

**证明（有限时域归纳法）**：

**步骤1：终端时刻（k=N）**

终端代价：$V_N(\mathbf{x}[N]) = \mathbf{x}[N]^T \mathbf{Q}_f \mathbf{x}[N]$

显然：$V_N(\mathbf{x})$ 是二次函数 ✅

**步骤2：倒数第二步（k=N-1）**

Bellman方程：
$$V_{N-1}(\mathbf{x}) = \min_{\mathbf{u}} \left\{ \mathbf{x}^T \mathbf{Q} \mathbf{x} + \mathbf{u}^T \mathbf{R} \mathbf{u} + V_N(\mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}) \right\}$$

代入$V_N$：
$$V_{N-1}(\mathbf{x}) = \min_{\mathbf{u}} \left\{ \mathbf{x}^T \mathbf{Q} \mathbf{x} + \mathbf{u}^T \mathbf{R} \mathbf{u} + (\mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u})^T \mathbf{Q}_f (\mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}) \right\}$$

展开 $(\mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u})^T \mathbf{Q}_f (\mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u})$：
$$= \mathbf{x}^T \mathbf{A}^T \mathbf{Q}_f \mathbf{A} \mathbf{x} + 2\mathbf{u}^T \mathbf{B}^T \mathbf{Q}_f \mathbf{A} \mathbf{x} + \mathbf{u}^T \mathbf{B}^T \mathbf{Q}_f \mathbf{B} \mathbf{u}$$

（注：由于最终结果为标量，且 $\mathbf{Q}_f$ 为对称阵，交叉项 $\mathbf{x}^T \mathbf{A}^T \mathbf{Q}_f \mathbf{B} \mathbf{u} = \mathbf{u}^T \mathbf{B}^T \mathbf{Q}_f \mathbf{A} \mathbf{x}$。为方便应用分母布局对 $\mathbf{u}$ 求导，这里写为后者形式）

因此：
$$V_{N-1}(\mathbf{x}) = \min_{\mathbf{u}} \left\{ \mathbf{x}^T(\mathbf{Q} + \mathbf{A}^T \mathbf{Q}_f \mathbf{A})\mathbf{x} + 2\mathbf{u}^T \mathbf{B}^T \mathbf{Q}_f \mathbf{A} \mathbf{x} + \mathbf{u}^T(\mathbf{R} + \mathbf{B}^T \mathbf{Q}_f \mathbf{B})\mathbf{u} \right\}$$

这是关于$\mathbf{u}$的二次优化问题！

**重要：为什么可以通过求偏导得到全局最优？**

**必要条件验证**：

1. **函数可微性** ✅
   目标函数是$\mathbf{u}$的多项式，处处可微

2. **凸函数判断** ✅
   目标函数对$\mathbf{u}$的Hessian矩阵：
   $$\mathbf{H}_{uu} = \frac{\partial^2 J}{\partial \mathbf{u}^2} = 2(\mathbf{R} + \mathbf{B}^T \mathbf{Q}_f \mathbf{B})$$

   由于$\mathbf{R} \succ 0$（正定）且$\mathbf{Q}_f \succeq 0$（半正定），所以：
   $$\mathbf{H}_{uu} = 2(\mathbf{R} + \mathbf{B}^T \mathbf{Q}_f \mathbf{B}) \succ 0$$

   Hessian矩阵正定 → 目标函数严格凸

3. **最优性条件** ✅
   对于严格凸函数，一阶必要条件即是充分条件。下面给出严谨的**代数证明**（为什么求偏导为0就能保证是全局唯一最优解）。

   假设目标函数具有通用的二次多项式形式：
   $$J(\mathbf{u}) = \frac{1}{2}\mathbf{u}^T \mathbf{H} \mathbf{u} + \mathbf{c}^T \mathbf{u} + const$$
   
   其中 $\mathbf{H} = 2(\mathbf{R} + \mathbf{B}^T \mathbf{Q}_f \mathbf{B}) \succ 0$ 是对称正定矩阵，$\mathbf{c} = 2\mathbf{B}^T \mathbf{Q}_f \mathbf{A} \mathbf{x}$。

   **先通过求偏导数得到候选最优点 $\mathbf{u}^*$：**
   对目标函数求梯度并令其为0：
   $$\nabla_{\mathbf{u}} J = \mathbf{H} \mathbf{u} + \mathbf{c} = 0 \Rightarrow \mathbf{H} \mathbf{u}^* = -\mathbf{c}$$

   **证明“偏导为0即全局最优”：**
   设任意其他控制输入为 $\mathbf{u} = \mathbf{u}^* + \Delta \mathbf{u}$（其中 $\Delta \mathbf{u} \neq 0$）。
   将其代入目标函数 $J(\mathbf{u})$ 中展开：
   $$J(\mathbf{u}^* + \Delta \mathbf{u}) = \frac{1}{2}(\mathbf{u}^* + \Delta \mathbf{u})^T \mathbf{H} (\mathbf{u}^* + \Delta \mathbf{u}) + \mathbf{c}^T (\mathbf{u}^* + \Delta \mathbf{u})$$
   $$= \underbrace{\left( \frac{1}{2}{\mathbf{u}^*}^T \mathbf{H} \mathbf{u}^* + \mathbf{c}^T \mathbf{u}^* \right)}_{\text{正是 } J(\mathbf{u}^*)} + \frac{1}{2}{\mathbf{u}^*}^T \mathbf{H} \Delta \mathbf{u} + \frac{1}{2}{\Delta \mathbf{u}}^T \mathbf{H} \mathbf{u}^* + \frac{1}{2}{\Delta \mathbf{u}}^T \mathbf{H} \Delta \mathbf{u} + \mathbf{c}^T \Delta \mathbf{u}$$

   由于 $\mathbf{H}$ 是对称矩阵（$\mathbf{H}^T = \mathbf{H}$），可知 ${\mathbf{u}^*}^T \mathbf{H} \Delta \mathbf{u} = {\Delta \mathbf{u}}^T \mathbf{H} \mathbf{u}^*$。因此可以合并线性项：
   $$= J(\mathbf{u}^*) + \underbrace{({\mathbf{u}^*}^T \mathbf{H} + \mathbf{c}^T)}_{\text{此项为0}} \Delta \mathbf{u} + \frac{1}{2}{\Delta \mathbf{u}}^T \mathbf{H} \Delta \mathbf{u}$$

   因为 $\mathbf{H} \mathbf{u}^* = -\mathbf{c}$，所以转置后有 ${\mathbf{u}^*}^T \mathbf{H}^T = {\mathbf{u}^*}^T \mathbf{H} = -\mathbf{c}^T$，即 ${\mathbf{u}^*}^T \mathbf{H} + \mathbf{c}^T = 0$。中间包含 $\Delta \mathbf{u}$ 的线性项**完美抵消为0**。

   等式化简为：
   $$J(\mathbf{u}^* + \Delta \mathbf{u}) = J(\mathbf{u}^*) + \frac{1}{2}{\Delta \mathbf{u}}^T \mathbf{H} \Delta \mathbf{u}$$

   最后，利用**Hessian矩阵正定性**的条件：由于 $\mathbf{H} \succ 0$，对于任意非零向量 $\Delta \mathbf{u} \neq 0$，永远有 $\frac{1}{2}{\Delta \mathbf{u}}^T \mathbf{H} \Delta \mathbf{u} > 0$。

   因此严格得出结论：
   $$J(\mathbf{u}^* + \Delta \mathbf{u}) > J(\mathbf{u}^*)$$
   
   无论在 $\mathbf{u}^*$ 基础上作何微小偏移，代价 $J$ **一定会严格变大**。这就是为什么当 $\mathbf{H}$ 正定时，令一阶偏导=0算出的不仅是极值，而且是**唯一的全局最小值**。

   ![LQR全局最优性图解](lqr_optimality.png)

**因此可以对$\mathbf{u}$求偏导（采用分母布局）：**
$$\frac{\partial J}{\partial \mathbf{u}} = 2\mathbf{B}^T \mathbf{Q}_f \mathbf{A} \mathbf{x} + 2(\mathbf{R} + \mathbf{B}^T \mathbf{Q}_f \mathbf{B}) \mathbf{u} = 0$$

解得最优控制：
$$\mathbf{u}^* = -(\mathbf{R} + \mathbf{B}^T \mathbf{Q}_f \mathbf{B})^{-1} \mathbf{B}^T \mathbf{Q}_f \mathbf{A} \mathbf{x}$$

**将最优控制 $\mathbf{u}^*$ 代入回值函数：**
已知代价函数的表达式为：
$$V_{N-1}(\mathbf{x}) = \mathbf{x}^T(\mathbf{Q} + \mathbf{A}^T \mathbf{Q}_f \mathbf{A})\mathbf{x} + 2{\mathbf{u}^*}^T \mathbf{B}^T \mathbf{Q}_f \mathbf{A} \mathbf{x} + {\mathbf{u}^*}^T(\mathbf{R} + \mathbf{B}^T \mathbf{Q}_f \mathbf{B})\mathbf{u}^*$$

为了方便化简，我们观察先前的极值条件：
$$(\mathbf{R} + \mathbf{B}^T \mathbf{Q}_f \mathbf{B}) \mathbf{u}^* = -\mathbf{B}^T \mathbf{Q}_f \mathbf{A} \mathbf{x}$$

将其直接代入代价函数的第三项中：
$${\mathbf{u}^*}^T \left[ (\mathbf{R} + \mathbf{B}^T \mathbf{Q}_f \mathbf{B}) \mathbf{u}^* \right] = {\mathbf{u}^*}^T \left[ -\mathbf{B}^T \mathbf{Q}_f \mathbf{A} \mathbf{x} \right] = -{\mathbf{u}^*}^T \mathbf{B}^T \mathbf{Q}_f \mathbf{A} \mathbf{x}$$

这与第二项刚好可以部分抵消：
$$2{\mathbf{u}^*}^T \mathbf{B}^T \mathbf{Q}_f \mathbf{A} \mathbf{x} - {\mathbf{u}^*}^T \mathbf{B}^T \mathbf{Q}_f \mathbf{A} \mathbf{x} = {\mathbf{u}^*}^T \mathbf{B}^T \mathbf{Q}_f \mathbf{A} \mathbf{x}$$

此时 $V_{N-1}(\mathbf{x})$ 大大简化为：
$$V_{N-1}(\mathbf{x}) = \mathbf{x}^T(\mathbf{Q} + \mathbf{A}^T \mathbf{Q}_f \mathbf{A})\mathbf{x} + {\mathbf{u}^*}^T \mathbf{B}^T \mathbf{Q}_f \mathbf{A} \mathbf{x}$$

现在，将 $\mathbf{u}^*$ 表达式的转置代回（注意包含对称矩阵的转置运算规律：**逆矩阵的转置 = 转置矩阵的逆**）：
$${\mathbf{u}^*}^T = -\mathbf{x}^T \mathbf{A}^T \mathbf{Q}_f \mathbf{B} (\mathbf{R} + \mathbf{B}^T \mathbf{Q}_f \mathbf{B})^{-1}$$

代入得到：
$$V_{N-1}(\mathbf{x}) = \mathbf{x}^T(\mathbf{Q} + \mathbf{A}^T \mathbf{Q}_f \mathbf{A})\mathbf{x} - \mathbf{x}^T \mathbf{A}^T \mathbf{Q}_f \mathbf{B} (\mathbf{R} + \mathbf{B}^T \mathbf{Q}_f \mathbf{B})^{-1} \mathbf{B}^T \mathbf{Q}_f \mathbf{A} \mathbf{x}$$

提取首尾的 $\mathbf{x}^T$ 和 $\mathbf{x}$，即可表示为纯粹的二次型形式：
$$V_{N-1}(\mathbf{x}) = \mathbf{x}^T \mathbf{P}_{N-1} \mathbf{x}$$

其中核心矩阵 $\mathbf{P}_{N-1}$ 即为：
$$\mathbf{P}_{N-1} = \mathbf{Q} + \mathbf{A}^T \mathbf{Q}_f \mathbf{A} - \mathbf{A}^T \mathbf{Q}_f \mathbf{B}(\mathbf{R} + \mathbf{B}^T \mathbf{Q}_f \mathbf{B})^{-1} \mathbf{B}^T \mathbf{Q}_f \mathbf{A}$$

**结论**：$V_{N-1}(\mathbf{x})$ 也是二次函数！✅

**步骤3：数学归纳**

**归纳假设**：如果 $V_{k+1}(\mathbf{x}) = \mathbf{x}^T \mathbf{P}_{k+1} \mathbf{x}$ 是二次的

**证明** $V_k(\mathbf{x})$ 也是二次的：

$$V_k(\mathbf{x}) = \min_{\mathbf{u}} \left\{ \mathbf{x}^T \mathbf{Q} \mathbf{x} + \mathbf{u}^T \mathbf{R} \mathbf{u} + V_{k+1}(\mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}) \right\}$$
$$= \min_{\mathbf{u}} \left\{ \mathbf{x}^T \mathbf{Q} \mathbf{x} + \mathbf{u}^T \mathbf{R} \mathbf{u} + (\mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u})^T \mathbf{P}_{k+1} (\mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}) \right\}$$

按照同样的展开和优化过程，得到：
$$V_k(\mathbf{x}) = \mathbf{x}^T \mathbf{P}_k \mathbf{x}$$

其中 $\mathbf{P}_k$ 满足递推关系：
$$\mathbf{P}_k = \mathbf{Q} + \mathbf{A}^T \mathbf{P}_{k+1} \mathbf{A} - \mathbf{A}^T \mathbf{P}_{k+1} \mathbf{B}(\mathbf{R} + \mathbf{B}^T \mathbf{P}_{k+1} \mathbf{B})^{-1} \mathbf{B}^T \mathbf{P}_{k+1} \mathbf{A}$$

**归纳结论**：所有有限步的值函数都是二次的！

**推广到无限时域**：
当 $N \to \infty$ 且系统稳定时，$\mathbf{P}_k$ 收敛到常数矩阵 $\mathbf{P}$，满足：
$$\mathbf{P} = \mathbf{Q} + \mathbf{A}^T \mathbf{P} \mathbf{A} - \mathbf{A}^T \mathbf{P} \mathbf{B}(\mathbf{R} + \mathbf{B}^T \mathbf{P} \mathbf{B})^{-1} \mathbf{B}^T \mathbf{P} \mathbf{A}$$

这就是**离散代数Riccati方程**！□

**核心洞察**：二次形式不是假设，而是线性系统+二次代价的数学必然性！

---

## 4. 代数Riccati方程

### 4.1 从Bellman方程到目标函数

将值函数假设 $V(\mathbf{x}) = \mathbf{x}^T \mathbf{P} \mathbf{x}$ 代入Bellman方程，展开二次项：

$$J(\mathbf{u}) = \mathbf{x}^T (\mathbf{Q} + \mathbf{A}^T \mathbf{P} \mathbf{A}) \mathbf{x} + 2\mathbf{u}^T \mathbf{B}^T \mathbf{P} \mathbf{A} \mathbf{x} + \mathbf{u}^T (\mathbf{R} + \mathbf{B}^T \mathbf{P} \mathbf{B}) \mathbf{u}$$

### 4.2 最优控制求解

对 $J(\mathbf{u})$ 关于 $\mathbf{u}$ 求偏导（采用分母布局）：
$$\frac{\partial J}{\partial \mathbf{u}} = 2\mathbf{B}^T \mathbf{P} \mathbf{A} \mathbf{x} + 2(\mathbf{R} + \mathbf{B}^T \mathbf{P} \mathbf{B}) \mathbf{u}$$

令偏导数为零：
$$2\mathbf{B}^T \mathbf{P} \mathbf{A} \mathbf{x} + 2(\mathbf{R} + \mathbf{B}^T \mathbf{P} \mathbf{B}) \mathbf{u}^* = 0$$

解得最优控制律：
$$\mathbf{u}^* = -(\mathbf{R} + \mathbf{B}^T \mathbf{P} \mathbf{B})^{-1} \mathbf{B}^T \mathbf{P} \mathbf{A} \mathbf{x}$$

定义反馈增益矩阵：
$$\mathbf{K} = (\mathbf{R} + \mathbf{B}^T \mathbf{P} \mathbf{B})^{-1} \mathbf{B}^T \mathbf{P} \mathbf{A}$$

因此：
$$\mathbf{u}^* = -\mathbf{K} \mathbf{x}$$

### 5.2 最优值函数

将最优控制 $\mathbf{u}^* = -\mathbf{K} \mathbf{x}$ 代入目标函数。
在这之前，我们可以先利用极值的性质进行化简。已知的目标函数为：
$$J(\mathbf{u}) = \mathbf{x}^T (\mathbf{Q} + \mathbf{A}^T \mathbf{P} \mathbf{A}) \mathbf{x} + 2{\mathbf{u}^*}^T \mathbf{B}^T \mathbf{P} \mathbf{A} \mathbf{x} + {\mathbf{u}^*}^T (\mathbf{R} + \mathbf{B}^T \mathbf{P} \mathbf{B}) \mathbf{u}^*$$

利用驻点条件 $(\mathbf{R} + \mathbf{B}^T \mathbf{P} \mathbf{B}) \mathbf{u}^* = -\mathbf{B}^T \mathbf{P} \mathbf{A} \mathbf{x}$，将其代入最后一项：
$${\mathbf{u}^*}^T \left[ (\mathbf{R} + \mathbf{B}^T \mathbf{P} \mathbf{B}) \mathbf{u}^* \right] = -{\mathbf{u}^*}^T \mathbf{B}^T \mathbf{P} \mathbf{A} \mathbf{x}$$

此项与第二项部分抵消：
$$2{\mathbf{u}^*}^T \mathbf{B}^T \mathbf{P} \mathbf{A} \mathbf{x} - {\mathbf{u}^*}^T \mathbf{B}^T \mathbf{P} \mathbf{A} \mathbf{x} = {\mathbf{u}^*}^T \mathbf{B}^T \mathbf{P} \mathbf{A} \mathbf{x}$$

所以代入最优控制后的代价函数 $V(\mathbf{x})$ 简化为：
$$V(\mathbf{x}) = \mathbf{x}^T (\mathbf{Q} + \mathbf{A}^T \mathbf{P} \mathbf{A}) \mathbf{x} + {\mathbf{u}^*}^T \mathbf{B}^T \mathbf{P} \mathbf{A} \mathbf{x}$$

再将 ${\mathbf{u}^*}^T = -\mathbf{x}^T \mathbf{K}^T = -\mathbf{x}^T \mathbf{A}^T \mathbf{P} \mathbf{B} (\mathbf{R} + \mathbf{B}^T \mathbf{P} \mathbf{B})^{-1}$ 代回：
$$V(\mathbf{x}) = \mathbf{x}^T (\mathbf{Q} + \mathbf{A}^T \mathbf{P} \mathbf{A}) \mathbf{x} - \mathbf{x}^T \mathbf{A}^T \mathbf{P} \mathbf{B} (\mathbf{R} + \mathbf{B}^T \mathbf{P} \mathbf{B})^{-1} \mathbf{B}^T \mathbf{P} \mathbf{A} \mathbf{x}$$

提取公共项 $\mathbf{x}^T$ 和 $\mathbf{x}$，可以化简为：
$$V(\mathbf{x}) = \mathbf{x}^T \left[ \mathbf{Q} + \mathbf{A}^T \mathbf{P} \mathbf{A} - \mathbf{A}^T \mathbf{P} \mathbf{B} (\mathbf{R} + \mathbf{B}^T \mathbf{P} \mathbf{B})^{-1} \mathbf{B}^T \mathbf{P} \mathbf{A} \right] \mathbf{x}$$

### 4.3 代数Riccati方程（ARE）

由于 $V(\mathbf{x}) = \mathbf{x}^T \mathbf{P} \mathbf{x}$，系数必须相等：

$$\mathbf{P} = \mathbf{Q} + \mathbf{A}^T \mathbf{P} \mathbf{A} - \mathbf{A}^T \mathbf{P} \mathbf{B} (\mathbf{R} + \mathbf{B}^T \mathbf{P} \mathbf{B})^{-1} \mathbf{B}^T \mathbf{P} \mathbf{A}$$

这就是**离散代数Riccati方程**！

---

## 5. 核心要点总结

### 5.1 为什么ARE是LQR的核心？

1. **数学核心**：ARE将动态规划的无穷维优化问题转化为有限维代数问题
2. **计算核心**：求解ARE得到 P，直接计算最优反馈增益
3. **理论核心**：ARE的解保证了闭环系统的稳定性和最优性
4. **实现核心**：一次求解ARE，终身使用线性反馈控制

### 5.2 数学推导的精髓

**整个推导的逻辑链条**：

```
动态规划原理 → 值函数假设(二次) → Bellman方程 → 最优控制求解 → ARE推导 → 反馈增益计算
```

每一步都是数学上的必然结果，ARE不是人为构造的，而是从最优控制理论自然推导出来的结果！

**关键数学技巧总结**：
1. **标量转置等于自身**：解决矩阵等式问题
2. **对称矩阵性质**：简化转置运算
3. **逆矩阵左乘**：解决矩阵方程组
4. **二次型优化**：通过求偏导得到最优解

这就是为什么ARE是LQR算法的绝对核心 —— 它是连接"最优性"与"可计算性"的桥梁。

---

## 6. C++ 工程实现：笛卡尔坐标系下的 LQR

> 前面推导的是通用 LQR 理论。实际工程中，需要根据具体车辆模型构建 A、B 矩阵。本节以项目中 C++ 版本的割草机控制器为例，讲解"自行车模型 → 线性化 → 离散化 → LQR 查表"的完整过程。

### 6.1 自行车运动学模型（连续时间）

割草机简化为前轮转向的自行车模型，连续时间运动学方程为：

```
dx/dt    = v * cos(yaw)
dy/dt    = v * sin(yaw)
dyaw/dt  = v * tan(delta) / L
```

其中：
- `(x, y)` — 车辆后轴中心在世界坐标系下的位置
- `yaw` — 航向角
- `v` — 纵向线速度
- `delta` — 前轮转角
- `L` — 轴距（前后轴距离）

### 6.2 状态量与控制量的定义

C++ 版本选择**笛卡尔坐标系下的绝对位姿误差**作为状态量：

```
状态量 x = [e_x, e_y, e_theta]^T    (3维)

  e_x     = robot_x - target_x       位置x误差
  e_y     = robot_y - target_y       位置y误差
  e_theta = robot_yaw - target_yaw   航向角误差
```

控制量定义为 2 维：

```
控制量 u = [v, delta]^T    (2维：线速度, 前轮转角)
```

> **注意**：最终只取 u 的第二个分量 `u[1]`（即 delta 修正量）作为反馈输出。

### 6.3 线性化 + 离散化

在参考轨迹点 (refYaw, refDelta) 处线性化，并以时间步长 dt 进行欧拉离散化后得到：

**A 矩阵 (3×3)** — 状态转移：

```
A = [ 1    0   -v*dt*sin(refYaw) ]
    [ 0    1    v*dt*cos(refYaw) ]
    [ 0    0    1                ]
```

物理含义：
- 第一行：x 误差 = 上一步 x 误差 + 航向误差导致的 x 偏移（`-v*dt*sin(yaw)`）
- 第二行：y 误差 = 上一步 y 误差 + 航向误差导致的 y 偏移（`v*dt*cos(yaw)`）
- 第三行：角度误差自传递

**B 矩阵 (3×2)** — 控制输入：

```
B = [ dt*cos(refYaw)                         0                              ]
    [ dt*sin(refYaw)                         0                              ]
    [ dt*tan(refDelta)/L    v*dt/(L*cos(refDelta)^2)  ]
```

物理含义：
- 第一列：线速度 v 对位置误差的影响
- 第二列：转角 delta 对航向角误差导数的影响（对 `tan(delta)/L` 求偏导得到 `1/(L*cos^2(delta))`）

### 6.4 查表策略

因为 A、B 同时依赖 **refYaw** 和 **refDelta** 两个参数，所以 K 矩阵也依赖这两个参数。

离线预计算：
```
for delta in range(-89°, +89°, 1°):      # 179 个值
    for yaw in range(-180°, +180°, 1°):  # 361 个值
        A, B = build_state_space(delta, yaw, v=0.4)
        P = riccati_iteration(A, B, Q, R)
        K = -(R + B'PB)^{-1} * B'PA
        table[f"({delta},{yaw})"] = K     # 2×3 矩阵
```

总共 179 × 361 ≈ **64,000 条** K 矩阵，以 JSON 存储。

在线查表：

```cpp
// 1. 计算误差状态
X = [robot_x - target_x, robot_y - target_y, normalize(robot_yaw - target_yaw)]

// 2. 将参考点的 delta 和 yaw 取整为度数，构造 key
key = "(" + to_string((int)(refDelta*57.3)) + "," + to_string((int)(refYaw*57.3)) + ")"

// 3. O(1) 哈希查表
K = k_table[key]        // 2×3 矩阵

// 4. 计算反馈
u = K * X               // (2×3) × (3×1) = (2×1)
feedbackDelta = u[1]    // 取第二个分量
```

### 6.5 与教科书版（Frenet）的关键差异

| 对比项   | C++ 版（笛卡尔）              | 教科书版（Frenet）               |
| -------- | ----------------------------- | -------------------------------- |
| 坐标系   | 世界坐标系，误差含绝对航向    | 路径坐标系，航向已"旋转掉"       |
| 状态量   | `[e_x, e_y, e_theta]` 3维     | `[e, e_dot, th_e, th_e_dot]` 4维 |
| A,B 依赖 | refDelta + refYaw 两个参数    | 仅速度 v 一个参数                |
| 查表大小 | ~64,000 条（2维表）           | ~160 条（1维表）                 |
| 速度适应 | 固定 v=0.4 生成，不随速度变化 | K 随速度自适应                   |

---

## 7. 工程补偿技术：预瞄与滤波

> LQR 理论假设控制指令瞬间生效、输出任意平滑。实车中执行器有延迟、机械有抖动，需要额外补偿。

### 7.1 预瞄补偿（Lookahead / Prediction）

**问题**：控制器算出转角指令后，电机实际执行有延迟（通信 + 机械响应）。如果用"当前位置"找参考点，等指令生效时车已经向前走了一段距离。

**解决**：用"预测未来位置"代替当前位置来查找参考点。

```
预测x   = 当前x   + v * cos(yaw) * T_pred
预测y   = 当前y   + v * sin(yaw) * T_pred
预测yaw = 当前yaw + 角速度 * T_pred
```

其中 `T_pred` 为预瞄时间（项目中取 0.2 秒），应与执行器延迟时间匹配。

**效果**：
- 无预瞄：机器人持续"追赶"已经错过的参考点 → 过冲、振荡
- 有预瞄：机器人提前对准即将到达的参考点 → 跟踪更平滑

**类比**：开车时看前方 30 米而非看车头正下方。

### 7.2 输出低通滤波（Low-pass Filter）

**问题**：LQR 每步独立计算转角，相邻两步的输出可能跳变（如 5° → 15°），直接发给电机会导致机械冲击。

**解决**：对输出做一阶低通滤波（指数移动平均）。

```
filtered_delta = alpha * last_delta + (1 - alpha) * current_delta
```

其中 `alpha` 为滤波系数（项目中取 0.5）。

**参数取值的工程权衡**：

| alpha 值 | 效果                   | 适用场景         |
| -------- | ---------------------- | ---------------- |
| 0.0      | 无滤波，完全跟随计算值 | 仿真环境         |
| 0.3      | 轻度平滑               | 响应要求高的场景 |
| **0.5**  | 中度平滑（项目当前值） | 一般割草场景     |
| 0.7      | 重度平滑，响应明显变慢 | 高速行驶         |
| 1.0      | 完全锁死，输出不变     | 无意义           |

### 7.3 完整控制输出流程

将 LQR 核心、前馈、预瞄、滤波组合起来，实际控制器的完整数据流为：

```
1. 预瞄：   predictedPose = predict(当前位姿, v, w, T_pred=0.2s)
2. 找参考点：targetIdx = findNearest(predictedPose, 参考轨迹)
3. 前馈：   refDelta = atan(L * kappa)
4. 反馈：   fbDelta  = LQR_lookup(refDelta, refYaw, 误差状态)
5. 合成：   rawDelta = refDelta + fbDelta
6. 滤波：   smoothDelta = 0.5 * lastDelta + 0.5 * rawDelta
7. 限幅：   finalDelta = clamp(smoothDelta, -30°, +30°)
8. 输出：   angular_z = v * tan(finalDelta) / L
```

前馈保证稳态跟踪（弯道上曲率对应的转角），反馈消除瞬态误差，滤波保护机械结构。