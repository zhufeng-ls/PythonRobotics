# 为什么 LQR 的值函数一定是二次的？

> 核心结论：**线性代换不改变多项式的次数** —— 这是整个结论的数学根基。

---

## 一句话直觉

代价是二次的，系统是线性的。线性的东西代入二次函数，结果还是二次的 —— **永远不会"升次"**。

---

## 三层递进理解

### 第 1 层：代价函数本身就是二次的

$$\text{状态代价: } \mathbf{x}^T \mathbf{Q} \mathbf{x} \quad\quad \text{控制代价: } \mathbf{u}^T \mathbf{R} \mathbf{u}$$

起点就是二次的，这为后面"保持二次"奠定了基础。

### 第 2 层：线性代换不升次

系统方程 $\mathbf{x}[k+1] = \mathbf{A}\mathbf{x}[k] + \mathbf{B}\mathbf{u}[k]$ 是**线性**的。

把它代入二次的 $V(\mathbf{x}[k+1])$：

$$V(\mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}) = (\mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u})^T \mathbf{P} (\mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u})$$

展开后只有 $\mathbf{x}^T(\cdots)\mathbf{x}$、$\mathbf{x}^T(\cdots)\mathbf{u}$、$\mathbf{u}^T(\cdots)\mathbf{u}$ 这些**二次项**，不会冒出三次项或更高次项。

> **类比**：把 $y = 2x + 3$ 代入 $f(y) = y^2$，得到 $f = 4x^2 + 12x + 9$，还是二次的！

### 第 3 层：最优化过程也保持二次

对关于 $\mathbf{u}$ 的二次函数求最小值（令 $\partial/\partial \mathbf{u} = 0$），解出的最优控制 $\mathbf{u}^* = -\mathbf{K}\mathbf{x}$ 是 $\mathbf{x}$ 的**线性函数**。

把线性的 $\mathbf{u}^*$ 代回二次的代价 → 结果仍然是 $\mathbf{x}$ 的二次函数。

![多项式次数保持原理](quadratic_preservation_1772766594909.png)

---

## 归纳法推理链

每一步 Bellman 递推都经历同样的过程，二次性层层传递：

![归纳法流程图](induction_flowchart_1772767392639.png)

```
终端: V_N(x) = x^T Q_f x                 ← 二次 ✓
       ↓ Bellman方程 + 线性代换
倒数第二步: 关于(x,u)的二次函数
       ↓ 对u求最优 → u* = -Kx (线性)
V_{N-1}(x) = x^T P_{N-1} x               ← 还是二次 ✓
       ↓ 同样过程重复...
V_k(x) = x^T P_k x                       ← 永远二次 ✓
       ↓ N→∞, P_k 收敛
V(x) = x^T P x, 其中 P 满足 Riccati 方程  ← 最终结论
```

### 数值示例（1维系统）

取 $x[k+1] = 0.8x[k] + u[k]$，$Q=1$，$R=0.5$，$Q_f=2$，倒推 5 步：

| 步骤 | $P_k$ | 值函数形式 |
|------|--------|-----------|
| k=5 (终端) | 2.000 | $V_5(x) = 2.000 \cdot x^2$ |
| k=4 | 1.980 | $V_4(x) = 1.980 \cdot x^2$ |
| k=3 | 1.979 | $V_3(x) = 1.979 \cdot x^2$ |
| k=2 | 1.979 | $V_2(x) = 1.979 \cdot x^2$ |
| k=1 | 1.979 | $V_1(x) = 1.979 \cdot x^2$ |
| k=0 | 1.979 | $V_0(x) = 1.979 \cdot x^2$ |

**每一步都是完美的抛物线**，只是系数 $P_k$ 在变化，且快速收敛。

![数值归纳示例](lqr_quadratic_induction.png)

---

## 反面理解：什么时候会不是二次的？

打破**任何一个条件**，二次性就不成立：

![三种情况对比](quadratic_conditions_1772766953076.png)

| 条件 | 打破后的结果 | 原因 |
|------|-------------|------|
| **系统线性** → 变成非线性 | 值函数变成高次/非解析 | 非线性代换 $f(\mathbf{x})$ 会产生高次项 |
| **代价二次** → 变成高次 | 值函数也变成高次 | 起点就不是二次的 |
| **控制无约束** → 加约束 | 值函数变成分段二次 | $\mathbf{u}^*$ 被"截断"，不再是 $\mathbf{x}$ 的线性函数 |

![数值对比图](lqr_quadratic_comparison.png)

---

## 总结

$$\boxed{\text{线性代换} + \text{二次函数} \xrightarrow{\text{代入}} \text{还是二次函数}}$$

这是纯粹的**代数事实**，不是假设，不是近似，而是数学运算的必然结果。
