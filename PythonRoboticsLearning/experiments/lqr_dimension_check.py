import numpy as np

print("=== LQR 矩阵运算维度验证 ===")

# 1. 从代码 lqr_steer_control.py 中提取的维度
n = 4  # 状态维度 (x)
m = 1  # 控制维度 (u)

print(f"状态维度 n = {n} (e, e_dot, th_e, th_e_dot)")
print(f"控制维度 m = {m} (delta)")

# 2. 模拟代码中的矩阵定义
# Q = np.eye(4) (Line 32)
Q = np.eye(n)
# R = np.eye(1) (Line 35)
R = np.eye(m)
# A: 4x4 (Line 176)
A = np.random.rand(n, n)
# B: 4x1 (Line 184)
B = np.random.rand(n, m)
# P (代码中的 X) 也是 4x4 (Line 113: X = Q)
P = np.random.rand(n, n)
# 确保 P 是对称的 (LQR中的P必然对称)
P = (P + P.T) / 2

# 3. 定义状态向量 x 和控制向量 u
x = np.random.rand(n, 1) # 4x1
u = np.random.rand(m, 1) # 1x1

print("\n[矩阵形状确认]")
print(f"x: {x.shape}")
print(f"u: {u.shape}")
print(f"A: {A.shape}")
print(f"B: {B.shape}")
print(f"P: {P.shape}")

# 4. 逐步验证 x^T A^T P B u 的运算过程
print("\n[运算链式追踪: x^T A^T P B u]")

res = x.T
print(f"Step 1: x.T               -> {res.shape} (行向量)")

res = res @ A.T
print(f"Step 2: x.T @ A.T         -> {res.shape} (1x4 * 4x4 -> 1x4)")

res = res @ P
print(f"Step 3: ... @ P           -> {res.shape} (1x4 * 4x4 -> 1x4)")

res = res @ B
print(f"Step 4: ... @ B           -> {res.shape} (1x4 * 4x1 -> 1x1) <--- 关键！变成 1x1 了")

res = res @ u
print(f"Step 5: ... @ u           -> {res.shape} (1x1 * 1x1 -> 1x1)")

val1 = res[0,0]
print(f"最终结果值: {val1:.6f}")

# 5. 验证转置项 u^T B^T P A x
print("\n[验证转置相等性]")
term2 = u.T @ B.T @ P @ A @ x
val2 = term2[0,0]

print(f"项 1 (x^T A^T P B u) = {val1:.6f}")
print(f"项 2 (u^T B^T P A x) = {val2:.6f}")
print(f"两者是否相等? {abs(val1 - val2) < 1e-10}")

if abs(val1 - val2) < 1e-10:
    print("结论: 证实它们是同一个标量，所以在求导时可以合并。")
