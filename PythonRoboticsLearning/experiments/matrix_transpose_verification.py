#!/usr/bin/env python3
"""
验证LQR理论中的矩阵转置等式
x^T A^T Q_f B u = u^T B^T Q_f A x

作者：学习笔记
日期：2026-03-06
目的：数值验证对称矩阵的转置性质
"""

import numpy as np

def verify_transpose_equality():
    """验证两个矩阵表达式的相等性"""

    # 设置随机种子以便重现结果
    np.random.seed(42)

    # 定义矩阵维度
    n, m, p = 3, 4, 2
    print(f"矩阵维度设置：")
    print(f"  x: {n}×1 (状态向量)")
    print(f"  A: {m}×{n} (系统矩阵)")
    print(f"  B: {m}×{p} (输入矩阵)")
    print(f"  Q_f: {m}×{m} (权重矩阵)")
    print(f"  u: {p}×1 (控制输入)")
    print()

    # 生成随机矩阵
    x = np.random.randn(n, 1)
    A = np.random.randn(m, n)
    B = np.random.randn(m, p)
    u = np.random.randn(p, 1)

    # 生成对称矩阵 Q_f
    Q_temp = np.random.randn(m, m)
    Q_f = Q_temp + Q_temp.T  # 确保对称

    print("验证 Q_f 是否对称：")
    print(f"  Q_f - Q_f^T 的最大值: {np.max(np.abs(Q_f - Q_f.T)):.2e}")
    print()

    # 计算两个表达式
    print("计算过程：")

    # 表达式1: x^T A^T Q_f B u
    print("表达式1: x^T A^T Q_f B u")
    step1_1 = x.T @ A.T
    print(f"  x^T A^T 维度: {step1_1.shape}")
    step1_2 = step1_1 @ Q_f
    print(f"  x^T A^T Q_f 维度: {step1_2.shape}")
    step1_3 = step1_2 @ B
    print(f"  x^T A^T Q_f B 维度: {step1_3.shape}")
    expr1 = step1_3 @ u
    print(f"  最终结果维度: {expr1.shape}")
    print(f"  结果值: {expr1[0, 0]:.6f}")
    print()

    # 表达式2: u^T B^T Q_f A x
    print("表达式2: u^T B^T Q_f A x")
    step2_1 = u.T @ B.T
    print(f"  u^T B^T 维度: {step2_1.shape}")
    step2_2 = step2_1 @ Q_f
    print(f"  u^T B^T Q_f 维度: {step2_2.shape}")
    step2_3 = step2_2 @ A
    print(f"  u^T B^T Q_f A 维度: {step2_3.shape}")
    expr2 = step2_3 @ x
    print(f"  最终结果维度: {expr2.shape}")
    print(f"  结果值: {expr2[0, 0]:.6f}")
    print()

    # 验证相等性
    difference = np.abs(expr1[0, 0] - expr2[0, 0])
    is_equal = np.isclose(expr1, expr2)

    print("=" * 50)
    print("验证结果：")
    print(f"  表达式1结果: {expr1[0, 0]:.10f}")
    print(f"  表达式2结果: {expr2[0, 0]:.10f}")
    print(f"  差值: {difference:.2e}")
    print(f"  是否相等: {is_equal[0, 0]}")
    print("=" * 50)

    return expr1[0, 0], expr2[0, 0], difference

def demonstrate_scalar_transpose():
    """演示标量转置等于自身的性质"""
    print("\n标量转置性质演示：")

    # 创建一个标量（1x1矩阵）
    scalar_matrix = np.array([[3.14159]])
    scalar_value = 3.14159

    print(f"标量矩阵: {scalar_matrix}")
    print(f"其转置: {scalar_matrix.T}")
    print(f"相等？ {np.array_equal(scalar_matrix, scalar_matrix.T)}")
    print(f"标量值: {scalar_value}")
    print(f"标量转置就是自身: {scalar_value} == {scalar_value}")

def test_with_different_dimensions():
    """测试不同维度下的情况"""
    print("\n\n测试不同维度：")

    test_cases = [
        (2, 3, 1),  # 小规模
        (5, 4, 3),  # 中等规模
        (10, 8, 5)  # 较大规模
    ]

    for i, (n, m, p) in enumerate(test_cases):
        print(f"\n测试案例 {i+1}: n={n}, m={m}, p={p}")

        # 重新设置种子确保一致性
        np.random.seed(42 + i)

        x = np.random.randn(n, 1)
        A = np.random.randn(m, n)
        B = np.random.randn(m, p)
        u = np.random.randn(p, 1)
        Q_temp = np.random.randn(m, m)
        Q_f = Q_temp + Q_temp.T

        expr1 = x.T @ A.T @ Q_f @ B @ u
        expr2 = u.T @ B.T @ Q_f @ A @ x

        difference = np.abs(expr1[0, 0] - expr2[0, 0])

        print(f"  表达式1: {expr1[0, 0]:.6f}")
        print(f"  表达式2: {expr2[0, 0]:.6f}")
        print(f"  差值: {difference:.2e}")
        print(f"  相等？ {np.isclose(expr1, expr2)[0, 0]}")

if __name__ == "__main__":
    print("LQR理论中矩阵转置等式的数值验证")
    print("=" * 60)

    # 主要验证
    verify_transpose_equality()

    # 标量转置演示
    demonstrate_scalar_transpose()

    # 不同维度测试
    test_with_different_dimensions()

    print("\n验证完成！所有测试都证实了理论结果的正确性。")