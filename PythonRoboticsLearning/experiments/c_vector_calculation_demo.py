"""
C向量计算过程详细演示
"""
import numpy as np
import math

def demonstrate_c_vector_calculation():
    """演示C向量的详细计算过程"""
    print("=== C向量计算过程演示 ===")

    # 工作点数据
    x, y, v, yaw = 1.0, 2.0, 10.0, 0.5
    acceleration = 0.0
    steering_angle = 0.1
    dt = 0.1
    WB = 2.5

    print(f"工作点: x={x}, y={y}, v={v}, yaw={yaw}")
    print(f"控制输入: acceleration={acceleration}, steering_angle={steering_angle}")
    print(f"参数: dt={dt}, WB={WB}")

    print("\\n--- 第1步: 计算非线性模型输出 f(x₀, u₀) ---")

    # 非线性运动学方程
    x_next_nl = x + v * math.cos(yaw) * dt
    y_next_nl = y + v * math.sin(yaw) * dt
    v_next_nl = v + acceleration * dt
    yaw_next_nl = yaw + v * math.tan(steering_angle) * dt / WB

    f_nonlinear = np.array([x_next_nl, y_next_nl, v_next_nl, yaw_next_nl])

    print(f"x_next = {x} + {v} * cos({yaw}) * {dt} = {x_next_nl:.8f}")
    print(f"y_next = {y} + {v} * sin({yaw}) * {dt} = {y_next_nl:.8f}")
    print(f"v_next = {v} + {acceleration} * {dt} = {v_next_nl:.8f}")
    print(f"yaw_next = {yaw} + {v} * tan({steering_angle}) * {dt} / {WB} = {yaw_next_nl:.8f}")
    print(f"f(x₀,u₀) = {f_nonlinear}")

    print("\\n--- 第2步: 计算A矩阵 ---")

    # A矩阵（状态雅可比）
    A = np.eye(4)
    A[0,2] = math.cos(yaw) * dt
    A[0,3] = -v * math.sin(yaw) * dt
    A[1,2] = math.sin(yaw) * dt
    A[1,3] = v * math.cos(yaw) * dt
    A[3,2] = math.tan(steering_angle) * dt / WB

    print("A矩阵 (状态雅可比):")
    print(f"A[0,2] = cos({yaw}) * {dt} = {A[0,2]:.8f}")
    print(f"A[0,3] = -{v} * sin({yaw}) * {dt} = {A[0,3]:.8f}")
    print(f"A[1,2] = sin({yaw}) * {dt} = {A[1,2]:.8f}")
    print(f"A[1,3] = {v} * cos({yaw}) * {dt} = {A[1,3]:.8f}")
    print(f"A[3,2] = tan({steering_angle}) * {dt} / {WB} = {A[3,2]:.8f}")
    print("完整A矩阵:")
    print(A)

    print("\\n--- 第3步: 计算B矩阵 ---")

    # B矩阵（控制雅可比）
    B = np.zeros((4, 2))
    B[2,0] = dt
    B[3,1] = v * dt / (WB * math.cos(steering_angle) ** 2)

    print("B矩阵 (控制雅可比):")
    print(f"B[2,0] = {dt} = {B[2,0]:.8f}")
    print(f"B[3,1] = {v} * {dt} / ({WB} * cos²({steering_angle})) = {B[3,1]:.8f}")
    print("完整B矩阵:")
    print(B)

    print("\\n--- 第4步: 计算 A @ x₀ ---")

    x_current = np.array([x, y, v, yaw])
    Ax = A @ x_current

    print(f"当前状态向量: {x_current}")
    print(f"A @ x₀ = {Ax}")
    print("详细计算:")
    for i in range(4):
        components = [f"{A[i,j]:.6f}*{x_current[j]:.1f}" for j in range(4) if A[i,j] != 0]
        print(f"  (A @ x₀)[{i}] = {' + '.join(components)} = {Ax[i]:.8f}")

    print("\\n--- 第5步: 计算 B @ u₀ ---")

    u_ref = np.array([acceleration, steering_angle])
    Bu = B @ u_ref

    print(f"控制向量: {u_ref}")
    print(f"B @ u₀ = {Bu}")
    print("详细计算:")
    for i in range(4):
        components = [f"{B[i,j]:.6f}*{u_ref[j]:.1f}" for j in range(2) if B[i,j] != 0]
        if components:
            print(f"  (B @ u₀)[{i}] = {' + '.join(components)} = {Bu[i]:.8f}")
        else:
            print(f"  (B @ u₀)[{i}] = 0 = {Bu[i]:.8f}")

    print("\\n--- 第6步: 计算C = f(x₀,u₀) - A@x₀ - B@u₀ ---")

    C = f_nonlinear - Ax - Bu

    print("最终计算:")
    print(f"f(x₀,u₀) = {f_nonlinear}")
    print(f"A @ x₀   = {Ax}")
    print(f"B @ u₀   = {Bu}")
    print(f"C = f(x₀,u₀) - A@x₀ - B@u₀")

    for i in range(4):
        print(f"C[{i}] = {f_nonlinear[i]:.8f} - {Ax[i]:.8f} - {Bu[i]:.8f} = {C[i]:.8f}")

    print(f"\\n最终C向量: {C}")

    print("\\n--- 验证：线性化是否精确 ---")
    linear_result = Ax + Bu + C
    print(f"线性化结果: A@x₀ + B@u₀ + C = {linear_result}")
    print(f"非线性结果: f(x₀,u₀) = {f_nonlinear}")
    print(f"误差: {np.abs(linear_result - f_nonlinear)}")

    if np.allclose(linear_result, f_nonlinear, atol=1e-15):
        print("✅ 验证通过: 线性化在工作点处完全精确!")
    else:
        print("❌ 验证失败: 存在计算误差")

if __name__ == "__main__":
    demonstrate_c_vector_calculation()