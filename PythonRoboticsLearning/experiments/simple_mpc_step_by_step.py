"""
Simple MPC Controller - Step by Step Implementation
逐步构建MPC控制器，加深对算法原理的理解

Author: Learning Project
Date: 2026-02-27
"""

import numpy as np
import matplotlib.pyplot as plt
import math
from dataclasses import dataclass
from typing import Tuple, List, Optional

# ============================================================================
# 第1步：车辆运动学模型
# ============================================================================

@dataclass
class VehicleState:
    """车辆状态类"""
    x: float = 0.0      # x位置 (m)
    y: float = 0.0      # y位置 (m)
    v: float = 0.0      # 速度 (m/s)
    yaw: float = 0.0    # 航向角 (rad)

class VehicleModel:
    """简化的车辆运动学模型"""

    def __init__(self, wheelbase: float = 2.5, dt: float = 0.1):
        """
        初始化车辆模型

        Args:
            wheelbase: 轴距 (m)
            dt: 时间步长 (s)
        """
        self.WB = wheelbase  # 轴距
        self.dt = dt         # 采样时间

    def update_state(self, state: VehicleState, acceleration: float, steering_angle: float) -> VehicleState:
        """
        更新车辆状态（非线性运动学模型）

        TODO: 请实现车辆运动学方程
        提示：
        - x[k+1] = x[k] + v*cos(yaw)*dt
        - y[k+1] = y[k] + v*sin(yaw)*dt
        - v[k+1] = v[k] + acceleration*dt
        - yaw[k+1] = yaw[k] + v*tan(steering_angle)/WB*dt

        Args:
            state: 当前状态
            acceleration: 加速度输入 (m/s²)
            steering_angle: 转向角输入 (rad)

        Returns:
            new_state: 更新后的状态
        """
        new_state = VehicleState()

        # TODO: 在这里实现运动学更新方程
        # 你需要根据当前状态和控制输入计算下一时刻的状态
        new_state.x = state.x + state.v * math.cos(state.yaw) * self.dt
        new_state.y = state.y + state.v * math.sin(state.yaw) * self.dt
        new_state.v = state.v + acceleration * self.dt
        new_state.yaw = state.yaw + (math.tan(steering_angle) / self.WB) * state.v * self.dt

        return new_state

# ============================================================================
# 第2步：测试车辆模型
# ============================================================================

def test_vehicle_model():
    """测试车辆运动学模型"""
    print("=== Testing Vehicle Model ===")

    # 创建车辆模型
    vehicle = VehicleModel(wheelbase=2.5, dt=0.1)

    # 初始状态
    state = VehicleState(x=0.0, y=0.0, v=5.0, yaw=0.0)

    # 测试直行
    print("\\nTest 1: Straight line motion")
    print(f"Initial: x={state.x:.2f}, y={state.y:.2f}, v={state.v:.2f}, yaw={state.yaw:.2f}")

    state = vehicle.update_state(state, acceleration=0.0, steering_angle=0.0)
    print(f"After 1 step: x={state.x:.2f}, y={state.y:.2f}, v={state.v:.2f}, yaw={state.yaw:.2f}")

    # 测试转向
    print("\\nTest 2: Turning motion")
    state = VehicleState(x=0.0, y=0.0, v=5.0, yaw=0.0)
    state = vehicle.update_state(state, acceleration=0.0, steering_angle=0.1)  # 转向角0.1rad
    print(f"After turn: x={state.x:.2f}, y={state.y:.2f}, v={state.v:.2f}, yaw={state.yaw:.2f}")

# ============================================================================
# 第3步：线性化函数 - MPC的核心！
# ============================================================================

class MPCLinearization:
    """MPC线性化模块"""

    def __init__(self, vehicle_model: VehicleModel):
        self.vehicle = vehicle_model
        self.dt = vehicle_model.dt
        self.WB = vehicle_model.WB

    def get_linearized_matrices(self, state: VehicleState, steering_angle: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        在给定状态点进行线性化，得到 A, B, C 矩阵

        线性化后的系统： x[k+1] = A @ x[k] + B @ u[k] + C

        TODO: 请实现雅可比矩阵计算

        状态向量: x = [x_pos, y_pos, velocity, yaw_angle]
        控制向量: u = [acceleration, steering_angle]

        提示：
        对于非线性方程组 f(x, u)：
        - A[i,j] = ∂f_i/∂x_j  (对状态的偏导数)
        - B[i,j] = ∂f_i/∂u_j  (对控制的偏导数)
        - C[i] = f_i(x₀,u₀) - A@x₀ - B@u₀  (线性化残差)

        Args:
            state: 线性化工作点的状态
            steering_angle: 线性化工作点的转向角

        Returns:
            A: 状态转移矩阵 (4x4)
            B: 控制输入矩阵 (4x2)
            C: 偏移向量 (4,)
        """
        # 提取状态变量（线性化工作点）
        x, y, v, yaw = state.x, state.y, state.v, state.yaw
        delta = steering_angle

        # 初始化矩阵
        A = np.eye(4)  # 从单位矩阵开始
        B = np.zeros((4, 2))
        C = np.zeros(4)

        # TODO: 计算A矩阵 (状态雅可比)
        # 提示：
        # A[0,2] = ∂(x + v*cos(yaw)*dt)/∂v = ?
        # A[0,3] = ∂(x + v*cos(yaw)*dt)/∂yaw = ?
        # A[1,2] = ∂(y + v*sin(yaw)*dt)/∂v = ?
        # A[1,3] = ∂(y + v*sin(yaw)*dt)/∂yaw = ?
        # A[3,2] = ∂(yaw + v*tan(delta)*dt/WB)/∂v = ?
        A[0,2] = math.cos(yaw) * self.dt
        A[0,3] = -v * math.sin(yaw) * self.dt
        A[1,2] = math.sin(yaw) * self.dt
        A[1,3] = v * math.cos(yaw) * self.dt
        A[3,2] = math.tan(delta) * self.dt / self.WB

        # TODO: 计算B矩阵 (控制雅可比)
        # 提示：
        # B[2,0] = ∂(v + acceleration*dt)/∂acceleration = ?
        # B[3,1] = ∂(yaw + v*tan(delta)*dt/WB)/∂delta = ?
        B[2,0] = self.dt
        B[3,1] = (v * self.dt) / (self.WB * math.cos(delta) ** 2)

        # TODO: 计算C向量 (线性化残差)
        # 提示：C用于补偿线性化误差
        # C[0] = 当前点的实际x变化 - 线性化预测的x变化

        # 计算非线性模型在工作点的输出
        # 假设控制输入为 u = [0, 0] （用于线性化，不是实际控制）
        acceleration_ref = 0.0

        # 非线性模型的状态更新：f(x₀, u₀)
        f_nonlinear = np.array([
            x + v * math.cos(yaw) * self.dt,  # 下一时刻的 x
            y + v * math.sin(yaw) * self.dt,   # 下一时刻的 y
            v + acceleration_ref * self.dt,    # 下一时刻的 v
            yaw + v * math.tan(delta) * self.dt / self.WB  # 下一时刻的 yaw
        ])

        # 当前状态向量
        x_current = np.array([x, y, v, yaw])

        # 参考控制向量
        u_ref = np.array([acceleration_ref, delta])

        # 计算线性化残差：C = f(x₀,u₀) - A@x₀ - B@u₀
        C = f_nonlinear - A @ x_current - B @ u_ref

        return A, B, C

# ============================================================================
# 第4步：测试线性化
# ============================================================================

def test_linearization():
    """测试线性化实现"""
    print("\\n=== Testing Linearization ===")

    vehicle = VehicleModel()
    linearizer = MPCLinearization(vehicle)

    # 测试点
    state = VehicleState(x=1.0, y=2.0, v=10.0, yaw=0.5)
    steering_angle = 0.1

    A, B, C = linearizer.get_linearized_matrices(state, steering_angle)

    print(f"State: x={state.x}, y={state.y}, v={state.v}, yaw={state.yaw:.3f}")
    print(f"Steering angle: {steering_angle:.3f}")
    print("\\nA matrix (4x4):")
    print(A)
    print("\\nB matrix (4x2):")
    print(B)
    print("\\nC vector (4,):")
    print(C)

    # 验证线性化精度
    print("\\n=== Verifying Linearization Accuracy ===")

    # 使用非线性模型计算下一状态
    next_state_nonlinear = vehicle.update_state(state, acceleration=0.0, steering_angle=steering_angle)
    nonlinear_result = np.array([next_state_nonlinear.x, next_state_nonlinear.y,
                                next_state_nonlinear.v, next_state_nonlinear.yaw])

    # 使用线性化模型计算下一状态
    current_state = np.array([state.x, state.y, state.v, state.yaw])
    control_input = np.array([0.0, steering_angle])  # acceleration=0.0
    linear_result = A @ current_state + B @ control_input + C

    print(f"Nonlinear model result: {nonlinear_result}")
    print(f"Linear model result:    {linear_result}")
    print(f"Difference:             {np.abs(nonlinear_result - linear_result)}")
    print(f"Max error:              {np.max(np.abs(nonlinear_result - linear_result)):.10f}")

    if np.max(np.abs(nonlinear_result - linear_result)) < 1e-10:
        print("✅ Linearization is accurate at working point!")

if __name__ == "__main__":
    test_vehicle_model()
    test_linearization()