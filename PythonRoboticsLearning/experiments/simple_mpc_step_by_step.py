"""
Simple MPC Controller - Step by Step Implementation
逐步构建MPC控制器，加深对算法原理的理解

Author: Learning Project
Date: 2026-02-27
"""

from cvxpy.reductions.solvers import solver
import numpy as np
import matplotlib.pyplot as plt
import math
import cvxpy
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

# ============================================================================
# 第5步：代价函数权重矩阵（待实现）
# ============================================================================

# TODO: 请实现 MPCWeights 类
#
# MPC的代价函数由多个权重矩阵组成，每个矩阵代表不同的控制目标：
#
# 提示：
# 1. 定义状态跟踪权重矩阵 Q = diag([Qx, Qy, Qv, Qyaw])
#    - Qx, Qy: 位置误差权重（优先保证轨迹跟踪）
#    - Qv, Qyaw: 速度和航向角误差权重
#
# 2. 定义控制输入权重矩阵 R = diag([Ra, Rdelta])
#    - Ra: 加速度代价（避免过大的加速度）
#    - Rdelta: 转向角代价（避免过大的转向角）
#
# 3. 定义控制平滑权重矩阵 Rd = diag([Rda, Rd_delta])
#    - Rda: 加速度变化率权重
#    - Rd_delta: 转向角变化率权重（为什么这个要大很多？）
#
# 4. 定义终端权重 Qf = Q（终端状态的重要性）
#
# 思考题：
# - 为什么 Rd_delta 要比 Rda 大100倍？（考虑车辆动力学特性）
# - Q 和 R 的相对大小如何影响控制效果？

class MPCWeights:
    """MPC权重矩阵定义"""

    def __init__(self):
        """初始化权重矩阵"""
        # TODO: 在这里实现权重矩阵
        # 提示：使用 np.diag() 创建对角矩阵,它是对角矩阵
        self.Q = np.diag([1.0, 1.0, 0.5, 0.5])
        self.R = np.diag([0.01, 0.01])
        self.Rd = np.diag([0.01, 1.0])

        self.Qf = self.Q

    def print_weights(self):
        """打印权重矩阵信息"""
        # TODO: 打印各个权重值，便于调试
        print("\n=== MPC Weights ===")
        print("Q (state tracking):")
        print(f"  Qx={self.Q[0,0]:.2f}, Qy={self.Q[1,1]:.2f}")
        print(f"  Qv={self.Q[2,2]:.2f}, Qyaw={self.Q[3,3]:.2f}")
        print("\nR (control cost):")
        print(f"  Ra={self.R[0,0]:.3f}, Rδ={self.R[1,1]:.3f}")
        print("\nRd (control smoothness):")
        print(f"  Rda={self.Rd[0,0]:.3f}, Rdδ={self.Rd[1,1]:.2f}")
        print(f"\n转向平滑权重 / 加速度平滑权重 = {self.Rd[1,1]/self.Rd[0,0]:.0f}倍")

# ============================================================================
# 第6步：MPC控制器（待实现）
# ============================================================================

class SimpleMPCController:
    """简化的MPC控制器（教学版）"""

    def __init__(self, horizon=5, dt=0.2, wheelbase=2.5):
        """
        初始化MPC控制器

        Args:
            horizon: 预测时域 T
            dt: 时间步长
            wheelbase: 轴距
        """
        self.T = horizon
        self.dt = dt
        self.WB = wheelbase

        # 创建车辆模型和线性化器
        self.vehicle = VehicleModel(wheelbase=wheelbase, dt=dt)
        self.linearizer = MPCLinearization(self.vehicle)

        # 权重矩阵
        self.weights = MPCWeights()

        # 车辆物理限制
        self.MAX_SPEED = 15.0      # m/s
        self.MIN_SPEED = -5.0      # m/s
        self.MAX_ACCEL = 1.0       # m/s²
        self.MAX_STEER = 0.6       # rad (约34度)
        self.MAX_DSTEER = 0.5      # rad/s (转向速率限制)

    def predict_trajectory(self, x0, u_sequence):
        """
        TODO: 使用非线性模型预测轨迹（用于线性化工作点）

        这个方法的作用是什么？
        - 给定初始状态和控制序列，预测未来T步的状态轨迹
        - 这个轨迹用作线性化的工作点（xbar）

        Args:
            x0: 初始状态 [x, y, v, yaw]
            u_sequence: 控制序列 T×2，每一行是[acceleration, steering_angle]

        Returns:
            xbar: 预测轨迹 (4, T+1)，每一列是一个时刻的状态

        提示：
        1. 初始化状态轨迹数组 xbar (4行, T+1列)
        2. 设置初始状态 xbar[:, 0] = x0
        3. 循环T步，每步：
           - 提取当前控制输入 u_sequence[t]
           - 使用 vehicle.update_state() 更新状态
           - 存储新状态到 xbar[:, t+1]
        """
        xbar = np.zeros((4, self.T + 1))
        xbar[:, 0] = x0

        state = VehicleState(x=x0[0], y=x0[1], v=x0[2], yaw=x0[3])

        for t in range(self.T):
            accel = u_sequence[t, 0]
            delta = u_sequence[t, 1]

            state = self.vehicle.update_state(state, accel, delta)
            xbar[:, t+1] = [state.x, state.y, state.v, state.yaw]

        return xbar

    def solve_linear_mpc(self, xref, xbar, x0, dref):
        """
        TODO: 求解线性MPC问题（单次优化）

        这是MPC的核心！需要使用CVXPY构建并求解QP问题。

        Args:
            xref: 参考轨迹 (4, T+1) - 我们想要跟踪的目标轨迹
            xbar: 线性化工作点轨迹 (4, T+1) - 用于每步的线性化
            x0: 初始状态 (4,) - 当前状态约束
            dref: 参考转向角序列 (T,) - 用于线性化的参考转向角

        Returns:
            u_opt: 最优控制序列 (T, 2)
            x_opt: 最优状态轨迹 (4, T+1)

        关键步骤：
        1. 定义优化变量：x (状态轨迹), u (控制序列)
        2. 构建代价函数：
           - 状态跟踪代价：||xref[t] - x[t]||²_Q
           - 控制输入代价：||u[t]||²_R
           - 控制平滑性代价：||u[t+1] - u[t]||²_Rd
           - 终端代价：||xref[T] - x[T]||²_Qf
        3. 添加约束：
           - 初始状态约束：x[:, 0] == x0
           - 动态约束（线性化）：x[t+1] = A[t] @ x[t] + B[t] @ u[t] + C[t]
           - 物理约束（速度、加速度、转向角限制）
        4. 求解QP问题

        注意：CVXPY变量形状
        - x = cvxpy.Variable((4, T+1)) → x.value 形状 (4, T+1)
        - u = cvxpy.Variable((2, T)) → u.value 形状 (2, T)
        - 返回时需要转置：u.value.T 得到 (T, 2)
        """

        x =  cvxpy.Variable((4, self.T + 1))
        u =  cvxpy.Variable((2, self.T))

        # 代价函数与约束条件
        cost = 0.0
        constraints = []

        #初始约束
        constraints += [x[:,0] == x0]

        # 构建状态约束和动态约束
        for t in range(self.T):
            # 控制代价
            cost += cvxpy.quad_form(u[:,t], self.weights.R)

            # 状态跟踪代价
            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], self.weights.Q)

            # 动态约束
            A, B, C = self.linearizer.get_linearized_matrices(VehicleState(xbar[0, t], xbar[1, t], xbar[2, t], xbar[3, t]), dref[t])
            constraints += [x[:, t+1] == A @ x[:, t] + B @ u[:, t] + C]

            # 控制平滑性代价
            if t < (self.T - 1):
                cost += cvxpy.quad_form(u[:, t+1] - u[:, t], self.weights.Rd)

                # 转向速率约束,u是2xN
                constraints += [cvxpy.abs(u[1, t+1] - u[1, t]) <= self.MAX_STEER * self.dt]

        # 终端代价
        cost += cvxpy.quad_form(xref[:, self.T] - x[:, self.T], self.weights.Qf)

        # 物理约束
        
        ## 控制量
        constraints += [cvxpy.abs(u[1, :]) <= self.MAX_STEER]
        constraints += [cvxpy.abs(u[0, :]) <= self.MAX_ACCEL]
        
        ## 状态量
        constraints += [x[2, :] <= self.MAX_SPEED]
        constraints += [x[2, :] >= self.MIN_SPEED]

        # 求解 QP 问题
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.CLARABEL, verbose=False)

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            return u.value.T, x.value
        else:
            print(f"MPC 求解失败: {prob.status}")
            return None, None
            

    def compute_control(self, current_state, reference_trajectory):
        """
        TODO: 计算MPC控制输入（迭代线性化）

        这个方法实现了迭代线性化MPC (ILMPC)：
        - 为什么需要迭代？因为线性化在工作点附近才准确
        - 每次迭代：预测轨迹 → 线性化 → 求解QP → 更新控制序列

        Args:
            current_state: 当前状态 [x, y, v, yaw]
            reference_trajectory: 参考轨迹 (4, T+1)

        Returns:
            optimal_control: 最优控制 [acceleration, steering_angle]

        算法流程：
        1. 初始化控制序列 u_init = 0
        2. 迭代（最多3次）：
           a. 用当前u预测轨迹 xbar
           b. 求解线性MPC得到新u
           c. 检查收敛（||u_new - u_old|| < threshold）
        3. 返回第一个控制输入（MPC的滚动时域特性）

        思考：为什么只返回第一个控制输入？
        """
        # 初始参数状态为0，Tx1
        dref = np.zeros(self.T)

        # 初始控制序列
        u_init = np.zeros((self.T, 2))

        # 迭代线性化MPC
        MAX_ITER = 3
        u_opt = u_init.copy()

        for iter in range(MAX_ITER):
            # 预测轨迹
            xbar = self.predict_trajectory(current_state, u_opt)

            # 求解线性MPC
            u_new, x_new = self.solve_linear_mpc(reference_trajectory,xbar, current_state, dref)

            if u_new is None:
                break

        return u_opt[0]

# ============================================================================
# 第7步：测试完整的MPC控制器
# ============================================================================

def test_mpc_controller():
    """测试MPC控制器"""
    print("\n=== Testing MPC Controller ===")

    # 创建MPC控制器
    mpc = SimpleMPCController(horizon=5, dt=0.2, wheelbase=2.5)

    # 打印权重信息
    mpc.weights.print_weights()

    # 当前状态
    current_state = np.array([0.0, 0.0, 5.0, 0.0])  # [x, y, v, yaw]

    # 参考轨迹（简单的直线轨迹）
    T = mpc.T
    xref = np.zeros((4, T + 1))
    for t in range(T + 1):
        xref[0, t] = current_state[0] + 5.0 * t * mpc.dt  # x位置
        xref[1, t] = 0.0                                   # y位置
        xref[2, t] = 5.0                                   # 速度
        xref[3, t] = 0.0                                   # 航向角

    print(f"\n当前状态: x={current_state[0]:.2f}, y={current_state[1]:.2f}, "
          f"v={current_state[2]:.2f}, yaw={current_state[3]:.2f}")

    # 计算MPC控制
    optimal_control = mpc.compute_control(current_state, xref)

    print(f"\n最优控制:")
    print(f"  加速度: {optimal_control[0]:.4f} m/s²")
    print(f"  转向角: {optimal_control[1]:.4f} rad ({np.degrees(optimal_control[1]):.2f}°)")

    print("\n✅ MPC控制器测试完成!")

if __name__ == "__main__":
    test_vehicle_model()
    test_linearization()
    test_mpc_controller()