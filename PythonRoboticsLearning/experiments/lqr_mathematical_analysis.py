"""
LQR 最优控制的数学推导演示
详细展示最优控制量、增益矩阵K和最小代价的关系

参考文献：
- Modern Control Engineering (Ogata)
- Optimal Control Theory (Anderson & Moore)
- Linear Optimal Control (Athans & Falb)
"""

import sys
sys.path.insert(0, '/home/zhufeng/code/PythonRobotics/PythonRobotics')

import numpy as np
import matplotlib.pyplot as plt
import scipy.linalg as la
import math

print("🎓 LQR Optimal Control: Mathematical Relationship Deep Dive")
print("=" * 70)

# 1. 基本系统定义
print("\n📚 1. System Definition and Problem Formulation")
print("-" * 50)

# 示例系统：双积分器系统 (车辆横向动力学的简化模型)
A = np.array([[1.0, 0.1],    # 位置-速度系统
              [0.0, 1.0]])   # dt = 0.1

B = np.array([[0.005],       # 控制输入到位置
              [0.1]])        # 控制输入到速度

Q = np.array([[10.0, 0.0],   # 位置权重
              [0.0,  1.0]])  # 速度权重

R = np.array([[1.0]])        # 控制权重

print(f"System matrix A:\n{A}")
print(f"Input matrix B:\n{B}")
print(f"State weight Q:\n{Q}")
print(f"Control weight R:\n{R}")

# 2. 求解 DARE 方程
print("\n🔬 2. Solving Discrete Algebraic Riccati Equation (DARE)")
print("-" * 60)

def solve_DARE_detailed(A, B, Q, R, show_steps=True):
    """详细求解DARE，显示迭代过程"""
    print("DARE: X = A'XA - A'XB(R + B'XB)^(-1)B'XA + Q")

    X = Q.copy()  # 初始猜测
    max_iter = 100
    eps = 1e-6

    if show_steps:
        print(f"Initial guess X₀:\n{X}")

    for i in range(max_iter):
        # DARE 迭代公式
        AXA = A.T @ X @ A
        AXB = A.T @ X @ B
        BXB_R = R + B.T @ X @ B
        BXA = B.T @ X @ A

        try:
            inv_term = la.inv(BXB_R)
            X_new = AXA - AXB @ inv_term @ BXA + Q
        except la.LinAlgError:
            print(f"❌ Singular matrix at iteration {i}")
            break

        # 检查收敛
        error = np.max(np.abs(X_new - X))

        if show_steps and i < 5:  # 只显示前5次迭代
            print(f"\nIteration {i+1}:")
            print(f"  X_{i+1}:\n  {X_new}")
            print(f"  Error: {error:.8f}")

        if error < eps:
            if show_steps:
                print(f"\n✅ Converged after {i+1} iterations")
                print(f"Final solution X:\n{X_new}")
            break

        X = X_new

    return X_new

# 求解 DARE
P = solve_DARE_detailed(A, B, Q, R)

# 3. 计算最优增益矩阵 K
print("\n⚡ 3. Computing Optimal Gain Matrix K")
print("-" * 45)

# K = (R + B'PB)^(-1) * B'PA
BPB_R = R + B.T @ P @ B
BPA = B.T @ P @ A

K = la.inv(BPB_R) @ BPA

print("Gain matrix formula: K = (R + B'PB)^(-1) * B'PA")
print(f"\nB'PB + R:\n{BPB_R}")
print(f"B'PA:\n{BPA}")
print(f"\n🎯 Optimal gain K:\n{K}")
print(f"K dimensions: {K.shape}")

# 4. 最优控制律
print("\n🎮 4. Optimal Control Law")
print("-" * 30)

print("Optimal control law: u* = -K * x")
print(f"For our system: u* = -[{K[0,0]:.4f}  {K[0,1]:.4f}] * [position; velocity]")
print(f"Physical meaning:")
print(f"  - Position feedback: {-K[0,0]:.4f} (proportional control)")
print(f"  - Velocity feedback: {-K[0,1]:.4f} (derivative control)")

# 5. 最小代价的计算
print("\n💰 5. Minimum Cost Analysis")
print("-" * 35)

def calculate_min_cost_analytical(x0, P):
    """解析计算最小代价"""
    return x0.T @ P @ x0

def calculate_min_cost_simulation(x0, A, B, K, Q, R, steps=100):
    """仿真计算实际代价"""
    x = x0.copy()
    total_cost = 0.0

    trajectory_x = []
    trajectory_u = []
    costs = []

    for k in range(steps):
        # 最优控制
        u = -K @ x

        # 计算瞬时代价
        stage_cost = x.T @ Q @ x + u.T @ R @ u
        total_cost += stage_cost[0,0]

        # 记录数据
        trajectory_x.append(x.copy())
        trajectory_u.append(u.copy())
        costs.append(stage_cost[0,0])

        # 状态更新
        x = A @ x + B @ u

        # 检查收敛
        if np.linalg.norm(x) < 1e-10:
            break

    return total_cost, trajectory_x, trajectory_u, costs

# 测试不同初始状态
test_states = [
    np.array([[1.0], [0.0]]),     # 只有位置误差
    np.array([[0.0], [1.0]]),     # 只有速度误差
    np.array([[1.0], [1.0]]),     # 位置和速度误差
    np.array([[2.0], [-0.5]])     # 复杂初始状态
]

print("Testing minimum cost for different initial states:")
print("Initial State    | Analytical J* | Simulation J | Error")
print("-" * 55)

for i, x0 in enumerate(test_states):
    # 解析解
    J_analytical = calculate_min_cost_analytical(x0, P)[0,0]

    # 仿真解
    J_simulation, traj_x, traj_u, costs = calculate_min_cost_simulation(x0, A, B, K, Q, R)

    error = abs(J_analytical - J_simulation)

    print(f"[{x0[0,0]:4.1f}, {x0[1,0]:4.1f}]      | {J_analytical:9.4f}   | {J_simulation:8.4f}   | {error:.2e}")

# 6. 关键关系的可视化
print("\n📊 6. Visualizing Key Relationships")
print("-" * 40)

def visualize_relationships():
    """可视化最优控制关系"""

    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('LQR Optimal Control: Mathematical Relationships', fontsize=16)

    # 测试状态
    x0 = np.array([[2.0], [1.0]])
    J_opt, traj_x, traj_u, costs = calculate_min_cost_simulation(x0, A, B, K, Q, R, steps=50)

    # 提取轨迹数据
    positions = [x[0,0] for x in traj_x]
    velocities = [x[1,0] for x in traj_x]
    controls = [u[0,0] for u in traj_u]
    time_steps = range(len(positions))

    # 1. 状态轨迹
    ax1 = axes[0, 0]
    ax1.plot(time_steps, positions, 'b-', label='Position', linewidth=2)
    ax1.plot(time_steps, velocities, 'r-', label='Velocity', linewidth=2)
    ax1.set_xlabel('Time Step k')
    ax1.set_ylabel('State Value')
    ax1.set_title('Optimal State Trajectory')
    ax1.legend()
    ax1.grid(True)

    # 2. 控制输入
    ax2 = axes[0, 1]
    time_control = time_steps[:len(controls)]  # 确保长度匹配
    ax2.plot(time_control, controls, 'g-', linewidth=2)
    ax2.set_xlabel('Time Step k')
    ax2.set_ylabel('Control Input u')
    ax2.set_title('Optimal Control Input')
    ax2.grid(True)

    # 3. 瞬时代价
    ax3 = axes[0, 2]
    time_costs = time_steps[:len(costs)]  # 确保长度匹配
    ax3.semilogy(time_costs, costs, 'purple', linewidth=2)
    ax3.set_xlabel('Time Step k')
    ax3.set_ylabel('Stage Cost (log scale)')
    ax3.set_title('Stage Cost Evolution')
    ax3.grid(True)

    # 4. 相平面图
    ax4 = axes[1, 0]
    ax4.plot(positions, velocities, 'b-', linewidth=2, marker='o', markersize=3)
    ax4.plot(positions[0], velocities[0], 'go', markersize=8, label='Start')
    ax4.plot(0, 0, 'ro', markersize=8, label='Target')
    ax4.set_xlabel('Position')
    ax4.set_ylabel('Velocity')
    ax4.set_title('Phase Plane Trajectory')
    ax4.legend()
    ax4.grid(True)

    # 5. 控制 vs 状态关系
    ax5 = axes[1, 1]
    state_norms = [np.linalg.norm(x) for x in traj_x[:len(traj_u)]]  # 确保长度匹配
    control_values = [abs(u[0,0]) for u in traj_u]
    ax5.plot(state_norms, control_values, 'mo-', linewidth=2)
    ax5.set_xlabel('State Norm ||x||')
    ax5.set_ylabel('Control Magnitude |u|')
    ax5.set_title('Control vs State Relationship')
    ax5.grid(True)

    # 6. K矩阵对代价的影响
    ax6 = axes[1, 2]

    # 测试不同的K值对代价的影响
    k1_range = np.linspace(0.5*K[0,0], 2.0*K[0,0], 20)
    k2_range = np.linspace(0.5*K[0,1], 2.0*K[0,1], 20)

    costs_k1 = []
    costs_k2 = []

    for k1 in k1_range:
        K_test = np.array([[k1, K[0,1]]])
        J_test, _, _, _ = calculate_min_cost_simulation(x0, A, B, K_test, Q, R)
        costs_k1.append(J_test)

    for k2 in k2_range:
        K_test = np.array([[K[0,0], k2]])
        J_test, _, _, _ = calculate_min_cost_simulation(x0, A, B, K_test, Q, R)
        costs_k2.append(J_test)

    ax6.plot(k1_range, costs_k1, 'b-', label=f'Varying K₁ (K₂={K[0,1]:.3f})', linewidth=2)
    ax6.plot(k2_range, costs_k2, 'r-', label=f'Varying K₂ (K₁={K[0,0]:.3f})', linewidth=2)
    ax6.axvline(K[0,0], color='b', linestyle='--', alpha=0.7, label='Optimal K₁')
    ax6.axvline(K[0,1], color='r', linestyle='--', alpha=0.7, label='Optimal K₂')
    ax6.set_xlabel('Gain Value')
    ax6.set_ylabel('Total Cost')
    ax6.set_title('Cost vs Gain Sensitivity')
    ax6.legend()
    ax6.grid(True)

    plt.tight_layout()
    plt.savefig('/home/zhufeng/code/PythonRobotics/PythonRoboticsLearning/experiments/lqr_optimal_control_relationships.png',
                dpi=300, bbox_inches='tight')

    return fig

# 生成可视化
fig = visualize_relationships()

# 7. 理论验证
print("\n🔍 7. Theoretical Verification")
print("-" * 35)

def verify_bellman_equation(A, B, P, Q, R, K):
    """验证贝尔曼方程"""
    print("Verifying Bellman equation:")
    print("P = Q + K'RK + (A-BK)'P(A-BK)")

    # 闭环系统矩阵
    A_cl = A - B @ K

    # 计算贝尔曼方程右边
    bellman_rhs = Q + K.T @ R @ K + A_cl.T @ P @ A_cl

    # 计算误差
    bellman_error = np.max(np.abs(P - bellman_rhs))

    print(f"Left side (P):\n{P}")
    print(f"Right side:\n{bellman_rhs}")
    print(f"Maximum error: {bellman_error:.2e}")

    if bellman_error < 1e-10:
        print("✅ Bellman equation satisfied!")
    else:
        print("❌ Bellman equation not satisfied")

    return bellman_error

def verify_optimality_condition(A, B, P, R, K):
    """验证最优性条件"""
    print("\nVerifying optimality condition:")
    print("K = (R + B'PB)^(-1) B'PA")

    # 计算理论最优增益
    K_theory = la.inv(R + B.T @ P @ B) @ (B.T @ P @ A)

    # 计算误差
    K_error = np.max(np.abs(K - K_theory))

    print(f"Computed K:\n{K}")
    print(f"Theoretical K:\n{K_theory}")
    print(f"Maximum error: {K_error:.2e}")

    if K_error < 1e-10:
        print("✅ Optimality condition satisfied!")
    else:
        print("❌ Optimality condition not satisfied")

    return K_error

# 执行验证
bellman_err = verify_bellman_equation(A, B, P, Q, R, K)
optimal_err = verify_optimality_condition(A, B, P, R, K)

# 8. 核心关系总结
print("\n🎯 8. Core Mathematical Relationships Summary")
print("-" * 55)

print("""
📐 FUNDAMENTAL RELATIONSHIPS:

1. OPTIMAL CONTROL LAW:
   u*(k) = -K·x(k)

   where K is the optimal gain matrix that minimizes the cost function.

2. RICCATI EQUATION (DARE):
   P = A'PA - A'PB(R + B'PB)⁻¹B'PA + Q

   P is the solution that gives the minimum cost for any initial state.

3. OPTIMAL GAIN:
   K = (R + B'PB)⁻¹B'PA

   This gain balances state regulation (via Q) and control effort (via R).

4. MINIMUM COST:
   J*(x₀) = x₀'Px₀

   The minimum achievable cost from initial state x₀.

5. BELLMAN OPTIMALITY:
   P = Q + K'RK + (A-BK)'P(A-BK)

   This ensures the solution satisfies the principle of optimality.

🔗 KEY INSIGHTS:

• K represents the optimal feedback policy
• P encodes the cost-to-go from any state
• The relationships are mutually consistent
• Higher Q weights → larger K → more aggressive control
• Higher R weights → smaller K → gentler control
• P determines both the optimal gain K and minimum cost J*
""")

# 9. 实际工程意义
print("\n🏗️ 9. Engineering Implications")
print("-" * 35)

print(f"""
PRACTICAL MEANINGS:

🎮 Control Gain K = [{K[0,0]:.4f}, {K[0,1]:.4f}]:
   • Position feedback: {K[0,0]:.4f} (like a spring force)
   • Velocity feedback: {K[0,1]:.4f} (like a damper force)
   • Total control: u = -{K[0,0]:.4f}·pos - {K[0,1]:.4f}·vel

💰 Cost Matrix P:
   • P[0,0] = {P[0,0]:.4f}: Cost per unit position²
   • P[1,1] = {P[1,1]:.4f}: Cost per unit velocity²
   • P[0,1] = {P[0,1]:.4f}: Cross-coupling cost

⚖️ Design Trade-offs:
   • Q ↑ → K ↑ → More aggressive control, lower tracking error
   • R ↑ → K ↓ → Gentler control, higher tracking error
   • P determines the Pareto frontier of this trade-off
""")

plt.show()

print("\n" + "=" * 70)
print("🎓 LQR Mathematical Relationships - Analysis Complete!")
print("=" * 70)