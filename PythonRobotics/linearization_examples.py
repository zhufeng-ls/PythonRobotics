import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize
import matplotlib.patches as patches

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Noto Sans CJK SC', 'Source Han Sans CN']
plt.rcParams['axes.unicode_minus'] = False

def example1_simple_function():
    """例子1：简单函数的线性化"""
    print("=== 例子1：简单函数的线性化 ===")
    
    # 原始非线性函数
    def f(x):
        return x**2 + 2*x + 1
    
    # 线性化函数（在x0处的切线）
    def f_linear(x, x0):
        # 在x0处的导数
        df_dx = 2*x0 + 2
        # 线性化：f(x) ≈ f(x0) + f'(x0)(x-x0)
        return f(x0) + df_dx * (x - x0)
    
    # 生成数据
    x = np.linspace(-2, 4, 100)
    x0 = 1.0  # 线性化点
    
    # 计算函数值
    y_original = f(x)
    y_linear = f_linear(x, x0)
    
    # 画图
    plt.figure(figsize=(12, 8))
    
    plt.subplot(2, 2, 1)
    plt.plot(x, y_original, 'b-', linewidth=2, label='原始函数 f(x) = x² + 2x + 1')
    plt.plot(x, y_linear, 'r--', linewidth=2, label=f'线性化 (在x={x0})')
    plt.plot(x0, f(x0), 'ko', markersize=8, label=f'线性化点 ({x0}, {f(x0):.2f})')
    plt.grid(True, alpha=0.3)
    plt.xlabel('x')
    plt.ylabel('f(x)')
    plt.title('函数线性化')
    plt.legend()
    
    # 误差分析
    plt.subplot(2, 2, 2)
    error = np.abs(y_original - y_linear)
    plt.plot(x, error, 'g-', linewidth=2)
    plt.grid(True, alpha=0.3)
    plt.xlabel('x')
    plt.ylabel('|误差|')
    plt.title('线性化误差')
    
    # 局部放大
    plt.subplot(2, 2, 3)
    x_local = np.linspace(x0-0.5, x0+0.5, 50)
    y_orig_local = f(x_local)
    y_lin_local = f_linear(x_local, x0)
    
    plt.plot(x_local, y_orig_local, 'b-', linewidth=2, label='原始函数')
    plt.plot(x_local, y_lin_local, 'r--', linewidth=2, label='线性化')
    plt.plot(x0, f(x0), 'ko', markersize=8)
    plt.grid(True, alpha=0.3)
    plt.xlabel('x')
    plt.ylabel('f(x)')
    plt.title('局部放大')
    plt.legend()
    
    # 不同线性化点的比较
    plt.subplot(2, 2, 4)
    x0_values = [-1, 0, 1, 2]
    colors = ['red', 'green', 'blue', 'orange']
    
    plt.plot(x, y_original, 'k-', linewidth=3, label='原始函数')
    for i, x0_val in enumerate(x0_values):
        y_lin = f_linear(x, x0_val)
        plt.plot(x, y_lin, '--', color=colors[i], linewidth=2, 
                label=f'线性化点 x={x0_val}')
        plt.plot(x0_val, f(x0_val), 'o', color=colors[i], markersize=6)
    
    plt.grid(True, alpha=0.3)
    plt.xlabel('x')
    plt.ylabel('f(x)')
    plt.title('不同点的线性化')
    plt.legend()
    
    plt.tight_layout()
    plt.show()

def example2_2d_function():
    """例子2：二维函数的线性化"""
    print("=== 例子2：二维函数的线性化 ===")
    
    # 原始非线性函数
    def f(x, y):
        return x**2 + y**2 + 0.5*x*y
    
    # 线性化函数
    def f_linear(x, y, x0, y0):
        # 偏导数
        df_dx = 2*x0 + 0.5*y0
        df_dy = 2*y0 + 0.5*x0
        # 线性化：f(x,y) ≈ f(x0,y0) + ∂f/∂x(x-x0) + ∂f/∂y(y-y0)
        return f(x0, y0) + df_dx * (x - x0) + df_dy * (y - y0)
    
    # 生成网格
    x = np.linspace(-3, 3, 50)
    y = np.linspace(-3, 3, 50)
    X, Y = np.meshgrid(x, y)
    
    # 线性化点
    x0, y0 = 1.0, 1.0
    
    # 计算函数值
    Z_original = f(X, Y)
    Z_linear = f_linear(X, Y, x0, y0)
    
    # 画图
    fig = plt.figure(figsize=(15, 10))
    
    # 原始函数
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    surf1 = ax1.plot_surface(X, Y, Z_original, cmap='viridis', alpha=0.8)
    ax1.scatter([x0], [y0], [f(x0, y0)], color='red', s=100, label='线性化点')
    ax1.set_title('原始函数 f(x,y) = x² + y² + 0.5xy')
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_zlabel('f(x,y)')
    
    # 线性化函数
    ax2 = fig.add_subplot(2, 3, 2, projection='3d')
    surf2 = ax2.plot_surface(X, Y, Z_linear, cmap='plasma', alpha=0.8)
    ax2.scatter([x0], [y0], [f(x0, y0)], color='red', s=100)
    ax2.set_title('线性化函数')
    ax2.set_xlabel('x')
    ax2.set_ylabel('y')
    ax2.set_zlabel('f_linear(x,y)')
    
    # 误差
    ax3 = fig.add_subplot(2, 3, 3, projection='3d')
    error = np.abs(Z_original - Z_linear)
    surf3 = ax3.plot_surface(X, Y, error, cmap='hot', alpha=0.8)
    ax3.set_title('线性化误差')
    ax3.set_xlabel('x')
    ax3.set_ylabel('y')
    ax3.set_zlabel('|误差|')
    
    # 等高线图
    ax4 = fig.add_subplot(2, 3, 4)
    contour1 = ax4.contour(X, Y, Z_original, levels=10, colors='blue', alpha=0.7)
    ax4.clabel(contour1, inline=True, fontsize=8)
    ax4.plot(x0, y0, 'ro', markersize=10, label='线性化点')
    ax4.set_title('原始函数等高线')
    ax4.set_xlabel('x')
    ax4.set_ylabel('y')
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    
    ax5 = fig.add_subplot(2, 3, 5)
    contour2 = ax5.contour(X, Y, Z_linear, levels=10, colors='red', alpha=0.7)
    ax5.clabel(contour2, inline=True, fontsize=8)
    ax5.plot(x0, y0, 'ro', markersize=10)
    ax5.set_title('线性化函数等高线')
    ax5.set_xlabel('x')
    ax5.set_ylabel('y')
    ax5.grid(True, alpha=0.3)
    
    # 局部区域
    ax6 = fig.add_subplot(2, 3, 6)
    # 局部区域
    x_local = np.linspace(x0-0.5, x0+0.5, 20)
    y_local = np.linspace(y0-0.5, y0+0.5, 20)
    X_local, Y_local = np.meshgrid(x_local, y_local)
    
    Z_orig_local = f(X_local, Y_local)
    Z_lin_local = f_linear(X_local, Y_local, x0, y0)
    
    contour3 = ax6.contour(X_local, Y_local, Z_orig_local, levels=5, colors='blue', alpha=0.7)
    contour4 = ax6.contour(X_local, Y_local, Z_lin_local, levels=5, colors='red', alpha=0.7, linestyles='--')
    ax6.plot(x0, y0, 'ko', markersize=10)
    ax6.set_title('局部区域对比')
    ax6.set_xlabel('x')
    ax6.set_ylabel('y')
    ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def example2_2d1_function():
    """例子2：二维函数的线性化"""
    print("=== 例子2：二维函数的线性化 ===")
    
    # 原始非线性函数
    def f(x, y):
        return 2*x + 3*y
    
    # 线性化函数
    def f_linear(x, y, x0, y0):
        # 偏导数
        df_dx = 2
        df_dy = 3
        # 线性化：f(x,y) ≈ f(x0,y0) + ∂f/∂x(x-x0) + ∂f/∂y(y-y0)
        return f(x0, y0) + df_dx * (x - x0) + df_dy * (y - y0)
    
    # 生成网格
    x = np.linspace(-3, 3, 50)
    y = np.linspace(-3, 3, 50)
    X, Y = np.meshgrid(x, y)
    
    # 线性化点
    x0, y0 = 1.0, 1.0
    
    # 计算函数值
    Z_original = f(X, Y)
    Z_linear = f_linear(X, Y, x0, y0)
    
    # 画图
    fig = plt.figure(figsize=(15, 10))
    
    # 原始函数
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    surf1 = ax1.plot_surface(X, Y, Z_original, cmap='viridis', alpha=0.8)
    ax1.scatter([x0], [y0], [f(x0, y0)], color='red', s=100, label='线性化点')
    ax1.set_title('原始函数 f(x,y) = x² + y² + 0.5xy')
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_zlabel('f(x,y)')
    
    # 线性化函数
    ax2 = fig.add_subplot(2, 3, 2, projection='3d')
    surf2 = ax2.plot_surface(X, Y, Z_linear, cmap='plasma', alpha=0.8)
    ax2.scatter([x0], [y0], [f(x0, y0)], color='red', s=100)
    ax2.set_title('线性化函数')
    ax2.set_xlabel('x')
    ax2.set_ylabel('y')
    ax2.set_zlabel('f_linear(x,y)')
    
    # 误差
    ax3 = fig.add_subplot(2, 3, 3, projection='3d')
    error = np.abs(Z_original - Z_linear)
    surf3 = ax3.plot_surface(X, Y, error, cmap='hot', alpha=0.8)
    ax3.set_title('线性化误差')
    ax3.set_xlabel('x')
    ax3.set_ylabel('y')
    ax3.set_zlabel('|误差|')
    
    # 等高线图
    ax4 = fig.add_subplot(2, 3, 4)
    contour1 = ax4.contour(X, Y, Z_original, levels=10, colors='blue', alpha=0.7)
    ax4.clabel(contour1, inline=True, fontsize=8)
    ax4.plot(x0, y0, 'ro', markersize=10, label='线性化点')
    ax4.set_title('原始函数等高线')
    ax4.set_xlabel('x')
    ax4.set_ylabel('y')
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    
    ax5 = fig.add_subplot(2, 3, 5)
    contour2 = ax5.contour(X, Y, Z_linear, levels=10, colors='red', alpha=0.7)
    ax5.clabel(contour2, inline=True, fontsize=8)
    ax5.plot(x0, y0, 'ro', markersize=10)
    ax5.set_title('线性化函数等高线')
    ax5.set_xlabel('x')
    ax5.set_ylabel('y')
    ax5.grid(True, alpha=0.3)
    
    # 局部区域
    ax6 = fig.add_subplot(2, 3, 6)
    # 局部区域
    x_local = np.linspace(x0-0.5, x0+0.5, 20)
    y_local = np.linspace(y0-0.5, y0+0.5, 20)
    X_local, Y_local = np.meshgrid(x_local, y_local)
    
    Z_orig_local = f(X_local, Y_local)
    Z_lin_local = f_linear(X_local, Y_local, x0, y0)
    
    contour3 = ax6.contour(X_local, Y_local, Z_orig_local, levels=5, colors='blue', alpha=0.7)
    contour4 = ax6.contour(X_local, Y_local, Z_lin_local, levels=5, colors='red', alpha=0.7, linestyles='--')
    ax6.plot(x0, y0, 'ko', markersize=10)
    ax6.set_title('局部区域对比')
    ax6.set_xlabel('x')
    ax6.set_ylabel('y')
    ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def example3_pendulum_linearization():
    """例子3：单摆系统的线性化"""
    print("=== 例子3：单摆系统的线性化 ===")
    
    # 单摆参数
    g = 9.81  # 重力加速度
    L = 1.0   # 摆长
    m = 1.0   # 质量
    
    # 非线性单摆方程
    def pendulum_dynamics(theta, theta_dot):
        """单摆动力学方程"""
        theta_ddot = -(g/L) * np.sin(theta)
        return theta_ddot
    
    # 线性化（在平衡点附近）
    def pendulum_linearized(theta, theta_dot, theta0=0):
        """线性化后的单摆方程"""
        # 在theta0处的线性化
        # d²θ/dt² ≈ -(g/L) * θ (当θ很小时，sin(θ) ≈ θ)
        theta_ddot = -(g/L) * (theta - theta0)
        return theta_ddot
    
    # 时间序列
    t = np.linspace(0, 10, 1000)
    dt = t[1] - t[0]
    
    # 初始条件
    theta0 = 0.3  # 初始角度（弧度）
    theta_dot0 = 0.0  # 初始角速度
    
    # 数值积分
    def integrate_system(dynamics_func, t, theta0, theta_dot0):
        """数值积分系统"""
        theta = np.zeros_like(t)
        theta_dot = np.zeros_like(t)
        
        theta[0] = theta0
        theta_dot[0] = theta_dot0
        
        for i in range(1, len(t)):
            theta_ddot = dynamics_func(theta[i-1], theta_dot[i-1])
            theta_dot[i] = theta_dot[i-1] + theta_ddot * dt
            theta[i] = theta[i-1] + theta_dot[i-1] * dt
        
        return theta, theta_dot
    
    # 计算轨迹
    theta_nonlinear, theta_dot_nonlinear = integrate_system(pendulum_dynamics, t, theta0, theta_dot0)
    theta_linear, theta_dot_linear = integrate_system(pendulum_linearized, t, theta0, theta_dot0)
    
    # 画图
    plt.figure(figsize=(15, 10))
    
    # 角度对比
    plt.subplot(2, 3, 1)
    plt.plot(t, theta_nonlinear, 'b-', linewidth=2, label='非线性系统')
    plt.plot(t, theta_linear, 'r--', linewidth=2, label='线性化系统')
    plt.grid(True, alpha=0.3)
    plt.xlabel('时间 (s)')
    plt.ylabel('角度 (rad)')
    plt.title('角度时间历程')
    plt.legend()
    
    # 角速度对比
    plt.subplot(2, 3, 2)
    plt.plot(t, theta_dot_nonlinear, 'b-', linewidth=2, label='非线性系统')
    plt.plot(t, theta_dot_linear, 'r--', linewidth=2, label='线性化系统')
    plt.grid(True, alpha=0.3)
    plt.xlabel('时间 (s)')
    plt.ylabel('角速度 (rad/s)')
    plt.title('角速度时间历程')
    plt.legend()
    
    # 相图
    plt.subplot(2, 3, 3)
    plt.plot(theta_nonlinear, theta_dot_nonlinear, 'b-', linewidth=2, label='非线性系统')
    plt.plot(theta_linear, theta_dot_linear, 'r--', linewidth=2, label='线性化系统')
    plt.plot(theta0, theta_dot0, 'ko', markersize=8, label='初始状态')
    plt.grid(True, alpha=0.3)
    plt.xlabel('角度 (rad)')
    plt.ylabel('角速度 (rad/s)')
    plt.title('相图')
    plt.legend()
    
    # 误差分析
    plt.subplot(2, 3, 4)
    angle_error = np.abs(theta_nonlinear - theta_linear)
    plt.plot(t, angle_error, 'g-', linewidth=2)
    plt.grid(True, alpha=0.3)
    plt.xlabel('时间 (s)')
    plt.ylabel('角度误差 (rad)')
    plt.title('角度误差')
    
    # 不同初始条件的比较
    plt.subplot(2, 3, 5)
    initial_angles = [0.1, 0.3, 0.5, 1.0]
    colors = ['blue', 'green', 'orange', 'red']
    
    for i, angle in enumerate(initial_angles):
        theta_nl, _ = integrate_system(pendulum_dynamics, t, angle, 0)
        theta_lin, _ = integrate_system(pendulum_linearized, t, angle, 0)
        error = np.abs(theta_nl - theta_lin)
        plt.plot(t, error, color=colors[i], linewidth=2, 
                label=f'初始角度 {angle:.1f} rad')
    
    plt.grid(True, alpha=0.3)
    plt.xlabel('时间 (s)')
    plt.ylabel('角度误差 (rad)')
    plt.title('不同初始条件的误差')
    plt.legend()
    
    # 单摆动画示意
    plt.subplot(2, 3, 6)
    # 绘制单摆示意图
    L_plot = 2
    x_pivot, y_pivot = 0, 0
    x_bob = L_plot * np.sin(theta_nonlinear[0])
    y_bob = -L_plot * np.cos(theta_nonlinear[0])
    
    plt.plot([x_pivot, x_bob], [y_pivot, y_bob], 'k-', linewidth=3)
    plt.plot(x_pivot, y_pivot, 'ko', markersize=10, label='支点')
    plt.plot(x_bob, y_bob, 'ro', markersize=15, label='摆锤')
    plt.xlim(-L_plot-0.5, L_plot+0.5)
    plt.ylim(-L_plot-0.5, 0.5)
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.title('单摆示意图')
    plt.legend()
    
    plt.tight_layout()
    plt.show()

def example4_vehicle_kinematics():
    """例子4：车辆运动学线性化"""
    print("=== 例子4：车辆运动学线性化 ===")
    
    # 车辆参数
    L = 2.5  # 轴距
    v = 10.0  # 速度
    
    # 非线性车辆模型
    def vehicle_nonlinear(x, y, psi, delta):
        """非线性车辆运动学"""
        dx_dt = v * np.cos(psi)
        dy_dt = v * np.sin(psi)
        dpsi_dt = (v/L) * np.tan(delta)
        return dx_dt, dy_dt, dpsi_dt
    
    # 线性化车辆模型
    def vehicle_linearized(x, y, psi, delta, psi_ref=0, delta_ref=0):
        """线性化车辆运动学"""
        # 在参考点附近的线性化
        dx_dt = v * np.cos(psi_ref) - v * np.sin(psi_ref) * (psi - psi_ref)
        dy_dt = v * np.sin(psi_ref) + v * np.cos(psi_ref) * (psi - psi_ref)
        dpsi_dt = (v/L) * (delta - delta_ref)  # tan(delta) ≈ delta
        return dx_dt, dy_dt, dpsi_dt
    
    # 时间序列
    t = np.linspace(0, 5, 100)
    dt = t[1] - t[0]
    
    # 初始条件
    x0, y0, psi0 = 0, 0, 0.1  # 初始位置和航向
    delta_input = 0.1  # 恒定转向角
    
    # 数值积分
    def integrate_vehicle(dynamics_func, t, x0, y0, psi0, delta):
        """积分车辆轨迹"""
        x = np.zeros_like(t)
        y = np.zeros_like(t)
        psi = np.zeros_like(t)
        
        x[0] = x0
        y[0] = y0
        psi[0] = psi0
        
        for i in range(1, len(t)):
            dx_dt, dy_dt, dpsi_dt = dynamics_func(x[i-1], y[i-1], psi[i-1], delta)
            x[i] = x[i-1] + dx_dt * dt
            y[i] = y[i-1] + dy_dt * dt
            psi[i] = psi[i-1] + dpsi_dt * dt
        
        return x, y, psi
    
    # 计算轨迹
    x_nl, y_nl, psi_nl = integrate_vehicle(vehicle_nonlinear, t, x0, y0, psi0, delta_input)
    x_lin, y_lin, psi_lin = integrate_vehicle(vehicle_linearized, t, x0, y0, psi0, delta_input)
    
    # 画图
    plt.figure(figsize=(15, 10))
    
    # 轨迹对比
    plt.subplot(2, 3, 1)
    plt.plot(x_nl, y_nl, 'b-', linewidth=2, label='非线性模型')
    plt.plot(x_lin, y_lin, 'r--', linewidth=2, label='线性化模型')
    plt.plot(x0, y0, 'ko', markersize=8, label='起始点')
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('车辆轨迹')
    plt.legend()
    
    # 航向角对比
    plt.subplot(2, 3, 2)
    plt.plot(t, psi_nl, 'b-', linewidth=2, label='非线性模型')
    plt.plot(t, psi_lin, 'r--', linewidth=2, label='线性化模型')
    plt.grid(True, alpha=0.3)
    plt.xlabel('时间 (s)')
    plt.ylabel('航向角 (rad)')
    plt.title('航向角时间历程')
    plt.legend()
    
    # 横向位置对比
    plt.subplot(2, 3, 3)
    plt.plot(t, y_nl, 'b-', linewidth=2, label='非线性模型')
    plt.plot(t, y_lin, 'r--', linewidth=2, label='线性化模型')
    plt.grid(True, alpha=0.3)
    plt.xlabel('时间 (s)')
    plt.ylabel('横向位置 (m)')
    plt.title('横向位置时间历程')
    plt.legend()
    
    # 误差分析
    plt.subplot(2, 3, 4)
    y_error = np.abs(y_nl - y_lin)
    psi_error = np.abs(psi_nl - psi_lin)
    plt.plot(t, y_error, 'g-', linewidth=2, label='横向位置误差')
    plt.plot(t, psi_error, 'm-', linewidth=2, label='航向角误差')
    plt.grid(True, alpha=0.3)
    plt.xlabel('时间 (s)')
    plt.ylabel('误差')
    plt.title('线性化误差')
    plt.legend()
    
    # 不同转向角的比较
    plt.subplot(2, 3, 5)
    delta_values = [0.05, 0.1, 0.15, 0.2]
    colors = ['blue', 'green', 'orange', 'red']
    
    for i, delta_val in enumerate(delta_values):
        x_nl, y_nl, _ = integrate_vehicle(vehicle_nonlinear, t, x0, y0, psi0, delta_val)
        x_lin, y_lin, _ = integrate_vehicle(vehicle_linearized, t, x0, y0, psi0, delta_val)
        error = np.abs(y_nl - y_lin)
        plt.plot(t, error, color=colors[i], linewidth=2, 
                label=f'转向角 {delta_val:.2f} rad')
    
    plt.grid(True, alpha=0.3)
    plt.xlabel('时间 (s)')
    plt.ylabel('横向位置误差 (m)')
    plt.title('不同转向角的误差')
    plt.legend()
    
    # 车辆示意图
    plt.subplot(2, 3, 6)
    # 绘制车辆在轨迹上的位置
    idx = len(t) // 2  # 中间时刻
    
    # 车辆轮廓
    car_length = 4.5
    car_width = 2.0
    
    # 非线性模型车辆
    x_car = x_nl[idx]
    y_car = y_nl[idx]
    psi_car = psi_nl[idx]
    
    # 车辆矩形
    car_corners = np.array([
        [-car_length/2, -car_width/2],
        [car_length/2, -car_width/2],
        [car_length/2, car_width/2],
        [-car_length/2, car_width/2]
    ])
    
    # 旋转矩阵
    R = np.array([
        [np.cos(psi_car), -np.sin(psi_car)],
        [np.sin(psi_car), np.cos(psi_car)]
    ])
    
    # 旋转车辆轮廓
    car_corners_rotated = (R @ car_corners.T).T
    car_corners_rotated[:, 0] += x_car
    car_corners_rotated[:, 1] += y_car
    
    # 绘制车辆
    car_patch = patches.Polygon(car_corners_rotated, facecolor='lightblue', 
                               edgecolor='blue', linewidth=2)
    plt.gca().add_patch(car_patch)
    
    # 绘制轨迹
    plt.plot(x_nl, y_nl, 'b-', alpha=0.5, linewidth=1)
    plt.plot(x_lin, y_lin, 'r--', alpha=0.5, linewidth=1)
    
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('车辆位置示意图')
    
    plt.tight_layout()
    plt.show()

def example5_linearization_accuracy():
    """例子5：线性化精度分析"""
    print("=== 例子5：线性化精度分析 ===")
    
    # 测试函数
    def test_function(x):
        return np.sin(x) + 0.1 * x**2
    
    def test_function_linear(x, x0):
        # 在x0处的线性化
        df_dx = np.cos(x0) + 0.2 * x0
        return test_function(x0) + df_dx * (x - x0)
    
    # 不同线性化点的精度分析
    x = np.linspace(-3, 3, 100)
    linearization_points = [-2, -1, 0, 1, 2]
    
    plt.figure(figsize=(15, 10))
    
    # 函数和不同线性化点
    plt.subplot(2, 3, 1)
    plt.plot(x, test_function(x), 'k-', linewidth=3, label='原始函数')
    
    colors = ['red', 'green', 'blue', 'orange', 'purple']
    for i, x0 in enumerate(linearization_points):
        y_lin = test_function_linear(x, x0)
        plt.plot(x, y_lin, '--', color=colors[i], linewidth=2, 
                label=f'线性化点 x={x0}')
        plt.plot(x0, test_function(x0), 'o', color=colors[i], markersize=8)
    
    plt.grid(True, alpha=0.3)
    plt.xlabel('x')
    plt.ylabel('f(x)')
    plt.title('不同点的线性化')
    plt.legend()
    
    # 误差分析
    plt.subplot(2, 3, 2)
    for i, x0 in enumerate(linearization_points):
        y_lin = test_function_linear(x, x0)
        error = np.abs(test_function(x) - y_lin)
        plt.plot(x, error, color=colors[i], linewidth=2, 
                label=f'线性化点 x={x0}')
    
    plt.grid(True, alpha=0.3)
    plt.xlabel('x')
    plt.ylabel('|误差|')
    plt.title('线性化误差')
    plt.legend()
    
    # 误差随距离的变化
    plt.subplot(2, 3, 3)
    distances = np.linspace(0, 2, 50)
    x0 = 0  # 固定线性化点
    
    errors = []
    for dist in distances:
        x_test = x0 + dist
        y_orig = test_function(x_test)
        y_lin = test_function_linear(x_test, x0)
        error = np.abs(y_orig - y_lin)
        errors.append(error)
    
    plt.plot(distances, errors, 'b-', linewidth=2)
    plt.grid(True, alpha=0.3)
    plt.xlabel('距离线性化点的距离')
    plt.ylabel('|误差|')
    plt.title('误差随距离的变化')
    
    # 二阶导数分析
    plt.subplot(2, 3, 4)
    def second_derivative(x):
        return -np.sin(x) + 0.2
    
    plt.plot(x, second_derivative(x), 'g-', linewidth=2)
    plt.grid(True, alpha=0.3)
    plt.xlabel('x')
    plt.ylabel('f\'\'(x)')
    plt.title('二阶导数')
    
    # 线性化质量评估
    plt.subplot(2, 3, 5)
    x0_values = np.linspace(-2, 2, 20)
    max_errors = []
    
    for x0 in x0_values:
        x_test = np.linspace(x0-1, x0+1, 50)
        y_orig = test_function(x_test)
        y_lin = test_function_linear(x_test, x0)
        max_error = np.max(np.abs(y_orig - y_lin))
        max_errors.append(max_error)
    
    plt.plot(x0_values, max_errors, 'r-', linewidth=2)
    plt.grid(True, alpha=0.3)
    plt.xlabel('线性化点')
    plt.ylabel('最大误差')
    plt.title('不同线性化点的最大误差')
    
    # 最佳线性化点
    plt.subplot(2, 3, 6)
    best_x0 = x0_values[np.argmin(max_errors)]
    y_lin_best = test_function_linear(x, best_x0)
    
    plt.plot(x, test_function(x), 'k-', linewidth=3, label='原始函数')
    plt.plot(x, y_lin_best, 'r--', linewidth=2, label=f'最佳线性化点 x={best_x0:.2f}')
    plt.plot(best_x0, test_function(best_x0), 'ro', markersize=8)
    plt.grid(True, alpha=0.3)
    plt.xlabel('x')
    plt.ylabel('f(x)')
    plt.title('最佳线性化点')
    plt.legend()
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 运行所有例子
    example1_simple_function()
    example2_2d_function()
    example2_2d1_function()
    example3_pendulum_linearization()
    example4_vehicle_kinematics()
    example5_linearization_accuracy()
    
    print("所有线性化例子演示完成！") 