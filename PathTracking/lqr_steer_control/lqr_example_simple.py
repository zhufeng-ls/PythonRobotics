"""
LQR 概念理解示例集合
包含从简单到复杂的LQR控制示例

author: AI Assistant for learning
"""
import numpy as np
import matplotlib.pyplot as plt
import scipy.linalg as la
import sys
import pathlib

# 添加项目根路径到Python搜索路径
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

# 配置matplotlib中文字体支持
def setup_chinese_font():
    """配置matplotlib支持中文显示"""
    import matplotlib
    import matplotlib.pyplot as plt
    import platform
    import warnings
    
    # 临时抑制字体警告
    warnings.filterwarnings('ignore', category=UserWarning, module='matplotlib')
    
    # 检查操作系统并设置合适的中文字体
    system = platform.system()
    
    if system == "Linux":
        # Linux系统中文字体（按优先级排序）
        fonts = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Noto Sans CJK SC', 'Source Han Sans CN']
    elif system == "Darwin":  # macOS
        fonts = ['PingFang SC', 'Hiragino Sans GB', 'STHeiti', 'Arial Unicode MS']
    else:  # Windows
        fonts = ['Microsoft YaHei', 'SimHei', 'SimSun', 'KaiTi']
    
    # 测试字体是否支持中文
    def test_chinese_support(font_name):
        """测试字体是否真正支持中文字符"""
        try:
            matplotlib.rcParams['font.sans-serif'] = [font_name]
            matplotlib.rcParams['axes.unicode_minus'] = False
            
            # 创建一个临时图形测试中文渲染
            fig, ax = plt.subplots(figsize=(1, 1))
            ax.text(0.5, 0.5, '测试', fontsize=12)
            
            # 检查是否有警告（更严格的检测）
            with warnings.catch_warnings(record=True) as w:
                warnings.simplefilter("always")
                fig.canvas.draw()
                plt.close(fig)
                
                # 如果有关于字符缺失的警告，说明不支持中文
                for warning in w:
                    if 'missing from font' in str(warning.message):
                        return False
                return True
        except:
            return False
    
    # 尝试设置字体
    for font in fonts:
        if test_chinese_support(font):
            print(f"✅ 中文字体设置成功: {font}")
            return True
    
    # 如果没有找到合适的中文字体，使用英文
    print("⚠️ 未找到支持中文的字体，将使用英文标签")
    matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans']
    matplotlib.rcParams['axes.unicode_minus'] = False
    return False

# 设置中文字体
CHINESE_FONT_AVAILABLE = setup_chinese_font()

def demo1_simple_position_control():
    """
    演示1: 最简单的LQR - 控制物体回到原点
    系统: 一个可以加速的质点，目标是回到位置0
    """
    print("🎯 演示1: 简单位置控制")
    print("="*50)
    print("场景：一个小车偏离了中心线，需要控制它回到中心")
    
    # 系统模型: x[k+1] = A*x[k] + B*u[k]
    # 状态: [位置, 速度]
    # 控制: [加速度]
    dt = 0.1
    A = np.array([[1, dt],    # 位置积分
                  [0, 1]])    # 速度保持
    B = np.array([[0],        # 加速度对位置的间接影响
                  [dt]])      # 加速度对速度的直接影响
    
    # 设计目标 (代价函数权重)
    Q = np.array([[10, 0],    # 位置误差很重要
                  [0, 1]])    # 速度误差一般重要
    R = np.array([[1]])       # 控制成本适中
    
    # 求解LQR
    K, S, E = solve_lqr(A, B, Q, R)
    
    print(f"LQR增益: K = [{K[0,0]:.2f}, {K[0,1]:.2f}]")
    print(f"控制律: u = -{K[0,0]:.2f}*位置 - {K[0,1]:.2f}*速度")
    print("解释: 位置越远，控制越强；速度越大，制动越强")
    
    # 仿真
    title = "Simple Position Control" if not CHINESE_FONT_AVAILABLE else "简单位置控制"
    simulate_system(A, B, K, initial_state=[5.0, 0.0], steps=50, title=title)
    
    return A, B, K

def demo2_car_lateral_control():
    """
    演示2: 车辆横向控制 (更接近实际应用)
    """
    print("\n🚗 演示2: 车辆横向控制")
    print("="*50)
    print("场景：车辆在高速公路上，需要保持在车道中心")
    
    # 更复杂的车辆模型
    dt = 0.1
    v = 20.0  # 纵向速度 m/s (72 km/h)
    
    # 状态: [横向位置, 横向速度, 航向角, 航向角速度]
    A = np.array([[1, dt, v*dt, 0],
                  [0, 1,  v,    0],
                  [0, 0,  1,    dt],
                  [0, 0,  0,    1]])
    
    B = np.array([[0],
                  [0],
                  [0],
                  [1]])  # 控制输入是航向角加速度
    
    # 权重设计
    Q = np.array([[100, 0,   0,  0],    # 横向位置误差最重要
                  [0,   10,  0,  0],    # 横向速度误差
                  [0,   0,   50, 0],    # 航向角误差重要
                  [0,   0,   0,  1]])   # 航向角速度误差
    
    R = np.array([[1]])  # 控制成本
    
    K, S, E = solve_lqr(A, B, Q, R)
    
    print("增益矩阵 K:")
    print(f"  横向位置系数: {K[0,0]:.2f}")
    print(f"  横向速度系数: {K[0,1]:.2f}")
    print(f"  航向角系数:   {K[0,2]:.2f}")
    print(f"  航向角速度系数: {K[0,3]:.2f}")
    
    # 仿真：车辆偏离车道中心2米
    if CHINESE_FONT_AVAILABLE:
        title = "车辆横向控制"
        state_labels = ['横向位置[m]', '横向速度[m/s]', '航向角[rad]', '航向角速度[rad/s]']
    else:
        title = "Vehicle Lateral Control"
        state_labels = ['Lateral Pos[m]', 'Lateral Vel[m/s]', 'Heading[rad]', 'Heading Rate[rad/s]']
    
    simulate_system(A, B, K, initial_state=[2.0, 0.0, 0.1, 0.0], 
                   steps=100, title=title, state_labels=state_labels)

def demo3_parameter_comparison():
    """
    演示3: 不同参数对控制效果的影响
    """
    print("\n🔧 演示3: 参数调整实验")
    print("="*50)
    
    # 基础系统
    dt = 0.1
    A = np.array([[1, dt], [0, 1]])
    B = np.array([[0], [dt]])
    
    # 不同参数配置
    configs = [
        {"Q": np.diag([10, 1]), "R": np.array([[1]]), "name": "平衡配置", "color": 'blue'},
        {"Q": np.diag([100, 1]), "R": np.array([[1]]), "name": "重视精度", "color": 'red'},
        {"Q": np.diag([10, 1]), "R": np.array([[10]]), "name": "节省控制", "color": 'green'},
        {"Q": np.diag([1, 20]), "R": np.array([[1]]), "name": "重视稳定", "color": 'orange'}
    ]
    
    plt.figure(figsize=(12, 8))
    
    for i, config in enumerate(configs):
        K, _, _ = solve_lqr(A, B, config["Q"], config["R"])
        
        # 仿真同样的初始条件
        x = np.array([[3.0], [0.0]])
        positions = []
        times = []
        
        for step in range(50):
            positions.append(x[0, 0])
            times.append(step * dt)
            u = -K @ x
            x = A @ x + B @ u
        
        plt.subplot(2, 2, i+1)
        plt.plot(times, positions, color=config["color"], linewidth=2)
        plt.axhline(y=0, color='black', linestyle='--', alpha=0.5)
        plt.title(f'{config["name"]}\nK=[{K[0,0]:.1f}, {K[0,1]:.1f}]')
        
        if CHINESE_FONT_AVAILABLE:
            plt.xlabel('时间 [s]')
            plt.ylabel('位置 [m]')
        else:
            plt.xlabel('Time [s]')
            plt.ylabel('Position [m]')
        plt.grid(True, alpha=0.3)
        
        print(f"{config['name']}: K = [{K[0,0]:.2f}, {K[0,1]:.2f}]")
    
    plt.tight_layout()
    if CHINESE_FONT_AVAILABLE:
        plt.suptitle('不同Q、R参数的控制效果比较', y=1.02)
    else:
        plt.suptitle('Comparison of Different Q,R Parameters', y=1.02)
    plt.show()

def demo4_path_tracking_simplified():
    """
    演示4: 简化的路径跟踪 (连接到实际应用)
    """
    print("\n🛣️ 演示4: 简化路径跟踪")
    print("="*50)
    print("场景：车辆需要跟随一条预定路径")
    
    # 路径跟踪的LQR模型
    dt = 0.1
    v = 10.0  # 车速
    
    # 状态: [横向误差, 横向误差变化率, 航向误差, 航向误差变化率]
    A = np.array([[1, dt, v*dt, 0],
                  [0, 1,  v,    0],
                  [0, 0,  1,    dt],
                  [0, 0,  0,    1]])
    
    B = np.array([[0], [0], [0], [v/2.0]])  # 简化的转向影响 (假设轴距L=2.0)
    
    # 权重矩阵 (类似真实LQR路径跟踪)
    Q = np.eye(4)  # 单位权重
    R = np.array([[1]])
    
    K, _, _ = solve_lqr(A, B, Q, R)
    
    print("这就是路径跟踪LQR的基本形式！")
    print(f"横向误差增益: {K[0,0]:.2f}")
    print(f"横向误差率增益: {K[0,1]:.2f}")
    print(f"航向误差增益: {K[0,2]:.2f}")
    print(f"航向误差率增益: {K[0,3]:.2f}")
    
    # 仿真路径跟踪
    if CHINESE_FONT_AVAILABLE:
        title = "简化路径跟踪"
        state_labels = ['横向误差[m]', '横向误差率[m/s]', '航向误差[rad]', '航向误差率[rad/s]']
    else:
        title = "Simplified Path Tracking"
        state_labels = ['Lateral Error[m]', 'Lateral Error Rate[m/s]', 'Heading Error[rad]', 'Heading Error Rate[rad/s]']
    
    simulate_system(A, B, K, initial_state=[1.0, 0.0, 0.2, 0.0], 
                   steps=80, title=title, state_labels=state_labels)

def solve_lqr(A, B, Q, R):
    """求解LQR控制器"""
    try:
        # 使用scipy求解离散代数Riccati方程
        S = la.solve_discrete_are(A, B, Q, R)
        # 计算LQR增益
        K = la.inv(R + B.T @ S @ B) @ (B.T @ S @ A)
        # 闭环特征值
        E = la.eigvals(A - B @ K)
        return K, S, E
    except Exception as e:
        print(f"LQR求解失败: {e}")
        return None, None, None

def simulate_system(A, B, K, initial_state, steps=50, title="LQR Simulation", state_labels=None):
    """仿真系统响应"""
    if K is None:
        print("无法仿真：LQR求解失败")
        return
    
    x = np.array(initial_state).reshape(-1, 1)
    n_states = len(initial_state)
    
    # 数据记录
    states_history = [[] for _ in range(n_states)]
    controls = []
    times = []
    
    print(f"\n仿真开始 - 初始状态: {initial_state}")
    
    for i in range(steps):
        # LQR控制
        u = -K @ x
        
        # 记录数据
        for j in range(n_states):
            states_history[j].append(x[j, 0])
        controls.append(u[0, 0])
        times.append(i * 0.1)
        
        # 系统更新
        x = A @ x + B @ u
        
        # 显示前几步
        if i < 3:
            print(f"  步骤{i+1}: 状态={[f'{x[j,0]:.3f}' for j in range(n_states)]}, 控制={u[0,0]:.3f}")
    
    print(f"最终状态: {[f'{x[j,0]:.3f}' for j in range(n_states)]}")
    
    # 绘图 - 根据字体支持情况选择标签语言
    fig_rows = n_states + 1
    plt.figure(figsize=(10, 2*fig_rows))
    
    # 设置标签文本
    if CHINESE_FONT_AVAILABLE:
        target_label = '目标值'
        control_label = '控制输入'
        time_label = '时间 [s]'
        control_ylabel = '控制量'
        title_suffix = ' - LQR控制效果'
    else:
        target_label = 'Target'
        control_label = 'Control Input'
        time_label = 'Time [s]'
        control_ylabel = 'Control'
        title_suffix = ' - LQR Control Effect'
    
    # 默认状态标签
    if state_labels is None:
        if CHINESE_FONT_AVAILABLE:
            state_labels = [f'状态{i+1}' for i in range(n_states)]
        else:
            state_labels = [f'State{i+1}' for i in range(n_states)]
    
    # 绘制每个状态
    for i in range(n_states):
        plt.subplot(fig_rows, 1, i+1)
        plt.plot(times, states_history[i], 'b-', linewidth=2, label=state_labels[i])
        plt.axhline(y=0, color='r', linestyle='--', alpha=0.5, label=target_label)
        plt.ylabel(state_labels[i])
        plt.legend()
        plt.grid(True, alpha=0.3)
        if i == 0:
            plt.title(f'{title}{title_suffix}')
    
    # 绘制控制输入
    plt.subplot(fig_rows, 1, fig_rows)
    plt.plot(times, controls, 'r-', linewidth=2, label=control_label)
    plt.xlabel(time_label)
    plt.ylabel(control_ylabel)
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def interactive_lqr_tuning():
    """
    交互式LQR参数调整 (演示如何调参)
    """
    print("\n🎛️ 交互式参数调整指南")
    print("="*50)
    print("LQR调参原则:")
    print("1. Q矩阵对角元素 ↑ → 对应状态误差控制更严格")
    print("2. R矩阵对角元素 ↑ → 控制输入更保守，动作更温和")
    print("3. 平衡精度与控制成本的权衡")
    
    # 基础系统
    dt = 0.1
    A = np.array([[1, dt], [0, 1]])
    B = np.array([[0], [dt]])
    
    # 展示调参效果
    Q_values = [1, 10, 100]
    R_values = [0.1, 1, 10]
    
    print(f"\n参数组合效果预览:")
    print(f"{'Q_pos':<6} {'R':<6} {'K_pos':<8} {'K_vel':<8} {'特点'}")
    print("-" * 50)
    
    for Q_pos in Q_values:
        for R_val in R_values:
            Q = np.diag([Q_pos, 1])
            R = np.array([[R_val]])
            K, _, _ = solve_lqr(A, B, Q, R)
            
            if K is not None:
                feature = ""
                if K[0,0] > 5: feature += "快速响应 "
                if K[0,0] < 2: feature += "慢响应 "
                if abs(K[0,1]) > 2: feature += "强阻尼"
                if abs(K[0,1]) < 1: feature += "弱阻尼"
                
                print(f"{Q_pos:<6} {R_val:<6} {K[0,0]:<8.2f} {K[0,1]:<8.2f} {feature}")

def main():
    """主函数 - 运行所有演示"""
    print("🎓 LQR 完整学习教程")
    print("="*60)
    print("本教程将通过4个递进的演示帮您理解LQR:")
    print("1. 最简单的位置控制")
    print("2. 车辆横向控制") 
    print("3. 参数调整对比")
    print("4. 简化路径跟踪")
    print("5. 参数调整指南")
    
    try:
        # 演示1: 基础概念
        demo1_simple_position_control()
        
        # 演示2: 实际应用
        demo2_car_lateral_control()
        
        # 演示3: 参数影响
        demo3_parameter_comparison()
        
        # 演示4: 路径跟踪
        demo4_path_tracking_simplified()
        
        # 演示5: 调参指南
        interactive_lqr_tuning()
        
        print("\n🎉 恭喜！您已经完成了LQR学习")
        print("="*50)
        print("关键概念总结:")
        print("• LQR = 最优控制理论的经典应用")
        print("• 状态空间模型: x[k+1] = A*x[k] + B*u[k]")
        print("• 控制律: u = -K*x (K是LQR计算的增益)")
        print("• Q矩阵: 状态误差权重 (越大越严格)")
        print("• R矩阵: 控制成本权重 (越大越保守)")
        print("• 实际应用: 路径跟踪、姿态控制、机器人控制等")
        
    except Exception as e:
        print(f"演示过程中发生错误: {e}")
        print("请检查依赖包是否正确安装 (numpy, scipy, matplotlib)")

if __name__ == '__main__':
    main() 