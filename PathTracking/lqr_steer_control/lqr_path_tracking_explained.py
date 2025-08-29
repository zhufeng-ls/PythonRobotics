"""
路径跟踪LQR详细解析
深入理解原始lqr_steer_control.py的实现原理

author: AI Assistant for learning
"""
import numpy as np
import matplotlib.pyplot as plt
import scipy.linalg as la
import sys
import pathlib
import math

# 添加项目根路径
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
from utils.angle import angle_mod

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

class LQRPathTrackingExplained:
    """路径跟踪LQR控制器详细解析类"""
    
    def __init__(self):
        # 与原文件相同的参数
        self.dt = 0.1  # 时间步长
        self.L = 0.5   # 车辆轴距
        self.Q = np.eye(4)  # 状态权重矩阵
        self.R = np.eye(1)  # 控制权重矩阵
        
        print("🚗 路径跟踪LQR控制器初始化")
        print(f"   时间步长: {self.dt}s")
        print(f"   车辆轴距: {self.L}m")
        print(f"   状态权重矩阵Q: 4x4单位矩阵")
        print(f"   控制权重矩阵R: 1x1单位矩阵")
    
    def explain_state_space_model(self, v=10.0):
        """
        详细解释状态空间模型的构建过程
        """
        print(f"\n📐 状态空间模型构建 (车速v={v}m/s)")
        print("="*60)
        
        # 状态向量定义
        print("状态向量 x = [e, de/dt, θe, dθe/dt]:")
        print("  e      : 横向误差 (车辆位置相对于路径的横向偏移)")
        print("  de/dt  : 横向误差变化率")
        print("  θe     : 航向误差 (车辆朝向与路径切线的夹角)")
        print("  dθe/dt : 航向误差变化率")
        print("\n控制输入 u = [δ]: 转向角")
        
        # 构建A矩阵
        A = np.zeros((4, 4))
        A[0, 0] = 1.0    # e[k+1] = e[k] + ...
        A[0, 1] = self.dt     # ... + de/dt * dt
        A[1, 2] = v      # de/dt[k+1] = v * θe[k] (运动学关系)
        A[2, 2] = 1.0    # θe[k+1] = θe[k] + ...
        A[2, 3] = self.dt     # ... + dθe/dt * dt
        
        print(f"\n矩阵A (状态转移矩阵):")
        print("  A[0,0]=1, A[0,1]=dt : 位置积分")
        print("  A[1,2]=v            : 航向误差导致横向误差变化")
        print("  A[2,2]=1, A[2,3]=dt : 航向角积分")
        print("  其他元素=0")
        print(f"\nA = \n{A}")
        
        # 构建B矩阵
        B = np.zeros((4, 1))
        B[3, 0] = v / self.L  # 转向角对航向角变化率的影响
        
        print(f"\n矩阵B (控制输入矩阵):")
        print(f"  B[3,0] = v/L = {v}/{self.L} = {v/self.L:.1f}")
        print("  解释: 转向角δ通过自行车模型影响航向角变化率")
        print("       dθe/dt = v/L * tan(δ) ≈ v/L * δ (小角度近似)")
        print(f"\nB = \n{B}")
        
        return A, B
    
    def explain_lqr_solution(self, A, B):
        """
        详细解释LQR求解过程
        """
        print(f"\n⚖️ LQR最优控制问题求解")
        print("="*60)
        
        print("目标函数 (代价函数):")
        print("J = Σ [x^T * Q * x + u^T * R * u]")
        print("目标: 最小化状态误差和控制输入的加权平方和")
        
        print(f"\nQ矩阵 (状态权重):")
        print(f"{self.Q}")
        print("解释: 对角元素表示对各状态误差的关注程度")
        print("  Q[0,0]=1: 横向误差权重")
        print("  Q[1,1]=1: 横向误差变化率权重")
        print("  Q[2,2]=1: 航向误差权重")
        print("  Q[3,3]=1: 航向误差变化率权重")
        
        print(f"\nR矩阵 (控制权重):")
        print(f"{self.R}")
        print("解释: R[0,0]=1表示对转向角大小的约束")
        
        # 求解LQR
        try:
            S = la.solve_discrete_are(A, B, self.Q, self.R)
            K = la.inv(self.R + B.T @ S @ B) @ (B.T @ S @ A)
            
            print(f"\n🎯 LQR求解结果:")
            print(f"增益矩阵 K = {K}")
            print(f"\n控制律: u = -K * x")
            print(f"  δ = -{K[0,0]:.3f}*e - {K[0,1]:.3f}*de/dt - {K[0,2]:.3f}*θe - {K[0,3]:.3f}*dθe/dt")
            
            print(f"\n物理意义:")
            print(f"  横向误差系数 {K[0,0]:.3f}: 偏离路径越远，转向越大")
            print(f"  横向误差率系数 {K[0,1]:.3f}: 横向运动越快，修正越强")
            print(f"  航向误差系数 {K[0,2]:.3f}: 朝向偏差越大，转向越大")
            print(f"  航向误差率系数 {K[0,3]:.3f}: 航向变化越快，抑制越强")
            
            return K
            
        except Exception as e:
            print(f"LQR求解失败: {e}")
            return None
    
    def simulate_step_by_step(self, K, initial_error=[1.0, 0.0, 0.2, 0.0], v=10.0):
        """
        逐步仿真展示控制过程
        """
        print(f"\n🎮 逐步仿真演示")
        print("="*60)
        print(f"初始误差状态: {initial_error}")
        print(f"车速: {v} m/s")
        
        A, B = self.explain_state_space_model(v)
        x = np.array(initial_error).reshape(4, 1)
        
        print(f"\n详细仿真过程:")
        for step in range(5):
            print(f"\n--- 步骤 {step+1} ---")
            print(f"当前状态: e={x[0,0]:.3f}m, de/dt={x[1,0]:.3f}m/s, θe={x[2,0]:.3f}rad, dθe/dt={x[3,0]:.3f}rad/s")
            
            # LQR控制
            u = -K @ x
            delta = u[0, 0]
            
            print(f"LQR控制计算:")
            print(f"  δ = -{K[0,0]:.3f}×{x[0,0]:.3f} - {K[0,1]:.3f}×{x[1,0]:.3f} - {K[0,2]:.3f}×{x[2,0]:.3f} - {K[0,3]:.3f}×{x[3,0]:.3f}")
            print(f"  δ = {delta:.3f} rad = {np.rad2deg(delta):.1f}°")
            
            # 状态更新
            x_new = A @ x + B @ u
            
            print(f"状态更新:")
            print(f"  新状态: e={x_new[0,0]:.3f}m, de/dt={x_new[1,0]:.3f}m/s, θe={x_new[2,0]:.3f}rad, dθe/dt={x_new[3,0]:.3f}rad/s")
            
            error_reduction = abs(x[0,0]) - abs(x_new[0,0])
            print(f"  横向误差变化: {error_reduction:.3f}m ({'减小' if error_reduction > 0 else '增大'})")
            
            x = x_new
            
            if step < 4:
                input("按回车继续下一步...")
    
    def compare_with_original_code(self):
        """
        对比原始代码实现
        """
        print(f"\n🔍 与原始lqr_steer_control.py的对比")
        print("="*60)
        
        print("原始代码关键函数解析:")
        print("\n1. lqr_steering_control() 函数:")
        print("   - calc_nearest_index(): 找到最近的路径点")
        print("   - 构建A、B矩阵 (与我们的解析完全一致)")
        print("   - dlqr(): 求解LQR增益矩阵K")
        print("   - 构建状态向量x: [e, de/dt, θe, dθe/dt]")
        print("   - 计算控制: ff + fb")
        
        print("\n2. 前馈+反馈控制:")
        print("   ff = atan2(L*k, 1)  # 前馈: 基于路径曲率的转向")
        print("   fb = -K@x           # 反馈: LQR基于误差的修正")
        print("   delta = ff + fb     # 总转向角")
        
        print("\n3. 关键差异:")
        print("   - 原始代码包含前馈控制 (路径曲率补偿)")
        print("   - 原始代码处理实际路径跟踪 (曲线路径)")
        print("   - 我们的示例简化为直线稳定问题")
    
    def visualize_control_effect(self, K, v=10.0):
        """
        可视化控制效果
        """
        print(f"\n📊 控制效果可视化")
        print("="*60)
        
        A, B = self.explain_state_space_model(v)
        
        # 不同初始条件的仿真
        initial_conditions = [
            [2.0, 0.0, 0.0, 0.0],    # 只有横向误差
            [0.0, 0.0, 0.3, 0.0],    # 只有航向误差
            [1.0, 0.0, 0.2, 0.0],    # 混合误差
            [-1.5, 0.5, -0.1, 0.0]   # 复杂初始条件
        ]
        
        plt.figure(figsize=(15, 10))
        
        for i, init_state in enumerate(initial_conditions):
            x = np.array(init_state).reshape(4, 1)
            
            # 仿真数据
            times = []
            e_history = []
            de_history = []
            theta_e_history = []
            dtheta_e_history = []
            delta_history = []
            
            for step in range(50):
                times.append(step * self.dt)
                e_history.append(x[0, 0])
                de_history.append(x[1, 0])
                theta_e_history.append(x[2, 0])
                dtheta_e_history.append(x[3, 0])
                
                # 控制
                u = -K @ x
                delta_history.append(u[0, 0])
                
                # 更新
                x = A @ x + B @ u
            
            # 绘图
            plt.subplot(2, 2, i+1)
            
            if CHINESE_FONT_AVAILABLE:
                e_label = '横向误差 e[m]'
                theta_label = '航向误差 θe×10[rad]'
                delta_label = '转向角 δ×5[rad]'
                title_text = f'初始条件{i+1}: e₀={init_state[0]:.1f}, θe₀={init_state[2]:.1f}'
                xlabel_text = '时间 [s]'
                ylabel_text = '状态量'
            else:
                e_label = 'Lateral Error e[m]'
                theta_label = 'Heading Error θe×10[rad]'
                delta_label = 'Steering δ×5[rad]'
                title_text = f'Initial Condition {i+1}: e₀={init_state[0]:.1f}, θe₀={init_state[2]:.1f}'
                xlabel_text = 'Time [s]'
                ylabel_text = 'States'
            
            plt.plot(times, e_history, 'b-', label=e_label, linewidth=2)
            plt.plot(times, np.array(theta_e_history)*10, 'r-', label=theta_label, linewidth=2)
            plt.plot(times, np.array(delta_history)*5, 'g-', label=delta_label, linewidth=2)
            
            plt.axhline(y=0, color='k', linestyle='--', alpha=0.5)
            plt.title(title_text)
            plt.xlabel(xlabel_text)
            plt.ylabel(ylabel_text)
            plt.legend()
            plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        if CHINESE_FONT_AVAILABLE:
            plt.suptitle('LQR路径跟踪控制效果 (不同初始条件)', y=1.02)
        else:
            plt.suptitle('LQR Path Tracking Control Effect (Different Initial Conditions)', y=1.02)
        plt.show()

def main():
    """主演示函数"""
    print("🎓 路径跟踪LQR详细解析教程")
    print("="*70)
    
    # 创建解析器实例
    explainer = LQRPathTrackingExplained()
    
    # 1. 解释状态空间模型
    A, B = explainer.explain_state_space_model(v=10.0)
    
    # 2. 解释LQR求解
    K = explainer.explain_lqr_solution(A, B)
    
    if K is not None:
        # 3. 逐步仿真
        print("\n是否要进行逐步仿真演示？(y/n): ", end="")
        # response = input().lower()
        # if response == 'y':
        #     explainer.simulate_step_by_step(K)
        
        # 4. 对比原始代码
        explainer.compare_with_original_code()
        
        # 5. 可视化
        explainer.visualize_control_effect(K)
        
        print("\n✅ 路径跟踪LQR学习完成！")
        print("="*50)
        print("🎯 学习要点总结:")
        print("1. 状态空间模型描述了车辆运动学")
        print("2. LQR通过最小化代价函数得到最优增益")
        print("3. 控制律 u = -K*x 实现误差反馈控制")
        print("4. 实际应用中还需要前馈控制补偿路径曲率")
        print("5. 参数Q、R的调整影响控制性能")
    else:
        print("❌ LQR求解失败，请检查系统参数")

if __name__ == '__main__':
    main() 