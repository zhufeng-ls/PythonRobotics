import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

def plot_convex_vs_non_convex():
    # Set up the figure
    fig = plt.figure(figsize=(12, 5))
    
    # 1. Convex Function (Bowl)
    ax1 = fig.add_subplot(121, projection='3d')
    x = np.linspace(-5, 5, 100)
    y = np.linspace(-5, 5, 100)
    X, Y = np.meshgrid(x, y)
    Z1 = X**2 + Y**2  # Simple quadratic function
    
    surf1 = ax1.plot_surface(X, Y, Z1, cmap=cm.viridis, alpha=0.8)
    ax1.set_title("Convex Function (The Bowl)\nGlobal Minimum = Local Minimum", fontsize=12, pad=20)
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")
    ax1.set_zlabel("Cost J")
    
    # Add a ball rolling down (schematic)
    ax1.scatter([0], [0], [0], color='red', s=100, label='Global Minimum')
    ax1.legend()

    # 2. Non-Convex Function (Mountains)
    ax2 = fig.add_subplot(122, projection='3d')
    Z2 = X**2 + Y**2 + 10 * np.sin(X) * np.sin(Y)  # Quadratic + Sine waves
    
    surf2 = ax2.plot_surface(X, Y, Z2, cmap=cm.plasma, alpha=0.8)
    ax2.set_title("Non-Convex Function (The Mountains)\nMany Local Minima traps", fontsize=12, pad=20)
    ax2.set_xlabel("x")
    ax2.set_ylabel("y")
    ax2.set_zlabel("Cost J")
    
    # Add local minima
    # Approximate local minima locations for visual
    local_min_x = [2.0, -2.0, 2.0, -2.0]
    local_min_y = [2.0, 2.0, -2.0, -2.0]
    local_min_z = [Z2[int((y+5)*10), int((x+5)*10)] for x, y in zip(local_min_x, local_min_y)]
    
    ax2.scatter(local_min_x, local_min_y, [10]*4, color='black', s=50, label='Local Minima (Traps)')
    ax2.scatter([0], [0], [0], color='red', s=100, label='Global Minimum')
    ax2.legend()
    
    plt.tight_layout()
    
    # Save the plot
    output_path = "/home/zhufeng/code/PythonRobotics/PythonRoboticsLearning/notes/img/convex_vs_non_convex.png"
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Image saved to {output_path}")

if __name__ == "__main__":
    plot_convex_vs_non_convex()
