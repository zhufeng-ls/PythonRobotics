import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

def plot_linear_constraints():
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Define a convex polygon using linear constraints (half-spaces)
    # x >= 1
    # y >= 1
    # 2x + y <= 10
    # x + 2y <= 10
    
    # Vertices of the polygon formed by these lines
    verts = [
        (1, 1),
        (4.5, 1),  # Intersection of y=1 and 2x+y=10 -> 2x+1=10 -> x=4.5
        (10/3, 10/3), # Intersection of 2x+y=10 and x+2y=10
        (1, 4.5),  # Intersection of x=1 and x+2y=10 -> 1+2y=10 -> y=4.5
        (1, 1)     # Close the loop
    ]
    
    codes = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY]
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor='#add8e6', lw=2, alpha=0.5, label='Feasible Region (Convex Set)')
    ax.add_patch(patch)
    
    # Plot the constraint lines
    x = np.linspace(0, 7, 100)
    
    # y >= 1
    plt.axhline(y=1, color='gray', linestyle='--', label='y >= 1')
    
    # x >= 1
    plt.axvline(x=1, color='gray', linestyle='--', label='x >= 1')
    
    # 2x + y <= 10  => y <= 10 - 2x
    plt.plot(x, 10 - 2*x, 'r--', label='2x + y <= 10')
    
    # x + 2y <= 10 => y <= (10 - x) / 2
    plt.plot(x, (10 - x) / 2, 'g--', label='x + 2y <= 10')
    
    # The "Rope Test" (Convexity definition)
    p1 = (1.5, 2.0)
    p2 = (3.0, 3.0)
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-', linewidth=2, marker='o', label='Rope Test')
    
    ax.text(1.3, 2.1, "A", fontsize=12, fontweight='bold')
    ax.text(3.1, 3.1, "B", fontsize=12, fontweight='bold')
    ax.text(2.2, 2.5, "All points on line AB\nare inside the region", fontsize=10, rotation=30)

    # Set limits and labels
    ax.set_xlim(0, 6)
    ax.set_ylim(0, 6)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_title('Why Linear Constraints Form a Convex Set', fontsize=14)
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_path = "/home/zhufeng/code/PythonRobotics/PythonRoboticsLearning/notes/img/linear_convex_set.png"
    plt.savefig(output_path, dpi=300)
    print(f"Image saved to {output_path}")

if __name__ == "__main__":
    plot_linear_constraints()
