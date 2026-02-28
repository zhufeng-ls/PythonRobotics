"""
测试导入路径
"""
import sys
import os

print("当前工作目录:", os.getcwd())
print("\nPython 路径:")
for p in sys.path[:5]:
    print(f"  - {p}")

# 测试路径
path_to_project = '/home/zhufeng/code/PythonRobotics/PythonRobotics'
print(f"\n测试路径: {path_to_project}")
print(f"路径存在: {os.path.exists(path_to_project)}")

if os.path.exists(path_to_project):
    sys.path.append(path_to_project)
    print(f"已添加到 sys.path")

    # 列出 PathPlanning 目录
    pp_path = os.path.join(path_to_project, 'PathPlanning')
    if os.path.exists(pp_path):
        print(f"\nPathPlanning 目录内容:")
        for item in os.listdir(pp_path)[:10]:
            print(f"  - {item}")

    # 尝试直接运行 a_star.py
    astar_path = os.path.join(pp_path, 'AStar', 'a_star.py')
    print(f"\n尝试直接运行: {astar_path}")
    print(f"文件存在: {os.path.exists(astar_path)}")
