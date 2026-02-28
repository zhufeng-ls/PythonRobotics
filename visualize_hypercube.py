
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math

class HypercubeVisualizer:
    def __init__(self):
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title("4D Hypercube Projection")
        
        # Define vertices of a 4D hypercube
        # 16 vertices: (+/-1, +/-1, +/-1, +/-1)
        vertices_list = []
        for i in [-1, 1]:
            for j in [-1, 1]:
                for k in [-1, 1]:
                    for w in [-1, 1]:
                        vertices_list.append([i, j, k, w])
        self.vertices = np.array(vertices_list)
        
        # Define edges (pairs of vertex indices)
        self.edges = []
        for i in range(len(self.vertices)):
            for j in range(i + 1, len(self.vertices)):
                # Connect if vertices differ by exactly one coordinate
                # Hamming distance = 1
                diff = np.sum(np.abs(self.vertices[i] - self.vertices[j]))
                if diff == 2: # Since coordinates differ by 2 (-1 to 1) 
                     self.edges.append((i, j))

    def rotation_matrix(self, angle, plane):
        """
        Create 4D rotation matrix.
        plane: tuple of indices (i, j) for the rotation plane.
        0:x, 1:y, 2:z, 3:w
        """
        c = np.cos(angle)
        s = np.sin(angle)
        mat = np.eye(4)
        
        idx1, idx2 = plane
        mat[idx1, idx1] = c
        mat[idx1, idx2] = -s
        mat[idx2, idx1] = s
        mat[idx2, idx2] = c
        return mat

    def project_4d_to_3d(self, vertices_4d, w_distance=3):
        """
        Project 4D points to 3D using perspective projection.
        v_3d = v_4d * (1 / (w_distance - w))
        """
        vertices_3d = []
        for v in vertices_4d:
            w = v[3]
            factor = 1 / (w_distance - w)
            proj_matrix = np.eye(4)
            proj_matrix = proj_matrix[:3, :] # Take top 3 rows
            
            # Simple perspective projection logic
            # x' = x * factor
            # y' = y * factor
            # z' = z * factor
            
            projected = v[:3] * factor
            vertices_3d.append(projected)
        return np.array(vertices_3d)

    def update(self, frame):
        self.ax.clear()
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        self.ax.set_zlim(-2, 2)
        self.ax.set_axis_off() # Hide axes for cleaner look
        
        # Rotation angles based on frame
        angle_xw = frame * 0.02
        angle_zw = frame * 0.01
        angle_yz = frame * 0.015
        
        # Apply combined rotations
        rot_xw = self.rotation_matrix(angle_xw, (0, 3)) # Rotate in X-W plane
        rot_zw = self.rotation_matrix(angle_zw, (2, 3)) # Rotate in Z-W plane
        rot_yz = self.rotation_matrix(angle_yz, (1, 2)) # Rotate in Y-Z plane for 3D spin
        
        # Transform vertices
        transformed_vertices = self.vertices.T # 4x16
        transform = rot_xw @ rot_zw @ rot_yz
        transformed_vertices = transform @ transformed_vertices
        transformed_vertices = transformed_vertices.T # 16x4
        
        # Project to 3D
        projected_vertices = self.project_4d_to_3d(transformed_vertices)
        
        # Draw edges
        for edge in self.edges:
            p1 = projected_vertices[edge[0]]
            p2 = projected_vertices[edge[1]]
            self.ax.plot(
                [p1[0], p2[0]], 
                [p1[1], p2[1]], 
                [p1[2], p2[2]], 
                color='cyan', alpha=0.6, linewidth=1.5
            )
            
        # Draw vertices
        self.ax.scatter(
            projected_vertices[:, 0],
            projected_vertices[:, 1],
            projected_vertices[:, 2],
            color='white', s=20
        )
        
        # Set dark background
        self.fig.patch.set_facecolor('black')
        self.ax.set_facecolor('black')
        
    def animate(self):
        ani = FuncAnimation(self.fig, self.update, frames=range(200), interval=50)
        plt.show()

if __name__ == "__main__":
    visualizer = HypercubeVisualizer()
    visualizer.animate()
