
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Hypercube5DVisualizer:
    def __init__(self):
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title("5D Hypercube (Penteract) Projection")
        
        # Define vertices of a 5D hypercube
        # 32 vertices: (+/-1, +/-1, +/-1, +/-1, +/-1)
        vertices_list = []
        for i in [-1, 1]:
            for j in [-1, 1]:
                for k in [-1, 1]:
                    for w in [-1, 1]:
                        for v in [-1, 1]:
                            vertices_list.append([i, j, k, w, v])
        self.vertices = np.array(vertices_list)
        
        # Define edges (pairs of vertex indices)
        self.edges = []
        n_vertices = len(self.vertices)
        for i in range(n_vertices):
            for j in range(i + 1, n_vertices):
                # Connect if vertices differ by exactly one coordinate
                diff = np.sum(np.abs(self.vertices[i] - self.vertices[j]))
                if diff == 2: 
                     self.edges.append((i, j))

    def rotation_matrix(self, angle, plane):
        """
        Create 5D rotation matrix.
        plane: tuple of indices (i, j) for the rotation plane.
        0:x, 1:y, 2:z, 3:w, 4:v
        """
        c = np.cos(angle)
        s = np.sin(angle)
        mat = np.eye(5)
        
        idx1, idx2 = plane
        mat[idx1, idx1] = c
        mat[idx1, idx2] = -s
        mat[idx2, idx1] = s
        mat[idx2, idx2] = c
        return mat

    def project_5d_to_4d(self, vertices_5d, v_distance=4):
        """
        Project 5D points to 4D.
        """
        vertices_4d = []
        for p in vertices_5d:
            v_coord = p[4]
            factor = 1 / (v_distance - v_coord)
            projected = p[:4] * factor
            vertices_4d.append(projected)
        return np.array(vertices_4d)

    def project_4d_to_3d(self, vertices_4d, w_distance=4):
        """
        Project 4D points to 3D.
        """
        vertices_3d = []
        for p in vertices_4d:
            w_coord = p[3]
            factor = 1 / (w_distance - w_coord)
            projected = p[:3] * factor
            vertices_3d.append(projected)
        return np.array(vertices_3d)

    def update(self, frame):
        self.ax.clear()
        # Limits need to be adjusted as projection can expand coordinates
        self.ax.set_xlim(-2.5, 2.5)
        self.ax.set_ylim(-2.5, 2.5)
        self.ax.set_zlim(-2.5, 2.5)
        self.ax.set_axis_off() 
        
        # Rotation angles
        # Rotate in 5th dimension (XV plane) and 4th dimension (ZW plane)
        angle_xv = frame * 0.02
        angle_zw = frame * 0.015
        angle_yz = frame * 0.01 
        
        rot_xv = self.rotation_matrix(angle_xv, (0, 4))
        rot_zw = self.rotation_matrix(angle_zw, (2, 3))
        rot_yz = self.rotation_matrix(angle_yz, (1, 2))
        
        # Transform
        transformed = self.vertices.T # 5x32
        transform = rot_xv @ rot_zw @ rot_yz
        transformed = transform @ transformed
        transformed = transformed.T # 32x5
        
        # Project 5D -> 4D
        projected_4d = self.project_5d_to_4d(transformed)
        
        # Project 4D -> 3D
        projected_3d = self.project_4d_to_3d(projected_4d)
        
        # Draw edges
        # In 5D there are many edges (32 * 5 / 2 = 80 edges)
        # We can color them based on their 5th coordinate or just single color
        for edge in self.edges:
            p1 = projected_3d[edge[0]]
            p2 = projected_3d[edge[1]]
            self.ax.plot(
                [p1[0], p2[0]], 
                [p1[1], p2[1]], 
                [p1[2], p2[2]], 
                color='magenta', alpha=0.3, linewidth=1
            )
            
        # Draw vertices
        self.ax.scatter(
            projected_3d[:, 0],
            projected_3d[:, 1],
            projected_3d[:, 2],
            color='white', s=10
        )
        
        # Set dark background
        self.fig.patch.set_facecolor('black')
        self.ax.set_facecolor('black')
        
    def animate(self):
        ani = FuncAnimation(self.fig, self.update, frames=range(200), interval=50)
        plt.show()

if __name__ == "__main__":
    visualizer = Hypercube5DVisualizer()
    visualizer.animate()
