import numpy as np
from scipy.spatial import HalfspaceIntersection, ConvexHull
from scipy.optimize import linprog
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import scipy.spatial
import plotly.graph_objects as go
from scipy.spatial import ConvexHull
import numpy as np


def find_feasible_point(halfspaces, margin=0.01):
    """
    Solve Linear Program to find a feasible point strictly inside the polyhedron.
    halfspaces: (N, 4) array where each row is [a, b, c, d] representing a*x + b*y + c*z + d <= 0
    """
    A = halfspaces[:, :3]
    b = -halfspaces[:, 3] - margin  # Shrink polytope a bit to ensure strict feasibility

    # print("A is ", A)
    # print("b is ", b)

    n_vars = 3
    n_halfspaces = A.shape[0]

    c = np.zeros(n_vars)  # Objective is arbitrary (we just need a feasible point)

    res = linprog(c, A_ub=A, b_ub=b, bounds=(None, None))
    if res.success:
        return res.x
    else:
        raise RuntimeError("Failed to find a feasible point inside the inflated polyhedron.")

def hrep_to_vertices(halfspaces):
    """
    Convert H-representation polyhedron to V-representation (list of vertices).
    halfspaces: (N, 4) array where each row is [a, b, c, d] representing a*x + b*y + c*z + d <= 0
    """
    feasible_point = find_feasible_point(halfspaces)
    hs = HalfspaceIntersection(halfspaces, feasible_point)
    return hs.intersections

# Example usage:
# halfspaces = np.array([...])  # Nx4 array with [a, b, c, d]
# vertices = hrep_to_vertices(halfspaces)
# print(vertices)


def plot_hyperplane(ax, normal, d, plane_size=5.0, color='b', alpha=0.2):
    # Ensure the normal is normalized
    normal = normal / np.linalg.norm(normal)
    
    # Find a point on the plane (origin shifted by d)
    if np.abs(normal[2]) > 1e-6:
        point = np.array([0, 0, -d / normal[2]])
    elif np.abs(normal[1]) > 1e-6:
        point = np.array([0, -d / normal[1], 0])
    else:
        point = np.array([-d / normal[0], 0, 0])

    # Create two vectors orthogonal to the normal
    v = np.array([1, 0, 0]) if abs(normal[0]) < 0.9 else np.array([0, 1, 0])
    side1 = np.cross(normal, v)
    side1 /= np.linalg.norm(side1)
    side2 = np.cross(normal, side1)

    # Generate the 4 corners of the plane patch
    corners = []
    for dx in [-1, 1]:
        for dy in [-1, 1]:
            corner = point + plane_size * (dx * side1 + dy * side2)
            corners.append(corner)

    # Plot the plane patch
    poly = Poly3DCollection([corners], alpha=alpha, facecolor=color, edgecolor='k')
    ax.add_collection3d(poly)



def plot_trajectory_and_corridor_with_obs(trajectory, corridors, obstacles):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot trajectory as line + points
    if trajectory.size > 0:
        ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color='red', marker='o', label='Trajectory')

    # Plot each corridor as a translucent 3D mesh
    for idx, corridor in enumerate(corridors):
        vertices = hrep_to_vertices(np.array(corridor))
        hull = ConvexHull(vertices)
        print(f"Corridor {idx}: {vertices.shape[0]} vertices, {hull.simplices.shape[0]} hull simplices")
        faces = [vertices[simplex] for simplex in hull.simplices]
        poly3d = Poly3DCollection(faces, alpha=0.1, facecolor='orange', edgecolor='k')
        ax.add_collection3d(poly3d)

    # Plot obstacles as black points
    if obstacles.size > 0:
        ax.scatter(obstacles[:, 0], obstacles[:, 1], obstacles[:, 2], color='black', s=20, label='Obstacles')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Trajectory, Corridor, and Obstacles Visualization')
    ax.legend()

    # Auto-scale axis limits considering all points
    all_points = np.vstack([trajectory] + [hrep_to_vertices(np.array(c)) for c in corridors] + [obstacles]) if corridors else np.vstack([trajectory, obstacles])
    if all_points.size > 0:
        max_range = np.array([all_points[:, 0].max()-all_points[:, 0].min(),
                              all_points[:, 1].max()-all_points[:, 1].min(),
                              all_points[:, 2].max()-all_points[:, 2].min()]).max() / 2.0

        mid_x = (all_points[:, 0].max()+all_points[:, 0].min()) * 0.5
        mid_y = (all_points[:, 1].max()+all_points[:, 1].min()) * 0.5
        mid_z = (all_points[:, 2].max()+all_points[:, 2].min()) * 0.5

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()


def plot_trajectory_and_corridor(trajectory, corridors):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot trajectory as line + points
    if trajectory.size > 0:
        ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color='red', marker='o', label='Trajectory')

    # Plot each corridor as a translucent 3D mesh
    for idx, corridor in enumerate(corridors):
        vertices = hrep_to_vertices(np.array(corridor))
        hull = ConvexHull(vertices)
        
        print(f"Corridor {idx}: {vertices.shape[0]} vertices, {hull.simplices.shape[0]} hull simplices")

        # Extract the triangles from hull.simplices and create faces for Poly3DCollection
        faces = [vertices[simplex] for simplex in hull.simplices]
        
        poly3d = Poly3DCollection(faces, alpha=0.1, facecolor='orange', edgecolor='k')
        ax.add_collection3d(poly3d)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Trajectory and Corridor Visualization')
    ax.legend()

    # Auto-scale to the data limits
    all_points = np.vstack([trajectory] + [hrep_to_vertices(np.array(c)) for c in corridors]) if corridors else trajectory
    if all_points.size > 0:
        max_range = np.array([all_points[:, 0].max()-all_points[:, 0].min(),
                              all_points[:, 1].max()-all_points[:, 1].min(),
                              all_points[:, 2].max()-all_points[:, 2].min()]).max() / 2.0

        mid_x = (all_points[:, 0].max()+all_points[:, 0].min()) * 0.5
        mid_y = (all_points[:, 1].max()+all_points[:, 1].min()) * 0.5
        mid_z = (all_points[:, 2].max()+all_points[:, 2].min()) * 0.5

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()
