import numpy as np
from scipy.spatial import HalfspaceIntersection, ConvexHull
from scipy.optimize import linprog

def find_feasible_point_2d(halfspaces, margin=0.01):
    """
    Find a feasible point strictly inside a 2D convex polygon.

    Parameters
    ----------
    halfspaces : (N, 3) array
        Each row is [a, b, c] representing a*x + b*y + c <= 0
    margin : float
        Small value to shrink the polygon to ensure strict feasibility

    Returns
    -------
    point : (2,) ndarray
        A point strictly inside the polygon.
    """
    A = halfspaces[:, :2]
    b = -halfspaces[:, 2] - margin  # shrink to ensure strict feasibility

    n_vars = 2
    c_obj = np.zeros(n_vars)  # arbitrary objective

    res = linprog(c_obj, A_ub=A, b_ub=b, bounds=(None, None))
    if res.success:
        return res.x
    else:
        raise RuntimeError("Failed to find a feasible point inside the polygon.")

def hrep_to_vertices_2d(halfspaces):
    """
    Convert H-representation (halfspaces) of a 2D convex polygon
    to its vertices (V-representation).

    Parameters
    ----------
    halfspaces : (N, 3) array
        Each row is [a, b, c] for a*x + b*y + c <= 0

    Returns
    -------
    vertices : (M, 2) ndarray
        Vertices of the polygon in counter-clockwise order.
    """
    # SciPy's HalfspaceIntersection expects [a, b, c] as a*x + b*y + c <= 0
    feasible_point = find_feasible_point_2d(halfspaces)
    hs = HalfspaceIntersection(halfspaces, feasible_point)

    # Order vertices CCW using ConvexHull
    hull = ConvexHull(hs.intersections)
    return hs.intersections[hull.vertices]

# Example usage
if __name__ == "__main__":
    # Square: x >= 0, x <= 1, y >= 0, y <= 1
    halfspaces = np.array([
        [-1, 0, 0],   # -x <= 0  → x >= 0
        [ 1, 0, -1],  #  x - 1 <= 0 → x <= 1
        [ 0,-1, 0],   # -y <= 0  → y >= 0
        [ 0, 1, -1]   #  y - 1 <= 0 → y <= 1
    ])
    vertices = hrep_to_vertices_2d(halfspaces)
    print(vertices)


    