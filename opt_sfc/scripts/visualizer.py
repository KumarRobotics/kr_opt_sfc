import numpy as np
import matplotlib.pyplot as plt
import utils

# Read polygons
polygons = []
current_lines = []
with open("projected_2d.txt") as f:
    for line in f:
        stripped = line.strip()
        if not stripped:
            if current_lines:
                polygons.append(current_lines)
                current_lines = []
        else:
            current_lines.append(tuple(map(float, stripped.split())))
if current_lines:
    polygons.append(current_lines)

# Compute and plot polygons
plt.figure(figsize=(8,8))
for poly in polygons:
    poly = np.array(poly)
    vertices = utils.hrep_to_vertices_2d(poly)
    if vertices is not None:
        plt.fill(vertices[:,0], vertices[:,1], alpha=0.4, edgecolor='black')
plt.axis('equal')
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Polygons from Half-Plane Representation")
plt.grid(True)
plt.show()
