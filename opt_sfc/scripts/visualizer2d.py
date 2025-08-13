import numpy as np
import matplotlib.pyplot as plt
import corridor_utils2d

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


corridor_utils2d.plot_polygons(polygons)