#!/usr/bin/env python3
"""Find the two pivot points on the bucket."""

import struct
import numpy as np

def read_stl_binary(filename):
    vertices = []
    with open(filename, 'rb') as f:
        f.read(80)
        num_triangles = struct.unpack('I', f.read(4))[0]
        for _ in range(num_triangles):
            f.read(12)
            for _ in range(3):
                vertex = struct.unpack('fff', f.read(12))
                vertices.append(vertex)
            f.read(2)
    return np.array(vertices)

# Analyze bucket
bucket = read_stl_binary("/home/shreyas/aries/src/aries/meshes/gripper/gripper_bucket.stl")
print("Bucket STL Analysis")
print("="*60)

# Look for the mounting end (where pivots would be)
min_z = bucket[:, 2].min()
print(f"Bottom Z: {min_z:.6f}")

# Get vertices at the bottom (mounting end)
bottom_verts = bucket[np.abs(bucket[:, 2] - min_z) < 0.002]
print(f"Vertices at bottom: {len(bottom_verts)}")

# Find unique X,Y positions (potential pivot holes)
rounded_xy = np.round(bottom_verts[:, :2], 4)
unique_xy, counts = np.unique(rounded_xy, axis=0, return_counts=True)

print("\nPotential pivot points (X, Y positions with multiple vertices):")
for xy, count in zip(unique_xy, counts):
    if count > 5:
        mask = np.all(np.abs(bottom_verts[:, :2] - xy) < 0.001, axis=1)
        verts_here = bottom_verts[mask]
        avg_z = verts_here[:, 2].mean()
        print(f"  Position: [{xy[0]:.6f}, {xy[1]:.6f}, {avg_z:.6f}] - {count} vertices")

# Also check for hole-like features (circles)
print("\nLooking for circular features (holes):")
from collections import defaultdict
# Round more to find centers
very_rounded = np.round(bottom_verts[:, :2], 3)
unique_centers, center_counts = np.unique(very_rounded, axis=0, return_counts=True)

for center, count in zip(unique_centers, center_counts):
    if count > 12:  # Circular holes have many vertices
        mask = np.all(np.abs(bottom_verts[:, :2] - center) < 0.002, axis=1)
        circle_verts = bottom_verts[mask]
        
        # Calculate if it forms a circle
        center_xy = center
        distances = np.sqrt(np.sum((circle_verts[:, :2] - center_xy)**2, axis=1))
        avg_radius = np.mean(distances[distances > 0.0001])
        
        if 0.001 < avg_radius < 0.01:
            avg_z = circle_verts[:, 2].mean()
            print(f"  Hole center: [{center[0]:.6f}, {center[1]:.6f}, {avg_z:.6f}], radius: {avg_radius:.6f}m ({avg_radius*1000:.2f}mm)")
