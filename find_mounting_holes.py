#!/usr/bin/env python3
"""Find mounting holes in gripper STL files."""

import struct
import numpy as np
from collections import defaultdict
import os

def read_stl_binary(filename):
    """Read binary STL file and return vertices."""
    vertices = []
    with open(filename, 'rb') as f:
        f.read(80)
        num_triangles = struct.unpack('I', f.read(4))[0]
        
        for _ in range(num_triangles):
            f.read(12)  # Skip normal
            for _ in range(3):
                vertex = struct.unpack('fff', f.read(12))
                vertices.append(vertex)
            f.read(2)  # Skip attribute
    
    return np.array(vertices)

def find_circular_patterns(vertices, z_level, tolerance=0.001):
    """Find circular patterns that might indicate holes."""
    # Get vertices at this Z level
    mask = np.abs(vertices[:, 2] - z_level) < tolerance
    level_verts = vertices[mask]
    
    if len(level_verts) < 10:
        return []
    
    # Round to find duplicate points (vertices forming circles)
    rounded = np.round(level_verts[:, :2], 4)
    unique, counts = np.unique(rounded, axis=0, return_counts=True)
    
    # Find points that appear multiple times (circular features)
    circles = []
    for point, count in zip(unique, counts):
        if count > 8:  # Likely a circle if many vertices at same XY
            # Find all vertices near this point
            mask = np.all(np.abs(level_verts[:, :2] - point) < 0.001, axis=1)
            circle_verts = level_verts[mask]
            
            # Calculate radius from center
            center_xy = point
            distances = np.sqrt(np.sum((circle_verts[:, :2] - center_xy)**2, axis=1))
            avg_radius = np.mean(distances[distances > 0.0001])
            
            if 0.001 < avg_radius < 0.05:  # Reasonable hole size
                circles.append({
                    'center': [center_xy[0], center_xy[1], z_level],
                    'radius': avg_radius,
                    'count': count
                })
    
    return circles

def analyze_for_holes(filename):
    """Find potential mounting holes in STL."""
    print(f"\n{'='*60}")
    print(f"Analyzing: {os.path.basename(filename)}")
    print(f"{'='*60}")
    
    vertices = read_stl_binary(filename)
    min_z = vertices[:, 2].min()
    max_z = vertices[:, 2].max()
    
    print(f"Z range: {min_z:.6f} to {max_z:.6f}")
    
    # Check bottom, top, and middle planes
    for z_level in [min_z, max_z, (min_z + max_z) / 2]:
        circles = find_circular_patterns(vertices, z_level)
        if circles:
            print(f"\nPotential holes at Z={z_level:.6f}:")
            for i, circle in enumerate(circles, 1):
                c = circle['center']
                print(f"  Hole {i}: Center=[{c[0]:.6f}, {c[1]:.6f}, {c[2]:.6f}], "
                      f"Radius={circle['radius']:.6f}m ({circle['radius']*1000:.2f}mm)")

# Analyze each component
gripper_path = "/home/shreyas/aries/src/aries/meshes/gripper/"
files = [
    "gripper_base.stl",
    "gripper_gear_left.stl",
    "gripper_gear_right.stl",
    "gripper_link_left.stl",
    "gripper_link_right.stl",
    "gripper_bucket.stl"
]

for filename in files:
    filepath = os.path.join(gripper_path, filename)
    if os.path.exists(filepath):
        analyze_for_holes(filepath)

print(f"\n{'='*60}")
print("ASSEMBLY RECOMMENDATIONS:")
print(f"{'='*60}")
print("""
Based on typical gripper designs:

1. Gears mount to base at their rotation axis
2. Links mount to base at one end, connect to gears via linkage
3. Buckets mount to the end of links

The gear and link should work together - when gear rotates, 
the link follows through a mechanical linkage.
""")
