#!/usr/bin/env python3
"""Analyze gripper STL files to find connection points and geometry."""

import struct
import numpy as np
import os

def read_stl_binary(filename):
    """Read binary STL file and return vertices."""
    vertices = []
    with open(filename, 'rb') as f:
        # Skip header (80 bytes)
        f.read(80)
        # Read number of triangles
        num_triangles = struct.unpack('I', f.read(4))[0]
        
        for _ in range(num_triangles):
            # Skip normal vector (3 floats)
            f.read(12)
            # Read 3 vertices (3 floats each)
            for _ in range(3):
                vertex = struct.unpack('fff', f.read(12))
                vertices.append(vertex)
            # Skip attribute byte count
            f.read(2)
    
    return np.array(vertices)

def analyze_stl(filename):
    """Analyze STL file and print key information."""
    print(f"\n{'='*60}")
    print(f"File: {os.path.basename(filename)}")
    print(f"{'='*60}")
    
    try:
        vertices = read_stl_binary(filename)
        
        # Calculate bounding box
        min_coords = vertices.min(axis=0)
        max_coords = vertices.max(axis=0)
        center = (min_coords + max_coords) / 2
        dimensions = max_coords - min_coords
        
        print(f"Number of vertices: {len(vertices)}")
        print(f"\nBounding Box:")
        print(f"  Min: [{min_coords[0]:.6f}, {min_coords[1]:.6f}, {min_coords[2]:.6f}]")
        print(f"  Max: [{max_coords[0]:.6f}, {max_coords[1]:.6f}, {max_coords[2]:.6f}]")
        print(f"  Center: [{center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f}]")
        print(f"  Dimensions (L x W x H): [{dimensions[0]:.6f}, {dimensions[1]:.6f}, {dimensions[2]:.6f}]")
        
        # Find potential mounting holes (vertices that are very close in groups)
        # This is a simple heuristic - look for circular patterns
        unique_z = np.unique(np.round(vertices[:, 2], 3))
        print(f"\nUnique Z levels (potential mounting planes): {len(unique_z)}")
        
        # Find vertices near specific Z planes that might be mounting holes
        for z in [min_coords[2], max_coords[2], center[2]]:
            nearby = vertices[np.abs(vertices[:, 2] - z) < 0.001]
            if len(nearby) > 0:
                nearby_center = nearby.mean(axis=0)
                print(f"  Vertices near Z={z:.6f}: {len(nearby)}, Center: [{nearby_center[0]:.6f}, {nearby_center[1]:.6f}, {nearby_center[2]:.6f}]")
        
    except Exception as e:
        print(f"Error reading file: {e}")

# Analyze all gripper STL files
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
        analyze_stl(filepath)
    else:
        print(f"\nFile not found: {filepath}")

print(f"\n{'='*60}")
print("Analysis complete!")
print(f"{'='*60}")
