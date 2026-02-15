# Godot Convex Hull Generator

This is a Godot editor tool that attempts to generate a convex hull mesh from a set of 3D points. The tool creates a mesh instance that visualizes the convex hull of points placed as children of a "Points" node.

**Note: This implementation is currently not working correctly.** It does not produce the expected convex hull output and is shared as a learning reference or starting point for potential convex hull algorithms in Godot.

## Current State

- Attempts to implement a quickhull algorithm for 3D convex hull generation
- Aims to generate quad-based faces rather than triangles
- Includes face orientation correction and duplicate face filtering
- Provides visualization in the Godot editor with semi-transparent material

## Known Issues

- The convex hull generation is not accurate at all and is horribly failing to achieve the required result
- Algorithm implementation likely has logical errors since I had never done anything like that before

## Usage

1. Create a new scene with a Spatial node as root
2. Attach this script to the Spatial node
3. Add a child node called "Points" to hold your points
4. Add child nodes (any spatial node type) to the "Points" node at desired hull vertices
5. Toggle the "update" property in the editor to attempt hull generation

## Code Overview

The script includes functions for:
- Point cloud to convex hull conversion via quickhull algorithm
- Plane distance calculations
- Coplanarity checks
- Face orientation fixing
- Duplicate and degenerate face filtering
- Ray-based interior face detection
- Mesh creation with quad-based faces

## Requirements

- Godot 3.x
- Points must be placed as children of a node named "Points"
