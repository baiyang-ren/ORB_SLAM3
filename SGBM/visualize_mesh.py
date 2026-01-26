#!/usr/bin/env python3
"""
Simple mesh visualization script using Open3D

This script provides an easy way to visualize the generated 3D mesh.
"""

import sys
from pathlib import Path


def visualize_mesh(mesh_path: str, show_wireframe: bool = False,
                  show_normals: bool = False):
    """
    Visualize mesh using Open3D

    Args:
        mesh_path: Path to mesh file (.ply, .obj, etc.)
        show_wireframe: Show wireframe overlay
        show_normals: Show vertex normals
    """
    try:
        import open3d as o3d
    except ImportError:
        print("✗ Open3D not found!")
        print("\nPlease install Open3D:")
        print("  pip install open3d")
        sys.exit(1)

    mesh_path = Path(mesh_path)

    if not mesh_path.exists():
        print(f"✗ Error: Mesh file not found: {mesh_path}")
        sys.exit(1)

    print(f"Loading mesh from {mesh_path}...")

    # Load mesh
    mesh = o3d.io.read_triangle_mesh(str(mesh_path))

    if not mesh.has_vertices():
        print("✗ Error: Failed to load mesh or mesh is empty")
        sys.exit(1)

    # Compute normals if not present
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()

    print(f"✓ Mesh loaded")
    print(f"  Vertices: {len(mesh.vertices):,}")
    print(f"  Triangles: {len(mesh.triangles):,}")
    print(f"  Has normals: {mesh.has_vertex_normals()}")
    print(f"  Has colors: {mesh.has_vertex_colors()}")

    # Create visualization
    geometries = [mesh]

    # Add coordinate frame for reference
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.5, origin=[0, 0, 0]
    )
    geometries.append(coord_frame)

    print("\nControls:")
    print("  - Mouse: Rotate")
    print("  - Ctrl+Mouse: Pan")
    print("  - Scroll: Zoom")
    print("  - H: Toggle help menu")
    print("  - N: Toggle normals visualization")
    print("  - W: Toggle wireframe")
    print("  - R: Reset view")
    print()

    # Visualize
    print("Opening viewer...")
    o3d.visualization.draw_geometries(
        geometries,
        window_name=f"3D Mesh: {mesh_path.name}",
        width=1280,
        height=720,
        mesh_show_wireframe=show_wireframe,
        mesh_show_back_face=True
    )


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Visualize 3D mesh using Open3D",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('mesh_path', type=str,
                       help='Path to mesh file (.ply, .obj, .stl, etc.)')
    parser.add_argument('--wireframe', action='store_true',
                       help='Show wireframe overlay')
    parser.add_argument('--normals', action='store_true',
                       help='Show vertex normals')

    # If no arguments provided, try to find the latest mesh
    if len(sys.argv) == 1:
        script_dir = Path(__file__).parent
        mesh_dir = script_dir / "output" / "tsdf_meshes"

        if mesh_dir.exists():
            meshes = list(mesh_dir.glob("*.ply"))
            if meshes:
                latest_mesh = max(meshes, key=lambda p: p.stat().st_mtime)
                print(f"No mesh specified, using latest: {latest_mesh.name}")
                visualize_mesh(str(latest_mesh))
                return

        print("Usage: python3 visualize_mesh.py <mesh_path>")
        print("\nExample:")
        print("  python3 visualize_mesh.py output/tsdf_meshes/dataset_20260125_101802_mesh.ply")
        sys.exit(1)

    args = parser.parse_args()

    visualize_mesh(
        args.mesh_path,
        show_wireframe=args.wireframe,
        show_normals=args.normals
    )


if __name__ == "__main__":
    main()
