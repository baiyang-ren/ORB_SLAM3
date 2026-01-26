#!/usr/bin/env python3
"""
Simple runner script for TSDF fusion

This script runs TSDF fusion on the dense reconstruction output
to create a dense 3D mesh.
"""

import subprocess
import os
import sys
from pathlib import Path


def run_tsdf_fusion():
    """Run TSDF fusion to create dense 3D mesh"""

    # Configuration
    dataset_name = "dataset_20260125_183432"
    dataset_path = f"/home/baiyang/octa/recording_share/{dataset_name}"

    # Get paths (reconstruction data is in dataset directory)
    script_dir = Path(__file__).parent
    reconstruction_dir = Path(dataset_path) / f"dense_reconstruction_{dataset_name}"
    metadata_file = reconstruction_dir / f"{dataset_name}_metadata.json"

    # Output paths (mesh saved in dataset directory)
    output_dir = Path(dataset_path) / "tsdf_meshes"
    output_dir.mkdir(parents=True, exist_ok=True)

    mesh_output = output_dir / f"{dataset_name}_mesh.ply"

    # Check if metadata exists
    if not metadata_file.exists():
        print(f"✗ Error: Reconstruction metadata not found: {metadata_file}")
        print("\nPlease run the SGBM pipeline first:")
        print("  cd /home/baiyang/octa/ORB_SLAM3/SGBM")
        print("  python3 run_dense_reconstruction.py")
        return

    print("=" * 80)
    print("TSDF Fusion - Dense 3D Mesh Generation")
    print("=" * 80)
    print(f"Input: {metadata_file}")
    print(f"Output: {mesh_output}")
    print("=" * 80)
    print("\nSettings:")
    print("  Voxel size: 10mm (good balance of quality/memory)")
    print("  SDF truncation: 40mm")
    print("  Using all keyframes")
    print()
    print("Expected time: ~30-60 seconds")
    print("=" * 80)
    print()

    # Build command
    tsdf_script = script_dir / "tsdf_fusion.py"
    command = [
        "python3",
        str(tsdf_script),
        "--input", str(metadata_file),
        "--output", str(mesh_output),
        "--voxel-size", "0.01",  # 1cm voxels
        "--sdf-trunc", "0.04",   # 4cm truncation
    ]

    print()
    print("=" * 80)
    print("Starting TSDF fusion...")
    print("=" * 80)
    print()

    try:
        # Run TSDF fusion
        result = subprocess.run(command, check=True)

        print()
        print("=" * 80)
        print("✓ TSDF fusion completed successfully!")
        print("=" * 80)
        print(f"\nMesh saved to: {mesh_output}")
        print()
        print("Visualization commands:")
        print()
        print("  # View with Open3D:")
        print(f"  python3 -c \"import open3d as o3d; mesh = o3d.io.read_triangle_mesh('{mesh_output}'); mesh.compute_vertex_normals(); o3d.visualization.draw_geometries([mesh])\"")
        print()
        print("  # Or use CloudCompare, MeshLab, Blender to view the PLY file")
        print("=" * 80)

    except subprocess.CalledProcessError as e:
        print(f"\n✗ Error running TSDF fusion: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\n✗ Fusion interrupted by user")
        sys.exit(1)


def main():
    """Main entry point"""

    # Check if tsdf_fusion.py exists
    script_dir = Path(__file__).parent
    tsdf_script = script_dir / "tsdf_fusion.py"

    if not tsdf_script.exists():
        print("✗ Error: tsdf_fusion.py not found in SGBM directory")
        print(f"  Expected: {tsdf_script}")
        sys.exit(1)

    run_tsdf_fusion()


if __name__ == "__main__":
    main()
