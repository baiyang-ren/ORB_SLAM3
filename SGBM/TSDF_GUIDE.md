# Step 3: TSDF Fusion - Dense 3D Mesh

Create a dense 3D mesh from depth maps using Truncated Signed Distance Function fusion.

## Quick Start

### 1. Install Open3D

```bash
pip install open3d
```

### 2. Run TSDF Fusion

```bash
cd /home/baiyang/octa/ORB_SLAM3/SGBM
python3 run_tsdf_fusion.py
```

**Prerequisites:** Complete Step 2 (SGBM depth maps) first.

## Configuration

Edit `run_tsdf_fusion.py`:

```python
dataset_name = "dataset_20260125_183432"  # Line 19
dataset_path = f"/home/baiyang/octa/recording_share/{dataset_name}"  # Line 20
```

## What It Does

1. Loads depth maps and camera poses from Step 2
2. Creates a 3D voxel grid (TSDF volume)
3. Integrates all depth maps with multi-view fusion
4. Extracts triangle mesh using Marching Cubes
5. Saves as PLY file

## Output

Mesh saved in dataset directory:

```
/home/baiyang/octa/recording_share/dataset_*/
└── tsdf_meshes/
    └── dataset_*_mesh.ply    # Dense 3D mesh
```

Typical mesh statistics:
- **Vertices:** 500K - 2M points
- **Triangles:** 1M - 4M faces
- **File size:** 20-100 MB

## Performance

- **Processing time:** 30-60 seconds (159 keyframes)
- **Rate:** ~5 fps
- **Memory:** ~1GB RAM

## Parameters

Default settings (can edit `tsdf_fusion.py`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| Voxel size | 10mm | Mesh resolution (smaller = more detail, more memory) |
| SDF truncation | 40mm | Integration distance (larger = smoother) |

### Adjusting Quality

**Higher resolution:**
```python
--voxel-size 0.005  # 5mm voxels (needs ~4GB memory)
```

**Faster/lower resolution:**
```python
--voxel-size 0.02  # 20mm voxels (uses ~256MB memory)
```

## Visualization

### View with Open3D

```bash
python3 visualize_mesh.py /home/baiyang/octa/recording_share/dataset_*/tsdf_meshes/*_mesh.ply
```

### View with External Tools

- **CloudCompare** - Recommended for large meshes
- **MeshLab** - Mesh processing and cleanup
- **Blender** - 3D modeling and rendering

## Troubleshooting

**"No module named 'open3d'":**
```bash
pip install open3d
```

**Out of memory:**
```bash
# Edit tsdf_fusion.py, increase voxel size:
--voxel-size 0.02  # Use larger voxels
```

**Mesh has holes:**
```bash
# Edit tsdf_fusion.py, increase SDF truncation:
--sdf-trunc 0.08  # Larger integration region
```

## What is TSDF?

**TSDF** = Truncated Signed Distance Function

Instead of just combining point clouds, TSDF:
1. Creates a 3D voxel grid
2. For each voxel, computes signed distance to nearest surface
3. Integrates depth from multiple views (reduces noise)
4. Extracts smooth mesh using Marching Cubes

**Benefits:**
- Combines multiple viewpoints seamlessly
- Reduces depth noise through averaging
- Creates watertight, smooth meshes
- Handles occlusions and missing data

## Advanced Options

The `tsdf_fusion.py` script supports additional options:

```bash
# Process only every 5th frame (faster preview)
python3 tsdf_fusion.py --input <metadata.json> --output mesh.ply --skip 5

# Simplify mesh (reduce triangle count)
python3 tsdf_fusion.py --input <metadata.json> --output mesh.ply --simplify --target-triangles 100000

# Extract point cloud instead of mesh
python3 tsdf_fusion.py --input <metadata.json> --output cloud.ply --point-cloud

# Use color from images
python3 tsdf_fusion.py --input <metadata.json> --output mesh.ply --use-color
```

## Complete Pipeline Summary

```
Step 1: ORB-SLAM3 (manual)
   Input: Stereo images
   Output: Keyframe trajectory

Step 2: SGBM (this directory)
   Input: Keyframe trajectory + stereo images
   Output: Dense depth maps
   Time: ~6 seconds

Step 3: TSDF Fusion (this guide)
   Input: Dense depth maps + poses
   Output: Dense 3D mesh
   Time: ~30-60 seconds
```

**Total time:** ~40-70 seconds from depth maps to mesh!

## Output Format

The PLY file contains:
- Vertex positions (x, y, z)
- Vertex normals (nx, ny, nz)
- Triangle faces (vertex indices)
- Optional: Vertex colors (r, g, b) if `--use-color` used

Compatible with most 3D software and game engines.
