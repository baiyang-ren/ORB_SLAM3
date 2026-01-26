# Step 2: Dense Depth Maps (SGBM)

Generate dense depth maps from ORB-SLAM3 keyframes using Semi-Global Block Matching.

## Quick Start

```bash
cd /home/baiyang/octa/ORB_SLAM3/SGBM
python3 run_dense_reconstruction.py
```

**Prerequisites:** Run ORB-SLAM3 first (Step 1) to generate keyframe trajectory.

## Configuration

Edit `run_dense_reconstruction.py`:

```python
dataset_name = "dataset_20260125_183432"  # Line 19
dataset_path = f"/home/baiyang/octa/recording_share/{dataset_name}"  # Line 20
```

## What It Does

1. Loads keyframe trajectory from Step 1
2. Finds corresponding stereo image pairs
3. Applies SGBM stereo matching for dense disparity
4. Converts disparity to depth using camera calibration
5. Saves depth maps and metadata for TSDF fusion

## Output

All files saved in dataset directory:

```
/home/baiyang/octa/recording_share/dataset_*/
└── dense_reconstruction_dataset_*/
    ├── depth_maps/depth_*.npy      # Dense depth maps
    ├── *_metadata.json             # Poses + camera info
    ├── *_trajectory.txt            # TUM format trajectory
    └── camera_intrinsics.txt       # Camera calibration
```

## Performance

- **Processing time:** ~6 seconds (159 keyframes)
- **Rate:** ~26 fps
- **Depth accuracy:** 1-2cm at 1-3m distance

## Troubleshooting

**Keyframe trajectory not found:**
```bash
cd /home/baiyang/octa/ORB_SLAM3
python3 run_orb_slam_noIMU.py
```

**cv2.ximgproc error:**
```bash
rm -rf /home/baiyang/octa/ORB_SLAM3/SGBM/__pycache__
```

## Next Step

See [TSDF_GUIDE.md](TSDF_GUIDE.md) for Step 3: Creating the dense 3D mesh.

## SGBM Parameters

- Disparity range: 128 pixels
- Block size: 5×5
- WLS filter: Enabled
- Mode: SGBM_3WAY (highest quality)
