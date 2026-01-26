# stereo_euroc Executable - Complete Usage Guide

## Command Format

```bash
./Examples/Stereo/stereo_euroc \
    <vocab_path> \
    <settings_path> \
    <dataset_folder> \
    <timestamps_file> \
    [trajectory_name]
```

## Arguments

| Arg | Required | Description |
|-----|----------|-------------|
| `vocab_path` | ✓ | Path to ORB vocabulary (e.g., `Vocabulary/ORBvoc.txt`) |
| `settings_path` | ✓ | Path to camera config YAML (e.g., `Examples/Stereo/RealSense_D435.yaml`) |
| `dataset_folder` | ✓ | Path to dataset root directory |
| `timestamps_file` | ✓ | Path to timestamps file (e.g., `<dataset>/mav0/timestamps.txt`) |
| `trajectory_name` | Optional | Output filename prefix for trajectories |

## How It Works

### 1. Image Loading
The executable expects images in **EuRoC MAV format**:
```
<dataset_folder>/
└── mav0/
    ├── cam0/
    │   └── data/
    │       ├── <timestamp1>.png
    │       ├── <timestamp2>.png
    │       └── ...
    └── cam1/
        └── data/
            ├── <timestamp1>.png
            ├── <timestamp2>.png
            └── ...
```

The code automatically appends `/mav0/cam0/data` and `/mav0/cam1/data` to the dataset_folder.

### 2. Timestamps File
Format: One timestamp per line (in nanoseconds)
```
1769354283901840384
1769354283935340288
1769354283968814848
...
```

The timestamps are divided by 1e9 to convert to seconds.

### 3. Output Files

**Output Location:** Files are saved in the **current working directory** (where you run the command from).

**If `trajectory_name` is provided:**
- `kf_<trajectory_name>.txt` - Keyframe trajectory
- `f_<trajectory_name>.txt` - Frame trajectory

**If `trajectory_name` is NOT provided:**
- `KeyFrameTrajectory.txt` - Keyframe trajectory
- `CameraTrajectory.txt` - Frame trajectory

## Output File Format (EuRoC format)

Both trajectory files use TUM format:
```
timestamp tx ty tz qx qy qz qw
```

Where:
- `timestamp` - Frame timestamp in seconds (or nanoseconds)
- `tx, ty, tz` - Camera position (translation)
- `qx, qy, qz, qw` - Camera orientation (quaternion)

## Examples

### Example 1: Basic Usage (No Output Name)
```bash
cd /home/baiyang/octa/ORB_SLAM3

./Examples/Stereo/stereo_euroc \
    Vocabulary/ORBvoc.txt \
    Examples/Stereo/RealSense_D435.yaml \
    /home/baiyang/octa/recording_share/dataset_20260125_101802 \
    /home/baiyang/octa/recording_share/dataset_20260125_101802/mav0/timestamps.txt
```

**Output files (in current directory):**
- `KeyFrameTrajectory.txt`
- `CameraTrajectory.txt`

### Example 2: With Custom Output Name (Recommended)
```bash
cd /home/baiyang/octa/ORB_SLAM3

./Examples/Stereo/stereo_euroc \
    Vocabulary/ORBvoc.txt \
    Examples/Stereo/RealSense_D435.yaml \
    /home/baiyang/octa/recording_share/dataset_20260125_101802 \
    /home/baiyang/octa/recording_share/dataset_20260125_101802/mav0/timestamps.txt \
    dataset_20260125_101802
```

**Output files (in current directory):**
- `kf_dataset_20260125_101802.txt` ← Keyframes (this is what we need!)
- `f_dataset_20260125_101802.txt` ← All frames

### Example 3: Output to Specific Directory
```bash
cd /home/baiyang/octa/ORB_SLAM3/output  # Change to desired output directory

../Examples/Stereo/stereo_euroc \
    ../Vocabulary/ORBvoc.txt \
    ../Examples/Stereo/RealSense_D435.yaml \
    /home/baiyang/octa/recording_share/dataset_20260125_101802 \
    /home/baiyang/octa/recording_share/dataset_20260125_101802/mav0/timestamps.txt \
    my_reconstruction
```

**Output files (in /home/baiyang/octa/ORB_SLAM3/output/):**
- `kf_my_reconstruction.txt`
- `f_my_reconstruction.txt`

## Multiple Sequences

The executable supports processing multiple sequences in one run:

```bash
./Examples/Stereo/stereo_euroc \
    Vocabulary/ORBvoc.txt \
    Examples/Stereo/RealSense_D435.yaml \
    /path/to/dataset1 /path/to/timestamps1.txt \
    /path/to/dataset2 /path/to/timestamps2.txt \
    /path/to/dataset3 /path/to/timestamps3.txt \
    combined_trajectory
```

## Key Points

### ✓ Output Directory Control
The output files are ALWAYS saved in the **current working directory**.

To control output location:
```bash
# Method 1: cd to output directory
cd /desired/output/dir
/path/to/stereo_euroc ...

# Method 2: Move files after execution
./Examples/Stereo/stereo_euroc ... trajectory_name
mv kf_trajectory_name.txt /desired/output/dir/
mv f_trajectory_name.txt /desired/output/dir/
```

### ✓ Dataset Folder Path
The executable expects:
```
<dataset_folder>/mav0/cam0/data/*.png
<dataset_folder>/mav0/cam1/data/*.png
```

It automatically appends `/mav0/cam0/data` and `/mav0/cam1/data`, so just provide the root dataset folder.

### ✓ Timestamps Format
Each line in timestamps.txt should contain a single timestamp in **nanoseconds**:
```
1769354283901840384
1769354283935340288
```

These are converted to seconds by dividing by 1e9.

### ✓ Trajectory Name Prefix
The last argument (if provided) becomes the **prefix** for output files:
- Input: `my_dataset`
- Output: `kf_my_dataset.txt` and `f_my_dataset.txt`

### ✓ Keyframe vs Frame Trajectory
- **`kf_*.txt`** - Keyframes only (~10% of frames, used for reconstruction)
- **`f_*.txt`** - All frames (full trajectory, used for evaluation)

**For dense reconstruction, we only need the keyframe file (`kf_*.txt`)!**

## Your Current Setup

Based on your usage, you're running:

```bash
cd /home/baiyang/octa/ORB_SLAM3

./Examples/Stereo/stereo_euroc \
    Vocabulary/ORBvoc.txt \
    Examples/Stereo/RealSense_D435.yaml \
    /home/baiyang/octa/recording_share/dataset_20260125_101802 \
    /home/baiyang/octa/recording_share/dataset_20260125_101802/mav0/timestamps.txt \
    dataset_20260125_101802
```

**Output location:** `/home/baiyang/octa/ORB_SLAM3/` (current directory)

**Output files:**
- `/home/baiyang/octa/ORB_SLAM3/kf_dataset_20260125_101802.txt` ✓ (this exists!)
- `/home/baiyang/octa/ORB_SLAM3/f_dataset_20260125_101802.txt` ✓

## Integration with Dense Reconstruction Pipeline

Our SGBM pipeline expects the keyframe file to be in the ORB_SLAM3 root directory:

```
/home/baiyang/octa/ORB_SLAM3/
├── kf_dataset_20260125_101802.txt  ← ORB-SLAM3 output
│
└── SGBM/
    └── run_dense_reconstruction.py  ← Looks for ../kf_dataset_*.txt
```

This is exactly your current setup! ✓

## Troubleshooting

### "Failed to load image at: ..."
- Check that dataset has `/mav0/cam0/data/` and `/mav0/cam1/data/`
- Verify image files have correct timestamp names
- Ensure images are PNG format

### No output files generated
- Check current working directory (`pwd`)
- Verify ORB-SLAM3 completed successfully (printed trajectory info)
- Look for files starting with `kf_` or `f_`

### Timestamps mismatch
- Ensure timestamps file has one timestamp per line
- Timestamps should match image filenames (without .png extension)
- Format: nanoseconds (e.g., 1769354283901840384)

## Source Code Reference

Output file creation (from [stereo_euroc.cc](../Examples/Stereo/stereo_euroc.cc#L172-L183)):

```cpp
// Save camera trajectory
if (bFileName)
{
    const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
    const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
    SLAM.SaveTrajectoryEuRoC(f_file);
    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
}
else
{
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
}
```

Files are saved in the current working directory with no path prefix!
