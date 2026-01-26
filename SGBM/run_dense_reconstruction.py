#!/usr/bin/env python3
"""
Example script to run dense reconstruction pipeline on recorded stereo data

This script demonstrates how to use the dense_reconstruction.py pipeline
with your recorded datasets.
"""

import subprocess
import os
import sys
from pathlib import Path


def run_dense_reconstruction():
    """Run dense reconstruction on recorded stereo dataset"""

    # Configuration
    dataset_name = "dataset_20260125_183432"
    dataset_path = f"/home/baiyang/octa/recording_share/{dataset_name}"

    # Get ORB_SLAM3 root directory (parent of SGBM)
    script_dir = Path(__file__).parent
    orb_slam3_root = script_dir.parent

    # Paths to existing ORB-SLAM3 trajectory files (in dataset directory)
    # run_orb_slam_noIMU.py moves the files to the dataset directory
    keyframe_traj_path = os.path.join(dataset_path, f"kf_{dataset_name}.txt")

    # Config file path
    config_path = str(orb_slam3_root / "Examples/Stereo/RealSense_D435.yaml")

    # Output directory (in dataset directory)
    output_dir = os.path.join(dataset_path, f"dense_reconstruction_{dataset_name}")

    # Check if dataset exists
    if not os.path.exists(dataset_path):
        print(f"✗ Error: Dataset not found: {dataset_path}")
        print("\nAvailable datasets:")
        recording_dir = "/home/baiyang/octa/recording_share"
        if os.path.exists(recording_dir):
            for item in os.listdir(recording_dir):
                item_path = os.path.join(recording_dir, item)
                if os.path.isdir(item_path):
                    print(f"  - {item}")
        return

    # Check if stereo data exists
    mav0_path = os.path.join(dataset_path, "mav0")
    if not os.path.exists(mav0_path):
        print(f"✗ Error: This dataset doesn't have stereo data (mav0 directory not found)")
        print(f"  Dataset: {dataset_path}")
        print("\nThis pipeline requires stereo image pairs in EuRoC format.")
        print("Please use a dataset with mav0/cam0 and mav0/cam1 directories.")
        return

    # Check if keyframe trajectory exists
    if not os.path.exists(keyframe_traj_path):
        print(f"✗ Error: Keyframe trajectory not found: {keyframe_traj_path}")
        print("\nPlease run ORB-SLAM3 first to generate the trajectory:")
        print("  cd /home/baiyang/octa/ORB_SLAM3")
        print("  python3 run_orb_slam_noIMU.py")
        print(f"\nExpected output file: kf_{dataset_name}.txt")
        return

    # Build command (skip ORB-SLAM3 execution, use existing trajectory)
    dense_recon_script = str(script_dir / "dense_reconstruction.py")
    command = [
        "python3",
        dense_recon_script,
        "--dataset", dataset_path,
        "--config", config_path,
        "--output", output_dir,
        "--no-slam",  # Skip ORB-SLAM3 execution
        "--keyframe-traj", keyframe_traj_path,
        "--trajectory-name", dataset_name
    ]

    print("=" * 80)
    print("Dense 3D Reconstruction Pipeline (SGBM Depth Map Generation)")
    print("=" * 80)
    print(f"Dataset: {dataset_path}")
    print(f"Keyframe trajectory: {keyframe_traj_path}")
    print(f"Config: {config_path}")
    print(f"Output: {output_dir}")
    print("=" * 80)
    print("\nThis will:")
    print("  1. Load existing keyframe trajectory (from ORB-SLAM3)")
    print("  2. Associate keyframes with stereo image pairs")
    print("  3. Apply SGBM dense stereo matching on keyframes")
    print("  4. Generate dense depth maps for each keyframe")
    print("  5. Save depth maps and metadata for TSDF fusion")
    print()
    print("Note: ORB-SLAM3 execution is SKIPPED (using existing trajectory)")
    print("=" * 80)
    print()
    print("=" * 80)
    print("Starting pipeline...")
    print("=" * 80)
    print()

    try:
        # Run the pipeline
        result = subprocess.run(command, check=True)

        print()
        print("=" * 80)
        print("✓ Dense reconstruction completed successfully!")
        print("=" * 80)
        print(f"\nOutput files saved to: {output_dir}/")
        print("  - depth_maps/        : Dense depth maps (.npy files)")
        print("  - *_metadata.json    : Keyframe data and camera calibration")
        print("  - *_trajectory.txt   : Camera trajectory (TUM format)")
        print("  - camera_intrinsics.txt : Camera calibration parameters")
        print()
        print("Next step: Run TSDF fusion to generate dense 3D mesh")
        print("=" * 80)

    except subprocess.CalledProcessError as e:
        print(f"\n✗ Error running pipeline: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\n✗ Pipeline interrupted by user")
        sys.exit(1)


def main():
    """Main entry point"""

    # Check if dense_reconstruction.py exists in the same directory
    script_dir = Path(__file__).parent
    dense_recon_script = script_dir / "dense_reconstruction.py"

    if not dense_recon_script.exists():
        print("✗ Error: dense_reconstruction.py not found in SGBM directory")
        print(f"  Expected: {dense_recon_script}")
        sys.exit(1)

    run_dense_reconstruction()


if __name__ == "__main__":
    main()
