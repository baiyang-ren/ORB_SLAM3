#!/usr/bin/env python3
"""
Dense 3D Reconstruction Pipeline using ORB-SLAM3 + SGBM Stereo Matching

This script implements dense depth map generation for ORB-SLAM3 keyframes:
1. Runs ORB-SLAM3 to get camera trajectory and keyframe poses
2. Extracts stereo image pairs at keyframe timestamps
3. Applies SGBM (Semi-Global Block Matching) for dense stereo matching
4. Generates and saves depth maps with associated camera poses

Author: Claude Code
Date: 2026-01-25
"""

import subprocess
import os
import sys
import numpy as np
import cv2
from pathlib import Path
import argparse
from typing import Dict, List, Tuple, Optional
import json
from dataclasses import dataclass, asdict
import time


@dataclass
class CameraIntrinsics:
    """Camera intrinsic parameters"""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    baseline: float  # Stereo baseline in meters


@dataclass
class KeyFrameData:
    """Keyframe information"""
    timestamp: float
    frame_id: int
    tx: float  # Translation x
    ty: float  # Translation y
    tz: float  # Translation z
    qx: float  # Quaternion x
    qy: float  # Quaternion y
    qz: float  # Quaternion z
    qw: float  # Quaternion w
    left_image_path: str
    right_image_path: str
    depth_map_path: Optional[str] = None


class SGBMDepthEstimator:
    """Dense depth estimation using Semi-Global Block Matching"""

    def __init__(self, camera_intrinsics: CameraIntrinsics,
                 min_disparity: int = 0,
                 num_disparities: int = 128,
                 block_size: int = 5,
                 use_wls_filter: bool = True):
        """
        Initialize SGBM depth estimator

        Args:
            camera_intrinsics: Camera calibration parameters
            min_disparity: Minimum possible disparity value
            num_disparities: Maximum disparity minus minimum disparity (must be divisible by 16)
            block_size: Matched block size (odd number, typically 3-11)
            use_wls_filter: Whether to use WLS filter for depth refinement
        """
        self.camera = camera_intrinsics
        self.use_wls_filter = use_wls_filter

        # Create left matcher (SGBM)
        self.left_matcher = cv2.StereoSGBM_create(
            minDisparity=min_disparity,
            numDisparities=num_disparities,
            blockSize=block_size,
            P1=8 * 3 * block_size ** 2,  # Penalty for disparity change by ±1
            P2=32 * 3 * block_size ** 2,  # Penalty for disparity change by >1
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

        # Create right matcher for WLS filtering
        if use_wls_filter:
            self.right_matcher = cv2.ximgproc.createRightMatcher(self.left_matcher)
            self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(self.left_matcher)
            self.wls_filter.setLambda(8000)
            self.wls_filter.setSigmaColor(1.5)

    def compute_disparity(self, left_img: np.ndarray, right_img: np.ndarray) -> np.ndarray:
        """
        Compute disparity map from stereo pair

        Args:
            left_img: Left rectified image
            right_img: Right rectified image

        Returns:
            Filtered disparity map (float32)
        """
        # Convert to grayscale if needed
        if len(left_img.shape) == 3:
            left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
        else:
            left_gray = left_img
            right_gray = right_img

        # Compute disparity
        if self.use_wls_filter:
            # Compute both left and right disparities
            disp_left = self.left_matcher.compute(left_gray, right_gray)
            disp_right = self.right_matcher.compute(right_gray, left_gray)

            # Apply WLS filter
            disp_filtered = self.wls_filter.filter(disp_left, left_gray, None, disp_right)
        else:
            disp_left = self.left_matcher.compute(left_gray, right_gray)
            disp_filtered = disp_left

        # Convert to float and scale
        disparity = disp_filtered.astype(np.float32) / 16.0

        return disparity

    def disparity_to_depth(self, disparity: np.ndarray) -> np.ndarray:
        """
        Convert disparity map to depth map

        Args:
            disparity: Disparity map in pixels

        Returns:
            Depth map in meters
        """
        # Avoid division by zero
        disparity[disparity <= 0] = 0.1

        # Depth = (focal_length * baseline) / disparity
        depth = (self.camera.fx * self.camera.baseline) / disparity

        # Set invalid depths to zero
        depth[disparity <= 0.1] = 0

        return depth

    def compute_depth(self, left_img: np.ndarray, right_img: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute depth map from stereo pair

        Args:
            left_img: Left rectified image
            right_img: Right rectified image

        Returns:
            Tuple of (disparity_map, depth_map)
        """
        disparity = self.compute_disparity(left_img, right_img)
        depth = self.disparity_to_depth(disparity)

        return disparity, depth


class DenseReconstructionPipeline:
    """Complete pipeline for dense 3D reconstruction"""

    def __init__(self, dataset_path: str, config_path: str, vocab_path: str,
                 output_dir: str, executable_path: str = "./Examples/Stereo/stereo_euroc"):
        """
        Initialize reconstruction pipeline

        Args:
            dataset_path: Path to stereo dataset (EuRoC format)
            config_path: Path to ORB-SLAM3 configuration YAML
            vocab_path: Path to ORB vocabulary
            output_dir: Directory to save output files
            executable_path: Path to ORB-SLAM3 stereo executable
        """
        self.dataset_path = Path(dataset_path)
        self.config_path = config_path
        self.vocab_path = vocab_path
        self.output_dir = Path(output_dir)
        self.executable_path = executable_path

        # Create output directories
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.depth_dir = self.output_dir / "depth_maps"
        self.depth_dir.mkdir(exist_ok=True)
        self.keyframes_dir = self.output_dir / "keyframes"
        self.keyframes_dir.mkdir(exist_ok=True)

        # Load camera intrinsics from config
        self.camera_intrinsics = self.load_camera_intrinsics(config_path)

        # Initialize SGBM depth estimator
        self.depth_estimator = SGBMDepthEstimator(
            self.camera_intrinsics,
            num_disparities=128,
            block_size=5,
            use_wls_filter=True
        )

        # Storage for keyframe data
        self.keyframes: List[KeyFrameData] = []

    def load_camera_intrinsics(self, config_path: str) -> CameraIntrinsics:
        """Load camera intrinsics from ORB-SLAM3 YAML config"""
        import yaml

        # Read file and handle OpenCV YAML format (%YAML:1.0)
        with open(config_path, 'r') as f:
            lines = f.readlines()

        # Remove OpenCV YAML directive (not compatible with PyYAML)
        if lines and lines[0].strip().startswith('%YAML:'):
            lines = lines[1:]

        content = ''.join(lines)
        config = yaml.safe_load(content)

        return CameraIntrinsics(
            fx=config['Camera1.fx'],
            fy=config['Camera1.fy'],
            cx=config['Camera1.cx'],
            cy=config['Camera1.cy'],
            width=config['Camera.width'],
            height=config['Camera.height'],
            baseline=config['Stereo.b']
        )

    def run_orb_slam3(self, trajectory_name: str = "trajectory") -> Tuple[str, str]:
        """
        Run ORB-SLAM3 to generate camera trajectory and keyframe poses

        Args:
            trajectory_name: Name for output trajectory files

        Returns:
            Tuple of (frame_trajectory_path, keyframe_trajectory_path)
        """
        print("=" * 80)
        print("STEP 1: Running ORB-SLAM3 for camera tracking")
        print("=" * 80)

        # Construct timestamps file path
        timestamps_file = self.dataset_path / "mav0" / "timestamps.txt"

        # Validate timestamps file exists
        if not timestamps_file.exists():
            print(f"✗ Error: Timestamps file not found: {timestamps_file}")
            print("\nThe dataset must have a timestamps.txt file at:")
            print(f"  {timestamps_file}")
            sys.exit(1)

        # stereo_euroc expects: vocab settings dataset_folder timestamps_file [trajectory_name]
        command = [
            self.executable_path,
            self.vocab_path,
            self.config_path,
            str(self.dataset_path),
            str(timestamps_file),
            trajectory_name
        ]

        print(f"Command: {' '.join(command)}")
        print(f"Dataset: {self.dataset_path}")
        print(f"Timestamps: {timestamps_file}")
        print()

        try:
            result = subprocess.run(command, check=True, cwd=os.getcwd())

            frame_traj = f"f_{trajectory_name}.txt"
            keyframe_traj = f"kf_{trajectory_name}.txt"

            print(f"\n✓ ORB-SLAM3 completed successfully!")
            print(f"  - Frame trajectory: {frame_traj}")
            print(f"  - Keyframe trajectory: {keyframe_traj}")

            return frame_traj, keyframe_traj

        except subprocess.CalledProcessError as e:
            print(f"✗ Error running ORB-SLAM3: {e}")
            sys.exit(1)
        except FileNotFoundError:
            print(f"✗ Executable not found: {self.executable_path}")
            print("Please build ORB-SLAM3 first using ./build.sh")
            sys.exit(1)

    def parse_trajectory_tum(self, trajectory_path: str) -> List[KeyFrameData]:
        """
        Parse TUM format trajectory file

        TUM format: timestamp tx ty tz qx qy qz qw

        Args:
            trajectory_path: Path to trajectory file

        Returns:
            List of keyframe data (without images yet)
        """
        print(f"\nParsing trajectory file: {trajectory_path}")

        keyframes = []
        with open(trajectory_path, 'r') as f:
            for idx, line in enumerate(f):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue

                parts = line.split()
                if len(parts) >= 8:
                    # Convert timestamp from nanoseconds to seconds if needed
                    timestamp = float(parts[0])
                    # If timestamp > 1e12, it's likely in nanoseconds, convert to seconds
                    if timestamp > 1e12:
                        timestamp = timestamp / 1e9

                    keyframes.append(KeyFrameData(
                        timestamp=timestamp,
                        frame_id=idx,
                        tx=float(parts[1]),
                        ty=float(parts[2]),
                        tz=float(parts[3]),
                        qx=float(parts[4]),
                        qy=float(parts[5]),
                        qz=float(parts[6]),
                        qw=float(parts[7]),
                        left_image_path="",
                        right_image_path=""
                    ))

        print(f"✓ Loaded {len(keyframes)} keyframes")
        return keyframes

    def find_closest_stereo_pair(self, timestamp: float, cam0_dir: Path, cam1_dir: Path) -> Tuple[str, str]:
        """
        Find the closest stereo image pair for a given timestamp

        Args:
            timestamp: Target timestamp in seconds
            cam0_dir: Directory containing left camera images
            cam1_dir: Directory containing right camera images

        Returns:
            Tuple of (left_image_path, right_image_path)
        """
        # Read timestamps
        timestamps_file = self.dataset_path / "mav0" / "timestamps.txt"
        if not timestamps_file.exists():
            # Try loading from data.csv in cam0
            timestamps_file = cam0_dir / "data.csv"

        # Load timestamps and find closest
        timestamps = []
        image_names = []

        if timestamps_file.exists() and timestamps_file.name == "data.csv":
            # EuRoC format: timestamp,filename
            with open(timestamps_file, 'r') as f:
                lines = f.readlines()[1:]  # Skip header
                for line in lines:
                    parts = line.strip().split(',')
                    if len(parts) >= 2:
                        ts_ns = int(parts[0])
                        timestamps.append(ts_ns / 1e9)  # Convert to seconds
                        image_names.append(parts[1])
        else:
            # Simple timestamp file
            for img_file in sorted(cam0_dir.glob("*.png")):
                # Filename is timestamp in nanoseconds
                ts_ns = int(img_file.stem)
                timestamps.append(ts_ns / 1e9)
                image_names.append(img_file.name)

        # Find closest timestamp
        timestamps = np.array(timestamps)
        idx = np.argmin(np.abs(timestamps - timestamp))

        left_path = str(cam0_dir / image_names[idx])
        right_path = str(cam1_dir / image_names[idx])

        return left_path, right_path

    def associate_keyframes_with_images(self, keyframes: List[KeyFrameData]) -> List[KeyFrameData]:
        """
        Associate keyframe poses with stereo image pairs

        Args:
            keyframes: List of keyframes with poses but no images

        Returns:
            List of keyframes with associated image paths
        """
        print("\n" + "=" * 80)
        print("STEP 2: Associating keyframes with stereo image pairs")
        print("=" * 80)

        cam0_dir = self.dataset_path / "mav0" / "cam0" / "data"
        cam1_dir = self.dataset_path / "mav0" / "cam1" / "data"

        if not cam0_dir.exists():
            print(f"✗ Error: Left camera directory not found: {cam0_dir}")
            sys.exit(1)

        if not cam1_dir.exists():
            print(f"✗ Error: Right camera directory not found: {cam1_dir}")
            sys.exit(1)

        for kf in keyframes:
            left_path, right_path = self.find_closest_stereo_pair(
                kf.timestamp, cam0_dir, cam1_dir
            )
            kf.left_image_path = left_path
            kf.right_image_path = right_path

        print(f"✓ Associated {len(keyframes)} keyframes with stereo pairs")
        return keyframes

    def generate_depth_maps(self, keyframes: List[KeyFrameData],
                           visualize: bool = False) -> List[KeyFrameData]:
        """
        Generate dense depth maps for all keyframes using SGBM

        Args:
            keyframes: List of keyframes with associated images
            visualize: Whether to save visualization images

        Returns:
            List of keyframes with depth map paths
        """
        print("\n" + "=" * 80)
        print("STEP 3: Generating dense depth maps using SGBM")
        print("=" * 80)

        for idx, kf in enumerate(keyframes):
            print(f"\rProcessing keyframe {idx+1}/{len(keyframes)}", end='', flush=True)

            # Load stereo pair
            left_img = cv2.imread(kf.left_image_path, cv2.IMREAD_GRAYSCALE)
            right_img = cv2.imread(kf.right_image_path, cv2.IMREAD_GRAYSCALE)

            if left_img is None or right_img is None:
                print(f"\n✗ Warning: Could not load images for keyframe {idx}")
                continue

            # Compute depth
            disparity, depth = self.depth_estimator.compute_depth(left_img, right_img)

            # Save depth map
            depth_filename = f"depth_{kf.frame_id:06d}.npy"
            depth_path = self.depth_dir / depth_filename
            np.save(depth_path, depth)
            kf.depth_map_path = str(depth_path)

            # Save visualization if requested
            if visualize and idx % 10 == 0:  # Save every 10th frame
                self.save_depth_visualization(
                    left_img, depth, disparity,
                    self.depth_dir / f"viz_{kf.frame_id:06d}.png"
                )

        print(f"\n✓ Generated {len(keyframes)} depth maps")
        return keyframes

    def save_depth_visualization(self, left_img: np.ndarray, depth: np.ndarray,
                                disparity: np.ndarray, output_path: Path):
        """Save visualization of depth map"""
        # Normalize depth for visualization
        depth_viz = depth.copy()
        depth_viz[depth_viz == 0] = np.nan
        depth_viz = np.clip(depth_viz, 0, 10)  # Clip to 10 meters
        depth_viz = (depth_viz / 10.0 * 255).astype(np.uint8)
        depth_viz = cv2.applyColorMap(depth_viz, cv2.COLORMAP_JET)

        # Create combined visualization
        h, w = left_img.shape
        viz = np.zeros((h, w * 2, 3), dtype=np.uint8)
        viz[:, :w] = cv2.cvtColor(left_img, cv2.COLOR_GRAY2BGR)
        viz[:, w:] = depth_viz

        cv2.imwrite(str(output_path), viz)

    def save_reconstruction_data(self, keyframes: List[KeyFrameData], output_name: str = "reconstruction"):
        """
        Save all reconstruction data for TSDF fusion

        Args:
            keyframes: List of keyframes with depth maps
            output_name: Base name for output files
        """
        print("\n" + "=" * 80)
        print("STEP 4: Saving reconstruction data")
        print("=" * 80)

        # Save keyframe metadata as JSON
        metadata_path = self.output_dir / f"{output_name}_metadata.json"
        metadata = {
            'camera_intrinsics': asdict(self.camera_intrinsics),
            'num_keyframes': len(keyframes),
            'keyframes': [asdict(kf) for kf in keyframes]
        }

        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)

        print(f"✓ Saved metadata: {metadata_path}")

        # Save camera intrinsics separately
        intrinsics_path = self.output_dir / "camera_intrinsics.txt"
        with open(intrinsics_path, 'w') as f:
            f.write(f"# Camera Intrinsics\n")
            f.write(f"fx: {self.camera_intrinsics.fx}\n")
            f.write(f"fy: {self.camera_intrinsics.fy}\n")
            f.write(f"cx: {self.camera_intrinsics.cx}\n")
            f.write(f"cy: {self.camera_intrinsics.cy}\n")
            f.write(f"width: {self.camera_intrinsics.width}\n")
            f.write(f"height: {self.camera_intrinsics.height}\n")
            f.write(f"baseline: {self.camera_intrinsics.baseline}\n")

        print(f"✓ Saved camera intrinsics: {intrinsics_path}")

        # Save trajectory in TUM format
        traj_path = self.output_dir / f"{output_name}_trajectory.txt"
        with open(traj_path, 'w') as f:
            f.write("# timestamp tx ty tz qx qy qz qw\n")
            for kf in keyframes:
                f.write(f"{kf.timestamp} {kf.tx} {kf.ty} {kf.tz} "
                       f"{kf.qx} {kf.qy} {kf.qz} {kf.qw}\n")

        print(f"✓ Saved trajectory: {traj_path}")

        print(f"\n{'=' * 80}")
        print("Summary:")
        print(f"{'=' * 80}")
        print(f"Total keyframes processed: {len(keyframes)}")
        print(f"Depth maps saved to: {self.depth_dir}")
        print(f"Metadata saved to: {metadata_path}")
        print(f"\nData ready for TSDF fusion!")
        print(f"{'=' * 80}")

    def run_pipeline(self, trajectory_name: str = "dense_recon",
                    run_slam: bool = True,
                    keyframe_traj_path: Optional[str] = None,
                    visualize: bool = True):
        """
        Run the complete dense reconstruction pipeline

        Args:
            trajectory_name: Name for trajectory files
            run_slam: Whether to run ORB-SLAM3 (if False, uses existing trajectory)
            keyframe_traj_path: Path to existing keyframe trajectory (if run_slam=False)
            visualize: Whether to save depth visualizations
        """
        start_time = time.time()

        # Step 1: Run ORB-SLAM3 (or use existing trajectory)
        if run_slam:
            _, keyframe_traj = self.run_orb_slam3(trajectory_name)
        else:
            if keyframe_traj_path is None:
                print("✗ Error: keyframe_traj_path must be provided when run_slam=False")
                sys.exit(1)
            keyframe_traj = keyframe_traj_path

        # Step 2: Parse trajectory
        self.keyframes = self.parse_trajectory_tum(keyframe_traj)

        # Step 3: Associate with images
        self.keyframes = self.associate_keyframes_with_images(self.keyframes)

        # Step 4: Generate depth maps
        self.keyframes = self.generate_depth_maps(self.keyframes, visualize=visualize)

        # Step 5: Save reconstruction data
        self.save_reconstruction_data(self.keyframes, trajectory_name)

        elapsed = time.time() - start_time
        print(f"\n✓ Pipeline completed in {elapsed:.2f} seconds")


def main():
    parser = argparse.ArgumentParser(
        description="Dense 3D Reconstruction using ORB-SLAM3 + SGBM",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run full pipeline (ORB-SLAM3 + SGBM)
  python dense_reconstruction.py \\
      --dataset /path/to/dataset_20260125_101802 \\
      --config Examples/Stereo/EuRoC.yaml \\
      --vocab Vocabulary/ORBvoc.txt \\
      --output output/dense_recon

  # Use existing trajectory
  python dense_reconstruction.py \\
      --dataset /path/to/dataset_20260125_101802 \\
      --config Examples/Stereo/EuRoC.yaml \\
      --vocab Vocabulary/ORBvoc.txt \\
      --output output/dense_recon \\
      --no-slam \\
      --keyframe-traj kf_trajectory.txt
        """
    )

    parser.add_argument('--dataset', type=str, required=True,
                       help='Path to dataset directory (EuRoC format with mav0/cam0, mav0/cam1)')
    parser.add_argument('--config', type=str, required=True,
                       help='Path to ORB-SLAM3 configuration YAML')
    parser.add_argument('--vocab', type=str,
                       help='Path to ORB vocabulary file (required if not using --no-slam)')
    parser.add_argument('--output', type=str, required=True,
                       help='Output directory for depth maps and reconstruction data')
    parser.add_argument('--executable', type=str, default='./Examples/Stereo/stereo_euroc',
                       help='Path to ORB-SLAM3 stereo executable (only needed without --no-slam)')
    parser.add_argument('--trajectory-name', type=str, default='dense_recon',
                       help='Name for trajectory files')
    parser.add_argument('--no-slam', action='store_true',
                       help='Skip ORB-SLAM3 and use existing trajectory')
    parser.add_argument('--keyframe-traj', type=str,
                       help='Path to existing keyframe trajectory (required with --no-slam)')
    parser.add_argument('--no-visualize', action='store_true',
                       help='Disable depth map visualizations')

    args = parser.parse_args()

    # Validate inputs based on mode
    if not os.path.exists(args.dataset):
        print(f"✗ Error: Dataset not found: {args.dataset}")
        sys.exit(1)

    if not os.path.exists(args.config):
        print(f"✗ Error: Config file not found: {args.config}")
        sys.exit(1)

    # Validate SLAM-specific arguments
    if not args.no_slam:
        if not args.vocab:
            print(f"✗ Error: --vocab is required when running ORB-SLAM3")
            print(f"  Use --no-slam to skip ORB-SLAM3 execution")
            sys.exit(1)
        if not os.path.exists(args.vocab):
            print(f"✗ Error: Vocabulary file not found: {args.vocab}")
            sys.exit(1)
    else:
        # Validate --no-slam mode requirements
        if not args.keyframe_traj:
            print(f"✗ Error: --keyframe-traj is required when using --no-slam")
            sys.exit(1)
        if not os.path.exists(args.keyframe_traj):
            print(f"✗ Error: Keyframe trajectory not found: {args.keyframe_traj}")
            sys.exit(1)

    # Create pipeline
    pipeline = DenseReconstructionPipeline(
        dataset_path=args.dataset,
        config_path=args.config,
        vocab_path=args.vocab or "",  # Optional when using --no-slam
        output_dir=args.output,
        executable_path=args.executable
    )

    # Run pipeline
    pipeline.run_pipeline(
        trajectory_name=args.trajectory_name,
        run_slam=not args.no_slam,
        keyframe_traj_path=args.keyframe_traj,
        visualize=not args.no_visualize
    )


if __name__ == "__main__":
    main()
