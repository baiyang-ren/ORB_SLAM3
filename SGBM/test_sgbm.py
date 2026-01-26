#!/usr/bin/env python3
"""
Test script for SGBM dense stereo matching

This script tests the SGBM depth estimator on a sample stereo pair
from your dataset to verify everything is working correctly.
"""

import sys
import os
import cv2
import numpy as np
from pathlib import Path


def test_sgbm_on_sample():
    """Test SGBM on a sample stereo pair"""

    print("=" * 80)
    print("Testing SGBM Dense Stereo Matching")
    print("=" * 80)

    # Find sample stereo pair from dataset
    dataset_path = Path("/home/baiyang/octa/recording_share/dataset_20260125_101802/mav0")

    if not dataset_path.exists():
        print(f"✗ Error: Dataset not found: {dataset_path}")
        print("\nThis test requires the dataset_20260125_101802 dataset.")
        return False

    cam0_dir = dataset_path / "cam0" / "data"
    cam1_dir = dataset_path / "cam1" / "data"

    if not cam0_dir.exists() or not cam1_dir.exists():
        print(f"✗ Error: Camera directories not found")
        return False

    # Get first stereo pair
    left_images = sorted(list(cam0_dir.glob("*.png")))
    right_images = sorted(list(cam1_dir.glob("*.png")))

    if len(left_images) == 0 or len(right_images) == 0:
        print(f"✗ Error: No images found in dataset")
        return False

    # Use middle image for testing
    idx = len(left_images) // 2
    left_path = left_images[idx]
    right_path = right_images[idx]

    print(f"\n✓ Found stereo dataset")
    print(f"  Left camera: {len(left_images)} images")
    print(f"  Right camera: {len(right_images)} images")
    print(f"\nTesting with image: {left_path.name}")

    # Load images
    print("\n1. Loading stereo pair...")
    left_img = cv2.imread(str(left_path), cv2.IMREAD_GRAYSCALE)
    right_img = cv2.imread(str(right_path), cv2.IMREAD_GRAYSCALE)

    if left_img is None or right_img is None:
        print(f"✗ Error: Could not load images")
        return False

    print(f"   ✓ Images loaded: {left_img.shape}")

    # Create SGBM matcher
    print("\n2. Creating SGBM matcher...")

    # Parameters for RealSense D435i stereo
    min_disparity = 0
    num_disparities = 128  # Must be divisible by 16
    block_size = 5

    left_matcher = cv2.StereoSGBM_create(
        minDisparity=min_disparity,
        numDisparities=num_disparities,
        blockSize=block_size,
        P1=8 * 3 * block_size ** 2,
        P2=32 * 3 * block_size ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )

    print(f"   ✓ SGBM matcher created")
    print(f"      - Disparity range: {min_disparity} to {num_disparities}")
    print(f"      - Block size: {block_size}")

    # Compute disparity
    print("\n3. Computing disparity map...")
    import time
    start_time = time.time()

    disparity = left_matcher.compute(left_img, right_img)
    disparity_float = disparity.astype(np.float32) / 16.0

    elapsed = time.time() - start_time
    print(f"   ✓ Disparity computed in {elapsed:.3f} seconds")

    # Statistics
    valid_disparity = disparity_float[disparity_float > 0]
    print(f"   ✓ Valid pixels: {len(valid_disparity)} / {disparity_float.size}")
    print(f"      - Min disparity: {valid_disparity.min():.2f} px")
    print(f"      - Max disparity: {valid_disparity.max():.2f} px")
    print(f"      - Mean disparity: {valid_disparity.mean():.2f} px")

    # Convert to depth (using baseline from D435i config)
    print("\n4. Converting to depth map...")
    baseline = 0.0499585  # meters (from RealSense_D435i.yaml)
    fx = 382.613  # focal length (from RealSense_D435i.yaml)

    # Avoid division by zero
    disparity_depth = disparity_float.copy()
    disparity_depth[disparity_depth <= 0] = 0.1

    depth = (fx * baseline) / disparity_depth
    depth[disparity_float <= 0.1] = 0  # Mark invalid depths

    valid_depth = depth[(depth > 0) & (depth < 10)]  # Valid depth 0-10m
    print(f"   ✓ Depth map created")
    print(f"      - Valid depth pixels: {len(valid_depth)}")
    print(f"      - Depth range: {valid_depth.min():.3f}m to {valid_depth.max():.3f}m")
    print(f"      - Mean depth: {valid_depth.mean():.3f}m")

    # Test WLS filter
    print("\n5. Testing WLS filter (optional refinement)...")
    try:
        right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
        wls_filter = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
        wls_filter.setLambda(8000)
        wls_filter.setSigmaColor(1.5)

        disp_right = right_matcher.compute(right_img, left_img)
        disp_filtered = wls_filter.filter(disparity, left_img, None, disp_right)
        disp_filtered = disp_filtered.astype(np.float32) / 16.0

        print(f"   ✓ WLS filter applied successfully")

        valid_filtered = disp_filtered[disp_filtered > 0]
        print(f"      - Filtered valid pixels: {len(valid_filtered)}")

    except Exception as e:
        print(f"   ! WLS filter not available: {e}")
        print("      (This is optional - basic SGBM still works)")

    # Save visualization
    print("\n6. Saving visualization...")
    output_dir = Path("test_output")
    output_dir.mkdir(exist_ok=True)

    # Normalize disparity for visualization
    disp_viz = disparity_float.copy()
    disp_viz = np.clip(disp_viz, 0, num_disparities)
    disp_viz = (disp_viz / num_disparities * 255).astype(np.uint8)
    disp_viz = cv2.applyColorMap(disp_viz, cv2.COLORMAP_JET)

    # Normalize depth for visualization
    depth_viz = depth.copy()
    depth_viz = np.clip(depth_viz, 0, 10)  # Clip to 10 meters
    depth_viz = (depth_viz / 10.0 * 255).astype(np.uint8)
    depth_viz = cv2.applyColorMap(depth_viz, cv2.COLORMAP_JET)

    # Create combined visualization
    h, w = left_img.shape
    viz = np.zeros((h * 2, w * 2, 3), dtype=np.uint8)
    viz[:h, :w] = cv2.cvtColor(left_img, cv2.COLOR_GRAY2BGR)
    viz[:h, w:] = cv2.cvtColor(right_img, cv2.COLOR_GRAY2BGR)
    viz[h:, :w] = disp_viz
    viz[h:, w:] = depth_viz

    # Add labels
    cv2.putText(viz, "Left Image", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(viz, "Right Image", (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(viz, "Disparity Map", (10, h + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(viz, "Depth Map (0-10m)", (w + 10, h + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    output_path = output_dir / "sgbm_test_result.png"
    cv2.imwrite(str(output_path), viz)
    print(f"   ✓ Visualization saved: {output_path}")

    # Save depth map
    depth_path = output_dir / "depth_test.npy"
    np.save(depth_path, depth)
    print(f"   ✓ Depth map saved: {depth_path}")

    print("\n" + "=" * 80)
    print("✓ SGBM Test Completed Successfully!")
    print("=" * 80)
    print("\nResults:")
    print(f"  - Disparity range: {valid_disparity.min():.2f} to {valid_disparity.max():.2f} px")
    print(f"  - Depth range: {valid_depth.min():.3f} to {valid_depth.max():.3f} m")
    print(f"  - Mean depth: {valid_depth.mean():.3f} m")
    print(f"  - Processing time: {elapsed:.3f} s")
    print(f"  - Output: {output_path}")
    print("\nThe SGBM depth estimator is working correctly!")
    print("You can now run the full dense reconstruction pipeline.")
    print("=" * 80)

    return True


def main():
    """Main entry point"""

    # Check dependencies
    print("\nChecking dependencies...")
    try:
        import cv2
        print(f"✓ OpenCV {cv2.__version__}")
    except ImportError:
        print("✗ OpenCV not found. Install with: pip install opencv-python opencv-contrib-python")
        sys.exit(1)

    try:
        import numpy
        print(f"✓ NumPy {numpy.__version__}")
    except ImportError:
        print("✗ NumPy not found. Install with: pip install numpy")
        sys.exit(1)

    print()

    # Run test
    success = test_sgbm_on_sample()

    if success:
        sys.exit(0)
    else:
        print("\n✗ Test failed")
        sys.exit(1)


if __name__ == "__main__":
    main()
