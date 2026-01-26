#!/usr/bin/env python3
"""
Verify that all components are ready for dense reconstruction
"""

import sys
import os
from pathlib import Path

def check_component(name, path, is_file=True):
    """Check if a component exists"""
    p = Path(path)
    exists = p.is_file() if is_file else p.is_dir()

    status = "✓" if exists else "✗"
    print(f"{status} {name}")
    if not exists:
        print(f"  Missing: {path}")
    return exists

def main():
    print("=" * 80)
    print("Dense Reconstruction Setup Verification")
    print("=" * 80)
    print()

    all_ok = True

    # Get paths
    script_dir = Path(__file__).parent
    orb_slam3_root = script_dir.parent

    print("1. Python Scripts:")
    all_ok &= check_component("   dense_reconstruction.py", script_dir / "dense_reconstruction.py")
    all_ok &= check_component("   run_dense_reconstruction.py", script_dir / "run_dense_reconstruction.py")
    print()

    print("2. Python Dependencies:")
    try:
        import cv2
        print(f"   ✓ OpenCV {cv2.__version__}")
        # Check for ximgproc
        try:
            cv2.ximgproc.createDisparityWLSFilter
            print(f"   ✓ OpenCV ximgproc (WLS filter)")
        except:
            print(f"   ✗ OpenCV ximgproc missing (install opencv-contrib-python)")
            all_ok = False
    except ImportError:
        print("   ✗ OpenCV not found")
        all_ok = False

    try:
        import numpy
        print(f"   ✓ NumPy {numpy.__version__}")
    except ImportError:
        print("   ✗ NumPy not found")
        all_ok = False

    try:
        import yaml
        print(f"   ✓ PyYAML")
    except ImportError:
        print("   ✗ PyYAML not found")
        all_ok = False
    print()

    print("3. ORB-SLAM3 Components:")
    all_ok &= check_component("   Vocabulary", orb_slam3_root / "Vocabulary/ORBvoc.txt")
    all_ok &= check_component("   Config file", orb_slam3_root / "Examples/Stereo/RealSense_D435.yaml")
    all_ok &= check_component("   Stereo executable", orb_slam3_root / "Examples/Stereo/stereo_euroc")
    print()

    print("4. Dataset:")
    dataset_path = Path("/home/baiyang/octa/recording_share/dataset_20260125_101802")
    all_ok &= check_component("   Dataset directory", dataset_path, is_file=False)
    all_ok &= check_component("   mav0 directory", dataset_path / "mav0", is_file=False)
    all_ok &= check_component("   cam0/data", dataset_path / "mav0/cam0/data", is_file=False)
    all_ok &= check_component("   cam1/data", dataset_path / "mav0/cam1/data", is_file=False)
    all_ok &= check_component("   timestamps.txt", dataset_path / "mav0/timestamps.txt", is_file=True)

    if (dataset_path / "mav0/cam0/data").exists():
        cam0_images = list((dataset_path / "mav0/cam0/data").glob("*.png"))
        cam1_images = list((dataset_path / "mav0/cam1/data").glob("*.png"))
        print(f"      Images: cam0={len(cam0_images)}, cam1={len(cam1_images)}")
    print()

    print("5. Configuration Test:")
    try:
        sys.path.insert(0, str(script_dir))
        from dense_reconstruction import DenseReconstructionPipeline

        pipeline = DenseReconstructionPipeline(
            dataset_path=str(dataset_path),
            config_path=str(orb_slam3_root / "Examples/Stereo/RealSense_D435.yaml"),
            vocab_path=str(orb_slam3_root / "Vocabulary/ORBvoc.txt"),
            output_dir=str(script_dir / "test_output")
        )

        print(f"   ✓ YAML config loaded successfully")
        print(f"      Camera: {pipeline.camera_intrinsics.width}x{pipeline.camera_intrinsics.height}")
        print(f"      Focal length: fx={pipeline.camera_intrinsics.fx:.2f}, fy={pipeline.camera_intrinsics.fy:.2f}")
        print(f"      Baseline: {pipeline.camera_intrinsics.baseline:.4f} m")
        print(f"   ✓ SGBM depth estimator initialized")
    except Exception as e:
        print(f"   ✗ Configuration test failed: {e}")
        import traceback
        traceback.print_exc()
        all_ok = False
    print()

    print("=" * 80)
    if all_ok:
        print("✓ All components verified successfully!")
        print("=" * 80)
        print()
        print("You can now run the dense reconstruction pipeline:")
        print()
        print("  cd /home/baiyang/octa/ORB_SLAM3/SGBM")
        print("  python3 run_dense_reconstruction.py")
        print()
        print("Or use the direct command:")
        print()
        print("  python3 dense_reconstruction.py \\")
        print("      --dataset /home/baiyang/octa/recording_share/dataset_20260125_101802 \\")
        print("      --config ../Examples/Stereo/RealSense_D435.yaml \\")
        print("      --vocab ../Vocabulary/ORBvoc.txt \\")
        print("      --output output/dense_reconstruction")
        print()
        return 0
    else:
        print("✗ Some components are missing or misconfigured")
        print("=" * 80)
        print("\nPlease fix the issues above before running the pipeline.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
