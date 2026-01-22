# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ORB-SLAM3 is a real-time SLAM library for Visual, Visual-Inertial, and Multi-Map SLAM with monocular, stereo, and RGB-D cameras. This repository includes a custom ROS 2 Jazzy port in `Examples/ROS/ORB_SLAM3/`.

## Build Commands

### Core Library Build
```bash
cd /home/cv/cuesinc-AI/ORB_SLAM3
chmod +x build.sh
./build.sh
```
This builds thirdparty libraries (DBoW2, g2o, Sophus), extracts the vocabulary, and compiles `lib/libORB_SLAM3.so`.

### ROS 2 Package Build
```bash
source /opt/ros/jazzy/setup.bash
source /home/cv/cuesinc-AI/.venv_cv/bin/activate
cd /home/cv/cuesinc-AI/ORB_SLAM3/Examples/ROS
colcon build --packages-select orb_slam3_ros2
```

### Running ORB-SLAM3

**Standalone (no ROS):**
```bash
./Examples/Stereo-Inertial/stereo_inertial_euroc \
    Vocabulary/ORBvoc.txt \
    Examples/Stereo-Inertial/RealSense_D435i_euroc.yaml \
    /path/to/dataset
```

**ROS 2 with RealSense D435i:**
```bash
source /opt/ros/jazzy/setup.bash
source /home/cv/cuesinc-AI/.venv_cv/bin/activate
source /home/cv/cuesinc-AI/ORB_SLAM3/Examples/ROS/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/cv/cuesinc-AI/ORB_SLAM3/lib:/home/cv/cuesinc-AI/ORB_SLAM3/Thirdparty/DBoW2/lib:/home/cv/cuesinc-AI/ORB_SLAM3/Thirdparty/g2o/lib

# Full launch (RealSense + ORB-SLAM3)
ros2 launch orb_slam3_ros2 stereo_inertial_d435i.launch.py

# ORB-SLAM3 only (when RealSense runs separately)
ros2 launch orb_slam3_ros2 orb_slam3_only.launch.py
```

## Architecture

### Core Components (src/)
- **System.cc** - Main entry point, coordinates all threads
- **Tracking.cc** - Visual tracking, feature matching, camera pose estimation
- **LocalMapping.cc** - Local bundle adjustment, keyframe/map point management
- **LoopClosing.cc** - Loop detection via DBoW2, pose graph optimization, map merging
- **Optimizer.cc** - g2o-based optimization (bundle adjustment, pose graph, IMU)
- **Atlas.cc** - Multi-map management for robust SLAM

### Data Structures
- **Frame/KeyFrame** - Image frame representation with ORB features
- **MapPoint** - 3D map points with descriptors and observations
- **ImuTypes.h** - IMU preintegration and bias handling

### Thirdparty Libraries
- **DBoW2** - Bag-of-words place recognition (modified)
- **g2o** - Graph optimization framework (modified)
- **Sophus** - Lie group operations for SE3/SO3

### ROS 2 Package (Examples/ROS/ORB_SLAM3/)
- **ros2_stereo_inertial.cc** - Stereo-inertial ROS 2 node
- Topics subscribe to `/camera/camera/imu`, `/camera/camera/infra1/image_rect_raw`, `/camera/camera/infra2/image_rect_raw` (configurable via parameters)
- Uses BEST_EFFORT QoS to match RealSense sensor QoS

## Configuration Files

Camera/IMU calibration YAMLs are in `Examples/Stereo-Inertial/` and `Examples/Stereo/`:
- `RealSense_D435i_euroc.yaml` - D435i stereo-inertial config
- `EuRoC.yaml`, `TUM-VI.yaml`, `KITTI*.yaml` - Dataset configs

Key YAML parameters:
- Camera intrinsics (fx, fy, cx, cy), stereo baseline
- IMU noise parameters (NoiseGyro, NoiseAcc, GyroWalk, AccWalk)
- IMU-camera extrinsic transformation (IMU.T_b_c1)
- ORB feature extraction settings

## Dependencies

**Required:** OpenCV 4.4+, Eigen3 3.1.0+, Pangolin, Boost, OpenSSL
**ROS 2:** rclcpp, sensor_msgs, cv_bridge, image_transport, message_filters, tf2_ros

## Key Notes

- RealSense D435i requires disabling IR projector for stereo SLAM: `depth_module.emitter_enabled:=0`
- IMU initialization requires camera motion (rotational excitation) - keep stationary initially, then move smoothly
- The ROS 2 port uses parameterized topic names defaulting to RealSense D435i topics
- OpenCV version mismatch warnings between cv_bridge (4.6) and system OpenCV (4.12) are expected but functional
