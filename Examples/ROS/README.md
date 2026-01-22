# ORB-SLAM3 ROS 2 Package

ROS 2 Jazzy wrapper for ORB-SLAM3 Stereo-Inertial SLAM with RealSense D435i support.

## Prerequisites

### System Dependencies
- Ubuntu 24.04
- ROS 2 Jazzy
- OpenCV 4.x
- Eigen3
- Pangolin
- Boost (system, serialization)
- OpenSSL

### ROS 2 Packages
```bash
sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport \
    ros-jazzy-message-filters ros-jazzy-tf2-ros ros-jazzy-realsense2-camera
```

## Build Instructions

### 1. Build ORB-SLAM3 Core Library First

```bash
cd ~/ORB_SLAM3
chmod +x build.sh
./build.sh
```

This builds:
- `lib/libORB_SLAM3.so`
- `Thirdparty/DBoW2/lib/libDBoW2.so`
- `Thirdparty/g2o/lib/libg2o.so`

### 2. Build ROS 2 Package

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Navigate to ROS workspace
cd ~/ORB_SLAM3/Examples/ROS

# Build with colcon
colcon build --packages-select orb_slam3_ros2

# Source the workspace
source install/setup.bash
```

### 3. Set Environment Variable (Optional)

If ORB-SLAM3 is not installed at `~/ORB_SLAM3`, set the environment variable:

```bash
export ORB_SLAM3_ROOT=/path/to/ORB_SLAM3
```

Add this to your `~/.bashrc` for persistence.

## Usage

### Run with RealSense D435i

```bash
# Terminal 1: Source and launch
source /opt/ros/jazzy/setup.bash
source ~/ORB_SLAM3/Examples/ROS/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ORB_SLAM3/lib:~/ORB_SLAM3/Thirdparty/DBoW2/lib:~/ORB_SLAM3/Thirdparty/g2o/lib

ros2 launch orb_slam3_ros2 stereo_inertial_d435i.launch.py
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `do_rectify` | `false` | Rectify images (false for pre-rectified D435i streams) |
| `do_equalize` | `false` | Apply CLAHE histogram equalization |
| `settings_file` | `RealSense_D435i.yaml` | Path to settings YAML |
| `record_bag` | `false` | Record rosbag of sensor data |
| `bag_path` | `~/rosbags` | Directory for rosbag output |

### Examples

```bash
# With custom settings file
ros2 launch orb_slam3_ros2 stereo_inertial_d435i.launch.py \
    settings_file:=/path/to/custom_settings.yaml

# With rosbag recording
ros2 launch orb_slam3_ros2 stereo_inertial_d435i.launch.py record_bag:=true

# ORB-SLAM3 only (RealSense running separately)
ros2 launch orb_slam3_ros2 orb_slam3_only.launch.py
```

## Troubleshooting

### Library Not Found Error

If you see `libcv_bridge.so: cannot open shared object file`, check if the executable has capabilities set:

```bash
getcap install/orb_slam3_ros2/lib/orb_slam3_ros2/stereo_inertial
```

If it shows `cap_sys_nice=ep`, remove it:

```bash
sudo setcap -r install/orb_slam3_ros2/lib/orb_slam3_ros2/stereo_inertial
```

### IMU Initialization

ORB-SLAM3 requires motion for IMU initialization:
1. Keep the camera stationary for ~2 seconds at startup
2. Then move the camera smoothly (rotation helps)
3. Watch for "IMU initialized" message

## Topics

### Subscribed
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/camera/imu` | `sensor_msgs/Imu` | IMU data |
| `/camera/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | Left IR image |
| `/camera/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | Right IR image |

## Camera Calibration

Create a YAML settings file for your camera. See `Examples/Stereo-Inertial/RealSense_D435i.yaml` as a template.

Get calibration data from RealSense:
```bash
rs-enumerate-devices -c > calibration.txt
```

Key parameters:
- Camera intrinsics (fx, fy, cx, cy)
- Stereo baseline
- IMU-camera extrinsics (T_b_c1)
- IMU noise parameters

## License

GPL-3.0 (same as ORB-SLAM3)
