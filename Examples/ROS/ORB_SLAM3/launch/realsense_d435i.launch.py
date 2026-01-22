"""
RealSense D435i Launch File for ORB-SLAM3
Configures stereo infrared + IMU with IR projector disabled
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # RealSense D435i camera node configured for stereo-inertial SLAM
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            # Disable color and depth (not needed for stereo SLAM)
            'enable_color': False,
            'enable_depth': False,
            # Enable stereo infrared cameras
            'enable_infra1': True,
            'enable_infra2': True,
            # Enable IMU
            'enable_gyro': True,
            'enable_accel': True,
            # IMU interpolation method (2 = linear_interpolation)
            'unite_imu_method': 2,
            # Resolution and framerate
            'infra_width': 848,
            'infra_height': 480,
            'infra_fps': 30.0,
            'gyro_fps': 200.0,
            'accel_fps': 200.0,
            # Synchronization
            'enable_sync': True,
            # IMPORTANT: Disable IR projector for stereo SLAM
            # The dot pattern interferes with feature detection
            'depth_module.emitter_enabled': 0,
        }],
        output='screen',
    )

    return LaunchDescription([
        realsense_node,
    ])
