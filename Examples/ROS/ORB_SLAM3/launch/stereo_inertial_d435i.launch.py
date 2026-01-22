"""
ORB-SLAM3 Stereo-Inertial Launch File for RealSense D435i
Launches RealSense camera and ORB-SLAM3 with proper topic remapping
Optionally records a rosbag of camera and IMU data

Environment Variables:
    ORB_SLAM3_ROOT: Path to ORB-SLAM3 root directory (default: ~/ORB_SLAM3)
"""

import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get ORB_SLAM3 root from environment variable or use default
    orb_slam3_dir = os.environ.get('ORB_SLAM3_ROOT', os.path.expanduser('~/cuesinc-AI/ORB_SLAM3'))

    # Paths
    vocabulary_path = os.path.join(orb_slam3_dir, 'Vocabulary', 'ORBvoc.txt')
    default_settings_path = os.path.join(orb_slam3_dir, 'Examples', 'Stereo-Inertial', 'RealSense_D435i.yaml')

    # LD_LIBRARY_PATH for ORB-SLAM3 libraries
    ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
    orb_slam3_libs = ':'.join([
        os.path.join(orb_slam3_dir, 'lib'),
        os.path.join(orb_slam3_dir, 'Thirdparty', 'DBoW2', 'lib'),
        os.path.join(orb_slam3_dir, 'Thirdparty', 'g2o', 'lib'),
    ])
    new_ld_path = f"{ld_library_path}:{orb_slam3_libs}" if ld_library_path else orb_slam3_libs

    # Ensure ROS library paths are included
    ros_lib_path = '/opt/ros/jazzy/lib'
    if ros_lib_path not in new_ld_path:
        new_ld_path = f"{ros_lib_path}:{new_ld_path}"

    # Launch arguments
    do_rectify_arg = DeclareLaunchArgument(
        'do_rectify',
        default_value='false',
        description='Whether to rectify images (use false for D435i with rectified streams)'
    )

    do_equalize_arg = DeclareLaunchArgument(
        'do_equalize',
        default_value='false',
        description='Whether to apply CLAHE histogram equalization'
    )

    settings_file_arg = DeclareLaunchArgument(
        'settings_file',
        default_value=default_settings_path,
        description='Path to ORB-SLAM3 settings YAML file'
    )

    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
        description='Whether to record a rosbag of camera and IMU data'
    )

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=os.path.expanduser('~/rosbags'),
        description='Directory to save rosbag files'
    )

    # Set LD_LIBRARY_PATH
    set_ld_path = SetEnvironmentVariable('LD_LIBRARY_PATH', new_ld_path)

    # RealSense camera node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'enable_color': False,
            'enable_depth': False,
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 2,  # 2 = linear_interpolation
            'infra_width': 848,
            'infra_height': 480,
            'infra_fps': 30,
            'gyro_fps': 200,
            'accel_fps': 63,
            'enable_sync': True,
            'depth_module.emitter_enabled': 0,  # Disable IR projector for stereo SLAM
        }],
        output='screen',
    )

    # ORB-SLAM3 stereo-inertial node
    orb_slam3_node = Node(
        package='orb_slam3_ros2',
        executable='stereo_inertial',
        name='orb_slam3',
        output='screen',
        arguments=[
            vocabulary_path,
            LaunchConfiguration('settings_file'),
            LaunchConfiguration('do_rectify'),
            LaunchConfiguration('do_equalize'),
        ],
        remappings=[
            ('/imu', '/camera/camera/imu'),
            ('/camera/left/image_raw', '/camera/camera/infra1/image_rect_raw'),
            ('/camera/right/image_raw', '/camera/camera/infra2/image_rect_raw'),
        ],
        additional_env={'LD_LIBRARY_PATH': new_ld_path},
    )

    # Delay ORB-SLAM3 start to let camera initialize first
    delayed_orb_slam3 = TimerAction(
        period=3.0,
        actions=[orb_slam3_node],
    )

    # Generate timestamped bag filename
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    # Rosbag recording (records stereo images and IMU)
    rosbag_record = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record_bag')),
        cmd=[
            'ros2', 'bag', 'record',
            '-o', [LaunchConfiguration('bag_path'), f'/slam_bag_{timestamp}'],
            '/camera/camera/infra1/image_rect_raw',
            '/camera/camera/infra2/image_rect_raw',
            '/camera/camera/imu',
            '/camera/camera/infra1/camera_info',
            '/camera/camera/infra2/camera_info',
        ],
        output='screen',
    )

    return LaunchDescription([
        do_rectify_arg,
        do_equalize_arg,
        settings_file_arg,
        record_bag_arg,
        bag_path_arg,
        set_ld_path,
        realsense_node,
        delayed_orb_slam3,
        rosbag_record,
    ])
