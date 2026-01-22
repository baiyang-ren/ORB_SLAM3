"""
ORB-SLAM3 Stereo-Inertial Launch File (ORB-SLAM3 only, no RealSense)
Assumes RealSense is already running separately

Environment Variables:
    ORB_SLAM3_ROOT: Path to ORB-SLAM3 root directory (default: ~/ORB_SLAM3)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get ORB_SLAM3 root from environment variable or use default
    orb_slam3_dir = os.environ.get('ORB_SLAM3_ROOT', os.path.expanduser('~/ORB_SLAM3'))

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
        description='Whether to rectify images'
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

    # Set LD_LIBRARY_PATH
    set_ld_path = SetEnvironmentVariable('LD_LIBRARY_PATH', new_ld_path)

    # ORB-SLAM3 stereo-inertial node
    orb_slam3_node = Node(
        package='orb_slam3_ros2',
        executable='stereo_inertial',
        name='orb_slam3',
        output='screen',
        emulate_tty=True,
        arguments=[
            vocabulary_path,
            LaunchConfiguration('settings_file'),
            LaunchConfiguration('do_rectify'),
            LaunchConfiguration('do_equalize'),
        ],
        additional_env={'LD_LIBRARY_PATH': new_ld_path},
    )

    return LaunchDescription([
        do_rectify_arg,
        do_equalize_arg,
        settings_file_arg,
        set_ld_path,
        orb_slam3_node,
    ])
