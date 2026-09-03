import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    board_size = LaunchConfiguration('board_size')
    square_size = LaunchConfiguration('square_size')
    camera_name = LaunchConfiguration('camera_name')
    local_only = LaunchConfiguration('local_only')

    camera_launch = os.path.join(
        get_package_share_directory('hik_camera'),
        'launch',
        'hik_camera.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('board_size', default_value='11x8'),
        DeclareLaunchArgument('square_size', default_value='0.020'),
        DeclareLaunchArgument('camera_name', default_value='narrow_stereo'),
        DeclareLaunchArgument('local_only', default_value='1'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', local_only),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
        ),
        Node(
            package='camera_calibration',
            executable='cameracalibrator',
            name='camera_calibrator',
            output='screen',
            arguments=[
                '--size', board_size,
                '--square', square_size,
                '--camera_name', camera_name,
                '--no-service-check',
            ],
            remappings=[('image', '/image_raw')],
        ),
    ])
