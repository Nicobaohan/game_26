import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    output_directory = LaunchConfiguration('output_directory')
    camera_info_url = LaunchConfiguration('camera_info_url')
    board_cols = LaunchConfiguration('board_cols')
    board_rows = LaunchConfiguration('board_rows')
    square_size_m = LaunchConfiguration('square_size_m')
    max_pose_age_ms = LaunchConfiguration('max_pose_age_ms')
    local_only = LaunchConfiguration('local_only')

    camera_launch = os.path.join(
        get_package_share_directory('hik_camera'), 'launch', 'hik_camera.launch.py')

    capture_node = Node(
        package='auto_aim_ros2',
        executable='extrinsic_capture_node.py',
        name='extrinsic_capture',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'output_directory': output_directory,
            'board_cols': ParameterValue(board_cols, value_type=int),
            'board_rows': ParameterValue(board_rows, value_type=int),
            'square_size_m': ParameterValue(square_size_m, value_type=float),
            'max_pose_age_ms': ParameterValue(max_pose_age_ms, value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('output_directory'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='package://hik_camera/config/camera_info_vehicle.yaml'),
        DeclareLaunchArgument('board_cols', default_value='11'),
        DeclareLaunchArgument('board_rows', default_value='8'),
        DeclareLaunchArgument('square_size_m', default_value='0.020'),
        DeclareLaunchArgument('max_pose_age_ms', default_value='30.0'),
        DeclareLaunchArgument('local_only', default_value='1'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', local_only),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            launch_arguments={
                'use_sensor_data_qos': 'false',
                'camera_info_url': camera_info_url,
            }.items(),
        ),
        Node(
            package='serical_device_ros2',
            executable='vision_pub_node',
            name='vision_pub',
            output='screen',
        ),
        capture_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=capture_node,
                on_exit=[EmitEvent(event=Shutdown(
                    reason='extrinsic capture window closed'))],
            )
        ),
    ])
