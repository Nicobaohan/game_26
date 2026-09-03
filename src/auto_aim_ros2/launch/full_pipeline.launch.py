import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_root = LaunchConfiguration('project_root')
    config_path = LaunchConfiguration('config_path')
    camera_info_url = LaunchConfiguration('camera_info_url')
    enable_control_output = LaunchConfiguration('enable_control_output')
    enable_fire = LaunchConfiguration('enable_fire')
    fire_hold_s = LaunchConfiguration('fire_hold_s')
    fire_interval_s = LaunchConfiguration('fire_interval_s')
    fire_shot_period_s = LaunchConfiguration('fire_shot_period_s')
    fire_burst_count = LaunchConfiguration('fire_burst_count')
    fire_yaw_tolerance_deg = LaunchConfiguration('fire_yaw_tolerance_deg')
    fire_pitch_tolerance_deg = LaunchConfiguration('fire_pitch_tolerance_deg')
    fire_max_yaw_velocity_deg_s = LaunchConfiguration(
        'fire_max_yaw_velocity_deg_s')
    fire_max_pitch_velocity_deg_s = LaunchConfiguration(
        'fire_max_pitch_velocity_deg_s')
    fire_max_reprojection_error_px = LaunchConfiguration(
        'fire_max_reprojection_error_px')
    require_auto_aim_mode = LaunchConfiguration('require_auto_aim_mode')
    control_target_filter_alpha = LaunchConfiguration(
        'control_target_filter_alpha')
    control_yaw_deadband_deg = LaunchConfiguration('control_yaw_deadband_deg')
    control_pitch_deadband_deg = LaunchConfiguration(
        'control_pitch_deadband_deg')
    start_serial = LaunchConfiguration('start_serial')
    local_only = LaunchConfiguration('local_only')

    live_launch = os.path.join(
        get_package_share_directory('auto_aim_ros2'),
        'launch',
        'live_detection.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'project_root',
            default_value='/home/nvidia/game_26_current/sp_vision_25'),
        DeclareLaunchArgument('config_path', default_value='configs/school.yaml'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='package://hik_camera/config/camera_info_vehicle.yaml'),
        DeclareLaunchArgument('enable_control_output', default_value='false'),
        DeclareLaunchArgument('enable_fire', default_value='false'),
        DeclareLaunchArgument('fire_hold_s', default_value='0.15'),
        DeclareLaunchArgument('fire_interval_s', default_value='0.5'),
        DeclareLaunchArgument('fire_shot_period_s', default_value='0.1'),
        DeclareLaunchArgument('fire_burst_count', default_value='3'),
        DeclareLaunchArgument('fire_yaw_tolerance_deg', default_value='0.50'),
        DeclareLaunchArgument('fire_pitch_tolerance_deg', default_value='0.50'),
        DeclareLaunchArgument(
            'fire_max_yaw_velocity_deg_s', default_value='5.0'),
        DeclareLaunchArgument(
            'fire_max_pitch_velocity_deg_s', default_value='5.0'),
        DeclareLaunchArgument(
            'fire_max_reprojection_error_px', default_value='3.0'),
        DeclareLaunchArgument('require_auto_aim_mode', default_value='true'),
        DeclareLaunchArgument('control_target_filter_alpha', default_value='0.25'),
        DeclareLaunchArgument('control_yaw_deadband_deg', default_value='0.25'),
        DeclareLaunchArgument('control_pitch_deadband_deg', default_value='0.20'),
        DeclareLaunchArgument('start_serial', default_value='false'),
        DeclareLaunchArgument('local_only', default_value='1'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', local_only),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(live_launch),
            launch_arguments={
                'project_root': project_root,
                'config_path': config_path,
                'camera_info_url': camera_info_url,
                'enable_control_output': enable_control_output,
                'enable_fire': enable_fire,
                'fire_hold_s': fire_hold_s,
                'fire_interval_s': fire_interval_s,
                'fire_shot_period_s': fire_shot_period_s,
                'fire_burst_count': fire_burst_count,
                'fire_yaw_tolerance_deg': fire_yaw_tolerance_deg,
                'fire_pitch_tolerance_deg': fire_pitch_tolerance_deg,
                'fire_max_yaw_velocity_deg_s': fire_max_yaw_velocity_deg_s,
                'fire_max_pitch_velocity_deg_s': fire_max_pitch_velocity_deg_s,
                'fire_max_reprojection_error_px': fire_max_reprojection_error_px,
                'require_auto_aim_mode': require_auto_aim_mode,
                'control_target_filter_alpha': control_target_filter_alpha,
                'control_yaw_deadband_deg': control_yaw_deadband_deg,
                'control_pitch_deadband_deg': control_pitch_deadband_deg,
                'local_only': local_only,
            }.items(),
        ),
        Node(
            package='serical_device_ros2',
            executable='vision_pub_node',
            name='vision_pub',
            output='screen',
            condition=IfCondition(start_serial),
        ),
        Node(
            package='serical_device_ros2',
            executable='robot_ctrl_main',
            name='robot_ctrl',
            output='screen',
            condition=IfCondition(start_serial),
        ),
    ])
