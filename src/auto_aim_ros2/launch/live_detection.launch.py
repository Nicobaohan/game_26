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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    project_root = LaunchConfiguration('project_root')
    config_path = LaunchConfiguration('config_path')
    image_topic = LaunchConfiguration('image_topic')
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
    control_moving_filter_alpha = LaunchConfiguration(
        'control_moving_filter_alpha')
    control_moving_speed_mps = LaunchConfiguration('control_moving_speed_mps')
    max_control_rate_deg_s = LaunchConfiguration('max_control_rate_deg_s')
    max_control_step_deg = LaunchConfiguration('max_control_step_deg')
    control_yaw_deadband_deg = LaunchConfiguration('control_yaw_deadband_deg')
    control_pitch_deadband_deg = LaunchConfiguration(
        'control_pitch_deadband_deg')
    local_only = LaunchConfiguration('local_only')
    camera_launch = os.path.join(
        get_package_share_directory('hik_camera'), 'launch', 'hik_camera.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'project_root',
            default_value='/home/nvidia/game_26_current/sp_vision_25'),
        DeclareLaunchArgument('config_path', default_value='configs/school.yaml'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
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
        DeclareLaunchArgument('control_moving_filter_alpha', default_value='0.45'),
        DeclareLaunchArgument('control_moving_speed_mps', default_value='0.80'),
        DeclareLaunchArgument('max_control_rate_deg_s', default_value='60.0'),
        DeclareLaunchArgument('max_control_step_deg', default_value='4.0'),
        DeclareLaunchArgument('control_yaw_deadband_deg', default_value='0.25'),
        DeclareLaunchArgument('control_pitch_deadband_deg', default_value='0.20'),
        DeclareLaunchArgument('local_only', default_value='1'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', local_only),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            launch_arguments={
                # Reliable publishers work with both reliable GUI viewers and
                # the best-effort auto-aim subscriber.
                'use_sensor_data_qos': 'false',
                'camera_info_url': camera_info_url,
            }.items(),
        ),
        Node(
            package='auto_aim_ros2',
            executable='auto_aim_node',
            name='auto_aim_node',
            output='screen',
            parameters=[{
                'project_root': project_root,
                'config_path': config_path,
                'image_topic': image_topic,
                'publish_debug_image': True,
                'enable_control_output': ParameterValue(
                    enable_control_output, value_type=bool),
                'enable_fire': ParameterValue(enable_fire, value_type=bool),
                'fire_hold_s': ParameterValue(fire_hold_s, value_type=float),
                'fire_interval_s': ParameterValue(
                    fire_interval_s, value_type=float),
                'fire_shot_period_s': ParameterValue(
                    fire_shot_period_s, value_type=float),
                'fire_burst_count': ParameterValue(
                    fire_burst_count, value_type=int),
                'fire_yaw_tolerance_deg': ParameterValue(
                    fire_yaw_tolerance_deg, value_type=float),
                'fire_pitch_tolerance_deg': ParameterValue(
                    fire_pitch_tolerance_deg, value_type=float),
                'fire_max_yaw_velocity_deg_s': ParameterValue(
                    fire_max_yaw_velocity_deg_s, value_type=float),
                'fire_max_pitch_velocity_deg_s': ParameterValue(
                    fire_max_pitch_velocity_deg_s, value_type=float),
                'fire_max_reprojection_error_px': ParameterValue(
                    fire_max_reprojection_error_px, value_type=float),
                'require_auto_aim_mode': ParameterValue(
                    require_auto_aim_mode, value_type=bool),
                'control_target_filter_alpha': ParameterValue(
                    control_target_filter_alpha, value_type=float),
                'control_moving_filter_alpha': ParameterValue(
                    control_moving_filter_alpha, value_type=float),
                'control_moving_speed_mps': ParameterValue(
                    control_moving_speed_mps, value_type=float),
                'max_control_rate_deg_s': ParameterValue(
                    max_control_rate_deg_s, value_type=float),
                'max_control_step_deg': ParameterValue(
                    max_control_step_deg, value_type=float),
                'control_yaw_deadband_deg': ParameterValue(
                    control_yaw_deadband_deg, value_type=float),
                'control_pitch_deadband_deg': ParameterValue(
                    control_pitch_deadband_deg, value_type=float),
            }],
        ),
    ])
