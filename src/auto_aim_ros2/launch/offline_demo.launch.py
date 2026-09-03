from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_root = LaunchConfiguration('project_root')
    config_path = LaunchConfiguration('config_path')
    video_path = LaunchConfiguration('video_path')
    image_topic = LaunchConfiguration('image_topic')
    local_only = LaunchConfiguration('local_only')

    return LaunchDescription([
        DeclareLaunchArgument(
            'project_root',
            default_value='/home/nvidia/game_26_current/sp_vision_25'),
        DeclareLaunchArgument('config_path', default_value='configs/demo.yaml'),
        DeclareLaunchArgument('video_path', default_value='assets/demo/demo.avi'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('local_only', default_value='1'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', local_only),
        Node(
            package='auto_aim_ros2',
            executable='video_replay_node.py',
            name='video_replay_node',
            output='screen',
            parameters=[{
                'project_root': project_root,
                'video_path': video_path,
                'image_topic': image_topic,
                'loop': True,
            }],
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
            }],
        ),
    ])
