from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_rviz',
            default_value='false',
            choices=['true','false'],
            description='Open RVIZ for Go1 visualization'
        ),

        DeclareLaunchArgument(
            name='fixed_frame',
            default_value='base_footprint',
            description='Fixed frame for RVIZ'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('unitree_legged_real'),
                    'launch',
                    'high.launch.py'
                ])
            ),
            launch_arguments=[
                ('use_rviz', LaunchConfiguration('use_rviz')),
                ('fixed_frame', LaunchConfiguration('fixed_frame')),
            ],
        ),

        Node(
            package='unitree_nav',
            executable='cmd_processor',
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('rslidar_sdk'),
                    'launch',
                    'start.py'
                ])
            )
        )
    ])