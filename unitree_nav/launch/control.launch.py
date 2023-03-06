from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from unitree_nav_launch_module import TernaryTextSubstitution


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            choices=['true','false'],
            description='Open RVIZ for Go1 visualization'
        ),

        DeclareLaunchArgument(
            name='use_onboard_odometry',
            default_value='true',
            choices=['true','false'],
            description='Publish odometry from received high state messages'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                    PathJoinSubstitution([
                        FindPackageShare('unitree_nav'),
                        'config',
                        'control.rviz'
                    ]),
                '-f',
                    TernaryTextSubstitution(
                        IfCondition(LaunchConfiguration('use_onboard_odometry')),
                        'odom',
                        'base_link'
                    ),
            ],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
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
                ('use_rviz', 'false'),
            ],
        ),

        Node(
            package='unitree_nav',
            executable='cmd_processor',
            output='screen'
        ),

        Node(
            package='unitree_nav',
            executable='odometry',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_onboard_odometry'))
        )
    ])