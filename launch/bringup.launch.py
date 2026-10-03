from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('mini_nav')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'params.yaml']),
            description='Parameter file for all mini_nav nodes'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz2'),

        Node(package='mini_nav', executable='simulator', name='simulator',
             parameters=[params_file], output='screen'),
        Node(package='mini_nav', executable='planner', name='planner',
             parameters=[params_file], output='screen'),
        Node(package='mini_nav', executable='controller', name='controller',
             parameters=[params_file], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'mini_nav.rviz'])],
             condition=IfCondition(use_rviz)),
    ])
