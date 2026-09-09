"""Hiển thị robot A3 trong RViz2.

    ros2 launch a3_description display.launch.py
    ros2 launch a3_description display.launch.py gui:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('a3_description')

    model = LaunchConfiguration('model')
    gui = LaunchConfiguration('gui')
    rvizconfig = LaunchConfiguration('rvizconfig')

    robot_description = ParameterValue(
        Command(['xacro ', model]), value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution([pkg, 'urdf', 'a3.urdf.xacro']),
            description='Đường dẫn file xacro của robot',
        ),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Bật joint_state_publisher_gui để quay bánh bằng slider',
        ),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([pkg, 'rviz', 'a3.rviz']),
            description='File cấu hình RViz',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rvizconfig],
            output='screen',
        ),
    ])
