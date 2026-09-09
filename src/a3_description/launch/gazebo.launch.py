"""Chạy robot A3 trong Gazebo Classic 11.

    ros2 launch a3_description gazebo.launch.py
    ros2 launch a3_description gazebo.launch.py rviz:=true
    ros2 launch a3_description gazebo.launch.py gui:=false          # headless
    ros2 launch a3_description gazebo.launch.py world:=/đường/dẫn/abc.world

Sau khi Gazebo chạy, lái xe bằng:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('a3_description')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    x, y, z = (LaunchConfiguration(n) for n in ('x', 'y', 'z'))

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg, 'urdf', 'a3.urdf.xacro']),
            ' sim_gazebo:=true',
        ]),
        value_type=str,
    )

    return LaunchDescription([
        # Truyền thẳng làm tham số vị trí cho gzserver. Để chuỗi rỗng thì gzserver
        # in "[Err] Could not open file[]" rồi mới tự fallback, nên chỉ thẳng luôn
        # vào empty.world (Gazebo phân giải qua GAZEBO_RESOURCE_PATH).
        DeclareLaunchArgument('world', default_value='worlds/empty.world',
                              description='File .world'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Mở cửa sổ Gazebo (false = headless)'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Mở kèm RViz2'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.02',
                              description='Thả robot cao hơn mặt đất một chút'),

        # gzserver: cần verbose để thấy lỗi plugin nếu có
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world, 'verbose': 'true'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(gui),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'a3',
                '-x', x, '-y', y, '-z', z,
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            condition=IfCondition(rviz),
            parameters=[{'use_sim_time': True}],
            arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'a3_gazebo.rviz'])],
        ),
    ])
