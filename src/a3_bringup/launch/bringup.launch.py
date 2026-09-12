"""Bringup toàn bộ robot A3 thật (không mô phỏng, không RViz).

Gồm:
  - robot_state_publisher (+ joint_state_publisher mặc định) từ URDF a3_description,
    để có đủ cây TF base_footprint -> base_link -> laser_link/...
  - kinematic.py + serial_bridge_node.py (a3_driver): /cmd_vel -> ESP32, và
    ESP32 -> /imu/data, qua đúng 1 cổng USB.
  - odom_publisher_node.py (a3_driver, qua odom.launch.py): /odom + TF
    odom -> base_footprint, dùng cmd_vel + imu (chưa có encoder feedback).
  - lidar.launch.py (a3_bringup): RPLidar A1M8 -> /scan đã lọc tia < 30cm.

    ros2 launch a3_bringup bringup.launch.py
    ros2 launch a3_bringup bringup.launch.py esp32_port:=/dev/esp32 lidar_port:=/dev/ttyUSB0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_pkg = FindPackageShare('a3_description')
    bringup_pkg = get_package_share_directory('a3_bringup')

    model = LaunchConfiguration('model')
    esp32_port = LaunchConfiguration('esp32_port')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')

    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution([description_pkg, 'urdf', 'a3.urdf.xacro']),
            description='Đường dẫn file xacro của robot'),
        DeclareLaunchArgument(
            'esp32_port', default_value='/dev/esp32',
            description='Cổng USB nối ESP32 (lệnh động cơ + IMU dùng chung 1 cổng)'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/rplidar',
            description='Cổng USB nối RPLidar A1M8'),
        DeclareLaunchArgument(
            'lidar_frame_id', default_value='laser_link',
            description='Frame TF của lidar, khớp a3_description/urdf/lidar.xacro'),

        # Cây TF tĩnh của robot (base_footprint -> base_link -> laser_link/...)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # Chưa có encoder publish /joint_states thật, dùng joint_state_publisher
        # mặc định (giá trị 0) để cây TF không thiếu link nào.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),

        # cmd_vel -> động học vi sai -> Velquery
        Node(
            package='a3_driver',
            executable='kinematic.py',
            name='kinematic_node',
            output='screen',
        ),
        # Velquery -> frame CAN-serial xuống ESP32, đồng thời đọc /imu/data
        # ngược lại, tất cả qua 1 cổng USB (xem a3_driver/scripts/serial_bridge_node.py).
        Node(
            package='a3_driver',
            executable='serial_bridge_node.py',
            name='serial_bridge_node',
            output='screen',
            parameters=[{'serial_port': esp32_port}],
        ),

        # /odom + TF odom -> base_footprint (cmd_vel + imu, xem odom.launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_pkg, 'launch', 'odom.launch.py')
            ),
        ),

        # RPLidar A1M8 -> /scan đã lọc tia < 30cm (xem lidar.launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_pkg, 'launch', 'lidar.launch.py')
            ),
            launch_arguments={
                'serial_port': lidar_port,
                'frame_id': lidar_frame_id,
            }.items(),
        ),
    ])
