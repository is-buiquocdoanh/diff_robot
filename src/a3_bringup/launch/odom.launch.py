"""Tạo /odom + TF odom -> base_footprint bằng EKF (robot_localization),
fusion /wheel/odom (vx suy từ /cmd_vel) với /imu/data (yaw + vyaw từ BNO055).

Trước đây 1 node tự tích phân trực tiếp (odom_publisher_node.py) ghi đè yaw
thô từ IMU mỗi tick, gây rung/giật khi IMU nhiễu. Nay tách làm 2:
  - wheel_odom_node.py (a3_driver): chỉ publish vx lên /wheel/odom.
  - ekf.launch.py (a3_bringup): EKF lọc mượt theo covariance rồi mới ra /odom.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('a3_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rate', default_value='50.0',
            description='Tần suất publish /wheel/odom (Hz)'),
        DeclareLaunchArgument(
            'odom_frame', default_value='odom',
            description='Frame gốc của odometry'),
        DeclareLaunchArgument(
            'base_frame', default_value='base_footprint',
            description='Frame gắn với robot, khớp base_footprint trong a3_description'),
        DeclareLaunchArgument(
            'cmd_vel_timeout', default_value='0.5',
            description='Sau bao nhiêu giây không nhận /cmd_vel thì coi robot đã dừng (giây)'),

        # /cmd_vel -> /wheel/odom (chỉ vx, xem a3_driver/scripts/wheel_odom_node.py)
        Node(
            package='a3_driver',
            executable='wheel_odom_node.py',
            name='wheel_odom_node',
            output='screen',
            parameters=[{
                'rate': LaunchConfiguration('rate'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout'),
            }],
        ),

        # /wheel/odom + /imu/data -> /odom + TF (xem config/ekf.yaml)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_pkg, 'launch', 'ekf.launch.py')
            ),
        ),
    ])
