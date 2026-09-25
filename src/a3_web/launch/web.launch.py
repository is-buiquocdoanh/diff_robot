"""Chạy web điều khiển robot A3 (1 lệnh duy nhất - mọi thứ còn lại bật/tắt từ giao diện web).

    ros2 launch a3_web web.launch.py
    ros2 launch a3_web web.launch.py port:=9000 autostart_bringup:=false
    ros2 launch a3_web web.launch.py maps_dir:=/home/pi/maps controller:=rpp

Sau đó mở http://<ip-robot>:8080 . Web tự bật/tắt bringup, SLAM, Nav2 (xem config/web_server.yaml).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('a3_web'), 'config', 'web_server.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='0.0.0.0', description='Địa chỉ lắng nghe'),
        DeclareLaunchArgument('port', default_value='8080', description='Cổng web'),
        DeclareLaunchArgument('maps_dir', default_value='', description='Thư mục bản đồ (rỗng = tự tìm src/a3_maps)'),
        DeclareLaunchArgument('autostart_bringup', default_value='true',
                              description='Tự chạy a3_bringup khi web khởi động'),
        DeclareLaunchArgument('controller', default_value='dwb', description='dwb | rpp | mppi'),
        Node(
            package='a3_web',
            executable='web_server',
            sigterm_timeout='20', sigkill_timeout='10',  # để web kịp tắt các stack con
            name='a3_web_server',
            output='screen',
            parameters=[config, {
                'host': LaunchConfiguration('host'),
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                'maps_dir': LaunchConfiguration('maps_dir'),
                'autostart_bringup': ParameterValue(LaunchConfiguration('autostart_bringup'), value_type=bool),
                'controller': LaunchConfiguration('controller'),
            }],
        ),
    ])
