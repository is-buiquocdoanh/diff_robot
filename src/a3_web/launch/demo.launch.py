"""Chạy thử giao diện web KHÔNG cần robot: robot giả (lidar/odom/SLAM/Nav2 giả) + web.

    ros2 launch a3_web demo.launch.py
    ros2 launch a3_web demo.launch.py world_map:=/đường/dẫn/map.yaml

Các stack (bringup/slam/nav) được thay bằng fake_stack nên bấm "Quét bản đồ"/"Điều hướng" trên
web vẫn hoạt động đầy đủ: lái robot giả để vẽ map, lưu map, chọn điểm, điều hướng.
Bản đồ demo lưu ở ~/.a3_web_demo/maps để không lẫn với bản đồ thật.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('world_map', default_value='',
                              description='yaml của bản đồ "thế giới thật" để robot giả raycast (rỗng = map1 hoặc phòng mẫu)'),
        DeclareLaunchArgument('maps_dir', default_value='~/.a3_web_demo/maps'),
        Node(package='a3_web', executable='fake_robot', name='fake_robot', output='screen',
             parameters=[{'world_map': LaunchConfiguration('world_map')}]),
        Node(
            package='a3_web', executable='web_server', name='a3_web_server', output='screen',
            sigterm_timeout='20', sigkill_timeout='10',  # để web kịp tắt các stack con
            parameters=[{
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                'maps_dir': LaunchConfiguration('maps_dir'),
                'config_dir': '~/.a3_web_demo',
                'autostart_bringup': True,
                'cmd_bringup': 'ros2 run a3_web fake_stack bringup',
                'cmd_slam': 'ros2 run a3_web fake_stack slam',
                'cmd_nav': 'ros2 run a3_web fake_stack nav',
            }],
        ),
    ])
