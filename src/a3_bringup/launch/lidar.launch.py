import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    frame_id = LaunchConfiguration('frame_id')

    filter_config = os.path.join(
        get_package_share_directory('a3_bringup'), 'config', 'lidar_filter.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ydlidar',
            description='Cổng serial của RPLidar A1M8 (nên dùng udev symlink cố định, vd /dev/rplidar)'),
        DeclareLaunchArgument(
            'frame_id', default_value='laser_link',
            description='Frame TF của lidar, khớp với laser_link trong a3_description/urdf/lidar.xacro'),

        # RPLidar A1M8: publish dữ liệu thô ra /scan_raw (không publish thẳng
        # /scan) vì còn phải đi qua bộ lọc khoảng cách bên dưới trước khi tới
        # slam_toolbox / nav2.
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': serial_port,
                'serial_baudrate': 115200,
                'frame_id': frame_id,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
            }],
            remappings=[('scan', 'scan_raw')],
            # rplidar_node hiện crash (buffer overflow) thay vì báo lỗi gọn
            # khi serial_port chưa tồn tại/chưa cắm lidar - respawn để tự
            # khởi động lại thay vì im lặng chết hẳn.
            respawn=True,
            respawn_delay=2.0,
        ),

        # Lọc bỏ tia < 30cm (xem config/lidar_filter.yaml) rồi republish ra
        # /scan như bình thường, để các node khác (slam_toolbox, nav2...)
        # không cần biết gì về bước lọc này.
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_to_scan_filter_chain',
            output='screen',
            parameters=[filter_config],
            remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan')],
        ),
    ])
