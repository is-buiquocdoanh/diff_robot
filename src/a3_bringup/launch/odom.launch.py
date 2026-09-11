from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rate', default_value='50.0',
            description='Tần suất publish /odom (Hz)'),
        DeclareLaunchArgument(
            'odom_frame', default_value='odom',
            description='Frame gốc của odometry'),
        DeclareLaunchArgument(
            'base_frame', default_value='base_footprint',
            description='Frame gắn với robot, khớp base_footprint trong a3_description'),
        DeclareLaunchArgument(
            'publish_tf', default_value='true',
            description='Có broadcast TF odom -> base_frame hay không'),
        DeclareLaunchArgument(
            'cmd_vel_timeout', default_value='0.5',
            description='Sau bao nhiêu giây không nhận /cmd_vel thì coi robot đã dừng (giây)'),

        # Publish /odom + TF odom->base_footprint mà không cần encoder: lấy
        # (vx, wz) từ /cmd_vel đang gửi cho robot và lấy hướng (yaw) từ
        # /imu/data (BNO055). Xem giải thích đầy đủ trong
        # a3_driver/scripts/odom_publisher_node.py.
        Node(
            package='a3_driver',
            executable='odom_publisher_node.py',
            name='odom_publisher_node',
            output='screen',
            parameters=[{
                'rate': LaunchConfiguration('rate'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout'),
            }],
        ),
    ])
