"""Tạo /odom + TF odom -> base_footprint bằng EKF (robot_localization), fusion
/wheel/odom (vx suy từ /cmd_vel) với /odom_rf2o (x,y,yaw + vx,vyaw, đo bằng so
khớp scan liên tiếp -- xem package rf2o_laser_odometry).

TẠM THỜI không dùng /imu/data (BNO055) -- xem comment đầu config/ekf.yaml.

Trước đây 1 node tự tích phân trực tiếp (odom_publisher_node.py) ghi đè yaw
thô từ IMU mỗi tick, gây rung/giật khi IMU nhiễu. Nay tách làm 3:
  - wheel_odom_node.py (a3_driver): chỉ publish vx lên /wheel/odom.
  - rf2o_laser_odometry_node (rf2o_laser_odometry): so khớp 2 scan liên tiếp
    trên /scan, publish x,y,yaw + vx,vyaw lên /odom_rf2o -- không phụ thuộc
    encoder/IMU nên không bị ảnh hưởng bởi nhiễu từ trường/chưa hiệu chuẩn.
  - ekf_node (robot_localization, config/ekf.yaml): lọc mượt theo covariance
    rồi mới ra /odom + TF, publish thẳng trong file này (không qua launch
    riêng -- ekf_node chỉ dùng đúng ở đây, tách file khác chỉ thêm 1 lớp
    include không cần thiết).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('a3_bringup')
    ekf_config = os.path.join(bringup_pkg, 'config', 'ekf.yaml')

    rate_arg = DeclareLaunchArgument(
        'rate', default_value='50.0',
        description='Tần suất publish /wheel/odom (Hz)')
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame', default_value='odom',
        description='Frame gốc của odometry')
    base_frame_arg = DeclareLaunchArgument(
        'base_frame', default_value='base_footprint',
        description='Frame gắn với robot, khớp base_footprint trong a3_description')
    cmd_vel_timeout_arg = DeclareLaunchArgument(
        'cmd_vel_timeout', default_value='0.5',
        description='Sau bao nhiêu giây không nhận /cmd_vel thì coi robot đã dừng (giây)')

    # /cmd_vel -> /wheel/odom (chỉ vx, xem a3_driver/scripts/wheel_odom_node.py)
    wheel_odom_node = Node(
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
    )

    # /scan -> /odom_rf2o (x,y,yaw + vx,vyaw, xem package rf2o_laser_odometry).
    # publish_tf: False -- EKF bên dưới là node DUY NHẤT publish TF odom ->
    # base_footprint (publish_tf: true trong ekf.yaml), 2 node cùng publish 1
    # TF sẽ xung đột/nhảy.
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 30.0,
        }],
    )

    # /wheel/odom + /odom_rf2o -> /odom + TF (xem config/ekf.yaml). ekf_node mặc
    # định publish odometry đã lọc ra topic `odometry/filtered` -- remap về
    # /odom cho khớp với những gì slam_toolbox/nav2 mong đợi, đồng thời tự
    # broadcast TF odom -> base_footprint (publish_tf: true trong ekf.yaml).
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', 'odom')],
    )

    return LaunchDescription([
        rate_arg,
        odom_frame_arg,
        base_frame_arg,
        cmd_vel_timeout_arg,

        wheel_odom_node,
        rf2o_node,
        ekf_node,
    ])
