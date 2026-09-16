import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                # /odom_rf2o (không phải /odom) -- output của EKF (ekf.yaml, a3_bringup)
                # mới là /odom canonical. rf2o chỉ đóng vai 1 sensor input cho EKF
                # (xem odom0/odom1 trong ekf.yaml), không phải nguồn odometry cuối cùng.
                'odom_topic': '/odom_rf2o',
                # EKF là node DUY NHẤT publish TF odom -> base_footprint (publish_tf:
                # true trong ekf.yaml) -- 2 node cùng publish 1 TF sẽ xung đột/nhảy.
                'publish_tf': False,
                'base_frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 30.0
            }],
        ),
    ])
