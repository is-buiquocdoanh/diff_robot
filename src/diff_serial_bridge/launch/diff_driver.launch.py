from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diff_serial_bridge',   # package chứa node
            executable='diff_serial_bridge',  # tên file py (không có .py)
            output='screen',
            # Nếu cần, thêm parameter hoặc remapping topic ở đây
        ),

        Node(
            package='diff_serial_bridge',
            executable='cmdvel_to_motorcommand',
            output='screen',
        ),
    ])
