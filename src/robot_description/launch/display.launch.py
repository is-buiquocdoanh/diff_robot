from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # process xacro to string
    robot_description_config = xacro.process_file(xacro_file).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    rviz_config = os.path.join(pkg_share, 'rviz', 'robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        # rviz_node
    ])
