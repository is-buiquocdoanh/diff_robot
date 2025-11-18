from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='robot_kinematic',
            executable='encoder_to_joint_state',  # Node chuyển encoder sang joint_states
            output='screen'
        )
        
        # Node(
        #     package='robot_kinematic',
        #     executable='encoder_to_odom',
        #     output='screen'
        # ),

        
    ])
