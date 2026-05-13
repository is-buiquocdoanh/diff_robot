("""Launch file to start ros_serial_bridge and kinematic nodes.

Usage:
  ros2 launch robot_control robot_control.launch.py serial_port:=ttyUSB0 baudrate:=115200

This file declares launch arguments for the serial port and baudrate and
starts both nodes with output forwarded to screen.
""")

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

	kinematic_serial = Node(
		package='robot_control',
		executable='kinematic_serial',
		name='kinematic_serial',
		output='screen',
	)	
 
	return LaunchDescription([
		kinematic_serial,
	])

