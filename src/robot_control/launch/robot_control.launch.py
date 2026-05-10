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
	serial_port_arg = DeclareLaunchArgument(
		'serial_port', default_value='esp32', description='Serial device name (no /dev/)')

	baudrate_arg = DeclareLaunchArgument(
		'baudrate', default_value='115200', description='Serial baudrate')

	kinematic_serial = Node(
		package='robot_control',
		executable='kinematic_serial',
		name='kinematic_serial',
		output='screen',
		parameters=[
			{'serial_port': LaunchConfiguration('serial_port')},
			{'baudrate': LaunchConfiguration('baudrate')},
		]
	)	
 
	return LaunchDescription([
		serial_port_arg,
		baudrate_arg,
		kinematic_serial,
	])

