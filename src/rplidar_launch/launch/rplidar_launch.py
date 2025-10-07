# rplidar_launch.py
# ROS2 Python launch file to start RPLIDAR (rplidar_ros)
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLIDAR'
    )
    
    serial_baud_arg = DeclareLaunchArgument(
        'serial_baud',
        default_value='115200',
        description='Baud rate for RPLIDAR'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='Frame ID for the laser scans'
    )
    
    inverted_arg = DeclareLaunchArgument(
        'inverted',
        default_value='false',
        description='Set to true if the scan is inverted'
    )
    
    angle_compensate_arg = DeclareLaunchArgument(
        'angle_compensate',
        default_value='true',
        description='Set to true to enable angle compensation'
    )
    
    serial_port = LaunchConfiguration('port')
    serial_baud = LaunchConfiguration('serial_baud')
    frame_id = LaunchConfiguration('frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    
    # Define the RPLIDAR node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': serial_baud,
            'frame_id': frame_id,
            'inverted': inverted,
            'angle_compensate': angle_compensate
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        serial_baud_arg,
        frame_id_arg,
        inverted_arg,
        angle_compensate_arg,
        rplidar_node
    ])