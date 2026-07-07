from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

launch_args = [
DeclareLaunchArgument(
    'serial_port',
    default_value='/dev/ttyUSB0',
    description='RPLIDAR serial port name.'),
DeclareLaunchArgument(
    'serial_baudrate',
    default_value='460800',
    description='RPLIDAR serial port baud rate.'),
DeclareLaunchArgument(
    'frame_id',
    default_value='laser',
    description='Custom frame ID for the scan data.'),
DeclareLaunchArgument(
    'inverted',
    default_value='false',
    description='Whether to invert the scan data.'),
DeclareLaunchArgument(
    'angle_compensate',
    default_value='True',
    description='Publish scan data with a consistent number of points. Disable this option to publish the variable-sized data received directly from the RPLIDAR.'),
DeclareLaunchArgument(
    'scan_mode',
    default_value='Standard',
    description='RPLIDAR scan mode.'),
]

def launch_setup(context):
  serial_port = LaunchConfiguration('serial_port')
  serial_baudrate = LaunchConfiguration('serial_baudrate')
  frame_id = LaunchConfiguration('frame_id')
  inverted = LaunchConfiguration('inverted')
  angle_compensate = LaunchConfiguration('angle_compensate')
  scan_mode = LaunchConfiguration('scan_mode')
  
  rviz_config_dir = os.path.join(get_package_share_directory('lgdx_rplidar_c1'), 'rviz', 'lgdx_rplidar_c1.rviz')
  
  lidar_node = Node(
    package='lgdx_rplidar_c1',
    executable='rplidar_c1_node',
    name='rplidar_c1_node',
    parameters=[{
      'serial_port': serial_port, 
      'serial_baudrate': serial_baudrate, 
      'frame_id': frame_id,
      'inverted': inverted, 
      'angle_compensate': angle_compensate,
      'scan_mode': scan_mode
    }],
    output='screen'
  )
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_dir],
    output='screen'
  )
  return [lidar_node, rviz_node]

  
def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld