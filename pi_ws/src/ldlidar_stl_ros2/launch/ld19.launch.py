from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        # base_link -> base_laser is owned by robot_state_publisher from the URDF.
        Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node',
             name='LD19', output='screen', parameters=[{
                 'product_name': 'LDLiDAR_LD19', 'topic_name': 'scan',
                 'frame_id': 'base_laser', 'port_name': LaunchConfiguration('port'),
                 'port_baudrate': 230400, 'laser_scan_dir': True,
                 'enable_angle_crop_func': False,
                 'angle_crop_min': 135.0, 'angle_crop_max': 225.0,
             }]),
    ])
