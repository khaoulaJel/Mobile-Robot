"""Raspberry Pi only: Arduino, wheel data and LiDAR."""
import atexit
import fcntl
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def hardware_setup(context):
    fd=os.open(f'/tmp/cc_pi_hardware_{os.getuid()}.lock',os.O_CREAT|os.O_RDWR|os.O_CLOEXEC|os.O_NOFOLLOW,0o600)
    try:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(fd);raise RuntimeError('Pi hardware launch is already running. Stop it with Ctrl+C first.') from None
    atexit.register(os.close,fd)
    manager=Node(package='controller_manager',executable='ros2_control_node',output='screen',
                 parameters=[PathJoinSubstitution([FindPackageShare('articubot_one'),'config','hardware_controllers.yaml'])],
                 remappings=[('~/robot_description','/robot_description')])
    wait=Node(package='articubot_one',executable='wait_for_hardware.py',output='screen')
    spawner=Node(package='controller_manager',executable='spawner',output='screen',
                 arguments=['joint_broad','diff_cont','--controller-manager','/controller_manager','--controller-manager-timeout','60'])
    lidar=Node(package='ldlidar_stl_ros2',executable='ldlidar_stl_ros2_node',name='LD19',output='screen',
               condition=IfCondition(LaunchConfiguration('lidar')),
               parameters=[{'product_name':'LDLiDAR_LD19','topic_name':'scan','frame_id':'base_laser',
                            'port_name':LaunchConfiguration('lidar_port'),'port_baudrate':230400,
                            'laser_scan_dir':True,'enable_angle_crop_func':False,
                            'angle_crop_min':135.0,'angle_crop_max':225.0}])
    def ready(event,_context):
        if event.returncode==0:return [spawner]
        return [EmitEvent(event=Shutdown(reason='Hardware initialization was interrupted.'))]
    def spawned(event,_context):
        return [] if event.returncode==0 else [EmitEvent(event=Shutdown(reason='Controller activation failed.'))]
    handlers=[RegisterEventHandler(OnProcessExit(target_action=wait,on_exit=ready)),
              RegisterEventHandler(OnProcessExit(target_action=spawner,on_exit=spawned))]
    for process in (manager,lidar):
        handlers.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='A Pi hardware process exited.'))])))
    return [*handlers,manager,wait,lidar]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp'),
        DeclareLaunchArgument('lidar_port',default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar',default_value='true',choices=['true','false']),
        OpaqueFunction(function=hardware_setup),
    ])
