import os, atexit, fcntl
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    fd=os.open(f'/tmp/cc_desktop_{os.getuid()}_{os.getenv("ROS_DOMAIN_ID","0")}.lock',os.O_CREAT|os.O_RDWR,0o600)
    fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB);atexit.register(os.close,fd)
    share=get_package_share_directory('articubot_one');cfg=lambda f:os.path.join(share,'config',f)
    enabled=lambda key:LaunchConfiguration(key).perform(context)=='true'
    description=xacro.process_file(os.path.join(share,'description','robot.hardware.urdf.xacro'),mappings={'device':LaunchConfiguration('device').perform(context)}).toxml()
    nodes=[Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':description}]),
      Node(package='robot_localization',executable='ekf_node',name='ekf_filter_node',parameters=[cfg('ekf.yaml')]),
      Node(package='twist_mux',executable='twist_mux',parameters=[cfg('mux.yaml')],remappings=[('cmd_vel_out','/diff_cont/cmd_vel_unstamped')])]
    if enabled('slam'):nodes.append(Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',parameters=[cfg('slam.yaml')]))
    handlers=[RegisterEventHandler(OnProcessExit(target_action=n,on_exit=[EmitEvent(event=Shutdown(reason='Essential desktop process exited.'))])) for n in nodes]
    nodes.append(Node(package='cc_pose_tools',executable='tf_to_pose.py'))
    if enabled('joy'):
      nodes.extend([Node(package='joy',executable='joy_node',parameters=[cfg('joy.yaml')]),Node(package='teleop_twist_joy',executable='teleop_node',name='teleop_twist_joy_node',parameters=[cfg('joy.yaml')],remappings=[('cmd_vel','/cmd_vel_joy')])])
    if enabled('rviz'):nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',cfg('robot.rviz')]))
    if enabled('matlab'):nodes.append(ExecuteProcess(cmd=['matlab','-desktop','-r',"run('"+os.path.join(share,'matlab','configure_and_open.m')+"')"]))
    return handlers+nodes

def generate_launch_description():
    return LaunchDescription([SetEnvironmentVariable('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp'),DeclareLaunchArgument('device',default_value='/dev/ttyACM0'),*[DeclareLaunchArgument(k,default_value=v,choices=['true','false']) for k,v in [('joy','true'),('slam','true'),('rviz','true'),('matlab','false')]],OpaqueFunction(function=setup)])
