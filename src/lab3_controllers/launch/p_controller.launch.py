import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    share = get_package_share_directory('lab3_controllers')
    cfg = LaunchConfiguration('config').perform(context)
    cfg = cfg if os.path.isabs(cfg) else os.path.join(share, 'config', cfg)    # a file name in config/, or a full path
    params = [cfg, {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true'}]
    # state_topic / cmd_topic: only override the config file when given
    over = {k: LaunchConfiguration(k).perform(context) for k in ('state_topic', 'cmd_topic')
            if LaunchConfiguration(k).perform(context)}
    if over:
        params.append(over)
    return [Node(package='lab3_controllers', executable='p_controller', name='p_controller',
                 output='screen', parameters=params)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='p_controller.yaml',
                              description='coefficients file: a name in config/ or a full path (e.g. the numbers '
                                          'you receive on the robot)'),
        DeclareLaunchArgument('state_topic', default_value='',
                              description="empty = the config file's topic (real robot /diff_cont/odom)"),
        DeclareLaunchArgument('cmd_topic', default_value='',
                              description="empty = the config file's topic (real robot /cmd_vel_tracker)"),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
