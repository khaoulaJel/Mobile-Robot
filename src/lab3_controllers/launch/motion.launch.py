import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    share = get_package_share_directory('lab3_controllers')
    cfg = os.path.join(share, 'config', LaunchConfiguration('config').perform(context))
    params = [cfg, {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true'}]
    # state_topic / cmd_topic: only override the config file when given (hardware.yaml sets the real robot's topics)
    over = {k: LaunchConfiguration(k).perform(context) for k in ('state_topic', 'cmd_topic')
            if LaunchConfiguration(k).perform(context)}
    if over:
        params.append(over)
    return [Node(package='lab3_controllers', executable='motion_controller', name='motion_controller',
                 output='screen', parameters=params)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='controllers.yaml',
                              description='controllers.yaml (competition, Gazebo), robust.yaml, lab_robot.yaml '
                                          '(professor robot model) or hardware.yaml (real robot, first run)'),
        DeclareLaunchArgument('state_topic', default_value='',
                              description="empty = the config file's topic. Gazebo /odom; real robot /diff_cont/odom"),
        DeclareLaunchArgument('cmd_topic', default_value='',
                              description="empty = the config file's topic. Gazebo /diff_cont/cmd_vel_unstamped; "
                                          'real robot /cmd_vel_tracker (laptop twist_mux input, joystick overrides)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='true for Gazebo, false on the real robot'),
        OpaqueFunction(function=launch_setup),
    ])
