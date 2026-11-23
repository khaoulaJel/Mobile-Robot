from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    params = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true'}
    for k in ('state_topic', 'cmd_topic'):          # only override the node defaults when given
        value = LaunchConfiguration(k).perform(context)
        if value:
            params[k] = value
    return [Node(package='lab3_controllers', executable='symbolic_controller', name='symbolic_controller',
                 output='screen', parameters=[params])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('state_topic', default_value='',
                              description='empty = /odom. Real robot: /diff_cont/odom'),
        DeclareLaunchArgument('cmd_topic', default_value='',
                              description='empty = /diff_cont/cmd_vel_unstamped. Real robot: /cmd_vel_tracker'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='true for Gazebo, false on the real robot'),
        OpaqueFunction(function=launch_setup),
    ])
