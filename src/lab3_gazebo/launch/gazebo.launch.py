"""Launch the CC robot with differential drive, odometry, pose and TF."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lab_share = Path(get_package_share_directory('lab3_gazebo'))
    robot_share = Path(get_package_share_directory('cc_robot_urdf'))
    gazebo_share = Path(get_package_share_directory('gazebo_ros'))
    robot_description = (robot_share / 'urdf' / 'CC_robot_gazebo.urdf').read_text()
    model_paths = [str(robot_share.parent)]
    if os.environ.get('GAZEBO_MODEL_PATH'):
        model_paths.append(os.environ['GAZEBO_MODEL_PATH'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true', choices=['true', 'false'],
            description='Open the Gazebo window; false runs the same simulation headless.'),
        DeclareLaunchArgument(
            'world', default_value=str(lab_share / 'worlds' / 'empty.world'),
            description='Gazebo Classic world file.'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(model_paths)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_share / 'launch' / 'gazebo.launch.py')),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'world': LaunchConfiguration('world'),
            }.items()),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'robot_description': ParameterValue(robot_description, value_type=str)}]),
        Node(
            package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=['-topic', '/robot_description', '-entity', 'cc_robot',
                       '-x', '0', '-y', '0', '-z', '0.04', '-timeout', '60']),
        Node(
            package='lab3_gazebo', executable='odom_to_pose.py',
            name='odom_to_pose', output='screen',
            parameters=[{'use_sim_time': True}]),
    ])
