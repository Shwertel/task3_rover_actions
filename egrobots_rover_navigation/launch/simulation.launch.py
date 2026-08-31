"""Shared bring-up: Gazebo, the rover, its controllers, the EKF, and RViz.

No behaviour node — included by rover.launch.py, or run alone for teleop.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('egrobots_rover_navigation')
    xacro_path = os.path.join(pkg_share, 'urdf', 'egrobots_rover.urdf.xacro')
    world_path = os.path.join(pkg_share, 'worlds', 'egrobots_world.world')
    rviz_config = os.path.join(pkg_share, 'rviz', 'egrobots_rover.rviz')

    use_rviz = LaunchConfiguration('rviz')

    # value_type=str matters: without it launch parses the URDF as YAML, and any
    # "word:" sequence in the XML aborts the launch after Gazebo has started.
    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    gazebo_pkg = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_path}.items())

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        gazebo_launch,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-topic', 'robot_description',
                        '-entity', 'egrobots_rover', '-z', '0.2'],
             output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster'], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['diff_drive_controller'], output='screen'),
        # Fuses wheel velocity with IMU heading and owns odom -> base_link.
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen',
             parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_config],
             condition=IfCondition(use_rviz), output='screen'),
    ])
