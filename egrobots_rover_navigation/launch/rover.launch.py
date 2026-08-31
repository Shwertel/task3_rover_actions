"""Simulation plus the rover node, which exposes both interfaces:

    action  /move_to_goal        goal-based movement (this week's task)
    service /start_avoidance     geofence patrol (kept from the previous task)

    ros2 launch egrobots_rover_navigation rover.launch.py

    ros2 action send_goal -f /move_to_goal \
      egrobots_rover_interfaces/action/MoveToGoal \
      "{target: {x: 5.0, y: 0.0, z: 0.0}, tolerance: 0.0}"
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('egrobots_rover_navigation')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'simulation.launch.py')),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items())

    rover_node = Node(
        package='egrobots_rover_navigation',
        executable='rover_node',
        name='rover_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'rover_params.yaml')],
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        simulation,
        rover_node,
    ])
