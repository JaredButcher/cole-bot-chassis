# SPDX-License-Identifier: GPL-3.0-or-later
"""
Bring up the chassis: description, ros2_control, controllers and twist_mux (PLAN.md §7.6).

The micro-ROS agent runs separately (ros/agent/compose.yaml).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    control_rate = LaunchConfiguration('control_rate')

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution(
                [FindPackageShare('colebot_description'), 'urdf', 'colebot.urdf.xacro']),
            ' use_mock_hardware:=', use_mock_hardware,
        ]),
        value_type=str)
    controllers = PathJoinSubstitution(
        [FindPackageShare('colebot_bringup'), 'config', 'controllers.yaml'])
    twist_mux_config = PathJoinSubstitution(
        [FindPackageShare('colebot_bringup'), 'config', 'twist_mux.yaml'])

    def spawner(name, *extra):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager', *extra],
            output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock_hardware', default_value='false',
            description='Use mock_components/GenericSystem instead of the firmware topics'),
        DeclareLaunchArgument(
            'control_rate', default_value='50',
            description='Controller manager update rate and odometry publish rate (integer Hz)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                controllers,
                {'update_rate': ParameterValue(control_rate, value_type=int)},
            ],
            output='screen'),

        spawner('joint_state_broadcaster'),
        spawner('diff_drive_controller',
                # One token with '=', so the spawner doesn't read '-p' as its own option.
                ['--controller-ros-args=-p publish_rate:=', control_rate, '.0']),
        spawner('wheel_velocity_controller', '--inactive'),

        Node(
            package='twist_mux',
            executable='twist_mux',
            parameters=[twist_mux_config],
            remappings=[('cmd_vel_out', '/diff_drive_controller/cmd_vel')],
            output='screen'),
    ])
