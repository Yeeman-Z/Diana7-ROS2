#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    ip_arg = DeclareLaunchArgument(
        'ip',
        default_value='192.168.10.75',
        description='Robot IP address'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='10000',
        description='Robot port'
    )
    
    timeout_arg = DeclareLaunchArgument(
        'timeout_ms',
        default_value='100',
        description='Connection timeout in milliseconds'
    )

    # Robot description using xacro
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare('diana7_description'), 'urdf', 'diana7_robot.urdf.xacro']),
        ' use_mock:=false',
        ' ip:=', LaunchConfiguration('ip'),
        ' port:=', LaunchConfiguration('port'),
        ' timeout_ms:=', LaunchConfiguration('timeout_ms')
    ])

    robot_description = {'robot_description': robot_description_content}

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Controller manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, PathJoinSubstitution([FindPackageShare('diana7_bringup'), 'config', 'controllers.yaml'])]
    )

    # Spawn joint state broadcaster (with delay)
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # Spawn arm controller (with delay)
    arm_controller_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        ip_arg,
        port_arg,
        timeout_arg,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner
    ])