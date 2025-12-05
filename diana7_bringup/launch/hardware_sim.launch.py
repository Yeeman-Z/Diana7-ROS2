from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    desc_share = get_package_share_directory('diana7_description')
    bringup_share = get_package_share_directory('diana7_bringup')

    # Prefer xacro entry if present
    urdf_xacro = os.path.join(desc_share, 'urdf', 'diana7_robot.urdf.xacro')
    urdf_xml = os.path.join(desc_share, 'urdf', 'diana_robot.urdf')
    use_xacro = os.path.exists(urdf_xacro)

    use_mock = LaunchConfiguration('use_mock')
    robot_description_param = {'robot_description': Command(['xacro', ' ', urdf_xacro, ' use_mock:=', use_mock])} if use_xacro else {'robot_description': open(urdf_xml).read()}

    controllers_yaml = os.path.join(bringup_share, 'config', 'controllers.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_mock', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description_param],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description_param, controllers_yaml],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
    ])


