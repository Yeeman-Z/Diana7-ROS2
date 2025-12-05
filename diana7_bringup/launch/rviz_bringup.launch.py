from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    desc_share = get_package_share_directory('diana7_description')
    # Prefer xacro entry if present, else use generated urdf
    urdf_xacro = os.path.join(desc_share, 'urdf', 'diana7_robot.urdf.xacro')
    urdf_xml = os.path.join(desc_share, 'urdf', 'diana_robot.urdf')
    use_xacro = os.path.exists(urdf_xacro)

    robot_description = {'robot_description': Command(['xacro', ' ', urdf_xacro])} if use_xacro else {'robot_description': open(urdf_xml).read()}

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])


