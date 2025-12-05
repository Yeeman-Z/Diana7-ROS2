from re import S
from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path

def generate_demo_launch_safe(moveit_config, launch_package_path: Path = None, start_controllers: bool = True):
    """
    Safe version of generate_demo_launch that allows skipping controller startup.
    
    Includes:
        * static_virtual_joint_tfs
        * robot_state_publisher
        * move_group
        * moveit_rviz
        * warehouse_db (optional)
        * ros2_control_node + controller spawners (optional)
    """
    if launch_package_path is None:
        launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(
        DeclareLaunchArgument(
            "db",
            default_value="false",
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))

    # Virtual joints static TF
    virtual_joints_launch = launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Robot state publisher
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/rsp.launch.py")),
        )
    )

    # Move group
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/move_group.launch.py")),
        )
    )

    # Rviz
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/moveit_rviz.launch.py")),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # Warehouse DB (optional)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/warehouse_db.launch.py")),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # Optional: ros2_control_node + spawn controllers
    if start_controllers:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    moveit_config.robot_description,
                    str(moveit_config.package_path / "config/ros2_controllers.yaml"),
                ],
            )
        )
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_package_path / "launch/spawn_controllers.launch.py")),
            )
        )

    return ld


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("diana7", package_name="diana7_moveit_config").to_moveit_configs()
    return generate_demo_launch_safe(moveit_config, start_controllers=False)
