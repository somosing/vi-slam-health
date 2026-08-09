from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("vif_bringup"))
    config = share / "config"
    inject_fault = LaunchConfiguration("inject_fault")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "inject_fault",
                default_value="true",
                description="Enable the scheduled fault window",
            ),
            Node(
                package="vif_fault_injection",
                executable="synthetic_sensors",
                name="synthetic_sensors",
                output="screen",
            ),
            Node(
                package="vif_fault_injection",
                executable="fault_injector",
                name="fault_injector",
                parameters=[
                    str(config / "fault_camera_blackout.yaml"),
                    {"enabled": ParameterValue(inject_fault, value_type=bool)},
                ],
                output="screen",
            ),
            Node(
                package="vif_sensor_frontend",
                executable="sensor_guard_node",
                name="sensor_guard",
                parameters=[str(config / "sensor_guard.yaml")],
                output="screen",
            ),
            Node(
                package="vif_mock_slam",
                executable="mock_slam_node",
                name="mock_slam",
                parameters=[str(config / "mock_slam.yaml")],
                output="screen",
            ),
            Node(
                package="vif_health_monitor",
                executable="health_monitor_node",
                name="health_monitor",
                parameters=[str(config / "health_monitor.yaml")],
                output="screen",
            ),
            Node(
                package="vif_relocalization",
                executable="relocalization_manager_node",
                name="relocalization_manager",
                parameters=[str(config / "relocalization.yaml")],
                output="screen",
            ),
        ]
    )
