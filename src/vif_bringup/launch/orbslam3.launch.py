from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("vif_bringup"))
    config = share / "config"
    vocabulary_path = LaunchConfiguration("vocabulary_path")
    settings_path = LaunchConfiguration("settings_path")
    use_viewer = LaunchConfiguration("use_viewer")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vocabulary_path",
                default_value="",
                description="Absolute path to ORBvoc.txt",
            ),
            DeclareLaunchArgument(
                "settings_path",
                default_value="",
                description="Absolute path to the ORB-SLAM3 stereo-inertial settings",
            ),
            DeclareLaunchArgument(
                "use_viewer",
                default_value="false",
                description="Start the Pangolin ORB-SLAM3 viewer",
            ),
            Node(
                package="vif_sensor_frontend",
                executable="sensor_guard_node",
                name="sensor_guard",
                parameters=[str(config / "sensor_guard.yaml")],
                output="screen",
            ),
            Node(
                package="vif_orbslam3",
                executable="orbslam3_node",
                name="orbslam3",
                parameters=[
                    str(config / "orbslam3.yaml"),
                    {
                        "vocabulary_path": vocabulary_path,
                        "settings_path": settings_path,
                        "use_viewer": ParameterValue(use_viewer, value_type=bool),
                    },
                ],
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
