from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    config = Path(get_package_share_directory("vif_bringup")) / "config"
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
                description="RGB-D-inertial camera settings; DepthMapFactor must match depth units",
            ),
            DeclareLaunchArgument(
                "use_viewer",
                default_value="false",
                description="Start the Pangolin ORB-SLAM3 viewer",
            ),
            Node(
                package="vif_orbslam3",
                executable="orbslam3_node",
                name="orbslam3",
                parameters=[
                    str(config / "orbslam3_rgbd.yaml"),
                    {
                        "vocabulary_path": vocabulary_path,
                        "settings_path": settings_path,
                        "use_viewer": ParameterValue(use_viewer, value_type=bool),
                    },
                ],
                output="screen",
            ),
        ]
    )
