from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import RegisterEventHandler


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("vif_bringup"))
    vocabulary_path = LaunchConfiguration("vocabulary_path")
    settings_path = LaunchConfiguration("settings_path")
    use_viewer = LaunchConfiguration("use_viewer")
    dataset_path = LaunchConfiguration("dataset_path")
    playback_rate = LaunchConfiguration("playback_rate")
    start_offset_sec = LaunchConfiguration("start_offset_sec")
    duration_sec = LaunchConfiguration("duration_sec")

    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_share / "launch" / "orbslam3.launch.py")),
        launch_arguments={
            "vocabulary_path": vocabulary_path,
            "settings_path": settings_path,
            "use_viewer": use_viewer,
        }.items(),
    )
    player = Node(
        package="vif_euroc_player",
        executable="euroc_player",
        name="euroc_player",
        parameters=[
            {
                "dataset_path": dataset_path,
                "playback_rate": ParameterValue(playback_rate, value_type=float),
                "start_offset_sec": ParameterValue(start_offset_sec, value_type=float),
                "duration_sec": ParameterValue(duration_sec, value_type=float),
                "required_subscriber_count": 2,
                "shutdown_on_complete": True,
            }
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dataset_path",
                default_value="",
                description="Absolute path to an extracted EuRoC sequence containing mav0",
            ),
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
            DeclareLaunchArgument("use_viewer", default_value="false"),
            DeclareLaunchArgument("playback_rate", default_value="1.0"),
            DeclareLaunchArgument("start_offset_sec", default_value="0.0"),
            DeclareLaunchArgument(
                "duration_sec",
                default_value="0.0",
                description="Playback duration; zero selects the full sequence",
            ),
            backend,
            player,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=player,
                    on_exit=[EmitEvent(event=Shutdown(reason="EuRoC playback finished"))],
                )
            ),
        ]
    )
