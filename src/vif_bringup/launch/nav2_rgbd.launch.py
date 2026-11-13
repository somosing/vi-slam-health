from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("vif_bringup"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = str(share / "params" / "nav2_rgbd.yaml")

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_share / "launch" / "navigation_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                description="Map-server YAML created from a dense RGB-D point cloud",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[
                    {"yaml_filename": map_yaml, "use_sim_time": use_sim_time}
                ],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["map_server"],
                    }
                ],
                output="screen",
            ),
            Node(
                package="vif_bringup",
                executable="slam_map_odom",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="vif_bringup",
                executable="slam_tracking_cmd_gate",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            GroupAction(
                scoped=True,
                actions=[
                    SetRemap(src="cmd_vel", dst="cmd_vel_nav"),
                    nav2_navigation,
                ],
            ),
        ]
    )
