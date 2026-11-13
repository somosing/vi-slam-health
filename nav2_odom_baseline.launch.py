from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parent
    params = str(root / "nav2_odom_baseline.yaml")
    map_yaml = LaunchConfiguration("map")

    servers = [
        ("nav2_map_server", "map_server"),
        ("nav2_controller", "controller_server"),
        ("nav2_planner", "planner_server"),
        ("nav2_behaviors", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator"),
        ("nav2_velocity_smoother", "velocity_smoother"),
    ]

    actions = [
        DeclareLaunchArgument(
            "map",
            default_value=str(
                root / "maps/rgbd_verified_scan_20261113_144153/nav_map.yaml"
            ),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="baseline_map_to_odom",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "map", "--child-frame-id", "odom",
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
    ]

    for package, executable in servers:
        node_params = [params, {"use_sim_time": True}]
        if executable == "map_server":
            node_params.append({"yaml_filename": map_yaml})

        actions.append(Node(
            package=package,
            executable=executable,
            name=executable,
            parameters=node_params,
            remappings=[
                ("odom", "/diff_drive_controller/odom"),
                ("cmd_vel", "/nav2_baseline/cmd_vel_raw"),
                ("cmd_vel_smoothed", "/nav2_baseline/cmd_vel_smoothed"),
            ],
            output="screen",
        ))

    actions.append(Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_baseline",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": [name for _, name in servers],
        }],
        output="screen",
    ))

    return LaunchDescription(actions)
