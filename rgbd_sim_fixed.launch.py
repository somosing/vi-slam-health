import os
from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    prefix = Path(get_package_prefix("depth_image_proc"))
    fix = Path("/tmp/vi_slam_rgbd_component_fix")
    resource = (
        fix / "share/ament_index/resource_index/rclcpp_components/depth_image_proc"
    )
    resource.parent.mkdir(parents=True, exist_ok=True)
    resource.write_text(
        "depth_image_proc::PointCloudXyzrgbNode;"
        f"{prefix}/lib/libdepth_image_proc.so\n"
    )
    os.environ["AMENT_PREFIX_PATH"] = (
        str(fix) + os.pathsep + os.environ.get("AMENT_PREFIX_PATH", "")
    )

    simulation = (
        Path(get_package_share_directory("vif_bringup"))
        / "launch/mujoco_sim.launch.py"
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(simulation))
        ),
        ComposableNodeContainer(
            name="rgbd_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::PointCloudXyzrgbNode",
                    name="xyzrgb_converter",
                    parameters=[{"use_sim_time": True}],
                    remappings=[
                        ("rgb/image_rect_color", "/vif/camera/color/image_raw"),
                        ("rgb/camera_info", "/vif/camera/color/camera_info"),
                        ("depth_registered/image_rect", "/vif/camera/depth/image_raw"),
                        ("points", "/vif/camera/points_rgb"),
                    ],
                )
            ],
        ),
        # Activate lazy subscriptions and display the cloud rate.
        ExecuteProcess(
            cmd=["ros2", "topic", "hz", "/vif/camera/points_rgb"],
            output="screen",
        ),
    ])
