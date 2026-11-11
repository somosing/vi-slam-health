from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("vif_bringup"))
    headless = LaunchConfiguration("headless")
    xacro_file = str(share / "description" / "vif_amr.urdf.xacro")
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " headless:=",
                headless,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True}
        ],
    )
    mujoco_control = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"use_sim_time": True},
            ParameterFile(str(share / "params" / "mujoco_controllers.yaml")),
            ParameterFile(str(share / "params" / "mujoco_plugins.yaml")),
        ],
    )
    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller,
                "--controller-manager",
                "/controller_manager",
                "--param-file",
                str(share / "params" / "mujoco_controllers.yaml"),
            ],
            output="screen",
        )
        for controller in (
            "joint_state_broadcaster",
            "diff_drive_controller",
            "vif",
        )
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run MuJoCo without its viewer window",
            ),
            robot_state_publisher,
            mujoco_control,
            *spawners,
        ]
    )
