"""RoboMarvel Nav2 bringup.

Mapping is intentionally not started here. The primary Cartographer launch owns
/map and map -> odom, while this file starts only the Nav2 navigation servers.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("main")
    launch_dir = os.path.join(package_share, "launch")

    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    actions = [
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_namespace", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(package_share, "config", "params.yaml"),
        ),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_composition", default_value="true"),
        DeclareLaunchArgument("use_respawn", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        GroupAction(
            [
                PushROSNamespace(
                    condition=IfCondition(use_namespace), namespace=namespace
                ),
                Node(
                    condition=IfCondition(use_composition),
                    package="rclcpp_components",
                    executable="component_container_isolated",
                    name="nav2_container",
                    output="screen",
                    parameters=[configured_params, {"autostart": autostart}],
                    arguments=["--ros-args", "--log-level", log_level],
                    remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, "navigation_launch.py")
                    ),
                    launch_arguments={
                        "namespace": namespace,
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "params_file": params_file,
                        "use_composition": use_composition,
                        "use_respawn": use_respawn,
                        "container_name": "nav2_container",
                        "log_level": log_level,
                    }.items(),
                ),
            ]
        ),
    ]

    return LaunchDescription(actions)
