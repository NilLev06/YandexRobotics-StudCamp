"""Select the mapping backend available in the rover's ROS image.

Newer RoboMarvel images contain Cartographer.  The older v6 images deployed on
part of the fleet contain slam_toolbox instead.  Selecting here keeps one fleet
release usable on both image generations without installing packages at boot.
"""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _package_share(package: str) -> Path | None:
    try:
        return Path(get_package_share_directory(package))
    except PackageNotFoundError:
        return None


def generate_launch_description() -> LaunchDescription:
    main_share = Path(get_package_share_directory("main"))

    if _package_share("cartographer_ros") is not None:
        backend = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(main_share / "launch" / "cartographer.launch.py")
            ),
            launch_arguments={
                "use_sim_time": "false",
                "scan_topic": "/scan",
                "odom_topic": "/icp/odom",
            }.items(),
        )
        return LaunchDescription([LogInfo(msg="Mapping backend: cartographer_ros"), backend])

    slam_share = _package_share("slam_toolbox")
    if slam_share is not None:
        backend = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(slam_share / "launch" / "online_async_launch.py")
            ),
            launch_arguments={
                "use_sim_time": "false",
                "slam_params_file": str(main_share / "config" / "params.yaml"),
            }.items(),
        )
        return LaunchDescription([LogInfo(msg="Mapping backend: slam_toolbox"), backend])

    raise RuntimeError(
        "No supported mapping backend found: install cartographer_ros or slam_toolbox"
    )
