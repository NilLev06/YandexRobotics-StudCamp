"""Primary 2D mapping launch for the RoboMarvel rover."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("main"))
    configuration_directory = package_share / "config"

    use_sim_time = LaunchConfiguration("use_sim_time")
    configuration_basename = LaunchConfiguration("configuration_basename")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    resolution = LaunchConfiguration("resolution")
    publish_period_sec = LaunchConfiguration("publish_period_sec")

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory",
            str(configuration_directory),
            "-configuration_basename",
            configuration_basename,
        ],
        remappings=[
            ("scan", scan_topic),
            ("odom", odom_topic),
        ],
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-resolution",
            resolution,
            "-publish_period_sec",
            publish_period_sec,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock instead of the rover clock.",
            ),
            DeclareLaunchArgument(
                "configuration_basename",
                default_value="cartographer_rover_2d.lua",
                description="Cartographer Lua configuration file.",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="LD19 LaserScan topic.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/icp/odom",
                description="External odometry consumed by Cartographer.",
            ),
            DeclareLaunchArgument(
                "resolution",
                default_value="0.025",
                description="Published OccupancyGrid resolution in metres.",
            ),
            DeclareLaunchArgument(
                "publish_period_sec",
                default_value="1.0",
                description="OccupancyGrid publication interval.",
            ),
            cartographer_node,
            occupancy_grid_node,
        ]
    )
