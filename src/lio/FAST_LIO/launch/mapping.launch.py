"""FAST-LIO mapping launch with an explicit, writable PCD staging path."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _prepare_pcd_path(context):
    raw_path = LaunchConfiguration("map_file_path").perform(context)
    path = Path(raw_path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    return []


def generate_launch_description():
    package_path = get_package_share_directory("fast_lio")
    default_config_path = str(Path(package_path) / "config")
    default_rviz_config_path = str(
        Path(package_path) / "rviz" / "fastlio.rviz"
    )
    default_pcd_path = str(
        Path.home()
        / "gb_maps"
        / ".pcd_staging"
        / "fast_lio_map.pcd"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_path = LaunchConfiguration("config_path")
    config_file = LaunchConfiguration("config_file")
    rviz_use = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    map_file_path = LaunchConfiguration("map_file_path")
    pcd_save_en = LaunchConfiguration("pcd_save_en")

    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="laser_mapping",
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {
                "use_sim_time": use_sim_time,
                # Override the relative ./test.pcd from mid360.yaml.
                "map_file_path": map_file_path,
                "pcd_save.pcd_save_en": ParameterValue(
                    pcd_save_en, value_type=bool
                ),
            },
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(rviz_use),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock",
            ),
            DeclareLaunchArgument(
                "config_path",
                default_value=default_config_path,
                description="FAST-LIO YAML directory",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value="mid360.yaml",
                description="FAST-LIO YAML file",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz",
            ),
            DeclareLaunchArgument(
                "rviz_cfg",
                default_value=default_rviz_config_path,
                description="RViz configuration",
            ),
            DeclareLaunchArgument(
                "map_file_path",
                default_value=default_pcd_path,
                description=(
                    "Temporary FAST-LIO PCD output. "
                    "Web moves it to <map_root>/<map_name>.pcd."
                ),
            ),
            DeclareLaunchArgument(
                "pcd_save_en",
                default_value="true",
                description="Enable FAST-LIO /map_save service output",
            ),
            OpaqueFunction(function=_prepare_pcd_path),
            fast_lio_node,
            rviz_node,
        ]
    )
