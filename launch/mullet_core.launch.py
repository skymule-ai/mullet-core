import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_instances(context) -> list[int]:
    raw_instances = LaunchConfiguration("px4_instances").perform(context).strip()
    if raw_instances:
        return sorted(int(value.strip()) for value in raw_instances.split(",") if value.strip())

    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    offset = int(LaunchConfiguration("px4_instance_offset").perform(context))
    return list(range(offset, offset + num_robots))


def _instance_to_namespace(instance: int) -> str:
    return "" if instance == 0 else f"/px4_{instance}"


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    config_dir = LaunchConfiguration("config_dir").perform(context)
    nodes = []

    for instance in _parse_instances(context):
        instance_config = os.path.join(config_dir, f"mullet_core_params_{instance}.yaml")
        fallback_config = os.path.join(config_dir, "mullet_core_params_default.yaml")
        config_path = instance_config if os.path.exists(instance_config) else fallback_config
        namespace = _instance_to_namespace(instance)

        print(
            f"[mullet_core launch] instance {instance}: loading '{config_path}'",
            flush=True,
        )

        nodes.append(
            Node(
                package="mullet_core",
                executable="mullet_core_node",
                name=f"mullet_core_{instance}",
                namespace=namespace,
                output="screen",
                parameters=[config_path, {"vehicle_instance": instance}],
            )
        )

    return nodes


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("mullet_core")
    default_config_dir = os.path.join(package_share, "config")

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            DeclareLaunchArgument(
                "px4_instances",
                default_value="0",
                description="Comma-separated PX4 instance list. Overrides num_robots/px4_instance_offset.",
            ),
            DeclareLaunchArgument(
                "num_robots",
                default_value="1",
                description="Number of mullet_core nodes to spawn when px4_instances is empty.",
            ),
            DeclareLaunchArgument(
                "px4_instance_offset",
                default_value="0",
                description="First PX4 instance when px4_instances is empty.",
            ),
            DeclareLaunchArgument(
                "config_dir",
                default_value=default_config_dir,
                description="Directory containing mullet_core_params_<instance>.yaml files.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
