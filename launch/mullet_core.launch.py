import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MIN_VEHICLE_INSTANCE = 0
MAX_VEHICLE_INSTANCE = 254


def _parse_int(value: str, field_name: str) -> int:
    try:
        return int(value)
    except ValueError as exc:
        raise ValueError(f"{field_name} must be an integer, got '{value}'") from exc


def _validate_instance(instance: int) -> None:
    if instance < MIN_VEHICLE_INSTANCE or instance > MAX_VEHICLE_INSTANCE:
        raise ValueError(
            "vehicle_instance must be in "
            f"[{MIN_VEHICLE_INSTANCE}, {MAX_VEHICLE_INSTANCE}] "
            "so target_system maps to PX4 system IDs [1, 255]"
        )


def _parse_explicit_instances(raw_instances: str) -> list[int] | None:
    if not raw_instances.strip():
        return None

    instances = []
    seen = set()
    for token in raw_instances.split(","):
        value = token.strip()
        if not value:
            raise ValueError("px4_instances must not contain empty tokens")
        instance = _parse_int(value, "px4_instances")
        _validate_instance(instance)
        if instance in seen:
            raise ValueError(f"px4_instances contains duplicate instance {instance}")
        seen.add(instance)
        instances.append(instance)

    return sorted(instances)


def _parse_offset_instances(num_robots_raw: str, offset_raw: str) -> list[int]:
    num_robots = _parse_int(num_robots_raw.strip(), "num_robots")
    offset = _parse_int(offset_raw.strip(), "px4_instance_offset")

    if num_robots < 1:
        raise ValueError("num_robots must be at least 1")
    if offset < MIN_VEHICLE_INSTANCE:
        raise ValueError(f"px4_instance_offset must be at least {MIN_VEHICLE_INSTANCE}")

    last_instance = offset + num_robots - 1
    if last_instance > MAX_VEHICLE_INSTANCE:
        raise ValueError(
            "px4_instance_offset + num_robots - 1 must be <= "
            f"{MAX_VEHICLE_INSTANCE}"
        )

    return list(range(offset, offset + num_robots))


def _parse_instances(context) -> list[int]:
    raw_instances = LaunchConfiguration("px4_instances").perform(context)
    explicit_instances = _parse_explicit_instances(raw_instances)
    if explicit_instances is not None:
        return explicit_instances

    return _parse_offset_instances(
        LaunchConfiguration("num_robots").perform(context),
        LaunchConfiguration("px4_instance_offset").perform(context),
    )


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
                description=(
                    "Comma-separated PX4 instance list in [0, 254]. "
                    "Overrides num_robots/px4_instance_offset."
                ),
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
