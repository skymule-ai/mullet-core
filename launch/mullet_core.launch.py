import os
import re

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

MIN_VEHICLE_INSTANCE = 0
MAX_VEHICLE_INSTANCE = 254


def _validate_instance(instance: int) -> None:
    if not isinstance(instance, int) or isinstance(instance, bool):
        raise ValueError("vehicle_instance must be an integer")
    if instance < MIN_VEHICLE_INSTANCE or instance > MAX_VEHICLE_INSTANCE:
        raise ValueError(
            "vehicle_instance must be in "
            f"[{MIN_VEHICLE_INSTANCE}, {MAX_VEHICLE_INSTANCE}] "
            "so target_system maps to PX4 system IDs [1, 255]"
        )


def _load_launch_config(config_path: str) -> tuple[int, str]:
    try:
        with open(config_path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f"Failed to load YAML config '{config_path}': {exc}") from exc

    if not isinstance(data, dict):
        raise RuntimeError(f"YAML config '{config_path}' must contain a mapping")

    for node_config in data.values():
        if not isinstance(node_config, dict):
            continue
        params = node_config.get("ros__parameters")
        if not isinstance(params, dict) or "vehicle_instance" not in params:
            continue

        instance = params["vehicle_instance"]
        _validate_instance(instance)
        namespace_prefix = params.get("px4_namespace_prefix", "")
        return instance, "" if namespace_prefix is None else str(namespace_prefix)

    raise RuntimeError(f"YAML config '{config_path}' must define vehicle_instance")


def _identity_from_env(instance: int, namespace_prefix: str, env=os.environ) -> tuple[int, str, str]:
    """DRONE_ID from /etc/mullet/drone.env (the container's env_file), the drone's one identity key.

    uav_N gives instance N, namespace prefix uav and drone_id uav_N, overriding the YAML, so the node's
    namespace, PX4 topics (/uav_N/fmu/...) and target system (N + 1) can't disagree with it. Unset
    (SITL, tests): the YAML applies, drone_id "" (the node then uses <prefix>_<instance>).
    """
    drone_id = env.get("DRONE_ID", "").strip()
    if not drone_id:
        return instance, namespace_prefix, ""
    match = re.fullmatch(r"([A-Za-z][A-Za-z0-9]*)_([0-9]+)", drone_id)
    if not match or int(match.group(2)) < 1:
        raise ValueError(f"DRONE_ID={drone_id!r} (drone.env) must be <prefix>_N with N >= 1, e.g. uav_1")
    override = int(match.group(2))
    _validate_instance(override)
    return override, match.group(1), drone_id


def _instance_to_namespace(instance: int, namespace_prefix: str) -> str:
    return "" if instance == 0 or not namespace_prefix else f"/{namespace_prefix}_{instance}"


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("mullet_core")
    config_path = os.path.join(package_share, "config", "mullet_core_params.yaml")
    instance, namespace_prefix = _load_launch_config(config_path)
    instance, namespace_prefix, drone_id = _identity_from_env(instance, namespace_prefix)
    overrides = {"vehicle_instance": instance, "px4_namespace_prefix": namespace_prefix}
    if drone_id:
        overrides["drone_id"] = drone_id

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            Node(
                package="mullet_core",
                executable="mullet_core_node",
                name=f"mullet_core_{instance}",
                namespace=_instance_to_namespace(instance, namespace_prefix),
                output="screen",
                parameters=[config_path, overrides],
            ),
        ]
    )
