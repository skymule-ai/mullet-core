import importlib.util
from pathlib import Path

import pytest
import yaml


def _load_launch_module():
    launch_path = Path(__file__).parents[1] / "launch" / "mullet_core.launch.py"
    spec = importlib.util.spec_from_file_location("mullet_core_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


LAUNCH = _load_launch_module()
PACKAGE_DIR = Path(__file__).parents[1]


def _write_config(tmp_path, parameters):
    config_path = tmp_path / "params.yaml"
    config_path.write_text(
        yaml.safe_dump({"/**": {"ros__parameters": parameters}}), encoding="utf-8"
    )
    return config_path


def test_load_launch_config_uses_yaml_values(tmp_path):
    config_path = _write_config(
        tmp_path, {"vehicle_instance": 2, "px4_namespace_prefix": "uav"}
    )

    assert LAUNCH._load_launch_config(str(config_path)) == (2, "uav")
    assert LAUNCH._instance_to_namespace(2, "uav") == "/uav_2"

def test_load_launch_config_accepts_empty_namespace_prefix(tmp_path):
    config_path = _write_config(tmp_path, {"vehicle_instance": 0})

    assert LAUNCH._load_launch_config(str(config_path)) == (0, "")
    assert LAUNCH._instance_to_namespace(0, "uav") == ""


def test_load_launch_config_rejects_invalid_instance(tmp_path):
    config_path = _write_config(tmp_path, {"vehicle_instance": -1})

    with pytest.raises(ValueError, match=r"vehicle_instance must be in \[0, 254\]"):
        LAUNCH._load_launch_config(str(config_path))


def test_load_launch_config_rejects_non_integer_instance(tmp_path):
    config_path = _write_config(tmp_path, {"vehicle_instance": "1"})

    with pytest.raises(ValueError, match="vehicle_instance must be an integer"):
        LAUNCH._load_launch_config(str(config_path))

def test_load_launch_config_requires_vehicle_instance(tmp_path):
    config_path = _write_config(tmp_path, {"px4_namespace_prefix": "uav"})

    with pytest.raises(RuntimeError, match="must define vehicle_instance"):
        LAUNCH._load_launch_config(str(config_path))


def test_default_config_omits_deprecated_compatibility_parameters():
    config_text = (PACKAGE_DIR / "config" / "mullet_core_params.yaml").read_text()
    config_keys = {
        line.split(":", 1)[0].strip()
        for line in config_text.splitlines()
        if ":" in line and not line.lstrip().startswith("#")
    }

    assert "takeoff_altitude_m" not in config_keys
    assert "hover_acceptance_radius_m" not in config_keys
