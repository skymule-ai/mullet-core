import importlib.util
from pathlib import Path

import pytest


def _load_launch_module():
    launch_path = Path(__file__).parents[1] / "launch" / "mullet_core.launch.py"
    spec = importlib.util.spec_from_file_location("mullet_core_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


LAUNCH = _load_launch_module()
PACKAGE_DIR = Path(__file__).parents[1]


def test_explicit_instances_accept_whitespace_and_sort():
    assert LAUNCH._parse_explicit_instances("2, 0, 1") == [0, 1, 2]


def test_explicit_instances_reject_duplicates():
    with pytest.raises(ValueError, match="duplicate instance 1"):
        LAUNCH._parse_explicit_instances("0,1,1")


def test_explicit_instances_reject_negative_values():
    with pytest.raises(ValueError, match=r"vehicle_instance must be in \[0, 254\]"):
        LAUNCH._parse_explicit_instances("-1")


def test_explicit_instances_reject_unsafe_upper_bound():
    with pytest.raises(ValueError, match=r"vehicle_instance must be in \[0, 254\]"):
        LAUNCH._parse_explicit_instances("255")


def test_explicit_instances_reject_malformed_tokens():
    with pytest.raises(ValueError, match="must be an integer"):
        LAUNCH._parse_explicit_instances("0,abc")


def test_explicit_instances_reject_empty_tokens():
    with pytest.raises(ValueError, match="empty tokens"):
        LAUNCH._parse_explicit_instances("0,,1")


def test_offset_instances_accept_safe_boundary():
    assert LAUNCH._parse_offset_instances("2", "253") == [253, 254]


def test_offset_instances_reject_invalid_ranges():
    with pytest.raises(ValueError, match="num_robots must be at least 1"):
        LAUNCH._parse_offset_instances("0", "0")
    with pytest.raises(ValueError, match="px4_instance_offset must be at least 0"):
        LAUNCH._parse_offset_instances("1", "-1")
    with pytest.raises(ValueError, match="must be <= 254"):
        LAUNCH._parse_offset_instances("2", "254")


def test_configs_include_disable_offboard_timeout():
    for config_name in ("mullet_core_params_default.yaml", "mullet_core_params_1.yaml"):
        config_text = (PACKAGE_DIR / "config" / config_name).read_text()

        assert "disable_offboard_timeout_s: 5.0" in config_text


def test_default_config_omits_deprecated_compatibility_parameters():
    config_text = (PACKAGE_DIR / "config" / "mullet_core_params_default.yaml").read_text()
    config_keys = {
        line.split(":", 1)[0].strip()
        for line in config_text.splitlines()
        if ":" in line and not line.lstrip().startswith("#")
    }

    assert "takeoff_altitude_m" not in config_keys
    assert "hover_acceptance_radius_m" not in config_keys
