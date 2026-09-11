"""nvblox costmap plugin 가용성 contract."""

from pathlib import Path

import pytest
import yaml

from vica_nav2 import dependency_checks as dc


def _local_costmap_plugins():
    config_path = Path(__file__).parents[1] / "config" / "nav2_params.yaml"
    params = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    return params["local_costmap"]["local_costmap"]["ros__parameters"]["plugins"]


def test_nav2_params_declares_nvblox_layer_with_official_plugin():
    """nvblox_layer가 plugins에 있고 공식 플러그인 클래스를 가리키는지."""
    config_path = Path(__file__).parents[1] / "config" / "nav2_params.yaml"
    params = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]

    assert "nvblox_layer" in local["plugins"]
    assert local["nvblox_layer"]["plugin"] == "nvblox::nav2::NvbloxCostmapLayer"
    assert local["nvblox_layer"]["nav2_costmap_global_frame"] == local["global_frame"]


def test_nvblox_costmap_plugin_is_loadable_when_declared():
    """nvblox_layer를 선언했다면 Host plugin 리소스가 온전해야 한다."""
    if "nvblox_layer" not in _local_costmap_plugins():
        pytest.skip("nvblox_layer 비활성 구성 — plugin 로드 불필요")
    if not dc.nvblox_packages_present():
        pytest.skip("Isaac ROS nvblox 미설치 환경")

    dc.verify_nvblox_costmap_plugin()
