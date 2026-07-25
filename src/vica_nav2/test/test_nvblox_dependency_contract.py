"""nvblox costmap plugin 가용성 contract.

nav2_params.yaml의 local costmap이 ``nvblox_layer``(nvblox::nav2::NvbloxCostmapLayer)
를 plugins에 포함하는 한, Host에는 그 plugin이 로드 가능한 상태로 있어야 한다.
소스 symlink 제거로 install의 plugin xml이 dangling(빈 파일)이 되면 pluginlib가
클래스를 못 찾아 local costmap configure가 실패하고 Nav2가 Goal을 거부한다.
이 테스트가 그 회귀를 런타임 이전에 잡는다.

Isaac ROS nvblox가 없는 환경에서는 nvblox_layer를 쓰지 않으므로 skip한다.
"""

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
    # 플러그인이 odom 기준으로 동작해야 slice frame(odom)과 identity로 정합한다.
    assert local["nvblox_layer"]["nav2_costmap_global_frame"] == local["global_frame"]


def test_nvblox_costmap_plugin_is_loadable_when_declared():
    """nvblox_layer를 선언했다면 Host plugin 리소스가 온전해야 한다."""
    if "nvblox_layer" not in _local_costmap_plugins():
        pytest.skip("nvblox_layer 비활성 구성 — plugin 로드 불필요")
    if not dc.nvblox_packages_present():
        pytest.skip("Isaac ROS nvblox 미설치 환경")

    # 존재하면 dangling symlink 등으로 깨져 있지 않아야 한다.
    dc.verify_nvblox_costmap_plugin()
