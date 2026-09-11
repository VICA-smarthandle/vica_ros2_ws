from pathlib import Path

import yaml


def test_wheel_odom_to_standard_odom_contract():
    config_path = Path(__file__).parents[1] / "config" / "ekf.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    params = config["ekf_filter_node"]["ros__parameters"]

    assert params["odom0"] == "/wheel/odom"
    assert params["odom_frame"] == "odom"
    assert params["base_link_frame"] == "base_footprint"
    assert params["world_frame"] == "odom"
    assert params["publish_tf"] is True


def test_encoder_does_not_publish_duplicate_tf():
    config_path = Path(__file__).parents[1] / "config" / "encoder.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    params = config["encoder_feedback"]["ros__parameters"]

    assert params["odom_topic"] == "/wheel/odom"
    assert params["publish_tf"] is False
    assert params["request_position_feedback"] is False
