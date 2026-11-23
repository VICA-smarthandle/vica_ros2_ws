"""세 파라미터 조합 경고 · launch 주석(전체 검토 I3 · I4 · I5)."""
import itertools
from pathlib import Path

import pytest

from vica_mission_manager.mission_logic import switch_bundle_warning

PKG = Path(__file__).resolve().parents[1]
NAMES = ("answer_field", "reask_ladder", "wake_revive")


@pytest.mark.parametrize("af, rl, wr", list(itertools.product([False, True], repeat=3)))
def test_i4_partial_bundle_is_warned(af, rl, wr):
    msg = switch_bundle_warning(answer_field=af, reask_ladder=rl, wake_revive=wr)
    if af == rl == wr:
        assert msg == ""
        return
    assert "VICA_RECUT" in msg and "VICA_ANSWER" in msg and "한 묶음" in msg
    values = dict(zip(NAMES, (af, rl, wr)))
    on = [n for n in NAMES if values[n]]
    off = [n for n in NAMES if not values[n]]
    assert f"켜짐 {', '.join(on)}" in msg and f"꺼짐 {', '.join(off)}" in msg
    assert "\n" not in msg


def test_i4_node_logs_the_warning_once_at_startup():
    node = (PKG / "vica_mission_manager" / "mission_manager_node.py").read_text(encoding="utf-8")
    start = node.index("    def __init__(")
    init = node[start: node.find("\n    def ", start + 1)]
    call = init.index("switch_bundle_warning(")
    assert init.index("self.logic = MissionLogic(") < call
    tail = init[call: call + 400]
    for name in NAMES:
        assert f"self.logic.{name}" in tail
    assert "self.get_logger().warn(" in tail


def test_i5_launch_comments_tell_the_bundle_and_defaults_stay_off():
    launch = (PKG / "launch" / "mission_manager.launch.py").read_text(encoding="utf-8")
    for name in NAMES:
        assert f'DeclareLaunchArgument("{name}", default_value="false")' in launch
    block = launch[launch.index("다섯 개"): launch.index('DeclareLaunchArgument("answer_field"')]
    assert "VICA_RECUT" in block and "VICA_ANSWER" in block and "한 묶음" in block
    assert "colcon build --packages-select vica_mission_manager" in block
    rl = launch[launch.index("# reask_ladder:"): launch.index('DeclareLaunchArgument("reask_ladder"')]
    assert "VICA_RECUT=on" in rl
    af = launch[launch.index("# answer_field:"): launch.index('DeclareLaunchArgument("answer_field"')]
    assert "다시 출발할까요?" in af and "VICA_ANSWER=on" in af
