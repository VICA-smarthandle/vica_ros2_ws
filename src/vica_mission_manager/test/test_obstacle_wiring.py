"""장애물 안내 노드 배선 계약 (2026-10-09 2단계, 설계서 5절) — rclpy 없이 소스 글자로 본다(test_handle_mode 방식)."""
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
NODE = (PKG / "vica_mission_manager" / "mission_manager_node.py").read_text(encoding="utf-8")
LAUNCH = (PKG / "launch" / "mission_manager.launch.py").read_text(encoding="utf-8")
XML = (PKG / "package.xml").read_text(encoding="utf-8")


def _setup_block() -> str:
    return NODE[NODE.index("    def _setup_obstacle_narration"):NODE.index("    def _on_obstacle_error")]


def test_switch_is_declared_and_exposed():
    assert 'self.declare_parameter("obstacle_narration", True)' in NODE
    assert 'DeclareLaunchArgument("obstacle_narration", default_value="true")' in LAUNCH
    assert '"obstacle_narration": ParameterValue(' in LAUNCH


def test_inputs_use_their_own_group_not_the_dialog_group():
    block = _setup_block()
    assert "group = MutuallyExclusiveCallbackGroup()" in block
    assert "_main_group" not in block and "_emergency_group" not in block
    assert block.count("callback_group=group") == 8          # 구독 7 + 판정 타이머 1


def test_sensor_inputs_keep_only_the_latest():
    block = _setup_block()
    assert "QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST," in block
    assert '"/scan", wrap(self._obs_scan), latest' in block
    assert '"/camera/depth_scan", wrap(self._obs_depth), latest' in block


def test_tf_listener_lives_in_this_node_without_a_new_node():
    block = _setup_block()
    assert "tf2_ros.TransformListener(self._tf_buffer, self)" in block
    assert "spin_thread" not in block and "create_node" not in block


def test_every_obstacle_callback_is_guarded():
    assert _setup_block().count("wrap(self._obs_") == 8


def test_speaking_is_decided_in_the_dialog_tick():
    tick = NODE[NODE.index("    def _tick(self)"):NODE.index("    def _publish_robot_state")]
    assert "self._drain_obstacle_cues()" in tick
    assert "self.logic.obstacle_cue(phrase, onset, self._now())" in NODE


def test_missing_map_turns_only_the_narration_off():
    block = _setup_block()
    assert "grid, why = load_grid(map_yaml)" in block
    assert "return" in block.split("if grid is None:")[1].split("self._obstacle = ObstacleJudge")[0]


def test_package_declares_the_new_dependencies():
    for dep in ("<depend>sensor_msgs</depend>", "<depend>nav_msgs</depend>", "<depend>rcl_interfaces</depend>",
                "<depend>tf2_ros</depend>", "<exec_depend>python3-numpy</exec_depend>",
                "<exec_depend>python3-scipy</exec_depend>"):
        assert dep in XML, dep


def test_cue_queue_is_never_popped_unguarded():
    """최종 검토 I-1: 판정 줄은 큐를 비우지 않고, 대화 줄은 경쟁에 안전한 take_all 로 꺼낸다."""
    assert "self._obstacle_cues.clear()" not in NODE
    drain = NODE[NODE.index("    def _drain_obstacle_cues"):NODE.index("    def _on_approach_request")]
    assert "take_all(self._obstacle_cues)" in drain
    assert ".popleft()" not in drain


def test_setup_failure_turns_only_the_narration_off():
    """최종 검토 I-2: 지도를 읽은 뒤의 설치(tf·구독·타이머)가 예외를 내도 미션은 뜨고 안내만 꺼진다."""
    block = _setup_block()
    after_grid = block.split("self._obstacle = ObstacleJudge(grid)")[0].split("if grid is None:")[1]
    assert "return" in after_grid
    guarded = block[block.index("        try:\n"):]
    assert "self._obstacle = ObstacleJudge(grid)" in guarded
    assert "except Exception as exc:" in guarded
    assert "self._obstacle_guard.enabled = False" in guarded
    assert "self._on_obstacle_error(exc)" in guarded
