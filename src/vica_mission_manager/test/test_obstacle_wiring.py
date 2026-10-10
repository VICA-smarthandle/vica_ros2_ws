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


def test_inputs_live_on_a_helper_node_with_a_single_threaded_executor():
    # 2026-10-10 CPU ③: 장애물 입력은 미션 본체의 다중 실행기(메시지당 부담이 크다) 대신 보조 노드 하나를
    # 단일 실행기로 자기 스레드에서 돌린다. 대화 줄(_main_group)·긴급 줄과는 처음부터 갈라져 있다.
    block = _setup_block()
    assert 'rclpy.create_node("vica_mission_obstacle")' in block
    assert "SingleThreadedExecutor()" in block and "self._obs_executor.add_node(self._obs_node)" in block
    assert "threading.Thread(target=self._obs_spin" in block
    assert "_main_group" not in block and "_emergency_group" not in block
    assert "self.create_subscription(" not in block           # 장애물 입력은 본체 노드에 붙지 않는다


def test_sensor_inputs_keep_only_the_latest():
    block = _setup_block()
    assert "QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST," in block
    assert '"/scan", wrap(self._obs_scan), latest' in block
    assert '"/camera/depth_scan", wrap(self._obs_depth), latest' in block


def test_pose_comes_from_amcl_and_odom_not_tf():
    # 2026-10-10 CPU ②-나: tf(초당 52통) 대신 마지막 AMCL 위치 + 바퀴 오도메트리 변화(녹화본 판정 같음).
    assert "TransformListener" not in NODE
    block = _setup_block()
    assert '"/amcl_pose", wrap(self._obs_amcl_msg)' in block
    assert '"/odom", wrap(self._obs_odom_msg)' in block
    assert "self._obs_trail.pose_since(self._obs_amcl)" in NODE
    # 라이다 위치는 /tf_static 만 받아 버퍼에 넣는다(드물다)
    assert '"/tf_static", wrap(self._obs_static)' in block and "set_transform_static" in NODE


def test_every_obstacle_callback_is_guarded():
    block = _setup_block()
    made = block.count("create_subscription(") + block.count("create_timer(")
    assert made == 11                                         # 늘 받는 4 + 판정 타이머 1 + 주행 중 6
    assert block.count("wrap(self._obs_") == made


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


# ---- 입력은 안내 주행 중에만 (2026-10-10 CPU ①: 쉴 때 미션 코어 하나 46 % → 2.4 % 실측) ----

def _part(start: str, end: str) -> str:
    return NODE[NODE.index(start):NODE.index(end)]


def test_tick_switches_inputs_by_guided_driving():
    tick = _part("    def _tick(self)", "    def _publish_robot_state")
    assert "self._obstacle_inputs_follow()" in tick
    follow = _part("    def _obstacle_inputs_follow", "    def _obs_start")
    assert "self.logic.dialog_state == State.NAVIGATING.value" in follow


def test_setup_keeps_only_rail_and_goal_always_on():
    setup = _part("    def _setup_obstacle_narration", "    def _obstacle_inputs_follow")
    for topic in ('"/rail_plan"', '"/vica_goal_event"', '"/amcl_pose"', '"/tf_static"'):   # 드물다
        assert topic in setup, topic
    for topic in ('"/odom"', '"/scan"', '"/camera/depth_scan"', '"/vcc/state"', '"/behavior_tree_log"', '"/rosout"'):
        assert topic not in setup, topic
    assert "TransformListener" not in setup
    assert "self._obs_timer.cancel()" in setup            # 판정 타이머도 주행 전엔 멈춰 둔다


def test_start_clears_old_inputs_before_listening():
    start = _part("    def _obs_start", "    def _obs_stop")
    for topic in ('"/odom"', '"/scan"', '"/camera/depth_scan"', '"/vcc/state"', '"/behavior_tree_log"', '"/rosout"'):
        assert topic in start, topic
    # 타이머를 구독보다 먼저 켠다 — 구독을 만들 때 실행기가 깨어나 켜진 타이머를 대기 목록에 넣는다
    order = [start.index(s) for s in ("self._obstacle.clear_inputs()", "self._obs_trail.clear()",
                                      "self._obs_timer.reset()", '"/odom"')]
    assert order == sorted(order)


def test_stop_releases_timer_and_subscriptions():
    stop = _part("    def _obs_stop", "    def _on_obstacle_error")
    assert "self._obs_timer.cancel()" in stop
    assert "self._obs_node.destroy_subscription(" in stop


def test_switching_error_turns_only_the_narration_off():
    follow = _part("    def _obstacle_inputs_follow", "    def _obs_start")
    assert "except Exception" in follow and "self._on_obstacle_error(exc)" in follow
    assert "self._obstacle_guard.enabled = False" in follow


# ---- 보조 노드 견고성 (2026-10-10 독립 검토) ----

def test_helper_callbacks_cannot_lock_a_group():
    # 구독을 닫는 순간 실행기가 막 집은 메시지를 꺼내면 rclpy(Humble)가 InvalidHandle 을 낸다 — 그룹의
    # '실행 중' 표시를 풀기 전이라, 한 번에 하나 그룹이면 그 그룹이 영영 잠긴다. 단일 실행기라 Reentrant 여도
    # 동시에 돌지 않는다.
    block = _setup_block()
    assert "group = ReentrantCallbackGroup()" in block
    made = block.count("create_subscription(") + block.count("create_timer(")
    assert block.count("callback_group=group") == made


def test_helper_spin_survives_a_closed_subscription():
    spin = _part("    def _obs_spin", "    def _obstacle_inputs_follow")
    assert "self._obs_executor.spin_once()" in spin
    assert "except InvalidHandle:" in spin and "continue" in spin.split("except InvalidHandle:")[1].split("except")[0]


def test_inputs_close_after_narration_turns_off():
    follow = _part("    def _obstacle_inputs_follow", "    def _obs_start")
    off = follow.split("if self._obstacle is None or not self._obstacle_guard.enabled:")[1].split("want =")[0]
    assert "self._obs_stop()" in off
    start = _part("    def _obs_start", "    def _obs_stop")
    # 여는 도중 오류가 나도 닫기가 정리할 수 있게, 열림 표시와 구독 목록을 먼저 둔다
    assert start.index("self._obs_on = True") < start.index("self._obs_timer.reset()") < start.index('"/odom"')
    assert "subs = self._obs_subs" in start and start.count("subs.append(node.create_subscription(") == 6


def test_helper_node_is_closed_with_the_mission():
    destroy = _part("    def destroy_node(self)", "    def _setup_obstacle_narration")
    assert 'getattr(self, "_obs_executor", None)' in destroy and "executor.shutdown(" in destroy
    assert 'getattr(self, "_obs_node", None)' in destroy and "node.destroy_node()" in destroy
    assert "super().destroy_node()" in destroy
