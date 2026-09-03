#!/usr/bin/env python3
"""vica_mission_manager — VICA 음성→주행 통합의 유일한 신규 노드 (진행순서 ②).

역할: /vica/intent(LLM 의 '제안')를 게이트 5조건으로 심사해, 통과한 경우에만
nav2_simple_commander 로 NavigateToPose 를 보낸다. 판단 로직은 전부
mission_logic.py(순수 함수/상태 머신)에 있고, 이 파일은 ROS 배선만 담당한다.

안전 원칙(불변):
- LLM/음성 파트는 /cmd_vel·Nav2 goal 을 직접 발행하지 않는다 — 여기가 유일한 관문.
- 모터 정지의 권위는 vica_safety의 /emergency_stop 중앙 래치 체인.
  여기서의 goal 취소는 심층 방어 보조 경로.

구독: /vica/intent, /vica/emergency(전용 callback group), /emergency_stop

`/emergency_stop`은 emergency_stop_node가 물리·앱·음성을 중앙 래치한 권위 상태다.
입력 펄스가 끝나도 명시적 reset 전까지 true를 유지한다.
발행: /vica/tts_request(std_msgs/String), /vica/robot_state(1Hz)
"""
from __future__ import annotations

import json
import math
import threading
from datetime import datetime
from pathlib import Path
from uuid import UUID

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.msg import SpeedLimit
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from vica_interfaces.msg import EmergencyEvent, RobotState, VicaIntent
from vica_interfaces.msg import PersonDetection
from vica_interfaces.srv import (
    DeleteHome,
    GetHome,
    MissionCommand,
    RequestApproach,
    RequestDestination,
    SaveHome,
)

from .approach_speed import DEFAULT_APPROACH_STAGES, stages_from_lists
from .destinations import load_destinations, load_home, load_map_bounds
from .approach_geometry import approach_goal
from .home_storage import HomeStorage, build_home
from .mission_logic import (
    MSG_APPROACH_QUESTION,
    ApproachRequest,
    CancelNav,
    Destination,
    GateReason,
    IntentData,
    MissionLogic,
    Navigate,
    NavStatus,
    Say,
    StopSpeech,
    SetNavSpeedLimit,
    SpinInPlace,
    Pose2D,
    State,
    _REJECT_MESSAGES,
    check_gate,
    yaw_deg_to_quaternion,
)

#: 홈에 붙이는 고정 id.
#:
#: 목적지 id 는 UUID v4 인데 홈은 일부러 그 형식을 쓰지 않는다. 그래야 앱이나
#: 음성이 이 id 로 `/vica/mission/request_destination` 을 불러도 UUID 검증에서
#: 먼저 걸린다 — **홈을 목적지처럼 요청하는 길을 형식 수준에서 막는 것**이다.
HOME_DESTINATION_ID = "__home__"

try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
except ImportError:  # nav2 미설치 환경(예: CI)에서도 import 에러로 죽지 않게
    BasicNavigator = None
    TaskResult = None


class MissionManagerNode(Node):
    """LLM intent를 안전 게이트로 심사하고 승인된 Nav2 작업만 실행한다.

    판단은 테스트하기 쉬운 :class:`MissionLogic`에 맡기고, 이 클래스는 ROS2
    메시지 변환·토픽 연결·Nav2 commander 호출 같은 배선만 담당한다.
    """

    def __init__(self) -> None:
        """설정과 목적지를 읽고 intent·긴급정지·Nav2·TTS 토픽을 연결한다."""
        super().__init__("vica_mission_manager")

        # 목적지/지도 파일 경로는 launch 의 ROS parameter(절대경로)로 받는다.
        self.declare_parameter("destinations_yaml", "")
        self.declare_parameter("map_id", "")
        self.declare_parameter("map_yaml", "")
        # 접근을 마쳤을 때 홈까지 스스로 돌아갈 것인가.
        #
        # 기본값 false 는 사람이 부르지 않았는데 로봇이 혼자 달리는 일을 막는다.
        # 홈 복귀는 2026-08-27 현재 실기 [미검증] 이고, 확인 전에 자동 주행부터
        # 켜면 아무도 안 보는 사이에 처음 달려 보게 된다. 관리자가 앱에서 부르는
        # 복귀(/vica/mission/return_home)는 이 값과 무관하게 늘 동작하므로
        # 실기 확인은 그쪽으로 먼저 한다.
        #
        # 확인이 끝나면 true 로 바꿔 '접근을 마친 자리 = 홈'으로 만든다.
        self.declare_parameter("auto_return_home", False)
        self.declare_parameter("confirm_timeout_sec", 30.0)
        # 수락 후 제자리 회전량(도). 0 이면 회전 없이 예전처럼 끝낸다.
        self.declare_parameter("approach_turn_yaw_deg", 180.0)
        self.declare_parameter("estop_release_grace_sec", 1.0)
        # 주행 실패 뒤 같은 목적지로 스스로 다시 시도하는 횟수와 간격.
        # 0 으로 두면 종전처럼 실패를 안내하고 끝낸다.
        self.declare_parameter("nav_retry_limit", 2)
        self.declare_parameter("nav_retry_delay_sec", 3.0)
        # 접근 감속 단계. ROS 2 파라미터에 쌍의 배열 타입이 없어 double 배열 둘로
        # 나눠 받는다. 같은 순번끼리 짝이며 개수가 다르면 기동 시 죽는다.
        # 기본값의 근거·위험은 approach_speed.py docstring 참조 — 특히 첫 감속
        # 지점을 3.0 m 에서 1.5 m 로 늦춘 이유가 거기 있다.
        self.declare_parameter(
            "approach_slowdown_distances_m",
            [distance for distance, _ in DEFAULT_APPROACH_STAGES],
        )
        self.declare_parameter(
            "approach_speed_limit_percents",
            [percent for _, percent in DEFAULT_APPROACH_STAGES],
        )
        self.declare_parameter("tick_hz", 5.0)
        # RobotState 층/건물 값 소스는 미결 사항 #5 — 일단 파라미터.
        self.declare_parameter("current_floor", -1)
        self.declare_parameter("current_building", "")
        # 도착 후 대화(arrival-dialog-flow). 기본 on (2026-08-30 사용자 결정) —
        # 도착 후 유형별 질문·대기·홈 복귀가 동작한다. 끄려면 :=false.
        self.declare_parameter("arrival_dialog", True)

        dest_path = str(self.get_parameter("destinations_yaml").value)
        if not dest_path:
            raise RuntimeError("destinations_yaml parameter 가 비어 있습니다 (launch 에서 절대경로 지정)")
        self._destinations_path = str(Path(dest_path).expanduser())
        configured_map_id = str(self.get_parameter("map_id").value).strip()
        self._map_id = configured_map_id or Path(self._destinations_path).parent.name
        self.destinations = load_destinations(self._destinations_path)
        if Path(self._destinations_path).exists():
            self.get_logger().info(
                f"목적지 {len(self.destinations)}개 로드: {self._destinations_path}"
            )
        else:
            self.get_logger().warn(
                f"목적지 catalog가 없어 빈 목록으로 시작합니다: {self._destinations_path}"
            )

        # 홈은 목적지 카탈로그와 같은 폴더의 home.yaml 이다. 지도별 자료가
        # 흩어지지 않도록 목적지 경로의 부모를 그대로 쓴다.
        self._home_storage = HomeStorage(Path(self._destinations_path).parent.parent)

        map_yaml = str(self.get_parameter("map_yaml").value)
        if map_yaml:
            self.map_bounds = load_map_bounds(map_yaml)
            self.get_logger().info(f"지도 경계: {self.map_bounds}")
        else:
            self.map_bounds = None
            self.get_logger().warn(
                "map_yaml 미지정 — 게이트 ⑤의 지도 경계 검증이 생략됩니다. "
                "실기 운용 전 반드시 지정할 것."
            )

        # 잘못된 단계 조합으로 조용히 굴러가는 것보다 기동 시점에 죽는 편이 안전하다.
        # 사용자가 핸들을 잡은 뒤 감속이 어긋난 것을 알게 되는 것이 가장 나쁘다.
        try:
            approach_stages = stages_from_lists(
                self.get_parameter("approach_slowdown_distances_m").value,
                self.get_parameter("approach_speed_limit_percents").value,
            )
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f"접근 감속 단계 파라미터가 잘못되었습니다: {exc}") from exc

        retry_limit = int(self.get_parameter("nav_retry_limit").value)
        retry_delay = float(self.get_parameter("nav_retry_delay_sec").value)
        # 도착 후 대화 + 홈 복귀. home.yaml(목적지 폴더의 __home__)이 있으면
        # 복귀 목적지로 넣는다 — 없으면 도착 후 대화는 제자리 대기로 폴백한다.
        arrival_dialog = bool(self.get_parameter("arrival_dialog").value)
        # 지도 변경 감지까지 하는 쪽을 쓴다. dev 의 _load_home 은 같은 파일을
        # 읽지만 "지도가 홈보다 새로 저장됐으면 visited_ok 를 내린다"가 없다 —
        # 그게 없으면 벽 위치가 옮겨진 지도에서 옛 홈을 확인된 것으로 믿는다.
        home = self._load_home_destination()
        self.logic = MissionLogic(
            confirm_timeout_sec=float(self.get_parameter("confirm_timeout_sec").value),
            approach_turn_yaw_rad=math.radians(
                float(self.get_parameter("approach_turn_yaw_deg").value)),
            estop_release_grace_sec=float(self.get_parameter("estop_release_grace_sec").value),
            approach_stages=approach_stages,
            nav_retry_limit=retry_limit,
            nav_retry_delay_sec=retry_delay,
            arrival_dialog=arrival_dialog,
            # 홈 좌표는 목적지 폴더의 home.yaml 하나다. 앱이 HomeStorage 로
            # 쓰고 노드가 읽는다 — 두 구현이 같은 형식을 쓴다.
            return_destination=home,
            auto_return_home=bool(self.get_parameter("auto_return_home").value),
        )
        if arrival_dialog:
            self.get_logger().info(
                f"도착 후 대화: 켜짐 · 홈={'있음' if home else '없음(제자리 대기)'}")
        if home is not None and self.logic.auto_return_home:
            self.get_logger().warn(
                "접근 뒤 자동 홈 복귀: 켜짐 — 사람이 부르지 않아도 로봇이 홈까지 달립니다."
            )
        elif home is not None:
            self.get_logger().info(
                "접근 뒤 자동 홈 복귀: 꺼짐(기본) — 접근을 마친 자리에 섭니다. "
                "앱의 홈 복귀 버튼은 그대로 동작합니다."
            )
        if retry_limit > 0:
            self.get_logger().info(
                f"주행 실패 시 자동 재시도: 최대 {retry_limit}회 · {retry_delay:.1f}초 간격"
            )
        else:
            self.get_logger().info("주행 실패 시 자동 재시도: 꺼짐")
        if approach_stages:
            ladder = ", ".join(
                f"{distance:.2f}m이하 {percent:.0f}%"
                for distance, percent in approach_stages
            )
            self.get_logger().info(f"접근 감속 단계: {ladder}")
        else:
            self.get_logger().warn(
                "접근 감속 단계가 비어 있습니다 — 목적지까지 최대속도로 접근합니다."
            )

        # Nav2 커맨더. 별도 노드로 두고 executor 에 넣지 않는다.
        # 호출은 _nav_lock 으로 직렬화 (goToPose/cancelTask/isTaskComplete 는 짧게 끝남).
        if BasicNavigator is None:
            raise RuntimeError("nav2_simple_commander 를 import 할 수 없습니다")
        self.navigator = BasicNavigator("vica_mission_navigator")
        self._nav_lock = threading.Lock()
        # _cancel_nav_blocking 이 이 lock 을 잡고 Nav2 응답을 기다릴 수 있다.
        # 콜백에서 무한히 기다리면 고치려던 문제가 그대로 돌아오므로 시한을 둔다.
        self._nav_lock_timeout_sec = 2.0
        self._nav_active = False  # goToPose 수락 후 완료 전까지 True
        # 주행 번호표. goToPose/spin 이 수락될 때마다 1 오른다. 취소 스레드는
        # 자기가 받은 번호와 지금 번호가 다르면 취소를 건너뛴다 — 번호가
        # 달라졌다는 것은 그 사이 새 goal 이 나갔다는 뜻이고(앱 전권화의
        # [취소 -> 즉시 출발] 경로), 그때 cancelTask() 를 부르면 옛 goal 이
        # 아니라 방금 보낸 새 goal 이 취소된다. 옛 goal 은 Nav2 가 새 goal 을
        # 받으면서 스스로 선점해 이미 내려가 있다.
        self._nav_gen = 0

        # emergency 계열은 전용 callback group — intent/tick 처리가
        # 긴급 취소를 블로킹하지 않게 한다 (MultiThreadedExecutor 전제).
        self._main_group = MutuallyExclusiveCallbackGroup()
        self._emergency_group = MutuallyExclusiveCallbackGroup()

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(
            VicaIntent, "/vica/intent", self._on_intent, 10, callback_group=self._main_group
        )
        self.create_subscription(
            EmergencyEvent,
            "/vica/emergency",
            self._on_emergency,
            reliable_qos,
            callback_group=self._emergency_group,
        )
        self._estop_active = False
        self.create_subscription(
            Bool,
            "/emergency_stop",
            self._on_estop,
            10,
            callback_group=self._emergency_group,
        )

        # 접근 질문의 "재생 종료" 시점. 응답 대기 8초를 여기서부터 세도록
        # mission_logic.on_approach_question_spoken 이 설계돼 있었는데(6.2절)
        # 배선이 없었다 — 그래서 질문 생성 시각부터 세어, 재생 2초 + STT 자동
        # 녹음 4초 + 변환 2초가 8초와 경주하게 됐다(2026-08-28 실기).
        self.create_subscription(
            String,
            "/vica/tts_done",
            self._on_tts_done,
            10,
            callback_group=self._main_group,
        )

        # 웨이크워드 호출. WAITING 각성·복귀 브레이크(도착 후 대화)에 쓴다.
        self.create_subscription(
            String, "/vica/wake", self._on_wake, 10,
            callback_group=self._main_group,
        )
        # 청취 상태 — 무응답 시계를 귀가 바쁜 동안 멈춘다 (mission_logic
        # on_listen_state 주석, 2026-08-30).
        self.create_subscription(
            String, "/vica/listen_state",
            lambda msg: self.logic.on_listen_state(msg.data, self._now()), 10,
            callback_group=self._main_group,
        )

        self.pub_tts = self.create_publisher(String, "/vica/tts_request", 10)
        # 취소·앱 선점의 큐 청소(StopSpeech)는 tts_request 의 제어 메시지
        # "control:stop" 으로 보낸다 — 별도 토픽(tts_stop)은 뒤이어 시키는
        # 새 멘트("안내를 시작합니다")와 도착 순서가 뒤집힐 수 있다
        # (실기: 같은 경합으로 "네?" 증발). 같은 토픽은 순서가 보장된다.
        # 질문(Say.expects_reply)을 말할 때 true — 웨이크워드 노드가 질문 TTS 종료
        # 직후 재청취 창을 연다 ("비카야" 재호출 없이 "네/아니요"로 답하게).
        self.pub_listen_request = self.create_publisher(Bool, "/vica/listen_request", 10)
        self.pub_state = self.create_publisher(RobotState, "/vica/robot_state", 10)
        self.pub_goal_event = self.create_publisher(String, "/vica_goal_event", 10)
        self.pub_speed_limit = self.create_publisher(
            SpeedLimit,
            "/speed_limit",
            10,
        )
        self._publish_nav_speed_limit(0.0)
        self.create_service(
            RequestDestination,
            "/vica/mission/request_destination",
            self._on_destination_request,
            callback_group=self._main_group,
        )
        # 물류 배송 전용 문. 요청 모양은 같고 private 목적지만 추가로 허용한다.
        # MissionCommand 처럼 "무엇을 할지는 service 이름이 정한다". 음성/LLM 은
        # 서비스 클라이언트가 없어 이 문에 닿지 못한다(홈 복귀와 같은 논리).
        self.create_service(
            RequestDestination,
            "/vica/mission/request_delivery",
            self._on_delivery_request,
            callback_group=self._main_group,
        )
        self.create_service(
            Trigger,
            "/vica/mission/reload_destinations",
            self._on_reload_destinations,
            callback_group=self._main_group,
        )
        # 취소·일시정지·재개. 안전 사건이 아니라 목표 조작이므로 E-stop 과 달리
        # _main_group 에 둔다(주행 판정과 같은 스레드에서 직렬 처리).
        self.create_service(
            MissionCommand,
            "/vica/mission/cancel_destination",
            self._on_cancel_request,
            callback_group=self._main_group,
        )
        self.create_service(
            MissionCommand,
            "/vica/mission/pause_navigation",
            self._on_pause_request,
            callback_group=self._main_group,
        )
        self.create_service(
            MissionCommand,
            "/vica/mission/resume_navigation",
            self._on_resume_request,
            callback_group=self._main_group,
        )
        # ── 사람 접근 (Phase B, 설계 3·5절) ────────────────────────────────
        # goal 권한의 경계다. person_detector_node 는 요청만 보내고, 좌표를
        # 계산해 Nav2 로 보내는 것은 여기뿐이다.
        self.create_service(
            RequestApproach,
            "/vica/mission/request_approach",
            self._on_approach_request,
            callback_group=self._main_group,
        )
        self.create_service(
            MissionCommand,
            "/vica/mission/cancel_approach",
            self._on_approach_cancel,
            callback_group=self._main_group,
        )
        # ── 홈 위치 (2026-08-26) ───────────────────────────────────────────
        # 홈은 목적지가 아니라 로봇의 설정값이라 destinations.yaml 에 넣지 않고
        # 같은 폴더의 home.yaml 하나로 관리한다. 파일이 하나이므로 "지도당 1개"가
        # 저절로 지켜진다.
        #
        # 홈 복귀를 별도 서비스로 두는 것이 **사용자와 관리자를 가르는 방법**이다.
        # 목적지 요청은 UUID 로 지목하는데 홈에는 UUID 가 없고, 음성 LLM 은
        # 토픽만 발행할 뿐 서비스 클라이언트가 없다. 즉 권한을 검사해서 막는
        # 것이 아니라 **사용자 쪽에 문이 없게** 만든 것이다. 목적지에 is_home
        # 플래그를 두면 "홈이면 거른다"를 모든 진입점마다 넣어야 하고 한 곳만
        # 빠뜨려도 조용히 샌다.
        self.create_service(
            SaveHome,
            "/vica/home/save",
            self._on_save_home,
            callback_group=self._main_group,
        )
        self.create_service(
            GetHome,
            "/vica/home/get",
            self._on_get_home,
            callback_group=self._main_group,
        )
        self.create_service(
            DeleteHome,
            "/vica/home/delete",
            self._on_delete_home,
            callback_group=self._main_group,
        )
        self.create_service(
            MissionCommand,
            "/vica/mission/return_home",
            self._on_return_home,
            callback_group=self._main_group,
        )
        # 접근 goal 계산(approach_goal)은 "사람에게서 로봇 쪽으로 1.1 m" 라
        # 로봇의 map 위치가 필요하다. TF 조회 대신 /amcl_pose 를 구독한다 —
        # 이 노드에 tf2 리스너를 새로 들이는 것보다 싸고, goal 의 후퇴 방향을
        # 정하는 용도라 AMCL 갱신 주기(이동 시에만) 수준의 신선도면 충분하다.
        self._robot_pose = None
        # AMCL 은 /amcl_pose 를 transient_local(보관) + 이동 시에만 발행한다.
        # 일반(volatile) 구독은 이 노드가 나중에 켜지면 보관본을 못 받아,
        # 로봇이 움직이기 전까지 접근 요청이 전부 "위치 없음"으로 거절된다
        # (2026-08-24 B5 실기에서 Mission 재시작 직후 실제로 났다).
        amcl_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl_pose,
            amcl_qos,
            callback_group=self._main_group,
        )

        tick_hz = float(self.get_parameter("tick_hz").value)
        self.create_timer(1.0 / tick_hz, self._tick, callback_group=self._main_group)
        self.create_timer(1.0, self._publish_robot_state, callback_group=self._main_group)

        self.get_logger().info("vica_mission_manager 시작 (상태: idle)")

    # -- 콜백 -------------------------------------------------------------------

    def _load_home(self):
        try:
            return load_home(self._destinations_path)
        except Exception as exc:  # 홈이 없거나 깨져도 서비스는 계속돼야 한다
            self.get_logger().warn(f"홈 로드 실패(제자리 대기로 폴백): {exc}")
            return None

    def _on_intent(self, msg: VicaIntent) -> None:
        # 복귀 주행 중 늦게 도착한 답 — 마지막 그물 (2026-08-30 실기: 무응답
        # 오판으로 떠난 직후 도착한 답이 버려져 세울 방법이 없었다). 복귀를
        # 조용히 멈추고(ASKING_NEXT) 아래 라우팅이 그 뜻을 그대로 처리한다
        # — wait 는 대기, navigate 제안은 확인 흐름. finish 는 이미 홈으로
        # 가는 중이라 제외(그대로 간다).
        if (self.logic.state == State.RETURNING
                and msg.intent in ("wait", "navigate")):
            self._run_actions(self.logic.on_return_brake(self._now(), quiet=True))
            self.get_logger().info(f"복귀 중 답 도착({msg.intent}) — 복귀 취소")

        # 도착 후 대화 중이면 답(wait/finish/cancel/affirm/deny)을
        # on_arrival_answer 로 보낸다 — 같은 말이라도 이 상태에선 뜻이 다르다
        # (도착 후 cancel = 홈 복귀 등). 상태 판정은 로직이 갖고 있다.
        # 예외: 새 목적지 '제안'(navigate + need_confirm)은 대화의 답이 아니라
        # 새 안내 요청이다 — 대화를 닫고 아래 일반 확인 흐름(CONFIRMING)으로
        # 합류시킨다. 제안에서 바로 출발하면 확인 질문 전에 달리고, 뒤따라온
        # 확정 답이 MSG_BUSY 로 거절된다 (2026-08-30 실기).
        if self.logic.is_awaiting_arrival_answer():
            if msg.intent == "navigate" and msg.need_confirm:
                self.logic.exit_arrival_dialog()
                # return 하지 않는다 — 아래 일반 경로가 이어서 처리한다.
            else:
                self._on_arrival_answer(msg)
                return
        # 음성 취소·일시정지·재개는 service 와 같은 로직을 탄다.
        # 다만 취소는 바로 실행하지 않고 되물어 확인한 뒤에만 처리한다.
        if msg.intent in ("cancel", "pause", "resume"):
            self._on_voice_mission_command(msg)
            return
        # 짧은 답(affirm/deny)은 어느 질문의 답인지 여기의 상태가 정한다.
        # CONFIRMING 이면 확인 질문("…로 안내해 드릴까요?")의 답 — 목적지는
        # 미션이 이미 알고 있어 LLM 의 추측 없이 직접 확정한다(2026-08-31).
        # 그 외에는 접근 질문 배선 — AWAITING_USER 가 아니면
        # on_approach_answer 가 빈 목록을 돌려주고, 그때는 무시가 정답이다.
        if msg.intent in ("affirm", "deny"):
            if self.logic.state == State.CONFIRMING:
                self._on_confirm_answer(msg.intent == "affirm")
            else:
                self._on_voice_answer(msg.intent == "affirm")
            return

        intent = IntentData(
            intent=msg.intent,
            matched_destination_id=msg.matched_destination_id,
            need_confirm=msg.need_confirm,
            safety_flag=msg.safety_flag,
            wait_minutes=int(getattr(msg, "wait_minutes", -1)),
        )
        dest = self.destinations.get(msg.matched_destination_id) or None
        actions = self.logic.on_intent(
            intent, dest, self.map_bounds, self._nav2_ready(), self._now()
        )
        self.get_logger().info(
            f"intent={msg.intent} dest={msg.matched_destination_id or '-'} "
            f"confirm={msg.need_confirm} -> state={self.logic.state.value}"
        )
        self._run_actions(actions)

    def _on_tts_done(self, msg: String) -> None:
        """TTS 가 끊기지 않고 끝까지 재생한 문장. 접근 질문일 때만 시계를 켠다.

        다른 멘트(수락·도착 등)의 재생 완료는 응답 대기와 무관하고, 로직 쪽이
        AWAITING_USER 가 아니면 무시하므로 이중 방어다.
        """
        if MSG_APPROACH_QUESTION in msg.data:
            self.logic.on_approach_question_spoken(self._now())
        # 도착 후 대화의 질문도 재생완료 시점부터 8초를 센다. 로직이
        # ASKING_* 가 아니면 무시하므로(이중 방어) 문구 대조 없이 넘긴다.
        self.logic.on_arrival_question_spoken(self._now())

    def _on_confirm_answer(self, affirmative: bool) -> None:
        """확인 질문의 네/아니오. 확인 중 목적지를 되찾아 로직에 넘긴다."""
        dest_id = self.logic.confirming_dest_id or ""
        dest = self.destinations.get(dest_id) or None
        before = self.logic.state
        actions = self.logic.on_confirm_answer(
            affirmative, dest, self.map_bounds, self._nav2_ready(), self._now()
        )
        self.get_logger().info(
            f"확인 응답 {'긍정' if affirmative else '부정'}: dest={dest_id or '-'} "
            f"{before.value} -> {self.logic.state.value}"
        )
        self._run_actions(actions)

    def _on_voice_answer(self, affirmative: bool) -> None:
        before = self.logic.state
        actions = self.logic.on_approach_answer(affirmative, self._now())
        if not actions:
            self.get_logger().info(
                f"affirm/deny 무시: state={before.value} (접근 질문 대기 중이 아님)"
            )
            return
        self._run_actions(actions)
        self.get_logger().info(
            f"접근 응답 {'긍정' if affirmative else '부정'}: "
            f"{before.value} -> {self.logic.state.value}"
        )

    def _on_arrival_answer(self, msg: VicaIntent) -> None:
        """도착 후 대화 중의 답. navigate 답이면 다음 목적지를 게이트 없이
        넘긴다(로직이 NAVIGATING 으로 전이) — 목적지 확정은 이미 음성 쪽
        matched_destination_id 로 됐다."""
        intent = IntentData(
            intent=msg.intent,
            matched_destination_id=msg.matched_destination_id,
            need_confirm=msg.need_confirm,
            safety_flag=msg.safety_flag,
            wait_minutes=int(getattr(msg, "wait_minutes", -1)),
        )
        next_dest = None
        if msg.intent == "navigate":
            next_dest = self.destinations.get(msg.matched_destination_id) or None
        before = self.logic.state
        actions = self.logic.on_arrival_answer(intent, self._now(), next_dest=next_dest)
        self._run_actions(actions)
        self.get_logger().info(
            f"도착 후 답 intent={msg.intent}: {before.value} -> {self.logic.state.value}")

    def _on_wake(self, msg: String) -> None:
        """/vica/wake — WAITING 은 각성(다시 안내 질문), RETURNING 은 복귀
        브레이크(E), 답-대기 상태는 옛 질문 접기(2026-09-01). 나머지는 무시."""
        if self.logic.state == State.RETURNING:
            actions = self.logic.on_return_brake(self._now())
            if actions:
                self._run_actions(actions)
                self.get_logger().info("복귀 중 '비카야' — 복귀 취소하고 응대")
            return
        before = self.logic.state
        actions = self.logic.on_wake(self._now())
        if actions or before != self.logic.state:
            self._run_actions(actions)
            self.get_logger().info(
                f"'비카야': {before.value} -> {self.logic.state.value}")

    def _on_voice_mission_command(self, msg: VicaIntent) -> None:
        """음성으로 온 취소·일시정지·재개를 처리한다.

        취소는 잘못 알아들으면 안내가 끊기므로 곧바로 실행하지 않는다.
        "취소할까요?"로 되묻고, 확인 응답이 와야 실제로 취소한다. 확인을 기다리는
        동안에도 주행은 계속되며, 응답이 없으면 그대로 안내를 이어간다.
        """
        now = self._now()
        before = self.logic.state

        if msg.intent == "cancel":
            if self.logic.cancel_confirm_pending:
                # 이미 되물은 상태에서 다시 "취소"라고 하면 긍정으로 본다.
                actions = self.logic.on_cancel_confirm_answer(True, now)
                self._run_actions(actions)
                self.get_logger().info(
                    f"음성 취소 확정: {before.value} -> {self.logic.state.value}"
                )
                return
            actions, reason = self.logic.on_cancel_confirm_request(now)
        elif msg.intent == "pause":
            actions, reason = self.logic.on_pause_request(now)
        else:
            actions, reason = self.logic.on_resume_request(self._nav2_ready(), now)

        if reason != GateReason.OK:
            message = _REJECT_MESSAGES.get(reason)
            if message:
                self._run_actions([Say(message, priority="response")])
            self.get_logger().warn(
                f"음성 {msg.intent} 거부: state={before.value} reason={reason.value}"
            )
            return

        self._run_actions(actions)
        self.get_logger().info(
            f"음성 {msg.intent} 처리: {before.value} -> {self.logic.state.value}"
        )

    # -- 취소 / 일시정지 / 재개 service -------------------------------------------
    #
    # 앱·CLI 는 요청만 보내고 허용 여부는 여기서 판정한다. 음성(LLM)은 같은 로직을
    # _on_intent 의 cancel/pause/resume 분기로 탄다.

    def _on_cancel_request(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        return self._handle_mission_command(request, response, "cancel")

    def _on_pause_request(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        return self._handle_mission_command(request, response, "pause")

    def _on_resume_request(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        return self._handle_mission_command(request, response, "resume")

    def _handle_mission_command(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
        command: str,
    ) -> MissionCommand.Response:
        try:
            request_id = str(UUID(request.request_id))
        except ValueError:
            response.accepted = False
            response.message = "request_id는 UUID여야 합니다."
            return response
        if request_id != request.request_id.lower():
            response.accepted = False
            response.message = "request_id가 canonical UUID 형식이 아닙니다."
            return response

        before = self.logic.state
        actions, reason = self._run_mission_command(command)
        if reason != GateReason.OK:
            response.accepted = False
            response.message = f"{command} 요청 거부: {reason.value}"
            self.get_logger().warn(
                f"{command} 요청 거부: state={before.value} reason={reason.value}"
            )
            return response

        self._run_actions(actions)
        response.accepted = True
        response.message = f"{command} 요청을 처리했습니다."
        self.get_logger().info(
            f"{command} 처리: {before.value} -> {self.logic.state.value}"
        )
        return response

    # -- 홈 위치 service (2026-08-26) --------------------------------------------
    #
    # 홈은 로봇의 설정값이라 목적지 카탈로그와 분리한다. 이 네 서비스가 홈에
    # 닿는 유일한 길이며, 음성(LLM)은 토픽만 발행하므로 여기 들어올 수 없다.

    def _load_home_destination(self):
        """home.yaml 을 읽어 MissionLogic 이 쓸 Destination 으로 만든다.

        홈이 없으면 None 이다. 이것은 오류가 아니라 아직 지정하지 않은 정상
        상태이며, 그때는 자동 복귀만 꺼지고 안내는 그대로 동작한다.

        ``visited_ok`` 가 False 여도 좌표는 그대로 쓴다. 확인 여부는 관리자에게
        보여줄 정보이지 로봇이 갈지 말지를 정하는 값이 아니다 — 오히려 가 봐야
        확인이 되므로, 여기서 막으면 영영 확인할 수 없다.
        """
        map_yaml = str(self.get_parameter("map_yaml").value)
        if map_yaml:
            # 같은 map_id 로 지도만 바뀌었으면 확인 상태를 되돌린다. 좌표는
            # 그대로인데 벽 위치가 옮겨져 홈이 벽 속에 들어갈 수 있다.
            before = self._home_storage.read(self._map_id)
            home = self._home_storage.invalidate_if_map_is_newer(
                self._map_id, map_yaml
            )
            if (
                before is not None
                and home is not None
                and before.visited_ok
                and not home.visited_ok
            ):
                self.get_logger().warn(
                    "지도가 홈보다 새로 저장되어 홈 확인 상태를 되돌렸습니다. "
                    "앱에서 '홈으로 가보기'로 다시 확인하세요."
                )
        else:
            home = self._home_storage.read(self._map_id)
        if home is None:
            return None
        return Destination(
            id=HOME_DESTINATION_ID,
            name=home.label or "홈",
            pose=Pose2D(x=home.x, y=home.y, yaw_deg=home.yaw, frame_id="map"),
        )

    def _refresh_home(self) -> None:
        """저장·삭제 직후 MissionLogic 이 쓰는 홈을 새로 읽어 맞춘다."""
        self.logic.return_destination = self._load_home_destination()

    def _on_save_home(
        self,
        request: SaveHome.Request,
        response: SaveHome.Response,
    ) -> SaveHome.Response:
        if request.map_id != self._map_id:
            response.accepted = False
            response.visited_ok = False
            response.message = (
                f"다른 지도의 홈은 저장할 수 없습니다. "
                f"현재={self._map_id}, 요청={request.map_id}"
            )
            return response

        try:
            home = build_home(
                map_id=request.map_id,
                x=request.x,
                y=request.y,
                yaw=request.yaw,
                source=request.source,
                score=request.score,
                label=request.label,
            )
            saved = self._home_storage.write(home)
        except (ValueError, OSError) as exc:
            response.accepted = False
            response.visited_ok = False
            response.message = f"홈 저장 실패: {exc}"
            self.get_logger().warn(response.message)
            return response

        self._refresh_home()
        response.accepted = True
        response.visited_ok = saved.visited_ok
        response.message = (
            "홈을 저장했습니다. 아직 가 본 적이 없으니 '홈으로 가보기'로 확인하세요."
        )
        self.get_logger().info(
            f"홈 저장: ({saved.x:.2f}, {saved.y:.2f}, {saved.yaw:.0f}도) "
            f"source={saved.source} score={saved.score:.1f}"
        )
        return response

    def _on_get_home(
        self,
        request: GetHome.Request,
        response: GetHome.Response,
    ) -> GetHome.Response:
        map_id = request.map_id or self._map_id
        try:
            home = self._home_storage.read(map_id)
        except ValueError as exc:
            response.exists = False
            response.message = f"map_id 오류: {exc}"
            return response

        if home is None:
            response.exists = False
            response.message = "홈이 아직 지정되지 않았습니다."
            return response

        response.exists = True
        response.message = ""
        response.x = home.x
        response.y = home.y
        response.yaw = home.yaw
        response.source = home.source
        response.score = home.score
        response.label = home.label
        response.visited_ok = home.visited_ok
        response.saved_at = home.saved_at
        return response

    def _on_delete_home(
        self,
        request: DeleteHome.Request,
        response: DeleteHome.Response,
    ) -> DeleteHome.Response:
        if request.map_id != self._map_id:
            response.accepted = False
            response.message = (
                f"다른 지도의 홈은 지울 수 없습니다. "
                f"현재={self._map_id}, 요청={request.map_id}"
            )
            return response

        try:
            removed = self._home_storage.delete(request.map_id)
        except (ValueError, OSError) as exc:
            response.accepted = False
            response.message = f"홈 삭제 실패: {exc}"
            return response

        self._refresh_home()
        response.accepted = True
        response.message = (
            "홈을 지웠습니다. 자동 복귀가 꺼집니다."
            if removed
            else "지울 홈이 없었습니다."
        )
        self.get_logger().info(response.message)
        return response

    def _on_return_home(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        """관리자가 앱에서 홈 복귀를 눌렀다.

        '홈으로 가보기'(장소 저장 화면)와 '홈으로 복귀'(원격 주행 화면)가
        **같은 서비스**를 부른다. 하는 일이 같기 때문이며, 다른 것은 관리자가
        그 결과를 무엇으로 쓰느냐뿐이다 — 지정 확인이냐 운영 호출이냐.
        """
        try:
            request_id = str(UUID(request.request_id))
        except ValueError:
            response.accepted = False
            response.message = "request_id는 UUID여야 합니다."
            return response
        if request_id != request.request_id.lower():
            response.accepted = False
            response.message = "request_id가 canonical UUID 형식이 아닙니다."
            return response

        before = self.logic.state
        accepted, reason, actions = self.logic.on_return_home_request(
            self._nav2_ready(), self._now()
        )
        if not accepted:
            response.accepted = False
            response.message = f"홈 복귀 거부: {reason.value}"
            self.get_logger().warn(
                f"홈 복귀 거부: state={before.value} reason={reason.value}"
            )
            return response

        self._run_actions(actions)
        home = self.logic.return_destination
        if home is not None:
            self._publish_goal_event("return_home_sent", home)
        response.accepted = True
        response.message = "홈으로 복귀합니다."
        self.get_logger().info(
            f"홈 복귀 시작: {before.value} -> {self.logic.state.value}"
        )
        return response

    # -- 사람 접근 service (Phase B) ---------------------------------------------

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose.orientation
        yaw_deg = math.degrees(
            math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        self._robot_pose = Pose2D(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw_deg=yaw_deg,
            frame_id=msg.header.frame_id or "map",
        )

    def _on_approach_request(
        self,
        request: RequestApproach.Request,
        response: RequestApproach.Response,
    ) -> RequestApproach.Response:
        """사람 접근 요청. 검증 -> goal 계산 -> 상태 기계 순서로 거른다.

        판단(60초 억제·상태·E-stop)은 mission_logic, 기하(1.1 m goal)는
        approach_geometry 가 한다. 여기는 srv 계약의 형식 검증과 배선만 한다.
        """
        try:
            request_id = str(UUID(request.request_id))
        except ValueError:
            response.accepted = False
            response.message = "request_id는 UUID여야 합니다."
            return response
        if request_id != request.request_id.lower():
            response.accepted = False
            response.message = "request_id가 canonical UUID 형식이 아닙니다."
            return response

        target = request.target
        # srv 계약: 상위 track_id 와 target.track_id 가 다르면 거절한다.
        if request.track_id != target.track_id:
            response.accepted = False
            response.message = "track_id가 target.track_id와 다릅니다."
            return response
        if target.track_id == PersonDetection.TRACK_ID_NONE:
            response.accepted = False
            response.message = "추적 id가 없는 탐지는 접근 대상이 아닙니다."
            return response

        px, py = target.pose.position.x, target.pose.position.y
        if not (math.isfinite(px) and math.isfinite(py)):
            # NaN 좌표로 approach_goal 을 부르면 NaN goal 이 나온다(None 이
            # 아니라). 기하에 들어가기 전에 여기서 자른다.
            response.accepted = False
            response.message = "탐지 좌표가 유효하지 않습니다."
            return response
        if self._robot_pose is None:
            response.accepted = False
            response.message = "로봇 위치(/amcl_pose)를 아직 받지 못했습니다."
            return response

        person = Pose2D(x=px, y=py, yaw_deg=0.0,
                        frame_id=target.header.frame_id or "map")
        goal = approach_goal(person, self._robot_pose)
        approach = ApproachRequest(
            goal=goal,                      # None 이면 게이트가 거부한다
            track_id=target.track_id,
            approachable=target.approachable,
        )

        before = self.logic.state
        actions, reason = self.logic.on_approach_request(
            approach, self.map_bounds, self._nav2_ready(), self._now()
        )
        if reason != GateReason.OK:
            response.accepted = False
            response.message = f"접근 요청 거부: {reason.value}"
            self.get_logger().warn(
                f"접근 거부: track={target.track_id} state={before.value} "
                f"reason={reason.value}"
            )
            return response

        self._run_actions(actions)
        response.accepted = True
        response.message = "접근을 시작합니다."
        self.get_logger().info(
            f"접근 승인: track={target.track_id} dist={target.distance_m:.2f}m "
            f"{before.value} -> {self.logic.state.value}"
        )
        return response

    def _on_approach_cancel(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        try:
            request_id = str(UUID(request.request_id))
        except ValueError:
            response.accepted = False
            response.message = "request_id는 UUID여야 합니다."
            return response
        if request_id != request.request_id.lower():
            response.accepted = False
            response.message = "request_id가 canonical UUID 형식이 아닙니다."
            return response

        before = self.logic.state
        actions, reason = self.logic.on_approach_cancel_request(self._now())
        if reason != GateReason.OK:
            response.accepted = False
            response.message = f"접근 취소 거부: {reason.value}"
            return response
        self._run_actions(actions)
        response.accepted = True
        response.message = "접근을 취소했습니다."
        self.get_logger().info(
            f"접근 취소: {before.value} -> {self.logic.state.value}"
        )
        return response

    def _run_mission_command(self, command: str) -> tuple:
        """command 이름에 맞는 로직 핸들러를 부른다. (actions, GateReason)"""
        now = self._now()
        if command == "cancel":
            # 앱 취소는 전면 개방 판 — 음성 취소(on_cancel_request)와 다르다.
            was_idle = self.logic.state is State.IDLE
            result = self.logic.on_app_cancel(now)
            if was_idle:
                # 취소할 것이 없어 조용히 수락한 경우다. 그대로 두면 앱 화면이
                # 로봇보다 뒤처져 있을 때(예: 종료 이벤트를 놓쳤을 때) 취소를
                # 눌러도 아무 일이 없는 것처럼 보인다 — 2026-09-02 실기에서
                # 홈 도착 뒤 정확히 그렇게 굳었다. 지금이 대기라는 사실을 한 번
                # 알려 앱이 스스로 되맞추게 한다. goal 은 이미 없으므로
                # destination 은 넣지 않는다.
                #
                # **왜 새 이름인가**(2026-09-02 사용자 판정, 유지 결정).
                # 이 발행은 그날 보고된 결함의 수리가 아니다. 그 결함은 앱
                # 상태 노드가 return_home_* 를 종료로 안 세던 것이고 거기서
                # 이미 고쳤다. 이것은 **앞으로 어긋날 때를 위한 보험**이다 —
                # /vica_goal_event 는 VOLATILE 이라 앱 노드가 재시작하면
                # '출발'만 듣고 '도착'을 놓칠 수 있고, 그때 유일한 회복은
                # goal_event_timeout_sec(600초)뿐이라 실기에서는 없는 것과
                # 같다. 이 신호가 그 600초를 취소 버튼 한 번으로 줄인다.
                #
                # 기존 이름을 재사용하지 않은 이유: goal_canceled 로 보내면
                # 앱이 팝업을 띄운다(needsPopup). 취소한 주행이 없는데 "주행이
                # 취소되었습니다"는 거짓이고, 그렇게 쌓인 팝업은 관리자가 읽지
                # 않고 닫는 습관을 만든다 — 성공을 팝업으로 안 알리는 것과
                # 같은 이유다. 비용은 이름 하나가 3곳(이 노드·앱 enum·앱
                # provider)에 퍼지는 것이고, 그 값을 치르기로 했다.
                self._publish_goal_event(
                    "state_idle", None, "취소할 주행이 없습니다(대기 중)."
                )
            return result
        if command == "pause":
            return self.logic.on_pause_request(now)
        if command == "resume":
            return self.logic.on_resume_request(self._nav2_ready(), now)
        return [], GateReason.NOT_NAVIGATE

    def _on_destination_request(
        self,
        request: RequestDestination.Request,
        response: RequestDestination.Response,
    ) -> RequestDestination.Response:
        """앱·CLI 요청을 UUID로 검증하고 기존 Mission gate를 통과시킨다."""
        return self._handle_destination_request(
            request, response, allow_private=False, label="공개 목적지"
        )

    def _on_delivery_request(
        self,
        request: RequestDestination.Request,
        response: RequestDestination.Response,
    ) -> RequestDestination.Response:
        """물류 배송 요청. 공개 요청과 같되 private 목적지를 허용한다.

        허용하는 것은 authorization 하나뿐이다. 접근 불가·지도 밖·Nav2 미준비·
        E-stop 은 그대로 거부한다 — 배송이라고 벽 속으로 보내지는 않는다.
        """
        return self._handle_destination_request(
            request, response, allow_private=True, label="배송 목적지",
            is_delivery=True,
        )

    def _handle_destination_request(
        self,
        request: RequestDestination.Request,
        response: RequestDestination.Response,
        allow_private: bool,
        label: str,
        is_delivery: bool = False,
    ) -> RequestDestination.Response:
        try:
            request_id = str(UUID(request.request_id))
            destination_id = str(UUID(request.destination_id))
        except ValueError:
            response.accepted = False
            response.message = "request_id와 destination_id는 UUID여야 합니다."
            return response
        if request_id != request.request_id.lower():
            response.accepted = False
            response.message = "request_id가 canonical UUID 형식이 아닙니다."
            return response
        parsed_destination = UUID(destination_id)
        if (
            parsed_destination.version != 4
            or destination_id != request.destination_id.lower()
        ):
            response.accepted = False
            response.message = "destination_id는 canonical UUID v4여야 합니다."
            return response
        if request.map_id != self._map_id:
            response.accepted = False
            response.message = (
                f"현재 지도와 요청 지도가 다릅니다: "
                f"current={self._map_id}, requested={request.map_id}"
            )
            return response
        # 앱은 어느 상태든 선점한다(ESTOPPED 예외) — 내부에서 전부 취소 후
        # 즉시 출발 (2026-08-31 사용자 결정, on_app_destination 주석 참고).
        destination = self.destinations.get(destination_id)
        before = self.logic.state
        actions, reason = self.logic.on_app_destination(
            destination, self.map_bounds, self._nav2_ready(), self._now(),
            allow_private=allow_private, is_delivery=is_delivery,
        )
        if reason != GateReason.OK:
            response.accepted = False
            response.message = f"목적지 요청 거부: {reason.value}"
            self.get_logger().warn(
                f"{label} 요청 거부: map_id={request.map_id} "
                f"id={destination_id} reason={reason.value}"
            )
            return response
        if before != State.IDLE:
            self.get_logger().info(
                f"앱 선점 주행: {before.value} 를 취소하고 새 목적지로")
        self._run_actions(actions)
        response.accepted = self._nav_active
        response.message = (
            f"목적지 요청을 수락했습니다: {destination.name}"
            if response.accepted and destination is not None
            else "Nav2가 목적지 요청을 수락하지 않았습니다."
        )
        return response

    def _on_reload_destinations(
        self,
        _: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """새 catalog 전체를 검증한 뒤에만 현재 메모리 목록을 교체한다."""
        try:
            destinations = load_destinations(self._destinations_path)
        except (OSError, ValueError, TypeError) as exc:
            response.success = False
            response.message = f"목적지 reload 실패: {exc}"
            self.get_logger().error(response.message)
            return response
        self.destinations = destinations
        response.success = True
        response.message = f"목적지 {len(destinations)}개를 다시 불러왔습니다."
        self.get_logger().info(response.message)
        return response

    def _on_emergency(self, msg: EmergencyEvent) -> None:
        self.get_logger().warn(f"긴급어 수신: '{msg.keyword}' (원문: {msg.source_text})")
        actions = self.logic.on_emergency(msg.keyword, self._now())
        self._run_actions(actions)

    def _on_estop(self, msg: Bool) -> None:
        active = bool(msg.data)
        changed = active != self._estop_active
        self._estop_active = active
        actions = self.logic.on_estop(self._estop_active, self._now())
        if actions:
            self.get_logger().warn(
                f"중앙 estop={self._estop_active} -> state={self.logic.state.value}"
            )
        elif changed:
            self.get_logger().info(
                f"중앙 estop={self._estop_active} -> state={self.logic.state.value}"
            )
        self._run_actions(actions)

    def _tick(self) -> None:
        status = self._poll_nav_status()
        distance = (
            self._nav_distance_remaining() if status == NavStatus.RUNNING else None
        )
        actions = self.logic.on_tick(self._now(), status, distance)
        self._run_actions(actions)

    def _publish_robot_state(self) -> None:
        msg = RobotState()
        msg.current_floor = int(self.get_parameter("current_floor").value)
        msg.current_building = str(self.get_parameter("current_building").value)
        msg.is_moving = self.logic.state == State.NAVIGATING
        # LLM 이 "다시 출발"을 이해하려면 그냥 정지와 일시정지를 구분해야 한다.
        msg.is_paused = self.logic.state == State.PAUSED
        self.pub_state.publish(msg)

    # -- Action 실행 -------------------------------------------------------------

    def _run_actions(self, actions) -> None:
        for action in actions:
            if isinstance(action, Say):
                out = String()
                # ros_tts_node 큐 우선순위 접두어 (vica-voice-llm src/tts_queue.py 계약)
                out.data = f"{action.priority}:{action.text}"
                self.pub_tts.publish(out)
                self.get_logger().info(f"TTS[{action.priority}]: {action.text}")
                if getattr(action, "expects_reply", False):
                    self.pub_listen_request.publish(Bool(data=True))
                    self.get_logger().info("질문 발화 — 재청취 요청 발행")
            elif isinstance(action, StopSpeech):
                out = String()
                out.data = "control:stop"
                self.pub_tts.publish(out)
                self.get_logger().info("발화 큐 청소 (취소·선점)")
            elif isinstance(action, CancelNav):
                self._cancel_nav(action.destination, action.event)
            elif isinstance(action, Navigate):
                self._start_nav(action)
            elif isinstance(action, SpinInPlace):
                self._start_spin(action)
            elif isinstance(action, SetNavSpeedLimit):
                self._publish_nav_speed_limit(action.percent)

    def _publish_nav_speed_limit(self, percent: float) -> None:
        msg = SpeedLimit()
        msg.percentage = True
        msg.speed_limit = float(percent)
        self.pub_speed_limit.publish(msg)
        if percent == 0.0:
            self.get_logger().info("Nav2 접근 속도 제한 해제")
        else:
            self.get_logger().info(f"Nav2 접근 속도 제한: 최대속도의 {percent:.1f}%")

    def _start_nav(self, action: Navigate) -> None:
        dest = action.destination
        goal = PoseStamped()
        goal.header.frame_id = dest.pose.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = dest.pose.x
        goal.pose.position.y = dest.pose.y
        qx, qy, qz, qw = yaw_deg_to_quaternion(dest.pose.yaw_deg)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        # 앞선 취소가 아직 Nav2 응답을 기다리는 중이면 여기서 영원히 기다리지
        # 않는다. 못 보낸 것을 실패로 처리하는 편이 콜백을 막는 것보다 낫다.
        if not self._nav_lock.acquire(timeout=self._nav_lock_timeout_sec):
            self._publish_goal_event(
                "goal_rejected", dest, "이전 goal 취소가 아직 처리 중입니다."
            )
            self.get_logger().error(
                f"NavigateToPose 전송 보류: 이전 취소가 "
                f"{self._nav_lock_timeout_sec:.1f}초 안에 끝나지 않았다 ({dest.id})"
            )
            self._run_actions(self.logic.on_tick(self._now(), NavStatus.FAILED))
            return
        try:
            accepted = self.navigator.goToPose(goal)
            self._nav_active = bool(accepted)
            if accepted:
                # 번호표를 올린다. 거부됐으면 올리지 않는다 — 옛 goal 이
                # 그대로 잡혀 있으므로, 대기 중인 취소가 그것을 마저 지워야 한다.
                self._nav_gen += 1
        finally:
            self._nav_lock.release()
        if accepted:
            self._publish_goal_event("goal_sent", dest)
            self._publish_goal_event("goal_accepted", dest)
            self.get_logger().info(
                f"NavigateToPose 전송: {dest.id} ({dest.pose.x:.2f}, {dest.pose.y:.2f}, "
                f"{dest.pose.yaw_deg:.1f}deg)"
            )
        else:
            self._publish_goal_event("goal_rejected", dest, "Nav2 goal rejected")
            # goal 거부 → 다음 tick 에서 FAILED 처리되도록 상태를 만든다.
            self.get_logger().error(f"NavigateToPose goal 거부됨: {dest.id}")
            self._run_actions(self.logic.on_tick(self._now(), NavStatus.FAILED))

    def _start_spin(self, action: SpinInPlace) -> None:
        """제자리 회전. BasicNavigator.spin() 은 behavior server 의 Spin 을 부른다.

        goToPose 와 같은 isTaskComplete/getResult 로 끝나므로 상태 감시는
        _poll_nav_status 를 그대로 탄다. 거부되면 다음 tick 의 FAILED 로 흘려
        로직이 TURNING 에서 스스로 내려오게 한다(별도 복구 없음 - 회전 실패는
        안내 실패가 아니다).
        """
        with self._nav_lock:
            accepted = self.navigator.spin(spin_dist=action.yaw_rad)
            self._nav_active = bool(accepted)
            if accepted:
                # 회전도 Nav2 task 다. 번호를 올려 늦은 취소가 못 건드리게 한다.
                self._nav_gen += 1
        if accepted:
            self.get_logger().info(
                f"제자리 회전 시작: {action.yaw_rad:.2f} rad (핸들을 사람 쪽으로)"
            )
        else:
            self.get_logger().error("Spin 거부됨 - 회전 없이 접근을 끝낸다")
            self._run_actions(self.logic.on_tick(self._now(), NavStatus.FAILED))

    def _cancel_nav(self, destination=None, event: str = "goal_canceled") -> None:
        """goal 취소를 알리고, Nav2 취소 호출은 별도 스레드에 맡긴다.

        [2026-08-21] 종전에는 이 함수가 콜백 안에서 navigator.cancelTask() 를 직접
        불렀다. humble 의 nav2_simple_commander 는 이렇게 되어 있다.

            def cancelTask(self):
                if self.result_future:
                    future = self.goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, future)   # 시한 없음

        같은 파일의 isTaskComplete() 에는 timeout_sec=0.10 이 있는데 여기에만 없다.
        Nav2 가 취소에 응답하지 않으면 이 호출이 영영 돌아오지 않고, 그러면 세 가지가
        한꺼번에 일어났다.

          1. 아래 _publish_goal_event 에 도달하지 못한다 -> 앱이 '일시정지'를 모른다.
             앱의 '다시 출발' 버튼이 뜰 때도 있고 안 뜰 때도 있던 이유가 이것이다.
          2. pause/resume/cancel/목적지요청이 모두 같은 _main_group(MutuallyExclusive)
             이라 그 뒤로 줄을 선다 -> 무엇을 눌러도 반응이 없다.
          3. supervisor_bringup 의 rosbridge 가 call_services_in_new_thread=false,
             default_call_service_timeout=0.0 이라 그 호출 하나가 rosbridge 전체를
             막는다 -> 앱 재연결도 비상정지도 안 통하고 rosbridge 재시작만이 답이었다.

        그래서 순서를 뒤집고 블로킹 호출을 콜백 밖으로 내보낸다. 이벤트를 먼저 내므로
        앱 표시는 Nav2 응답 여부와 무관해진다. 모터 정지 권위는 원래 이 호출이 아니라
        Safety 래치 체인에 있으므로 취소가 늦어져도 안전 경로는 그대로다.
        """
        # 1) 알림이 먼저다. Nav2 응답을 기다리지 않는다.
        if destination is not None:
            reason = (
                "일시정지 요청으로 목적지를 보관하고 멈췄습니다."
                if event == "goal_paused"
                else "비상정지 또는 Mission 요청으로 목적지가 취소되었습니다."
            )
            self._publish_goal_event(event, destination, reason)

        # 2) 보낼 goal 이 없으면 여기서 끝난다.
        if not self._nav_active:
            return
        self._nav_active = False

        # 3) 시한 없는 취소 호출은 별도 스레드로 보낸다. _nav_lock 을 그 스레드가
        #    잡으므로 _start_nav 는 아래에서 시한부로 기다린다. 쪽지에 지금
        #    번호표를 적어 준다 — 스레드가 lock 을 잡기 전에 새 goal 이 나가면
        #    번호가 달라지고, 그때는 취소하면 안 된다.
        threading.Thread(
            target=self._cancel_nav_blocking,
            args=(self._nav_gen,),
            name="vica_nav_cancel",
            daemon=True,
        ).start()
        self.get_logger().warn(
            "Nav2 goal 취소 요청 (보조 경로 — 모터 정지 권위는 래치 체인)"
        )

    def _cancel_nav_blocking(self, gen: int) -> None:
        """콜백 밖에서 navigator.cancelTask() 를 부른다. 예외를 삼키지 않고 남긴다.

        gen 은 취소를 시킨 시점의 주행 번호표다. cancelTask() 는 '지금 잡혀
        있는' task 를 취소하므로, 이 스레드가 lock 을 잡기 전에 _start_nav 가
        새 goal 을 보냈다면(앱 전권화의 [취소 -> 즉시 출발]) 그대로 부르면
        방금 보낸 새 goal 이 죽는다. 번호가 달라졌으면 옛 goal 은 Nav2 선점으로
        이미 내려간 뒤라 할 일이 없다.
        """
        with self._nav_lock:
            if self._nav_gen != gen:
                self.get_logger().info(
                    "Nav2 취소 건너뜀 — 새 goal 이 이전 goal 을 이미 대체했다"
                )
                return
            try:
                self.navigator.cancelTask()
            except Exception as exc:  # noqa: BLE001 - 스레드가 조용히 죽지 않게 한다
                self.get_logger().error(f"Nav2 goal 취소 호출 실패: {exc}")
            else:
                self.get_logger().info("Nav2 goal 취소 완료")

    def _poll_nav_status(self) -> NavStatus:
        if not self._nav_active:
            return NavStatus.NONE
        with self._nav_lock:
            if not self.navigator.isTaskComplete():
                return NavStatus.RUNNING
            self._nav_active = False
            result = self.navigator.getResult()
        # 복귀 중이면 목적지가 홈이다. 도착 여부가 곧 "이 홈에 갈 수 있는가"의
        # 답이므로 여기서 home.yaml 의 visited_ok 를 기록한다.
        returning = self.logic.state == State.RETURNING
        if result == TaskResult.SUCCEEDED:
            if returning:
                self._record_home_visit(True)
            if self.logic.active_destination is not None:
                self._publish_goal_event(
                    "return_home_succeeded" if returning else "goal_succeeded",
                    self.logic.active_destination,
                )
            return NavStatus.SUCCEEDED
        if result == TaskResult.CANCELED:
            if self.logic.active_destination is not None:
                self._publish_goal_event(
                    "return_home_canceled" if returning else "goal_canceled",
                    self.logic.active_destination,
                )
            return NavStatus.CANCELED
        if returning:
            # 실패도 기록한다. 한 번 확인된 홈이라도 가구가 놓이거나 문이
            # 잠기면 못 가는 자리가 된다 — 좌표는 그대로인데 도달할 수 없다.
            self._record_home_visit(False)
        if self.logic.active_destination is not None:
            self._publish_goal_event(
                "return_home_failed" if returning else "goal_failed",
                self.logic.active_destination,
                "Nav2 task failed",
            )
        return NavStatus.FAILED

    def _record_home_visit(self, visited_ok: bool) -> None:
        """복귀 결과를 home.yaml 에 남긴다. 실패해도 주행 판정을 막지 않는다."""
        try:
            self._home_storage.mark_visited(self._map_id, visited_ok)
        except (ValueError, OSError) as exc:
            self.get_logger().warn(f"홈 확인 상태를 기록하지 못했습니다: {exc}")

    def _nav_distance_remaining(self):
        """Nav2 feedback 의 남은 거리(m). 아직 없으면 None.

        Nav2 는 경로를 계산하기 전에 0.0 을 주기도 해서 양수만 신뢰한다.
        """
        with self._nav_lock:
            feedback = self.navigator.getFeedback()
        distance = getattr(feedback, "distance_remaining", None)
        if distance is None:
            return None
        distance = float(distance)
        return distance if distance > 0.0 else None

    # -- 유틸 --------------------------------------------------------------------

    def _nav2_ready(self) -> bool:
        """Nav2 미준비 시 goal 거부용. goToPose 내부의 무한 대기를 피하려고
        action server 준비 여부만 즉시 확인한다."""
        try:
            return bool(self.navigator.nav_to_pose_client.server_is_ready())
        except AttributeError:
            return False

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_goal_event(self, event: str, destination, reason: str = "") -> None:
        """goal 사건을 앱에 알린다. destination 이 None 이면 좌표 없이 보낸다.

        목적지 없는 사건이 하나 있다 — 취소할 주행이 없을 때의 `state_idle`.
        빈 값으로 채우는 이유는 앱이 키 부재와 빈 값을 다르게 다루지 않기
        때문이고, 키를 빼면 옛 앱에서 KeyError 가 날 수 있어서다.
        """
        msg = String()
        msg.data = json.dumps(
            {
                "event": event,
                "map_id": self._map_id,
                "location_id": destination.id if destination else "",
                "destination_id": destination.id if destination else "",
                "name": destination.name if destination else "",
                "x": destination.pose.x if destination else 0.0,
                "y": destination.pose.y if destination else 0.0,
                "yaw": destination.pose.yaw_deg if destination else 0.0,
                "reason": reason,
                "timestamp": datetime.now().isoformat(timespec="seconds"),
            },
            ensure_ascii=False,
        )
        self.pub_goal_event.publish(msg)


def main(args=None) -> None:
    """긴급 콜백이 일반 처리에 막히지 않도록 다중 스레드 executor로 실행한다."""
    rclpy.init(args=args)
    node = MissionManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
