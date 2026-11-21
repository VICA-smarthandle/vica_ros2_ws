#!/usr/bin/env python3
"""vica_mission_manager — VICA 음성→주행 통합의 유일한 신규 노드 (진행순서 ②).

역할: /vica/intent(LLM 의 '제안')를 게이트 5조건으로 심사해, 통과한 경우에만
nav2_simple_commander 로 NavigateToPose 를 보낸다. 판단 로직은 전부
mission_logic.py(순수 함수/상태 머신)에 있고, 이 파일은 ROS 배선만 담당한다.

안전 원칙(불변):
- LLM/음성 파트는 /cmd_vel·Nav2 goal 을 직접 발행하지 않는다 — 여기가 유일한 관문.
- 모터 정지의 권위는 vica_safety의 /emergency_stop 중앙 래치 체인.
  여기서의 goal 취소는 심층 방어 보조 경로.

구독: /vica/intent, /vica/emergency·/vica/tipped·/app_estop_state·/emergency_stop(전용 callback group),
      /vica/patrol/event(경비 모드 — 감지·호출어 노드가 순찰 중에만 낸다)

`/emergency_stop`은 emergency_stop_node가 물리·앱·음성을 중앙 래치한 권위 상태다.
입력 펄스가 끝나도 명시적 reset 전까지 true를 유지한다.
발행: /vica/tts_request(std_msgs/String), /vica/robot_state(1Hz)
"""
from __future__ import annotations

import json
import math
import re
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional
from uuid import UUID

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from nav2_msgs.msg import BehaviorTreeLog, SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.exceptions import InvalidHandle
from rclpy.executors import (ExternalShutdownException, MultiThreadedExecutor, ShutdownException,
                             SingleThreadedExecutor)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Log
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from vica_interfaces.msg import EmergencyEvent, RobotState, VicaIntent
from vica_interfaces.msg import PersonDetection, SmartHandleState
try:        # 질문 기록(2026-11-17). 옛 vica_interfaces 설치본이면 발행만 건너뛴다 — 미션은 떠야 한다
    from vica_interfaces.msg import VicaQuestion
except ImportError:
    VicaQuestion = None
from vica_interfaces.srv import (
    DeleteHome,
    GetHome,
    MissionCommand,
    RequestApproach,
    RequestDestination,
    SaveHome,
)

try:
    from vica_interfaces.srv import GetPatrolRoute, SavePatrolRoute
except ImportError:  # vica_interfaces 를 다시 빌드하기 전 — 미션은 뜨고 순찰 경로 서비스만 빠진다
    GetPatrolRoute = SavePatrolRoute = None

from .approach_speed import DEFAULT_APPROACH_STAGES, stages_from_lists
from .destinations import load_destinations, load_home, load_map_bounds
from .approach_geometry import approach_goal
from .dock_flow import (CALL_CANCEL, CALL_START, CALL_UNDOCK, DOCK_FAR_COV, DOCK_FAR_M, DockFlow,
                        classify_start, parse_node_state, staging_relation)
from .dock_spot import DOCK_DESTINATION_ID, dock_yaml_path, read_dock_spot, staging_destination
from .home_storage import HOME_UNREACHABLE_MESSAGE, HomeStorage, build_home, near_home
from .ledger import (COV_UNKNOWN, Ledger, LedgerStore, apply_goal_event, confirm_abort_name,
                     place_here, state_fields)
from .map_meta import load_map_meta
from .mission_logic import (
    APPROACH_BT_FILE,
    APPROACH_TURN_YAW_DEG,
    ApproachRequest,
    CancelNav,
    DEST_RETRY_RETURN_SEC,
    DIALOG_PAUSED_HANDLE,
    Destination,
    GRIP_ENTER_WINDOW_SEC,
    GRIP_HINT_PULSE_SEC,
    GRIP_RATIO,
    GRIP_RELEASE_GRACE_SEC,
    GRIP_RESUME_WINDOW_SEC,
    GRIP_WAIT_TIMEOUT_SEC,
    GUIDED_BT_FILE,
    GUIDED_NO_RAIL_BT_FILE,
    GateReason,
    GoalEvent,
    HANDLE_LOST_GIVE_UP_SEC,
    HANDLE_LOST_REPEAT_SEC,
    HANDLE_SIDE_MIN_YAW_RAD,
    HANDLE_STATE_STALE_SEC,
    HOME_BEACON_INTERVAL_SEC,
    Haptic,
    HelpEvent,
    IntentData,
    MSG_APPROACH_ONBOARDING,
    MSG_APPROACH_REASK,
    MSG_DEST_RETRY,
    MSG_DOCK_GO,
    MissionLogic,
    NAV_TREE_GUIDED,
    NAV_TREE_WAIT,
    NEAR_CALL_MAX_M,
    NEAR_CALL_NO_SPIN_M,
    NavStatus,
    Navigate,
    PATROL_ALARM_KINDS,
    PATROL_ALARM_RESEND_SEC,
    PATROL_ARMING_TIMEOUT_SEC,
    PATROL_DESTINATION_PREFIX,
    PATROL_DETECTOR_TIMEOUT_SEC,
    PATROL_DWELL_SEC,
    PATROL_INTRUDER_REPEAT_SEC,
    PATROL_LIGHT_REPEAT_SEC,
    PATROL_MAX_LEG_FAILURES,
    PATROL_REJECT_TEXT,
    PATROL_SPEED_MPS,
    PATROL_START_REJECT_TEXT,
    PATROL_WATCH_KINDS,
    PERSON_APPROACH_SPEED_PERCENT,
    PatrolEvent,
    PatrolRecordClip,
    Pose2D,
    RETURN_RESUME_SEC,
    Say,
    SetNavSpeedCap,
    SetNavSpeedLimit,
    SpinInPlace,
    State,
    StopSpeech,
    WAIT_SPOT_BT_FILE,
    WakeReply,
    check_gate,
    doa_to_spin_yaw,
    nav_behavior_tree,
    yaw_deg_to_quaternion,
)
from .question_record import QuestionRecord, QuestionTracker, answer_for
from .obstacle_inputs import (
    BASE_FRAME,
    LASER_DEFAULT,
    Guard,
    OdomTrail,
    depth_frame_ok,
    format_decision,
    load_grid,
    scan_points,
    take_all,
    yaw_of,
)
from .obstacle_judge import ObstacleJudge, cm_what, goal_event, parse_state
from .patrol_storage import (
    ROUTE_IO_ERROR,
    ROUTE_NOTHING_WATCHED,
    ROUTE_OK,
    ROUTE_TOO_FEW,
    ROUTE_UNKNOWN_MAP,
    PatrolEventLog,
    PatrolRouteStorage,
    alarm_record,
    check_route_ids,
    clean_watch_off,
    clip_record,
    nearest_place_name,
)

#: 경보 id 모양(YYYYMMDD_HHMMSS_<kind>). patrol_ack 의 request_id 가 이 모양이면 그 경보만 확인한다.
PATROL_ALARM_ID_PATTERN = re.compile(r"^\d{8}_\d{6}_[a-z_]+$")

#: 트리가 바뀌는 goal 을 보내기 전, 앞 task 의 취소가 끝나기를 기다리는 상한(초).
#: _nav_lock_timeout_sec(2.0) 과 같은 크기 — 콜백을 오래 붙잡지 않는다.
BT_SWITCH_WAIT_SEC = 2.0

#: 주행 중 매 틱(_tick)마다 "Nav2 주행이 끝났나"를 물을 때 Nav2 소식을 기다리는 시간(초).
#: nav2_simple_commander 의 isTaskComplete 기본 0.1 초는 쉬는 시간 없이 도는 예제 반복문의 쉼 겸
#: 우편함 확인이다. 미션은 자기 틱이 있어 그 위에 0.1 초가 겹쳤고, 그동안 오는 진행 소식
#: (bt_loop_duration 10 ms → 최대 100 Hz)을 다 처리하느라 CPU 를 쓰고 _main_group 의 음성·앱 요청이
#: 줄을 섰다. 결과는 기다리지 않아도 대기열에 남아 다음 틱에 읽힌다. 0 이면 남은 거리(getFeedback)가
#: 최대 3 초 묵는다. 2026-10-13 격리 벤치(진행 소식 100 Hz): 0.1 초 CPU 19.8 %·0.02 초 10.7 %·
#: 0.05 초 14.1 %, 남은 거리는 0.02 초 이상이면 신선(중앙 6~8 ms). 사용자 결정 0.03 초.
#: 트리 전환 기다림·출발 전 확인은 라이브러리 기본 그대로다(test_nav_poll_wait.py).
NAV_POLL_WAIT_SEC = 0.03

#: 출발 전 costmap(local·global) 비우기의 응답을 기다리는 상한(초). 두 요청을 함께 보내고
#: 이 시한 하나로 함께 기다린다. 실측은 local 이 BT 한 박자(0.01 s) 안(run81·82 10회),
#: global 은 요청 뒤 0.03 s 안에 지우기를 마쳤다(10-10 planner_server 로그). 넘으면 그대로 출발한다.
CLEAR_BEFORE_DEPARTURE_TIMEOUT_SEC = 1.0

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

if BasicNavigator is not None:
    class MissionNavigator(BasicNavigator):
        """isTaskComplete 의 기다림만 넘겨받을 수 있게 한 BasicNavigator.

        판정은 라이브러리 원본(robot_navigator.py:278)과 같고, 기본값도 원본과 같은 0.1 초다.
        미션은 주행 중 확인(_poll_nav_status)에서만 NAV_POLL_WAIT_SEC 를 넘긴다.
        """

        def isTaskComplete(self, timeout_sec: float = 0.10) -> bool:  # noqa: N802 - 원본 이름
            if not self.result_future:
                return True  # 취소됐거나 끝난 task
            rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=timeout_sec)
            result = self.result_future.result()
            if not result:
                return False  # 아직 도는 중
            self.status = result.status
            return True
else:
    MissionNavigator = None


class MissionManagerNode(Node):
    """LLM intent를 안전 게이트로 심사하고 승인된 Nav2 작업만 실행한다.

    판단은 테스트하기 쉬운 :class:`MissionLogic`에 맡기고, 이 클래스는 ROS2
    메시지 변환·토픽 연결·Nav2 commander 호출 같은 배선만 담당한다.
    """

    # 도킹(2026-11-18). 시험이 __init__ 없이 노드를 만들 때를 위해 클래스 기본값을 둔다.
    _dock: Optional[DockFlow] = None

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
        # 기본 175(2026-10-10) — Spin 이 6~8° 더 도는 몫을 뺀 값. 근거는 mission_logic.APPROACH_TURN_YAW_DEG.
        self.declare_parameter("approach_turn_yaw_deg", APPROACH_TURN_YAW_DEG)
        # 사람 접근 goal 에만 쓰는 Nav2 행동 트리(레일 없는 자유주행). 2026-10-02 run60.
        #   "auto" : vica_nav2 share 의 behavior_trees/vica_navigate_to_pose_approach.xml
        #   ""     : 끔 — 접근도 기본 트리(레일)를 쓴다(종전 동작)
        #   경로   : 그 파일. 없으면 경고를 남기고 끔과 같게 동작한다.
        self.declare_parameter("approach_bt_xml", "auto")
        # 사용자 안내 도착 트리(레일 + 위치만 판정 — 방향 정렬 없음, 2026-10-07).
        # 대기 장소·목적지 복귀·관리자 가보기 트리(레일 없음 + 위치·방향, 2026-10-07).
        # 값의 규칙은 approach_bt_xml 과 같다("auto"/""/경로). 못 찾으면 기본 트리.
        self.declare_parameter("guided_bt_xml", "auto")
        # 레일(route_server)이 없을 때 쓰는 안내 트리. 레일 안내 트리는 ComputeRoute 를 불러
        # route_server 가 없으면 만들 때부터 실패한다. 규칙은 guided_bt_xml 과 같다.
        self.declare_parameter("guided_no_rail_bt_xml", "auto")
        self.declare_parameter("wait_spot_bt_xml", "auto")
        # 마이크 각도 증가 방향. +1 반시계 / -1 시계 — 장비 실측값이다
        # (호출 접근 설계 §5). 틀리면 로봇이 호출 방향의 정반대로 돈다.
        self.declare_parameter("wake_doa_sign", 1.0)
        # 고개를 돌린 뒤 사람을 찾는 시간(초). 8.0 근거는
        # mission_logic.SEEK_LOOK_SEC 주석(2026-09-10 재검토).
        self.declare_parameter("seek_look_sec", 8.0)
        # 근접 호출(2026-09-10 확장). 부른 사람이 이보다 가까우면 접근 goal
        # (1.1 m)이 이미 지나간 자리라 걸어가지 않고 그 자리에서 바로 질문한다.
        # detection_gate.DEFAULT_MIN_DISTANCE_M 과 값은 같지만(1.5) 별개
        # 파라미터다 — Mission 은 그 감지기 상수를 알 수 없다(패키지 경계).
        self.declare_parameter("near_call_max_m", NEAR_CALL_MAX_M)
        # 이보다 가까우면 수락해도 회전하지 않는다 — 손잡이가 뒤로 길게 나와
        # 있어 이 거리의 180도 회전은 손잡이가 사람을 칠 수 있다
        # (mission_logic.NEAR_CALL_NO_SPIN_M 주석, 2026-09-10 사용자 결정).
        self.declare_parameter("near_call_no_spin_m", NEAR_CALL_NO_SPIN_M)
        # 핸들 쪽(로봇 뒤) 호출 사각지대(도) — 위 정면 사각지대(SEEK_MIN_YAW_RAD,
        # 10°)의 거울쌍이다. 회전량이 이보다 크면(부채꼴 180°±45°) 소리가
        # 핸들 옆에서 왔다는 뜻이라 카메라 확인 없이 곧바로 접근 질문을 낸다
        # (mission_logic.HANDLE_SIDE_MIN_YAW_RAD 주석, 2026-09-10 사용자 결정).
        self.declare_parameter(
            "handle_side_min_yaw_deg", math.degrees(HANDLE_SIDE_MIN_YAW_RAD))
        # 홈 복귀 중 호출로 브레이크가 걸린 뒤 이만큼 침묵하면 떠나기 예고를
        # 내고(MSG_LEAVING_NOTICE 재사용) LEAVING_GRACE_SEC 뒤 복귀를 재개한다
        # (2026-09-10 사용자 승인 흐름). 기준은 브레이크가 걸린 시각 —
        # 청취 창(음성 쪽) 길이와는 무관하다.
        self.declare_parameter("return_resume_sec", RETURN_RESUME_SEC)
        # 온보딩("이제 어디로 가고 싶으신가요?") 뒤 STT 가 빈손으로 닫히면
        # 한 번 되묻고, 그래도 빈손이면 이만큼 더 기다렸다 떠남을 예고한다
        # (실기 2026-09-11). return_resume_sec 과 값·뜻이 같다 — 근거는
        # mission_logic.DEST_RETRY_RETURN_SEC 주석.
        self.declare_parameter("dest_retry_return_sec", DEST_RETRY_RETURN_SEC)
        # 사람에게 다가가는 구간의 최대속도(주행 상한의 %). 기본 60 % = 0.3 m/s.
        # 등록 목적지 주행과 달리 감속 사다리를 쓰지 않고 처음부터 끝까지 이 값이다.
        # 2026-09-09 실측: 7.77 m 접근에 19.6 초로, 이 값이 그 시간의 주범이다
        # (knob 35 % 도 Nav2 요청도 아니었다 — 0.5 x 60 % = 0.30 이 그대로 나갔다).
        # 올리면 빨라지지만 **사람에게 다가오는 속도**라 위협감을 함께 봐야 한다.
        # 도착 직전 1.1 m 는 collision_monitor 가 0.12 m/s 로 따로 줄이므로 그
        # 구간(약 9초)은 이 값과 무관하다.
        self.declare_parameter(
            "person_approach_speed_percent", PERSON_APPROACH_SPEED_PERCENT)
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
        # 손잡이 터치 × 진동 (2026-09-30). 뜻과 근거는 mission_logic 의 같은
        # 이름 상수 주석(GRIP_* · HANDLE_*)에 있다. 정본 설계:
        # docs/superpowers/specs/2026-09-28-touch-haptic-integration-final.md 5절.
        self.declare_parameter("grip_enter_window_sec", GRIP_ENTER_WINDOW_SEC)
        self.declare_parameter("grip_ratio", GRIP_RATIO)
        self.declare_parameter("grip_wait_timeout_sec", GRIP_WAIT_TIMEOUT_SEC)
        # 시연 스위치(2026-10-05): 잡기 대기 시간이 다 되면 잡은 것으로 넘어가고
        # 손 놓침 정지를 쓰지 않는다. 터치 모듈 고장 중 시연용.
        self.declare_parameter("grip_assume_held", False)
        # 주행 중 손 놓침 정지(2026-10-21 사용자 결정으로 기본 꺼짐) — 터치는 처음 잡기에만 쓴다.
        self.declare_parameter("grip_release_stop", False)
        # 대기 중 본인 확인(2026-11-11 설계 speaker-follow). 음성 VICA_SPK_FOLLOW=on 과 함께 켠다.
        self.declare_parameter("speaker_gate", False)
        self.declare_parameter("spk_joined_threshold", 0.50)
        self.declare_parameter("speaker_gate_grip", False)   # 대기 중 명령에 손잡이 접촉도 요구(11-14 기본 꺼짐)
        # 세 단계 되묻기(원칙 8 · 31, 2026-11-21). 켜면 다시 말씀 → 더 크게 또박또박 → (주행 중) 직원 도움. 기본 꺼짐 = 지금 그대로.
        self.declare_parameter("reask_ladder", False)
        # "비카야" 뒤 처리(원칙 13 · 3, 2026-11-21). 켜면 호출이 취소·도움 확인도 접고, 40초 안의 "응"엔 그 질문을 다시 한다. 기본 꺼짐 = 지금 그대로.
        self.declare_parameter("wake_revive", False)
        # 대답 칸으로 처리(원칙 6 · 3, 2026-11-21). 켜면 열린 질문의 대답 칸(yes·no·목적지·분·못 알아들음)을 의도보다 먼저 본다. 기본 꺼짐 = 지금 그대로.
        self.declare_parameter("answer_field", False)
        # 홈 알림(2026-10-08): 홈에서 쉬는 동안 이 간격(초)마다 M3 를 말한다. 0 이면 끈다.
        self.declare_parameter("home_beacon_interval_sec", HOME_BEACON_INTERVAL_SEC)
        # 주행 중 장애물 안내(2026-10-09, 설계서 2026-10-08-obstacle-narration-design.md 5절 2단계).
        # false 면 장애물 입력을 아예 구독하지 않는다.
        self.declare_parameter("obstacle_narration", True)
        self.declare_parameter("grip_hint_pulse_sec", GRIP_HINT_PULSE_SEC)
        self.declare_parameter("grip_release_grace_sec", GRIP_RELEASE_GRACE_SEC)
        self.declare_parameter("grip_resume_window_sec", GRIP_RESUME_WINDOW_SEC)
        self.declare_parameter("handle_lost_repeat_sec", HANDLE_LOST_REPEAT_SEC)
        self.declare_parameter("handle_lost_give_up_sec", HANDLE_LOST_GIVE_UP_SEC)
        self.declare_parameter("handle_state_stale_sec", HANDLE_STATE_STALE_SEC)
        # 경비(순찰) 모드(2026-11-14, ~/vica_data/patrol_plan/SPEC.md 4.A). 뜻은 mission_logic 의 PATROL_* 주석.
        self.declare_parameter("patrol_dwell_sec", PATROL_DWELL_SEC)
        self.declare_parameter("patrol_intruder_repeat_sec", PATROL_INTRUDER_REPEAT_SEC)
        self.declare_parameter("patrol_light_repeat_sec", PATROL_LIGHT_REPEAT_SEC)
        self.declare_parameter("patrol_alarm_resend_sec", PATROL_ALARM_RESEND_SEC)
        self.declare_parameter("patrol_detector_timeout_sec", PATROL_DETECTOR_TIMEOUT_SEC)
        self.declare_parameter("patrol_arming_timeout_sec", PATROL_ARMING_TIMEOUT_SEC)
        self.declare_parameter("patrol_speed_mps", PATROL_SPEED_MPS)
        self.declare_parameter("patrol_max_leg_failures", PATROL_MAX_LEG_FAILURES)
        # 경보 기록(events.jsonl)을 두는 폴더. 감지 노드의 영상·사진과 같은 폴더다(컨테이너 안에서는 /workspaces/…).
        self.declare_parameter("patrol_media_dir", "~/workspaces/isaac_ros-dev/vica_patrol")

        dest_path = str(self.get_parameter("destinations_yaml").value)
        if not dest_path:
            raise RuntimeError("destinations_yaml parameter 가 비어 있습니다 (launch 에서 절대경로 지정)")
        self._destinations_path = str(Path(dest_path).expanduser())
        configured_map_id = str(self.get_parameter("map_id").value).strip()
        self._map_id = configured_map_id or Path(self._destinations_path).parent.name
        self.destinations = load_destinations(self._destinations_path)

        # ---- 로봇 대장 (P1, 스펙 3절) — 사실만 적고 방송한다 ---------------------
        self._map_meta = load_map_meta(self._destinations_path)
        self._ledger_store = LedgerStore(self._destinations_path)
        self._ledger: Ledger = self._ledger_store.read()
        self._ledger_prev_confirming: Optional[str] = None   # 확인 대기 → 안 감 전이 감지용
        self._pose_cov_xy = 0.0                               # AMCL x·y 분산 합(초기 위치 전 판정)
        self.get_logger().info(
            f"대장: 건물 {self._map_meta.building or '?'} {self._map_meta.floor}층, "
            f"직전 도착 {self._ledger.last_destination or '-'}, 파일 {self._ledger_store.path}")

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
        # 아직 '가 봄'이 아니거나 접근 실패가 남은 홈의 (x, y). 위치를 받을 때 홈 30 cm 안인지 본다.
        # 이미 가 봤고 막힘도 없으면 None — 그때는 위치마다 아무것도 하지 않는다(11-19).
        self._home_watch: Optional[tuple[float, float]] = None

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
            wake_doa_sign=float(self.get_parameter("wake_doa_sign").value),
            seek_look_sec=float(self.get_parameter("seek_look_sec").value),
            near_call_max_m=float(self.get_parameter("near_call_max_m").value),
            near_call_no_spin_m=float(
                self.get_parameter("near_call_no_spin_m").value),
            return_resume_sec=float(self.get_parameter("return_resume_sec").value),
            handle_side_min_yaw_rad=math.radians(
                float(self.get_parameter("handle_side_min_yaw_deg").value)),
            dest_retry_return_sec=float(
                self.get_parameter("dest_retry_return_sec").value),
            estop_release_grace_sec=float(self.get_parameter("estop_release_grace_sec").value),
            approach_stages=approach_stages,
            nav_retry_limit=retry_limit,
            nav_retry_delay_sec=retry_delay,
            arrival_dialog=arrival_dialog,
            # 홈 좌표는 목적지 폴더의 home.yaml 하나다. 앱이 HomeStorage 로
            # 쓰고 노드가 읽는다 — 두 구현이 같은 형식을 쓴다.
            return_destination=home,
            auto_return_home=bool(self.get_parameter("auto_return_home").value),
            person_approach_speed_percent=float(
                self.get_parameter("person_approach_speed_percent").value),
            **{name: float(self.get_parameter(name).value) for name in (
                "grip_enter_window_sec", "grip_ratio", "grip_wait_timeout_sec",
                "grip_hint_pulse_sec", "grip_release_grace_sec",
                "grip_resume_window_sec", "handle_lost_repeat_sec",
                "handle_lost_give_up_sec", "handle_state_stale_sec")},
            grip_assume_held=bool(self.get_parameter("grip_assume_held").value),
            grip_release_stop=bool(self.get_parameter("grip_release_stop").value),
            speaker_gate=bool(self.get_parameter("speaker_gate").value),
            spk_joined_threshold=float(self.get_parameter("spk_joined_threshold").value),
            speaker_gate_grip=bool(self.get_parameter("speaker_gate_grip").value),
            reask_ladder=bool(self.get_parameter("reask_ladder").value),
            wake_revive=bool(self.get_parameter("wake_revive").value),
            answer_field=bool(self.get_parameter("answer_field").value),
            home_beacon_interval_sec=float(
                self.get_parameter("home_beacon_interval_sec").value),
            **{name: float(self.get_parameter(name).value) for name in (
                "patrol_dwell_sec", "patrol_intruder_repeat_sec", "patrol_light_repeat_sec",
                "patrol_alarm_resend_sec", "patrol_detector_timeout_sec",
                "patrol_arming_timeout_sec", "patrol_speed_mps")},
            patrol_max_leg_failures=int(self.get_parameter("patrol_max_leg_failures").value),
        )
        # 순찰 경로는 목적지와 같은 폴더의 patrol.yaml, 경보 기록은 patrol_media_dir 의 events.jsonl.
        self._patrol_routes = PatrolRouteStorage(Path(self._destinations_path).parent.parent)
        self._patrol_log = PatrolEventLog(str(self.get_parameter("patrol_media_dir").value))
        self.get_logger().info(
            "손잡이: 잡기 "
            f"{self.logic.grip_enter_window_sec:.1f}s 중 {self.logic.grip_ratio:.0%} · "
            f"대기 {self.logic.grip_wait_timeout_sec:.0f}s · "
            + (f"놓침 {self.logic.grip_release_grace_sec:.2f}s 뒤 정지" if self.logic.grip_release_stop
               else "주행 중 놓침 정지 꺼짐(터치는 처음 잡기만)"))
        if self.logic.grip_assume_held:
            self.get_logger().warn(
                f"손잡이 시연 스위치 켜짐: 대기 {self.logic.grip_wait_timeout_sec:.0f}s 뒤 "
                "잡은 것으로 넘어가고 손 놓침 정지를 쓰지 않는다(grip_assume_held)")
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
        self.navigator = MissionNavigator("vica_mission_navigator")
        self._nav_lock = threading.Lock()
        # _cancel_nav_blocking 이 이 lock 을 잡고 Nav2 응답을 기다릴 수 있다.
        # 콜백에서 무한히 기다리면 고치려던 문제가 그대로 돌아오므로 시한을 둔다.
        self._nav_lock_timeout_sec = 2.0
        self._nav_active = False  # goToPose 수락 후 완료 전까지 True
        # 사람 접근 전용 트리 파일(빈 문자열 = 끔). _task_bt 는 마지막으로 수락된
        # NavigateToPose 가 쓴 트리다("" = 기본 트리). 트리가 바뀌는 goal 은 Humble
        # bt_navigator 가 선점을 거절하므로 _start_nav 가 앞 task 를 먼저 끝낸다.
        self._approach_bt = self._resolve_bt(
            str(self.get_parameter("approach_bt_xml").value).strip(),
            APPROACH_BT_FILE, "사람 접근 트리(레일 없이 자유주행)")
        # Navigate.tree 종류 → 트리 파일(2026-10-07). 비어 있으면 기본 트리로 간다.
        self._tree_files = {
            NAV_TREE_GUIDED: self._resolve_bt(
                str(self.get_parameter("guided_bt_xml").value).strip(),
                GUIDED_BT_FILE, "사용자 안내 트리(위치만 판정)"),
            NAV_TREE_WAIT: self._resolve_bt(
                str(self.get_parameter("wait_spot_bt_xml").value).strip(),
                WAIT_SPOT_BT_FILE, "대기 장소 트리(레일 없음, 위치·방향)"),
        }
        self._guided_no_rail_bt = self._resolve_bt(
            str(self.get_parameter("guided_no_rail_bt_xml").value).strip(),
            GUIDED_NO_RAIL_BT_FILE, "사용자 안내 트리(레일 없는 지도, 위치만 판정)")
        self._task_bt = ""
        # 주행 번호표. goToPose/spin 이 수락될 때마다 1 오른다. 취소 스레드는
        # 자기가 받은 번호와 지금 번호가 다르면 취소를 건너뛴다 — 번호가
        # 달라졌다는 것은 그 사이 새 goal 이 나갔다는 뜻이고(앱 전권화의
        # [취소 -> 즉시 출발] 경로), 그때 cancelTask() 를 부르면 옛 goal 이
        # 아니라 방금 보낸 새 goal 이 취소된다. 옛 goal 은 Nav2 가 새 goal 을
        # 받으면서 스스로 선점해 이미 내려가 있다.
        self._nav_gen = 0
        # 출발 전 costmap 비우기(2026-10-09 run82 뒤 사용자 결정: 직전 안내가 다 끝나고 새로
        # 출발할 때마다 local, 10-10 run84 뒤 global 도). 앞 NavigateToPose 가 성공으로 끝났을 때만 True 가
        # 되고, 새 goal 이 수락되면 False 로 돌아간다. 그래서 일시정지 뒤 재개·실패 뒤
        # 재시도·주행 중 바꾸기(앞 goal 이 성공으로 끝나지 않았다)는 비우지 않는다.
        # 제자리 회전(Spin) 결과는 건드리지 않는다 — 접근 뒤 손잡이 돌리기가 사람에 막혀
        # 실패해도(run82 18:13:41) 다음 출발은 비운다. 켤 때의 첫 출발도 비운다.
        self._clear_before_next_nav = True
        self._nav_task_is_spin = False   # 지금(마지막으로 수락된) task 가 Spin 인가
        # 배송 도착 표시(2026-10-10 사용자 결정, 웹 배송 문자 A안). 배송 요청으로 시작한 주행의
        # 목적지 id. 그곳 goal_succeeded 에 "delivery": true 를 실어, 누가 배송을 보냈든(웹 포함)
        # 관리자 유심 폰 앱이 도착 문자를 보내게 한다. 도착·실패·취소·거부나 다른 곳으로 출발하면
        # 지운다 — 남겨 두면 나중에 같은 곳으로 간 안내 주행에 배송 문자가 나간다. 일시정지는 남긴다.
        self._delivery_dest_id: Optional[str] = None

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
        # 몸통 넘어짐(2026-10-15)과 앱 원인. 둘 다 /emergency_stop 보다 **먼저** 만든다 —
        # 같은 콜백 그룹에서 동시에 준비된 콜백은 만든 순서대로 처리된다. 두 신호는
        # 래치보다 먼저 오므로, 그 순서가 지켜지면 넘어짐은 넘어짐 멘트 하나만,
        # 앱 버튼은 "관리자를 호출했습니다" 없는 멘트가 나간다(mission_logic.on_tipped).
        # /vica/tipped 는 바뀔 때만 오므로 transient_local 로 받는다(재기동해도 지금 상태).
        self.create_subscription(
            Bool,
            "/vica/tipped",
            self._on_tipped,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self._emergency_group,
        )
        # 앱 원인은 /app_estop_state(1 Hz JSON)의 app_active 로 받는다. 앱 버튼을 누른 순간
        # app_emergency_node 가 1 Hz 를 기다리지 않고 한 번 더 낸다. 10 Hz /app_emergency_stop 을
        # 받던 판은 서 있을 때 CPU 를 약 3 %p 더 썼다(run96, 2026-10-15).
        self.create_subscription(
            String,
            "/app_estop_state",
            self._on_app_estop_state,
            10,
            callback_group=self._emergency_group,
        )
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

        # 웨이크워드 호출("wake"/창 안 구제 "rescue"). 2026-10-07 부터 "네?"와 대답 여부
        # (호출 반응표)를 여기서 정한다 — 같은 주 그룹이라 주행 시작(_start_nav)이 길게
        # 붙잡으면 "네?"도 늦는다(최악 약 4초, 음성 쪽 판정 대기 시한의 근거).
        self.create_subscription(
            String, "/vica/wake", self._on_wake, 10,
            callback_group=self._main_group,
        )
        # 호출이 온 방향. 대기 중에만 받아 그쪽으로 고개를 돌린다
        # (호출 접근 설계). 같은 콜백 그룹이라 /vica/wake 가 먼저 처리되는데,
        # 그 처리 직후(WAKE_CONSUMED_GUARD_SEC 이내)의 이 토픽은 on_wake_doa
        # 가 시각으로 거절한다 — 두 토픽의 도착 순서와 무관하게 새 호출만
        # SEEKING 을 연다(2026-09-10 재현).
        self.create_subscription(
            Float32, "/vica/wake_doa", self._on_wake_doa, 10,
            callback_group=self._main_group,
        )
        # 근접 호출(2026-09-10 확장). detection_gate 가 거리 하나만으로
        # TOO_NEAR 거절한 원본 결과를 여기서 직접 본다 — RequestApproach
        # service 는 person_detector_node 가 approachable=true 일 때만 부르므로
        # TOO_NEAR 는 그 service 로 오지 않는다. mission_logic.on_person_detection
        # 이 탐색 창(IDLE + _seek_deadline) 밖에서는 아무 일도 하지 않는다.
        self.create_subscription(
            PersonDetection, "/vica/person_detection", self._on_person_detection, 10,
            callback_group=self._main_group,
        )
        # 청취 상태 — 무응답 시계를 귀가 바쁜 동안 멈춘다 (mission_logic
        # on_listen_state 주석, 2026-08-30). 빈손(empty:*)은 온보딩 되묻기
        # 사다리도 전진시키므로(2026-09-11) 반환값을 버리지 않고 실행한다.
        self.create_subscription(
            String, "/vica/listen_state", self._on_listen_state, 10,
            callback_group=self._main_group,
        )
        # LLM 이 사용자 말을 판단하는 중(2026-10-10 사용자 결정) — 켜진 동안은 대답 대기 시계를 잡는다.
        # 원래 TTS 의 '생각 중' 운율용 신호(음성 LLM 노드가 Realtime·글자 경로 호출 전후로 켜고 끈다).
        self.create_subscription(
            Bool, "/vica/thinking",
            lambda msg: self.logic.on_llm_thinking(msg.data, self._now()),
            10,
            callback_group=self._main_group,
        )

        # 손잡이 터치(2026-09-30). 드라이버가 2 Hz 주기 + 바뀐 순간 즉시 낸다.
        # 사실만 적고 판정(잡기·놓침)은 _tick → logic.on_tick 이 한다.
        self.create_subscription(
            SmartHandleState, "/vica/smart_handle_state",
            lambda msg: self.logic.on_handle_state(
                msg.user_contact, msg.uplink_fresh, self._now()),
            10,
            callback_group=self._main_group,
        )
        # 화자 판정 모드(2026-11-11) — 음성 .env 의 VICA_SPK_FOLLOW 하나로 미션 대기 중 본인 확인까지 켠다.
        # 마지막 값을 보관하는 토픽이라 어느 쪽이 먼저 떠도 받는다.
        self.create_subscription(
            String, "/vica/speaker_mode", self._on_speaker_mode,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
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
        # 질문 기록(설계 2026-11-14 4.1) — 대답을 기다리는 말마다 낸다. 늦게 뜬 음성 노드도 마지막 기록을 받는다.
        self._questions = QuestionTracker(prefix=f"q{int(time.time())}")
        self._question_lock = threading.Lock()
        self.pub_question = None
        if VicaQuestion is not None:
            self.pub_question = self.create_publisher(
                VicaQuestion, "/vica/question",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
        else:
            self.get_logger().warn("질문 기록 없음(옛 vica_interfaces) — /vica/question 발행 안 함")
        self._publish_question(None)       # 미션만 다시 떴을 때 음성이 들고 있는 옛 질문을 지운다
        # "비카야" 판정 (2026-10-07, 호출 반응표). "listen"/"ignore" — 웨이크워드 노드가
        # 호출 창에서 들은 말을 쥐고 기다리다 listen 이면 LLM 으로 넘기고 ignore 면 버린다.
        # 이 노드가 멈추면 판정이 안 가고, 음성 쪽이 시간을 넘겨 버린다(대답 없음).
        self.pub_wake_reply = self.create_publisher(String, "/vica/wake_reply", 10)
        self._wake_reply_seq = ""   # 지금 판정하는 호출의 번호(_on_wake 가 채운다)
        # 손잡이 진동 요청 (2026-09-10, 2026-09-30 확장). "long"(손잡이 찾기)·
        # "tick"(잡음 확인)을 낸다. user_guidance_driver_node 가 구독해 바이트로
        # 바꾼다. 도착·비상 진동은 여기서 내지 않는다 — 펌웨어가 상태 진입 때 낸다.
        self.pub_haptic = self.create_publisher(String, "/vica/haptic_request", 10)
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
        # 관리자 '대기 장소로 가보기'(2026-10-07). 등록 확인용 이동이다 — 지도 설정 화면만
        # 부른다. 요청 모양은 목적지 요청과 같고(destination_id = 대기 장소를 가진 목적지),
        # 음성/LLM 은 서비스 클라이언트가 없어 이 문에 닿지 못한다(홈 복귀와 같은 논리).
        self.create_service(
            RequestDestination,
            "/vica/mission/request_wait_spot",
            self._on_wait_spot_request,
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
        # 관리자 호출(도움 요청, 2026-10-15 설계): 앱 팝업의 "확인". request_id 가 확인할 help_id 다.
        self.create_service(
            MissionCommand,
            "/vica/mission/help_ack",
            self._on_help_ack,
            callback_group=self._main_group,
        )
        # ── 경비(순찰) 모드 (2026-11-14, SPEC 3.1·3.2) ─────────────────────────
        # 감지 노드·호출어 노드가 순찰 중에만 내는 사건(JSON). 순찰로 늘어난 구독은 이것 하나다.
        self.create_subscription(
            String, "/vica/patrol/event", self._on_patrol_event, 10,
            callback_group=self._main_group,
        )
        # 관리자가 꺼 둔 감시 항목(2026-11-14 사용자 요청) — 감지·호출어 노드가 꺼진 항목은 계산도 하지
        # 않게 알린다. 기동·저장(순찰 중 제외)·경비 시작 때만 내고, 마지막 값 보관이라 늦게 뜬 노드도 받는다.
        # 못 받은 노드는 모두 켬으로 돈다(경보를 버리는 판단은 on_patrol_event 가 한다).
        self._pub_patrol_watch = self.create_publisher(
            String, "/vica/patrol/watch",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._publish_patrol_watch()
        # 시작·종료·경보 확인. 일시정지·재개는 기존 pause_navigation·resume_navigation 을 같이 쓴다.
        # patrol_records_clear(2026-11-16)는 경비 기록·영상·사진을 모두 지운다 — 앱 '경비 기록' 삭제.
        for name, handler in (("patrol_start", self._on_patrol_start),
                              ("patrol_stop", self._on_patrol_stop),
                              ("patrol_ack", self._on_patrol_ack),
                              ("patrol_records_clear", self._on_patrol_records_clear)):
            self.create_service(
                MissionCommand, f"/vica/mission/{name}", handler,
                callback_group=self._main_group,
            )
        # ── 도킹 (2026-11-18) ────────────────────────────────────────────
        # 실제로 들어가고 나오는 것은 vica_docking 노드다. 미션은 앱 요청(도킹 시작·나오기·중지)을
        # 받아 시키고, 도크 안에서 원격 주행이 오면 그 주행을 잡아 두고 먼저 나온다(dock_flow.py).
        # 도킹 노드 서비스는 main 그룹에서 비동기로 부른다 — 응답도 main 그룹에서 받아 다른 콜백과 겹치지 않는다.
        self._dock = DockFlow()
        self._dock_lock = threading.RLock()
        self._dock_far_check_at = 0.0
        self._dock_clients = {
            name: self.create_client(Trigger, f"/vica/dock/{name}", callback_group=self._main_group)
            for name in (CALL_START, CALL_UNDOCK, CALL_CANCEL)
        }
        self.create_subscription(
            String, "/vica/dock/state", self._on_dock_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self._main_group,
        )
        for name, handler in (("dock_start", self._on_dock_start),
                              ("dock_undock", self._on_dock_undock),
                              ("dock_cancel", self._on_dock_cancel)):
            self.create_service(
                MissionCommand, f"/vica/mission/{name}", handler,
                callback_group=self._main_group,
            )
        if SavePatrolRoute is not None:
            self.create_service(
                SavePatrolRoute, "/vica/patrol/route/save", self._on_save_patrol_route,
                callback_group=self._main_group,
            )
            self.create_service(
                GetPatrolRoute, "/vica/patrol/route/get", self._on_get_patrol_route,
                callback_group=self._main_group,
            )
        else:
            self.get_logger().warn(
                "순찰 경로 서비스 없음 — vica_interfaces 를 다시 빌드해야 한다(SavePatrolRoute·GetPatrolRoute)")
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

        # ---- 주행 중 장애물 안내 (2026-10-09) — 입력은 전용 줄, 말할지는 _tick 이 정한다 ----------
        self._obstacle: Optional[ObstacleJudge] = None
        self._obstacle_guard = Guard(self._on_obstacle_error)
        self._obstacle_cues: deque = deque(maxlen=8)
        if bool(self.get_parameter("obstacle_narration").value):
            self._setup_obstacle_narration(map_yaml)
        else:
            self.get_logger().info("장애물 안내: 꺼짐(obstacle_narration=false)")

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

    def _on_speaker_mode(self, msg: String) -> None:
        before = self.logic.speaker_gate
        self.logic.set_speaker_mode(msg.data)
        if self.logic.speaker_gate != before:
            self.get_logger().info(f"[화자] 음성 모드 {msg.data!r} — 대기 중 본인 확인 "
                                   + ("켬" if self.logic.speaker_gate else "끔"))

    def _on_intent(self, msg: VicaIntent) -> None:
        """음성 요청. 판단은 로직(on_voice_intent)이 하고 여기선 실행·기록만 한다(2026-10-08)."""
        # 대답 칸(3.2) — 열린 질문 id 는 기록을 닫기 전에 읽는다. 옛 메시지에는 칸이 없다.
        open_q = self._questions.current
        answer = answer_for(open_q.id if open_q else "", str(getattr(msg, "question_id", "")),
                            str(getattr(msg, "answer", "")))
        answer_value = str(getattr(msg, "answer_value", ""))
        intent = IntentData(
            intent=msg.intent,
            matched_destination_id=msg.matched_destination_id,
            need_confirm=msg.need_confirm,
            safety_flag=msg.safety_flag,
            wait_minutes=int(getattr(msg, "wait_minutes", -1)),
            reply=msg.reply,
            speaker_verdict=str(getattr(msg, "speaker_verdict", "")),
            speaker_score=float(getattr(msg, "speaker_score", -1.0)),
            speaker_joined=float(getattr(msg, "speaker_score_joined", -1.0)),
            # 질문 넘김(3-가) — 옛 설치본 메시지에는 없다.
            ask_text=str(getattr(msg, "ask_text", "")),
            ask_kind=str(getattr(msg, "ask_kind", "")),
            ask_option_ids=tuple(getattr(msg, "ask_option_ids", ())),
            answer=answer, answer_value=answer_value if answer else "",
        )
        before = self.logic.state
        self._question_safely(self._close_question, "answer")   # 뒤이은 동작이 새 질문을 열 수 있다 — 먼저 닫는다
        actions = self.logic.on_voice_intent(
            intent, self._now(), self._lookup_destination,
            bounds=self.map_bounds, nav_ready=self._nav2_ready())
        self._run_actions(actions)
        said = [a.text for a in actions if isinstance(a, Say)]
        self.get_logger().info(
            f"음성 요청 intent={msg.intent} dest={msg.matched_destination_id or '-'} "
            f"confirm={msg.need_confirm}: {before.value} -> {self.logic.state.value}"
            + (f" answer={answer}" if answer else "")
            + (f" 말={said}" if said else " (말 없음)")
            + (f" — {self.logic.voice_note}" if self.logic.voice_note else ""))

    def _lookup_destination(self, dest_id: str) -> Optional[Destination]:
        if not dest_id:
            return None
        return self.destinations.get(dest_id) or None

    def _on_tts_done(self, msg: String) -> None:
        """TTS 가 끊기지 않고 끝까지 재생한 문장. 접근 질문일 때만 시계를 켠다.

        다른 멘트(수락·도착 등)의 재생 완료는 응답 대기와 무관하고, 로직 쪽이
        AWAITING_USER 가 아니면 무시하므로 이중 방어다.
        """
        # 첫 접근 질문의 끝과 다시 묻기(2026-10-09)가 같은 글자다 — 둘 다 여기서 8초를 다시 센다.
        if MSG_APPROACH_REASK in msg.data:
            self.logic.on_approach_question_spoken(self._now())
        # 접근 질문 중 질문에 LLM 이 답한 말이 끝났으면 다시 묻거나 물러난다(로직이 글자를 대조한다).
        self._run_actions(self.logic.on_approach_reply_spoken(msg.data, self._now()))
        # 손잡이 힌트는 재생 완료를 기다리지 않는다 — 진동이 힌트와 같은 순간
        # 시작해 잡을 때까지 이어진다(2026-09-30, 09-11 I-2 장치 폐기).
        # 온보딩·되묻기 재생이 끝난 시점부터 답 대기 15초를 센다(2026-09-11).
        # 로직이 사다리 중이 아니면 무시하므로 이중 방어다.
        if MSG_APPROACH_ONBOARDING in msg.data or MSG_DEST_RETRY in msg.data:
            self._run_actions(self.logic.on_dest_prompt_spoken(self._now()))
        # 도착 후 대화의 질문도 재생완료 시점부터 8초를 센다. 로직이
        # ASKING_* 가 아니면 무시하므로(이중 방어) 문구 대조 없이 넘긴다.
        self.logic.on_arrival_question_spoken(self._now())
        # 대기 장소 멘트(M2) 재생이 끝났는가 — 손 놓기 판정은 그때부터다(2026-10-07).
        # 로직이 자기 M2 문장과 대조하므로 다른 문장이면 무시된다.
        self._run_actions(self.logic.on_wait_speech_spoken(msg.data, self._now()))

    def _on_wake(self, msg: String) -> None:
        """/vica/wake — 호출 반응표대로 대답할지 정한다(2026-10-07, 판단은 로직).

        "wake": 호출 한 번. 로직(on_wake_call)이 "네?"·상태 정리·판정(WakeReply)을 낸다.
          RETURNING 이면 복귀 브레이크(E)를 먼저 건다 — 그 뒤 IDLE 로서 대답한다.
        "rescue": 창 안에서 소리로 건진 호출. 음성 쪽이 이미 대답하고 듣는 중이라 상태
          정리(on_wake)만 한다 — 판정·인사를 또 내면 "네?"가 두 번 난다.
        """
        # "wake:N" — N 은 음성 쪽 호출 번호. 판정에 그대로 붙여 돌려주면 음성 쪽이 앞
        # 호출의 늦은 판정을 거른다(2026-10-07 검토). 번호 없는 "wake"(옛 판)도 받는다.
        kind, _, seq = (msg.data or "").strip().partition(":")
        rescue = kind == "rescue"
        self._wake_reply_seq = seq if seq.isdigit() else ""
        now = self._now()
        before = self.logic.state
        actions: list = []
        if self.logic.state == State.RETURNING:
            actions.extend(self.logic.on_return_brake(now))
            if actions:
                self.get_logger().info("복귀 중 '비카야' — 복귀 취소하고 응대")
        if rescue:
            actions.extend(self.logic.on_wake(now))
        else:
            actions.extend(self.logic.on_wake_call(now))
        self._run_actions(actions)
        reply = next((a for a in actions if isinstance(a, WakeReply)), None)
        verdict = "창 안 구제" if rescue else ("들음" if reply and reply.listen else "대답 안 함")
        self.get_logger().info(
            f"'비카야'({verdict}): {before.value} -> {self.logic.state.value}")

    def _on_wake_doa(self, msg: Float32) -> None:
        """/vica/wake_doa — 호출 방향으로 고개를 돌린다 (IDLE 에서만).

        받았지만 아무 일도 안 한 경우(10도 미만이라 회전 생략·IDLE 아님·
        E-stop·nav 미준비·복귀 대기 중·방금 wake 소비 직후·접근 온보딩
        직후)까지 전부 한 줄로 남긴다 — 이것은 멘트가 아니라 로그다. 실기에서
        "안 돌았다"의 원인(부호 오류·관문 거절·사각지대)을 가릴 유일한
        단서이며, 실기 검증만 남은 브랜치에서는 이 로그가 전제다.

        판정 분기는 결과(actions·state 전이)를 먼저 보고, 못 맞히면 그제서야
        on_wake_doa 의 관문을 순서대로 되짚는다: 회전 시작(SpinInPlace 발행)
        → 핸들 쪽(180도 근방, 회전 없이 AWAITING_USER 로 전이) → state/estop/
        nav_ready → 복귀 대기 중 → wake 소비 직후 → 접근 온보딩 직후 →
        생략(10도 미만, 마지막 else). return_interrupted/wake_guard_active/
        user_attached_guard_active 는 on_wake_doa 가 쓰는 것과 같은
        속성·메서드라 로그와 실제 판정이 갈라질 일이 없다.

        핸들 쪽(180°±45°, HANDLE_SIDE_MIN_YAW_RAD) 판정은 회전 없이 상태가
        곧장 AWAITING_USER 로 바뀌는 것으로 알아본다 — SpinInPlace 가 없다는
        점은 정면 사각지대("생략, 10도 미만")와 같지만 상태 전이가 다르다.
        """
        now = self._now()
        before = self.logic.state
        nav_ready = self._nav2_ready()
        actions = self.logic.on_wake_doa(float(msg.data), nav_ready, now)
        self._run_actions(actions)
        if State.SEEKING in (before, self.logic.state) and before != self.logic.state:
            # SEEKING 진입은 YOLO 를 끈다(is_moving). /vica/robot_state 는
            # 1 Hz 라 최대 1초 늦게 갱신되면 seek_look_sec 창이 그만큼 줄어든다
            # — 즉시 갱신해 그 지연을 회수한다(2026-09-10 재검토).
            self._publish_robot_state()
        yaw_rad = doa_to_spin_yaw(float(msg.data), self.logic.wake_doa_sign)
        if any(isinstance(a, SpinInPlace) for a in actions):
            verdict = "회전 시작"
        elif before == State.IDLE and self.logic.state == State.AWAITING_USER:
            verdict = "핸들 쪽 — 회전 없이 곧바로 질문"
        elif before != State.IDLE:
            verdict = f"거절(대기 중 아님, state={before.value})"
        elif self.logic.estop_active:
            verdict = "거절(E-stop)"
        elif not nav_ready:
            verdict = "거절(nav 미준비)"
        elif self.logic.return_interrupted:
            verdict = "거절(복귀 대기 중)"
        elif self.logic.wake_guard_active(now):
            verdict = "거절(방금 wake 소비 직후로 추정)"
        elif self.logic.user_attached_guard_active(now):
            verdict = "거절(안내 시작 직후)"
        else:
            verdict = "생략(10도 미만, 창만 유지)"
        self.get_logger().info(
            f"'비카야' 방향 doa={msg.data:.0f}° yaw={math.degrees(yaw_rad):.0f}°: "
            f"{before.value} -> {self.logic.state.value} ({verdict})")

    def _on_person_detection(self, msg: PersonDetection) -> None:
        """/vica/person_detection 원본 결과 (근접 호출, 2026-09-10 확장).

        탐색 창 밖에서는 mission_logic.on_person_detection 이 빈 목록을 돌려주고
        상태도 그대로다 — 5 Hz 로 늘 들어오는 이 콜백이 평소에는 아무 일도 안
        하는 것이 정상이다. 로그는 상태가 실제로 바뀐 경우에만 남긴다(그 외는
        5 Hz 소음이 된다).
        """
        if self.logic.state == State.APPROACHING:
            self._recheck_approach(msg)
            return
        before = self.logic.state
        actions = self.logic.on_person_detection(
            track_id=msg.track_id,
            distance_m=msg.distance_m,
            stable=msg.stable,
            approachable=msg.approachable,
            now=self._now(),
            person=self._detection_pose(msg),
        )
        if before == self.logic.state:
            return
        self._run_actions(actions)
        self.get_logger().info(
            f"근접 호출: track={msg.track_id} dist={msg.distance_m:.2f}m "
            f"{before.value} -> {self.logic.state.value}"
        )

    @staticmethod
    def _detection_pose(msg: PersonDetection) -> Optional[Pose2D]:
        """검출의 사람 자리(map). 좌표가 NaN 이면 None — 자리로 판단하지 않는다."""
        px, py = msg.pose.position.x, msg.pose.position.y
        if not (math.isfinite(px) and math.isfinite(py)):
            return None
        return Pose2D(x=px, y=py, yaw_deg=0.0, frame_id=msg.header.frame_id or "map")

    def _recheck_approach(self, msg: PersonDetection) -> None:
        """접근 중 검출 → 중간 재측정(2026-10-10, 설계 docs/superpowers/specs/2026-10-10-…).

        판단(언제·어느 검출·12 cm)은 mission_logic, 안전거리 앞 목표 계산은 여기(approach_goal) —
        접근 요청과 같은 나눔이다. 로그는 단계가 바뀔 때와 결과 때만 한 줄씩(5 Hz 소음 방지).
        """
        person = self._detection_pose(msg)
        if person is None:
            return
        recheck = self.logic.approach_recheck
        before = recheck.phase
        new_person = self.logic.on_approach_detection(person, msg.confidence, self._now())
        after = recheck.phase
        if after != before:
            self.get_logger().info(
                f"중간 재측정: {before.value} -> {after.value} (처음 거리 "
                f"{recheck.start_distance_m:.2f} m, 시점 {recheck.trigger_m:.2f} m, "
                f"표본 {recheck.sample_count}개)")
        if new_person is None or self._robot_pose is None:
            return
        old = self.logic.approach_person
        shift = (math.hypot(new_person.x - old.x, new_person.y - old.y)
                 if old is not None else float("nan"))
        goal = approach_goal(new_person, self._robot_pose)
        actions, moved = self.logic.on_approach_recheck_goal(
            new_person, goal, self.map_bounds, self._now())
        self._run_actions(actions)
        self.get_logger().info(
            f"중간 재측정: 사람 자리 {shift:.2f} m 고침 → 목표 {moved:.2f} m "
            f"{'다시 보냄' if actions else '그대로(12 cm 미만이거나 지도 밖)'} · 이후 목표 고정")

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
        # 순찰 중의 일시정지·재개·취소(2026-11-14) — 거절 사유는 한국어로, 바뀐 상태는 곧바로 방송한다.
        patrol = before.value.startswith("patrol")
        actions, reason = self._run_mission_command(command)
        if reason != GateReason.OK:
            # 거절이어도 로직이 돌려준 행동은 낸다 — 비상 정지 중 앱 취소가 끝낸 도움 요청의
            # help_canceled(2026-10-16 결정 3). 다른 거절은 행동이 없다.
            self._run_actions(actions)
            response.accepted = False
            response.message = f"{command} 요청 거부: {reason.value}"
            if patrol and reason in PATROL_REJECT_TEXT:
                response.message = PATROL_REJECT_TEXT[reason]
            self.get_logger().warn(
                f"{command} 요청 거부: state={before.value} reason={reason.value}"
            )
            return response

        self._run_actions(actions)
        # 순찰 중 취소 = 경비 종료 → 홈 복귀. 복귀를 못 했으면 기존 홈 복귀 거절 문구를 싣는다.
        home_message = self._patrol_stop_follow_up() if patrol and command == "cancel" else ""
        if patrol and self.logic.state != before:
            self._publish_robot_state()
        response.accepted = True
        response.message = home_message or f"{command} 요청을 처리했습니다."
        self.get_logger().info(
            f"{command} 처리: {before.value} -> {self.logic.state.value}"
        )
        return response

    def _on_help_ack(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        """관리자가 앱에서 도움 요청을 확인했다. request_id = 확인할 help_id(uuid4)."""
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
        actions, accepted = self.logic.on_help_ack(request_id, self._now())
        self._run_actions(actions)
        response.accepted = accepted
        if accepted:
            response.message = "ok"
        elif self.logic.help_id is None:
            response.message = "no_active_help"
        else:
            response.message = "id_mismatch"
        self.get_logger().info(f"도움 요청 확인: {request_id} → {response.message}")
        return response

    # -- 경비(순찰) 모드 (2026-11-14, ~/vica_data/patrol_plan/SPEC.md 3절) ----------
    #
    # 판단은 mission_logic(on_patrol_*)이 한다. 여기는 경로 파일·서비스 응답·사건 JSON·기록 파일 배선이다.
    # 세 서비스의 request_id 는 형식을 따지지 않는다(순찰 서비스는 앱만 부른다).

    def _patrol_follow(self, before: str, why: str) -> None:
        """순찰 상태가 바뀌었으면 한 줄 남기고 /vica/robot_state 를 곧바로 낸다.

        감지·음성·손잡이가 dialog_state 로 순찰을 따라온다 — 1 Hz 를 기다리면 3초 지점 정지의
        1초를 잃는다. 틱 안에서 바뀐 것은 _tick 이 같은 일을 한다.
        """
        after = self.logic.dialog_state
        if after == before:
            return
        self.get_logger().info(f"순찰({why}): {before} -> {after}")
        self._publish_robot_state()

    def _on_patrol_event(self, msg: String) -> None:
        """/vica/patrol/event — 순찰이 아니면 JSON 도 풀지 않는다."""
        if not self.logic.patrol_active:
            return
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        before = self.logic.dialog_state
        self._run_actions(self.logic.on_patrol_event(data, self._now()))
        self._patrol_follow(before, f"감지 {data.get('kind')}")

    def _load_patrol_route(self) -> tuple:
        """지금 지도의 순찰 경로 → (장소 목록 또는 None, 못 쓰는 이유 한 줄, 꺼 둔 감시 항목)."""
        try:
            ids, watch_off = self._patrol_routes.read(self._map_id)
        except ValueError:
            ids, watch_off = None, []
        if not ids:
            return None, "", watch_off
        code, message = check_route_ids(ids, self.destinations)
        if code == ROUTE_TOO_FEW:
            return None, "", watch_off   # 2곳 미만 — 시작 거절의 정해진 문구(PATROL_START_REJECT_TEXT)가 나간다
        if code != ROUTE_OK:
            return None, message, watch_off
        return [self.destinations[dest_id] for dest_id in ids], "", watch_off

    def _publish_patrol_watch(self, watch_off=None) -> None:
        """꺼 둔 감시 항목을 /vica/patrol/watch 로 알린다. None 이면 지금 지도의 파일에서 읽는다."""
        if watch_off is None:
            try:
                watch_off = self._patrol_routes.read(self._map_id)[1]
            except ValueError:
                watch_off = []
        self._pub_patrol_watch.publish(String(data=json.dumps({"off": list(watch_off)})))

    def _patrol_reply(self, response, reason: GateReason, ok_message: str, what: str):
        """순찰 서비스 응답. 거절 사유는 한국어 한 줄로 돌려준다."""
        response.accepted = reason == GateReason.OK
        response.message = (ok_message if response.accepted
                            else PATROL_REJECT_TEXT.get(reason, reason.value))
        if response.accepted:
            self.get_logger().info(f"{what}: {response.message}")
        else:
            self.get_logger().warn(f"{what} 거부: {reason.value}")
        return response

    def _on_patrol_start(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        """경비 시작. patrol.yaml 을 읽어 로직에 넘긴다 — 수락하면 감지 노드 준비부터 기다린다."""
        if self._dock_refuse(response, "경비 시작"):
            return response
        route, problem, watch_off = self._load_patrol_route()
        pose_known = self._robot_pose is not None and self._pose_cov_xy <= COV_UNKNOWN
        before = self.logic.dialog_state
        actions, reason = self.logic.on_patrol_start(
            route, self.map_bounds, self._nav2_ready(), pose_known, self._now(), watch_off)
        if reason == GateReason.OK:
            # 순찰 상태(/vica/robot_state)보다 먼저 — 감지 노드가 엔진을 올릴지 이 값으로 정한다.
            self._publish_patrol_watch(watch_off)
            if watch_off:
                self.get_logger().info(f"경비 시작 — 꺼 둔 감시 항목: {', '.join(watch_off)}")
        self._run_actions(actions)
        self._patrol_follow(before, "시작")
        self._patrol_reply(response, reason, "경비를 시작합니다.", "경비 시작")
        if reason in PATROL_START_REJECT_TEXT:
            # 앱이 그대로 보여 주는 목업 문구(비상정지 중·지점 2곳 미만). 시작 거절에만 쓴다.
            response.message = PATROL_START_REJECT_TEXT[reason]
        if reason == GateReason.NO_PATROL_ROUTE and problem:
            response.message = problem   # 경로는 2곳 이상인데 못 쓰는 경우(지워진 장소 등)
        return response

    def _on_patrol_stop(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        """경비 종료 → 홈으로 복귀(2026-11-14 사용자 결정). 홈이 없거나 복귀를 시작할 수 없으면
        그 자리에서 끝내고, 응답 message 에 기존 홈 복귀 거절 문구를 싣는다."""
        before = self.logic.dialog_state
        actions, reason = self.logic.on_patrol_stop(self._now(), self._nav2_ready())
        self._run_actions(actions)
        home_message = self._patrol_stop_follow_up() if reason == GateReason.OK else ""
        self._patrol_follow(before, "종료")
        self._patrol_reply(response, reason, "경비를 종료했습니다.", "경비 종료")
        if home_message:
            response.message = home_message
        return response

    def _patrol_stop_follow_up(self) -> str:
        """관리자 경비 종료 뒤의 홈 복귀 마무리(로직 on_patrol_stop 의 행동을 실행한 다음 부른다).

        복귀를 시작했으면 앱의 홈 복귀(_on_return_home)와 똑같이 return_home_sent 를 내고 "" 를,
        못 했으면 그 서비스의 거절 문구와 같은 글자를 돌려준다.
        """
        reason = self.logic.patrol_stop_home_reason
        self.logic.patrol_stop_home_reason = None   # 한 번만 쓴다 — 다음 취소에 옛 결과가 남지 않게
        if reason is None:
            return ""
        if reason != GateReason.OK:
            self.get_logger().warn(f"경비 종료 뒤 홈 복귀 거부: reason={reason.value}")
            return f"홈 복귀 거부: {reason.value}"
        home = self.logic.return_destination
        if home is not None:
            self._publish_goal_event("return_home_sent", home)
        self.get_logger().info("경비 종료 — 홈으로 복귀합니다.")
        return ""

    def _on_patrol_ack(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        """경보 확인(음성 반복·재전송만 멈춘다). request_id 가 경보 id 모양이면 그 경보만,
        아니면 지금 걸린 경보를 확인한다."""
        raw = str(request.request_id or "").strip()
        alarm_id = raw if PATROL_ALARM_ID_PATTERN.fullmatch(raw) else ""
        actions, reason = self.logic.on_patrol_ack(alarm_id, self._now())
        self._run_actions(actions)
        return self._patrol_reply(response, reason, "경보를 확인했습니다.", "경보 확인")

    def _on_patrol_records_clear(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        """경비 기록·영상·사진을 모두 지운다(앱 '알림 및 로그 > 경비 기록'의 삭제, 2026-11-16 사용자 요청).
        경비 중에는 거절한다 — 감지 노드가 장면을 쓰고 있을 수 있다."""
        if self.logic.patrol_active:
            response.accepted = False
            response.message = "경비 중에는 기록을 지울 수 없습니다."
            self.get_logger().warn("경비 기록 삭제 거부: 경비 중")
            return response
        try:
            removed = self._patrol_log.clear()
        except OSError as exc:
            response.accepted = False
            response.message = "경비 기록을 지우지 못했습니다."
            self.get_logger().warn(f"경비 기록 삭제 실패: {exc}")
            return response
        response.accepted = True
        response.message = "경비 기록을 지웠습니다."
        self.get_logger().info(f"경비 기록 삭제: {removed}개 ({self._patrol_log.path.parent})")
        return response

    def _patrol_catalog(self, map_id: str):
        """그 지도의 목적지 catalog. 지금 지도면 메모리의 것, 다른 지도면 파일을 읽는다. 없으면 None."""
        if map_id == self._map_id:
            return self.destinations
        try:
            path = self._patrol_routes.path_for(map_id).with_name("destinations.yaml")
            return load_destinations(str(path)) if path.exists() else None
        except Exception:  # noqa: BLE001 — 깨진 catalog 는 '모르는 지도'로 본다
            return None

    def _on_save_patrol_route(self, request, response):
        """순찰 경로와 꺼 둔 감시 항목 저장. 장소가 빈 배열이면 경로를 지운다. reason 코드는 SPEC 3.2.

        감시 항목은 셋 다 끌 수 없다. 경로를 지워도 꺼 둔 항목이 있으면 파일에 그 값만 남긴다.
        """
        map_id = str(request.map_id or "").strip() or self._map_id
        ids = [str(dest_id).strip().lower() for dest_id in request.destination_ids]
        watch_off = clean_watch_off(list(request.watch_off))
        catalog = self._patrol_catalog(map_id)
        if catalog is None:
            code, message = ROUTE_UNKNOWN_MAP, f"지도를 찾을 수 없습니다: {map_id}"
        elif len(watch_off) == len(PATROL_WATCH_KINDS):
            code, message = ROUTE_NOTHING_WATCHED, "감시 항목은 하나 이상 켜 두어야 합니다."
        else:
            try:
                if not ids:
                    had_route = bool(self._patrol_routes.read(map_id)[0])
                    if watch_off:
                        self._patrol_routes.write(map_id, [], watch_off)
                    else:
                        self._patrol_routes.delete(map_id)
                    code = ROUTE_OK
                    message = "순찰 경로를 지웠습니다." if had_route else "지울 순찰 경로가 없었습니다."
                else:
                    code, message = check_route_ids(ids, catalog)
                    if code == ROUTE_OK:
                        self._patrol_routes.write(map_id, ids, watch_off)
                        message = f"순찰 경로를 저장했습니다(장소 {len(ids)}곳)."
            except (OSError, ValueError) as exc:
                code, message = ROUTE_IO_ERROR, f"순찰 경로 저장 실패: {exc}"
        response.accepted = code == ROUTE_OK
        response.reason = code
        response.message = message
        if response.accepted and map_id == self._map_id and not self.logic.patrol_active:
            # 다음 경비 전에 미리 알려 둔다. 순찰 중에는 내지 않는다 — 이번 순찰은 시작 때 값으로 끝까지
            # 가는데, 감지·호출어 노드는 받는 즉시 바꾼다. 다음 경비 시작 때 파일 값이 다시 나간다.
            self._publish_patrol_watch(watch_off)
        self.get_logger().info(
            f"순찰 경로 저장({map_id}, {len(ids)}곳, 끈 항목 {watch_off or '없음'}): {code} — {message}")
        return response

    def _on_get_patrol_route(self, request, response):
        """순찰 경로 읽기. 이름은 catalog 에서 찾고, 지워진 장소는 이름을 "" 로 둔다."""
        map_id = str(request.map_id or "").strip() or self._map_id
        response.found = False
        response.map_id = map_id
        response.destination_ids = []
        response.names = []
        response.watch_off = []
        try:
            ids, response.watch_off = self._patrol_routes.read(map_id)
        except ValueError:
            response.message = f"지도를 찾을 수 없습니다: {map_id}"
            return response
        if not ids:
            response.message = "저장된 순찰 경로가 없습니다."
            return response
        catalog = self._patrol_catalog(map_id) or {}
        names = [catalog[dest_id].name if dest_id in catalog else "" for dest_id in ids]
        response.found = True
        response.destination_ids = ids
        response.names = names
        response.message = "" if all(names) else "경로에 지워진 장소가 있습니다. 경로를 다시 저장하세요."
        return response

    def _patrol_record(self, record: dict) -> None:
        """경보 기록 파일에 한 줄 덧붙인다. 실패는 경고만 — 순찰은 계속한다."""
        if not self._patrol_log.append(record):
            self.get_logger().warn(f"경보 기록 쓰기 실패: {self._patrol_log.path}")

    def _publish_patrol_event(self, action: PatrolEvent) -> None:
        """순찰 사건을 /vica_goal_event 로 낸다(SPEC 3.3).

        경보 4종은 보낼 때마다 그 순간 위치(place·robot_x·robot_y)를 싣는다. 기록 파일은 첫
        전송에만 쓴다 — 10초마다의 재전송은 쓰지 않는다.
        """
        extra: dict = {}
        if action.event == "patrol_started":
            extra["route"] = list(action.route)
        elif action.event == "patrol_waypoint":
            extra.update({"index": action.index, "lap": action.lap, "count": action.count})
        elif action.event == "patrol_acked":
            extra["alarm_id"] = action.alarm_id
        elif action.event == "patrol_stopped":
            extra["return_home"] = action.return_home   # 이어서 홈 복귀를 시작했나(admin 만 true)
        elif action.kind in PATROL_ALARM_KINDS:
            pose = self._robot_pose
            place = nearest_place_name(pose, self.destinations)
            robot_x = pose.x if pose is not None else None
            robot_y = pose.y if pose is not None else None
            extra.update({"alarm_id": action.alarm_id, "place": place,
                          "robot_x": robot_x, "robot_y": robot_y, "resend": action.resend})
            if action.clip_id:
                extra["clip_id"] = action.clip_id
            if action.photo:
                extra["photo"] = action.photo
            if action.level_db is not None:
                extra["level_db"] = action.level_db
            if not action.resend:
                self._patrol_record(alarm_record(
                    action.alarm_id, action.kind, action.time, place, robot_x, robot_y,
                    self._map_id, action.clip_id, action.photo, action.level_db))
        self._publish_goal_event(action.event, action.destination, action.reason, extra=extra)
        self.get_logger().info(
            f"앱 알림: {action.event} {action.alarm_id or action.reason}"
            + (" (재전송)" if action.resend else ""))

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
        self._home_watch = (
            (home.x, home.y)
            if home is not None and (not home.visited_ok or home.reach_failed)
            else None
        )
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
        # 마지막 복귀가 실패했으면 그 사실을 사람 말로 싣는다 — 앱이 '가 본 적 없음' 대신 보인다
        # (11-19 사용자). 공용 계약을 바꾸지 않으려고 기존 message 칸을 쓴다.
        response.message = HOME_UNREACHABLE_MESSAGE if home.reach_failed else ""
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
        if self._dock_refuse(response, "홈 복귀"):
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
        # 도착 순간 입구 방향(M1) 계산용(2026-10-07).
        self.logic.robot_yaw_deg = yaw_deg
        # 홈 알림의 '홈에 있는가' 판정용(2026-10-08).
        self.logic.robot_pose = self._robot_pose
        cov = msg.pose.covariance
        self._pose_cov_xy = float(cov[0] + cov[7]) if len(cov) >= 8 else 0.0
        self._check_home_reached()

    def _robot_near_home(self) -> bool:
        """위치를 제대로 잡은 채 홈 30 cm 안에 있는가."""
        home = self.logic.return_destination
        pose = self._robot_pose
        if home is None or pose is None or self._pose_cov_xy > COV_UNKNOWN:
            return False
        return near_home(home.pose.x, home.pose.y, pose.x, pose.y)

    def _check_home_reached(self) -> None:
        """홈 30 cm 안에 들어오면 '가 봄'으로 적는다(11-19 사용자 — 주행에서 어떻게든 홈에 한 번
        가 봤으면 '가 본 적 없음'을 내린다). Nav2 가 방향을 못 맞춰 '성공'을 못 낸 복귀, 수동 주행,
        지나가기도 같다. 위치를 아직 못 잡았으면(초기 위치 전) 보지 않는다."""
        watch = self._home_watch
        pose = self._robot_pose
        if watch is None or pose is None or self._pose_cov_xy > COV_UNKNOWN:
            return
        if near_home(watch[0], watch[1], pose.x, pose.y):
            self._home_watch = None
            self.get_logger().info("홈 30 cm 안에 들어와 '가 봄'으로 기록합니다.")
            self._record_home_visit(True)

    def destroy_node(self) -> bool:
        # 장애물 안내 보조 노드·실행기를 먼저 닫는다 — 본체가 사라진 뒤 판정 타이머가 더 돌지 않게.
        executor = getattr(self, "_obs_executor", None)
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
        node = getattr(self, "_obs_node", None)
        if node is not None:
            node.destroy_node()
        return super().destroy_node()

    # -- 주행 중 장애물 안내 (2026-10-09) ------------------------------------------------------
    # 판정은 obstacle_judge(설계서 3절, 시운전 run81 에서 16번 모두 실물). 입력은 보조 노드(vica_mission_obstacle)가
    # 단일 실행기로 자기 스레드에서 받아 대화 줄(_main_group)과 서로 기다리지 않는다. 라이다·깊이는 최근 1장만 둔다.
    # 판정 줄은 말할 후보만 _obstacle_cues 에 넣고, 말할지는 대화 줄의 _tick 이 logic.obstacle_cue 로 정한다.
    # 장애물 부분의 예외는 Guard 가 받아 안내만 끈다 — 안내 주행은 계속된다.
    #
    # CPU(2026-10-10 실측, 코어 하나 기준): 처음 배선(본체 다중 실행기·tf·늘 받기)은 쉴 때도 46 %. 판정 계산이 아니라
    # 메시지 받기 부담이었다 — ① 입력은 안내 주행 중에만, ② 위치는 tf(초당 52통) 대신 /amcl_pose + /odom 변화
    # (녹화본 run81·run82 판정 같음), ③ rclpy 다중 실행기 대신 보조 노드의 단일 실행기.

    def _setup_obstacle_narration(self, map_yaml: str) -> None:
        grid, why = load_grid(map_yaml)
        if grid is None:
            self.get_logger().warn(f"장애물 안내: 지도를 못 읽어 끕니다 — {why}")
            return
        # 설치(노드·구독·타이머)도 보호막 안이다 — 여기서 예외가 나도 미션은 뜨고 안내만 꺼진다(2026-10-09 최종 검토 I-2).
        try:
            self._obstacle = ObstacleJudge(grid)
            self._laser_offset: Optional[tuple] = None
            self._tf_buffer = tf2_ros.Buffer()       # /tf_static(라이다 위치)만 넣는다 — /tf 는 받지 않는다
            self._obs_amcl: Optional[tuple] = None   # 마지막 AMCL 위치 (헤더 시각, x, y, yaw)
            self._obs_trail = OdomTrail()
            self._obs_subs: list = []
            self._obs_on = False
            node = rclpy.create_node("vica_mission_obstacle")
            self._obs_node = node
            # 한 줄(단일 실행기)이라 Reentrant 여도 콜백이 동시에 돌지 않는다. 한 번에 하나 그룹은 구독을 닫는 순간의
            # InvalidHandle(Humble 은 그룹의 '실행 중' 표시를 풀기 전에 낸다)에 영영 잠긴다(2026-10-10 검토).
            group = ReentrantCallbackGroup()
            self._obs_group = group
            wrap = self._obstacle_guard.wrap
            # 늘 받는 것은 드물고 가볍다. 목적지 사건은 출발 직전에 와서, 주행 중에만 받으면 '목적지 1 m 안에서는
            # 말하지 않기'가 목적지를 모른다. AMCL·tf_static 은 보관(transient_local)이라 늦게 붙어도 마지막 값을 받는다.
            kept = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
            static = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(PathMsg, "/rail_plan", wrap(self._obs_rail), 5, callback_group=group)
            node.create_subscription(String, "/vica_goal_event", wrap(self._obs_goal), 10, callback_group=group)
            node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", wrap(self._obs_amcl_msg), kept,
                                     callback_group=group)
            node.create_subscription(TFMessage, "/tf_static", wrap(self._obs_static), static, callback_group=group)
            # 판정 타이머와 나머지 입력은 _obs_start 가 연다.
            self._obs_timer = node.create_timer(0.1, wrap(self._obs_tick), callback_group=group)
            self._obs_timer.cancel()
            self._obs_executor = SingleThreadedExecutor()
            self._obs_executor.add_node(self._obs_node)
            threading.Thread(target=self._obs_spin, name="obstacle_inputs", daemon=True).start()
        except Exception as exc:  # noqa: BLE001
            self._obstacle_guard.enabled = False
            self._on_obstacle_error(exc)
            return
        self.get_logger().info(f"장애물 안내: 켜짐 (지도 {map_yaml}) — 입력은 안내 주행 중에만 받습니다")

    def _obs_spin(self) -> None:
        # spin() 대신 spin_once() 를 돈다 — 대화 줄이 구독을 닫는 순간 이 줄이 막 집은 메시지를 꺼내면 rclpy(Humble)가
        # InvalidHandle 을 낸다(executors._take_subscription). 그 메시지 하나만 버리고 계속 돈다.
        # spin_once 에 시한을 주지 않는다 — Humble 은 시한마다 타이머를 새로 만든다.
        while rclpy.ok():
            try:
                self._obs_executor.spin_once()
            except InvalidHandle:
                continue
            except (ExternalShutdownException, ShutdownException):
                return
            except Exception as exc:  # noqa: BLE001 — 보조 실행기가 죽으면 안내만 끈다
                if rclpy.ok():
                    self._obstacle_guard.enabled = False
                    self._on_obstacle_error(exc)
                return

    def _obstacle_inputs_follow(self) -> None:
        """장애물 입력은 안내 주행(NAVIGATING) 중에만 받는다 — 안내 멘트도 그때만 나간다(obstacle_cue).

        2026-10-10 실측: 입력을 늘 받으면 쉴 때도 미션이 코어 하나의 46 %(끄면 2.4 %).
        대화 줄의 _tick(5 Hz)에서 부르므로 상태가 바뀐 뒤 0.2초 안에 열고 닫는다.
        """
        if self._obstacle is None or not self._obstacle_guard.enabled:
            # 오류로 안내가 꺼졌는데 입력이 열려 있으면 한 번 닫는다 — 꺼진 안내가 메시지 받기 부담만 남기지 않게.
            if getattr(self, "_obs_on", False):
                try:
                    self._obs_stop()
                except Exception:  # noqa: BLE001 — 이미 꺼진 안내다. 닫다 난 오류는 더 할 일이 없다
                    pass
            return
        want = self.logic.dialog_state == State.NAVIGATING.value
        if want == self._obs_on:
            return
        try:
            if want:
                self._obs_start()
            else:
                self._obs_stop()
        except Exception as exc:  # noqa: BLE001 — 열고 닫다 난 오류도 안내만 끈다
            self._obstacle_guard.enabled = False
            self._on_obstacle_error(exc)

    def _obs_start(self) -> None:
        # 지난 주행의 위치·점·VCC·결정 대기와 바퀴 기록을 먼저 지운다 — 옛 위치로 새 점을 찍으면 없는 장애물을 말한다.
        # 새 /odom 이 오기 전에는 '위치 모름'(원인 판정 쉼)이다. 판정 타이머·센서 구독은 앞서 _obs_stop 이 닫아
        # 두었으므로 이 줄과 동시에 판정이 돌지 않는다(늘 받는 레일·목적지·AMCL 은 지우는 것을 건드리지 않는다).
        self._obstacle.clear_inputs()
        self._obs_trail.clear()
        node, group = self._obs_node, self._obs_group
        latest = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
        wrap = self._obstacle_guard.wrap
        # 열림 표시와 구독 목록을 먼저 둔다 — 여는 도중 오류가 나도 다음 _tick 의 닫기가 연 것만큼 정리한다.
        self._obs_on = True
        self._obs_subs = []
        # 타이머를 구독보다 먼저 켠다 — rclpy 는 타이머 reset 으로 실행기를 깨우지 않는다. 아래 구독을 만들 때
        # 실행기가 깨어나 대기 목록을 다시 짜므로, 그때 켜진 타이머가 들어간다.
        self._obs_timer.reset()
        subs = self._obs_subs
        subs.append(node.create_subscription(Odometry, "/odom", wrap(self._obs_odom_msg), 10, callback_group=group))
        subs.append(node.create_subscription(LaserScan, "/scan", wrap(self._obs_scan), latest, callback_group=group))
        subs.append(node.create_subscription(LaserScan, "/camera/depth_scan", wrap(self._obs_depth), latest,
                                             callback_group=group))
        subs.append(node.create_subscription(String, "/vcc/state", wrap(self._obs_vcc), 10, callback_group=group))
        subs.append(node.create_subscription(BehaviorTreeLog, "/behavior_tree_log", wrap(self._obs_bt), 50,
                                             callback_group=group))
        subs.append(node.create_subscription(Log, "/rosout", wrap(self._obs_rosout), 50, callback_group=group))
        self.get_logger().info("장애물 안내: 입력 받기 시작(안내 주행)")

    def _obs_stop(self) -> None:
        self._obs_on = False
        self._obs_timer.cancel()
        for sub in self._obs_subs:
            self._obs_node.destroy_subscription(sub)
        self._obs_subs = []
        self.get_logger().info("장애물 안내: 입력 멈춤(안내 주행 아님)")

    def _on_obstacle_error(self, exc: BaseException) -> None:
        # 큐는 비우지 않는다 — 대화 줄이 꺼내는 중에 다른 스레드가 비우면 경쟁이 생긴다(최종 검토 I-1).
        # 판정이 꺼지면 새 후보는 들어오지 않고, 남은 후보는 대화 줄이 늘 하던 대로 거른다.
        self._obstacle = None
        self.get_logger().error(f"장애물 안내: 오류로 끕니다 — 안내 주행은 계속됩니다 ({exc!r})")

    @staticmethod
    def _stamp(msg) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _obs_amcl_msg(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        self._obs_amcl = (self._stamp(msg), p.position.x, p.position.y, yaw_of(p.orientation))

    def _obs_odom_msg(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._obs_trail.add(self._stamp(msg), p.position.x, p.position.y, yaw_of(p.orientation))

    def _obs_static(self, msg: TFMessage) -> None:
        for tr in msg.transforms:
            self._tf_buffer.set_transform_static(tr, "default_authority")

    def _obs_pose(self, t: float) -> None:
        pose = self._obs_trail.pose_since(self._obs_amcl)
        if pose is None:
            return   # 위치를 아직 모르면 원인 판정만 쉰다
        self._obstacle.on_pose(t, *pose)

    def _obs_laser_offset(self) -> tuple:
        if self._laser_offset is None:
            try:
                tr = self._tf_buffer.lookup_transform(BASE_FRAME, "laser_frame", rclpy.time.Time())
            except tf2_ros.TransformException:
                return LASER_DEFAULT
            self._laser_offset = (tr.transform.translation.x, tr.transform.translation.y)
        return self._laser_offset

    def _obs_scan(self, msg: LaserScan) -> None:
        t = self._now()
        self._obs_pose(t)
        self._obstacle.on_points("scan", t, *scan_points(msg, self._obs_laser_offset()))

    def _obs_depth(self, msg: LaserScan) -> None:
        if depth_frame_ok(msg.header.frame_id):
            self._obstacle.on_points("depth", self._now(), *scan_points(msg))

    def _obs_vcc(self, msg: String) -> None:
        st = parse_state(msg.data)
        if st is not None:
            t = self._now()
            # 안내 주행인지는 미션 자신의 대화 단계로 본다(1단계 도구는 /vica/robot_state 로 받았다).
            self._obstacle.on_dialog(t, self.logic.dialog_state)
            self._obstacle.on_vcc(t, st)

    def _obs_bt(self, msg: BehaviorTreeLog) -> None:
        for e in msg.event_log:
            self._obstacle.on_bt(e.timestamp.sec + e.timestamp.nanosec * 1e-9, e.node_name, e.current_status)

    def _obs_rosout(self, msg: Log) -> None:
        if msg.name == "collision_monitor":
            what = cm_what(msg.msg)
            if what:
                self._obstacle.on_cm(self._now(), what)

    def _obs_rail(self, msg: PathMsg) -> None:
        self._obstacle.on_rail(self._now(), [(p.pose.position.x, p.pose.position.y) for p in msg.poses])

    def _obs_goal(self, msg: String) -> None:
        # 이 노드가 낸 goal 사건을 이 노드가 다시 받는다 — 판정 입력을 1단계 도구와 똑같이 둔다.
        ev = goal_event(msg.data)
        if ev:
            self._obstacle.on_goal(self._now(), ev["event"], ev["x"], ev["y"], ev["loc"])

    def _obs_tick(self) -> None:
        t = self._now()
        self._obs_pose(t)
        for d in self._obstacle.tick(t):
            if d["decision"] == "announce":
                self._obstacle_cues.append((d["phrase"], d["t"]))
            self.get_logger().info("장애물: " + format_decision(d))

    def _drain_obstacle_cues(self) -> None:
        """판정 줄이 넘긴 후보를 대화 줄에서 말할지 정한다(_tick 끝에서 부른다)."""
        for phrase, onset in take_all(self._obstacle_cues):
            actions, why = self.logic.obstacle_cue(phrase, onset, self._now())
            if why:
                self.get_logger().info(f"장애물 안내 뺌: {why}")
            self._run_actions(actions)

    def _on_approach_request(
        self,
        request: RequestApproach.Request,
        response: RequestApproach.Response,
    ) -> RequestApproach.Response:
        """사람 접근 요청. 검증 -> goal 계산 -> 상태 기계 순서로 거른다.

        판단(60초 억제·상태·E-stop)은 mission_logic, 기하(1.1 m goal)는
        approach_geometry 가 한다. 여기는 srv 계약의 형식 검증과 배선만 한다.
        """
        if self._dock is not None and (self._dock.mode or self._dock.undock_pending
                                       or self._dock.near_dock):
            # 도킹 중(도크 안·나오다 멈춤 포함)에는 사람에게 다가가지 않는다(2026-11-18·19 사용자: 원격만 받는다).
            # 도킹을 중지해 도크 바로 앞에 선 때도 같다 — 받으면 로봇이 스스로 도크에서 나와 다가간다(11-19 검토).
            response.accepted = False
            response.message = "도킹 중이라 접근하지 않습니다."
            return response
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
            person=person,                  # 같은 사람은 자리로도 안다(2026-10-10 B6)
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
        if command == "cancel" and self.logic.state.value.startswith("patrol"):
            # 순찰 중 앱 취소 = 경비 종료 → 홈 복귀(2026-11-14). 홈 복귀 관문이 Nav2 준비를 본다.
            return self.logic.on_patrol_stop(now, self._nav2_ready())
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

    def _check_destination_request(
        self,
        request: RequestDestination.Request,
        response: RequestDestination.Response,
    ):
        """요청 모양(UUID·지도)을 검사한다. 통과하면 목적지 id, 아니면 None
        (response 에 사유를 채운다). 목적지 요청·가보기가 같은 검사를 쓴다."""
        try:
            request_id = str(UUID(request.request_id))
            destination_id = str(UUID(request.destination_id))
        except ValueError:
            response.accepted = False
            response.message = "request_id와 destination_id는 UUID여야 합니다."
            return None
        if request_id != request.request_id.lower():
            response.accepted = False
            response.message = "request_id가 canonical UUID 형식이 아닙니다."
            return None
        parsed_destination = UUID(destination_id)
        if (
            parsed_destination.version != 4
            or destination_id != request.destination_id.lower()
        ):
            response.accepted = False
            response.message = "destination_id는 canonical UUID v4여야 합니다."
            return None
        if request.map_id != self._map_id:
            response.accepted = False
            response.message = (
                f"현재 지도와 요청 지도가 다릅니다: "
                f"current={self._map_id}, requested={request.map_id}"
            )
            return None
        return destination_id

    def _handle_destination_request(
        self,
        request: RequestDestination.Request,
        response: RequestDestination.Response,
        allow_private: bool,
        label: str,
        is_delivery: bool = False,
    ) -> RequestDestination.Response:
        destination_id = self._check_destination_request(request, response)
        if destination_id is None:
            return response
        if self._dock_refuse(response, label):
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
        # 앞 주행을 선점·취소하는 이벤트가 _run_actions 안에서 먼저 나가므로 그 뒤에 기억한다.
        # 출발이 거부됐으면(_nav_active 거짓) 배송으로 기억하지 않는다.
        self._run_actions(actions)
        # 도크 안이라 잡아 둔 주행(먼저 나온 뒤 보낸다)도 받아들인 것이다 — 거절로 답하면 앱은 실패를 띄우는데
        # 로봇은 나와서 간다. 배송 기억도 그대로 둔다(11-19 검토).
        sent = self._nav_active or self._dock_holding()
        self._delivery_dest_id = destination_id if is_delivery and sent else None
        response.accepted = sent
        response.message = (
            f"목적지 요청을 수락했습니다: {destination.name}"
            if response.accepted and destination is not None
            else "Nav2가 목적지 요청을 수락하지 않았습니다."
        )
        return response

    def _on_wait_spot_request(
        self,
        request: RequestDestination.Request,
        response: RequestDestination.Response,
    ) -> RequestDestination.Response:
        """대기 장소로 가보기. 안내 중(IDLE 아님)에는 거절한다 — 로직이 판단한다."""
        destination_id = self._check_destination_request(request, response)
        if destination_id is None:
            return response
        if self._dock_refuse(response, "대기 장소 가보기"):
            return response
        destination = self.destinations.get(destination_id)
        actions, reason = self.logic.on_app_wait_spot(
            destination, self.map_bounds, self._nav2_ready(), self._now())
        if reason != GateReason.OK:
            response.accepted = False
            response.message = f"대기 장소 가보기 거부: {reason.value}"
            self.get_logger().warn(
                f"대기 장소 가보기 거부: id={destination_id} reason={reason.value}")
            return response
        self._run_actions(actions)
        response.accepted = self._nav_active or self._dock_holding()
        response.message = (
            f"대기 장소로 이동합니다: {destination.name}-대기"
            if response.accepted and destination is not None
            else "Nav2가 대기 장소 요청을 수락하지 않았습니다."
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

    def _on_tipped(self, msg: Bool) -> None:
        tipped = bool(msg.data)
        was_tipped = self.logic.tipped
        actions = self.logic.on_tipped(tipped, self._now())
        if tipped and not was_tipped:
            self.get_logger().error(
                f"넘어짐 감지 -> state={self.logic.state.value} (세울 때까지 멘트 반복)")
        elif was_tipped and not tipped:
            self.get_logger().warn(
                f"넘어짐 풀림(세움) -> state={self.logic.state.value} (해제는 관리자 앱)")
        else:
            # 켤 때 받는 지금 상태(transient_local) — 바뀐 것이 아니다.
            self.get_logger().info(f"넘어짐 신호: {'넘어짐' if tipped else '서 있음'}(지금 상태)")
        self._run_actions(actions)

    def _on_app_estop_state(self, msg: String) -> None:
        try:
            app_active = bool(json.loads(msg.data).get("app_active", False))
        except (ValueError, AttributeError):
            return
        self.logic.on_app_estop(app_active)

    def _on_estop(self, msg: Bool) -> None:
        active = bool(msg.data)
        changed = active != self._estop_active
        self._estop_active = active
        actions = self.logic.on_estop(self._estop_active, self._now())
        if self._dock is not None and active and self._dock.phase == "going":
            # 준비 위치로 가는 중일 때만 할 일이 있다(Nav2 취소). 도킹 잠금은 기다리지 않는다 — 주 콜백이 Nav2 호출
            # (goal 수락 대기 등)로 잠금을 쥐고 있으면 비상 콜백 그룹까지 같이 멈춘다(11-19 검토). 못 잡으면 다음
            # tick(0.2 s)이 한다. 모터는 Safety 가 이미 세웠다.
            if self._dock_lock.acquire(blocking=False):
                try:
                    self._dock_apply(self._dock.on_estop(True))
                finally:
                    self._dock_lock.release()
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
        self._question_safely(self._tick_question)
        before = self.logic.state
        status = self._poll_nav_status()
        if self._dock is not None and self._dock.owns_nav():
            # 도크 앞(준비 위치)으로 가는 주행 — 결과는 도킹 흐름이 받는다. 로직은 IDLE 이다.
            with self._dock_lock:
                self._dock_apply(self._dock.on_nav_status(status.value))
            status = NavStatus.NONE
        if self._dock is not None and self._estop_active and self._dock.phase == "going":
            with self._dock_lock:   # 비상 콜백이 잠금을 못 잡아 남긴 것(_on_estop)
                self._dock_apply(self._dock.on_estop(True))
        if self._dock is not None and self._now() >= self._dock_far_check_at:
            self._dock_far_check_at = self._now() + 1.0
            self._dock_far_check()
        distance = (
            self._nav_distance_remaining() if status == NavStatus.RUNNING else None
        )
        # Nav2 준비 여부는 손 놓침 뒤 자동 재출발에만 쓴다 — 그때만 묻는다.
        nav_ready = (self._nav2_ready()
                     if self.logic.dialog_state == DIALOG_PAUSED_HANDLE else True)
        dialog_before = self.logic.dialog_state
        actions = self.logic.on_tick(self._now(), status, distance, nav_ready=nav_ready)
        # 대장(P1): 확인 대기(CONFIRMING)였다가 출발 없이 접혔으면(거절·시간초과·호출로 접음)
        # 그 목적지가 "하려다 만 곳"이다. 전이표는 ledger.confirm_abort_name (순수
        # 함수, 최종 리뷰 6절 M4) — 여기는 결과만 복사·저장한다.
        confirming = self.logic.confirming_dest_id
        active_dest_id = self.logic.active_destination.id if self.logic.active_destination else None
        aborted_name = confirm_abort_name(
            self._ledger_prev_confirming, confirming, before, self.logic.state,
            active_dest_id, self.destinations,
        )
        if aborted_name is not None:
            self._ledger.aborted_destination = aborted_name
            self._save_ledger()
        self._ledger_prev_confirming = confirming
        self._run_actions(actions)
        self._drain_obstacle_cues()
        self._obstacle_inputs_follow()
        if State.SEEKING in (before, self.logic.state) and before != self.logic.state:
            # SEEKING 진입·이탈은 여기서도 일어난다(탐색 창 닫힘, 복귀 회전
            # 시작·종료). /vica/robot_state 1 Hz 의 최대 1초 지연을 즉시
            # 갱신으로 회수한다 — _on_wake_doa 와 같은 이유(2026-09-10 재검토).
            self._publish_robot_state()
        elif "patrol" in (dialog_before[:6], self.logic.dialog_state[:6]):
            # 순찰 상태가 틱 안에서 바뀌었으면(도착·지점 정지 끝·카메라 끊김·종료) 곧바로 방송한다.
            self._patrol_follow(dialog_before, "틱")
        elif dialog_before != self.logic.dialog_state:
            # 손잡이 대화 단계(grip_wait·paused_handle)가 바뀌었다. 손 놓쳐 선
            # 사용자가 곧바로 "왜 안 가요?"라고 물을 수 있어 1 Hz 를 기다리지 않는다.
            self.get_logger().info(
                f"손잡이: {dialog_before} → {self.logic.dialog_state} "
                f"(활성={self.logic.handle_active})")
            self._publish_robot_state()

    def _publish_robot_state(self) -> None:
        msg = RobotState()
        floor_param = int(self.get_parameter("current_floor").value)
        building_param = str(self.get_parameter("current_building").value)
        # 층·건물: launch 인자가 있으면 우선, 없으면 목적지 폴더의 map.yaml (지도 = 한 층)
        msg.current_floor = floor_param if floor_param >= 0 else self._map_meta.floor
        msg.current_building = building_param or self._map_meta.building
        # SEEKING(제자리 회전) 중 영상은 화면이 통째로 흐른다. detection_gate 의
        # 변위 관문(0.3 m)이 어차피 트랙을 계속 깨뜨리므로, 쓰이지 않을 추론에
        # CPU 를 쓰지 않는다 — 이 로봇의 제1 병목은 CPU 다.
        # APPROACHING 을 넣지 않는 것은 의도된 현행 유지다: 접근 중에는 목표점
        # 갱신에 탐지가 필요하다.
        # 대기 장소로 혼자 가는 두 상태도 바퀴가 돈다(2026-10-07).
        # 순찰 구간 이동(PATROLLING)도 바퀴가 돈다(2026-11-14).
        msg.is_moving = self.logic.state in (
            State.NAVIGATING, State.SEEKING,
            State.MOVING_TO_WAIT_SPOT, State.MOVING_BACK_TO_DEST, State.PATROLLING)
        # LLM 이 "다시 출발"을 이해하려면 그냥 정지와 일시정지를 구분해야 한다.
        # 순찰 중 관리자 일시정지(PATROL_PAUSED)도 일시정지다.
        msg.is_paused = self.logic.state in (State.PAUSED, State.PATROL_PAUSED)
        # dialog_state 는 state.value 에 손잡이 두 단계(grip_wait·paused_handle)를
        # 더한 것이다(2026-09-30, 설계 4.7). is_paused 의 뜻은 그대로다.
        fields = state_fields(
            self._ledger, self.logic.dialog_state, self._robot_pose, self._pose_cov_xy,
            self.destinations, now_epoch=time.time(),
            wait_minutes=self.logic.wait_minutes_requested(),
            wait_left_sec=self.logic.wait_left_sec(self._now()),
            door_side=self.logic.door_side,
            wait_place=self.logic.wait_place,
        )
        for key, value in fields.items():
            setattr(msg, key, value)
        self.pub_state.publish(msg)

    # -- 질문 기록 (설계 2026-11-14 4.1) -------------------------------------------

    def _on_listen_state(self, msg: String) -> None:
        """음성의 듣기 결과. 빈손으로 끝났으면 열린 질문을 먼저 닫는다 — 뒤이은 되묻기가 새 질문을 연다."""
        self._question_safely(self._listen_question, msg.data)
        self._run_actions(self.logic.on_listen_state(msg.data, self._now()))

    def _question_safely(self, fn, *args) -> None:
        """질문 기록은 기록일 뿐이다 — 여기서 난 예외가 미션의 말·동작을 막거나 노드를 죽이지 않게 한다."""
        try:
            fn(*args)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"[질문] 기록 실패 — 미션은 계속: {exc!r}")

    def _listen_question(self, state: str) -> None:
        with self._question_lock:
            closed = self._questions.on_listen_state(state, self._now())
        if closed is not None:
            self._close_question("silence", closed)

    def _tick_question(self) -> None:
        """상태가 바뀌었거나 음성에서 아무 신호 없이 오래 열려 있으면 닫는다."""
        with self._question_lock:
            closed = self._questions.tick(self.logic.state.value, self._now())
        if closed is not None:
            self._close_question(closed[1], closed[0])

    def _publish_question(self, record: Optional[QuestionRecord]) -> None:
        """기록을 /vica/question 으로 낸다. None 이면 빈 기록(열린 질문 없음)."""
        if self.pub_question is None:
            return
        msg = VicaQuestion()
        if record is not None:
            msg.id, msg.kind, msg.text = record.id, record.kind, record.text
            msg.option_ids = [o[0] for o in record.options]
            msg.option_names = [o[1] for o in record.options]
            msg.step, msg.on_silence = record.step, record.on_silence
            msg.opened_at = self.get_clock().now().to_msg()
        self.pub_question.publish(msg)

    def _open_question(self, say: Say) -> None:
        if not say.kind:
            self.get_logger().warn(f"[질문] kind 없음 — open 으로 냄: {say.text}")
        with self._question_lock:
            old, record = self._questions.open(say, self.logic.state.value, self._now())
        if old is not None:
            self._log_question_closed(old, "replaced")
        self._publish_question(record)
        self.get_logger().info(
            f"[질문] 열림 id={record.id} kind={record.kind} 후보={len(record.options)} "
            f"요청={'y' if say.expects_reply else 'n'} 글자={record.text}"
            + (f" 되묻기={record.step}" if record.step else ""))

    def _close_question(self, reason: str, record: Optional[QuestionRecord] = None) -> None:
        """열린 질문을 닫고 빈 기록을 낸다. record 를 주면 이미 닫힌 것을 알리기만 한다."""
        if record is None:
            with self._question_lock:
                record = self._questions.close(reason)
        if record is None:
            return
        self._log_question_closed(record, reason)
        self._publish_question(None)

    def _log_question_closed(self, record: QuestionRecord, reason: str) -> None:
        text = f"[질문] 닫힘 id={record.id} 사유={reason} 열린 시간={self._now() - record.opened_at:.1f}s"
        # 심각도마다 다른 줄에서 부른다 — rclpy 는 같은 호출 줄에서 심각도가 바뀌면 ValueError 를 낸다.
        if reason == "no_signal":
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    # -- Action 실행 -------------------------------------------------------------

    def _run_actions(self, actions) -> None:
        actions = list(actions)
        for index, action in enumerate(actions):
            if isinstance(action, Say):
                out = String()
                # ros_tts_node 큐 우선순위 접두어 (vica-voice-llm src/tts_queue.py 계약)
                out.data = f"{action.priority}:{action.text}"
                self.pub_tts.publish(out)
                self.get_logger().info(f"TTS[{action.priority}]: {action.text}")
                if action.kind or getattr(action, "expects_reply", False):
                    self._question_safely(self._open_question, action)
                if getattr(action, "expects_reply", False):
                    self.pub_listen_request.publish(Bool(data=True))
                    # 같은 질문인지는 원래 질문 글자로 본다 — 세 단계 되묻기 ①·② 는 문장이 달라도 같은 질문이다(원칙 8).
                    self.logic.note_question_asked(action.question_text or action.text, self._now())
                    self.get_logger().info("질문 발화 — 재청취 요청 발행")
            elif isinstance(action, StopSpeech):
                out = String()
                out.data = "control:stop"
                self.pub_tts.publish(out)
                self.get_logger().info("발화 큐 청소 (취소·선점)")
            elif isinstance(action, WakeReply):
                # "비카야" 판정 — 웨이크워드 노드가 호출 창의 말을 넘길지 버릴지 정한다.
                verdict = "listen" if action.listen else "ignore"
                seq = self._wake_reply_seq
                self.pub_wake_reply.publish(String(data=f"{verdict}:{seq}" if seq else verdict))
            elif isinstance(action, CancelNav):
                if self._dock is not None and self._dock.held is not None:
                    # 도크에서 나온 뒤 보내려고 잡아 둔 주행이 취소됐다. 같은 묶음 뒤쪽에 새 주행이 있으면(앱이 다른
                    # 곳으로 다시 보냄) 나오기는 멈추지 않고 잡아 둔 주행만 바꾼다 — 멈추면 새 주행이 도크 안에서
                    # 그대로 Nav2 로 나갔다(11-19 검토).
                    follows = any(isinstance(a, (Navigate, SpinInPlace)) for a in actions[index + 1:])
                    # 비상 콜백에서도 온다 — 잠금을 오래 기다리지 않는다. 못 잡으면 건너뛴다: 비상정지면 도킹 노드가
                    # 스스로 멈추고 그 결과로 잡아 둔 주행이 버려진다(release_held 는 비상정지 중에 보내지 않는다).
                    if self._dock_lock.acquire(timeout=0.2):
                        try:
                            self._dock_apply(self._dock.on_mission_cancel(keep_leaving=follows))
                        finally:
                            self._dock_lock.release()
                    else:
                        self.get_logger().warn("도킹 잠금을 0.2 s 안에 못 잡아 잡아 둔 주행 취소를 건너뛴다")
                self._cancel_nav(action.destination, action.event)
            elif isinstance(action, Navigate):
                if not self._dock_hold(action):
                    self._start_nav(action)
            elif isinstance(action, SpinInPlace):
                if not self._dock_hold(action):
                    self._start_spin(action)
            elif isinstance(action, SetNavSpeedLimit):
                self._publish_nav_speed_limit(action.percent)
            elif isinstance(action, GoalEvent):
                # 미션이 판단한 사건(대기 장소 막힘·대기 시간 만료) — 앱 알림.
                self._publish_goal_event(
                    action.event, action.destination, action.reason,
                    extra={"wait_place": action.wait_place,
                           "wait_minutes": action.wait_minutes})
                self.get_logger().info(f"앱 알림: {action.event} {action.reason}")
            elif isinstance(action, HelpEvent):
                self._publish_help_event(action)
            elif isinstance(action, PatrolEvent):
                self._publish_patrol_event(action)
            elif isinstance(action, PatrolRecordClip):
                # 경보 뒤에 알게 된 영상 id — 기록 파일에만 덧붙인다.
                self._patrol_record(clip_record(action.alarm_id, action.clip_id))
            elif isinstance(action, SetNavSpeedCap):
                self._publish_nav_speed_cap(action.mps)
            elif isinstance(action, Haptic):
                out = String()
                out.data = action.pattern
                self.pub_haptic.publish(out)
                self.get_logger().info(f"진동 요청: {action.pattern}")

    def _publish_help_event(self, action: HelpEvent) -> None:
        """도움 요청 사건을 /vica_goal_event 로 낸다(목적지 칸은 빈 값). help_requested 는 보낼
        때마다 그 순간의 위치를 싣는다 — place 는 상황판 place_here 와 같은 문장, robot_x·y 는
        AMCL. 기존 x·y 는 목적지 좌표 칸이라 쓰지 않는다."""
        extra: dict = {"help_id": action.help_id}
        if action.event == "help_requested":
            pose = self._robot_pose
            place, _ = place_here(pose, self.destinations, self._pose_cov_xy)
            extra.update({
                "place": place,
                "robot_x": pose.x if pose is not None else None,
                "robot_y": pose.y if pose is not None else None,
            })
        self._publish_goal_event(action.event, None, extra=extra)
        self.get_logger().info(f"앱 알림: {action.event} {action.help_id} {extra.get('place', '')}")

    def _publish_nav_speed_limit(self, percent: float) -> None:
        msg = SpeedLimit()
        msg.percentage = True
        msg.speed_limit = float(percent)
        self.pub_speed_limit.publish(msg)
        if percent == 0.0:
            self.get_logger().info("Nav2 접근 속도 제한 해제")
        else:
            self.get_logger().info(f"Nav2 접근 속도 제한: 최대속도의 {percent:.1f}%")

    def _publish_nav_speed_cap(self, mps: float) -> None:
        """순찰 구간 속도 상한(m/s). 같은 /speed_limit 을 절대값으로 낸다 — 푸는 것은 0.0(위)."""
        msg = SpeedLimit()
        msg.percentage = False
        msg.speed_limit = float(mps)
        self.pub_speed_limit.publish(msg)
        self.get_logger().info(f"Nav2 속도 상한(순찰): {mps:.2f} m/s")

    def _resolve_bt(self, value: str, file_name: str, label: str) -> str:
        """*_bt_xml 파라미터를 실제 파일 경로로 바꾼다. 못 쓰면 "" (끔 = 기본 트리).

        "auto" 는 vica_nav2 share 의 behavior_trees/<file_name> 이다. 사람 접근 트리에서
        시작한 규칙을 안내·대기 장소 트리도 그대로 쓴다(2026-10-07).
        """
        if not value:
            self.get_logger().info(f"{label}: 끔 — 기본 트리(레일)를 쓴다")
            return ""
        if value == "auto":
            try:
                from ament_index_python.packages import get_package_share_directory
                value = str(Path(get_package_share_directory("vica_nav2"))
                            / "behavior_trees" / file_name)
            except Exception as exc:  # noqa: BLE001 - 패키지가 없어도 노드는 뜬다
                self.get_logger().warn(
                    f"{label}를 못 찾아 끔 — 기본 트리(레일)를 쓴다: {exc}")
                return ""
        if not Path(value).is_file():
            self.get_logger().warn(
                f"{label} 파일이 없어 끔 — 기본 트리(레일)를 쓴다: {value}")
            return ""
        self.get_logger().info(f"{label}: {value}")
        return value

    def _finish_task_for_bt_switch(self) -> None:
        """트리가 바뀌는 goal 을 보내기 전에 앞 task 를 끝낸다. _nav_lock 안에서 부른다.

        Humble bt_navigator 는 실행 중인 goal 과 다른 트리 파일을 쓰는 goal 의 선점을
        거절하고 옛 goal 을 계속 따라간다(navigate_to_pose.cpp onPreempt). 그대로 보내면
        새 goal 이 수락 직후 종료돼 FAILED 로 읽힌다. 그래서 앞 task 가 아직 돌고 있으면
        취소하고, 끝났다는 응답을 BT_SWITCH_WAIT_SEC 까지 기다린다. 트리는 접근을
        시작하거나 끝낼 때만 바뀌므로 드물게만 기다린다.
        """
        try:
            if self.navigator.isTaskComplete():
                return
            self.get_logger().info("트리가 바뀌는 goal — 앞 task 를 먼저 취소한다")
            self.navigator.cancelTask()
            deadline = time.monotonic() + BT_SWITCH_WAIT_SEC
            while not self.navigator.isTaskComplete():   # 한 번에 0.1 s 씩 돈다
                if time.monotonic() > deadline:
                    self.get_logger().warn(
                        f"앞 task 가 {BT_SWITCH_WAIT_SEC:.1f}초 안에 안 끝났다 — 그대로 보낸다")
                    return
        except Exception as exc:  # noqa: BLE001 - 보내기 자체는 막지 않는다
            self.get_logger().warn(f"트리 전환 전 취소 실패: {exc}")

    def _clear_costmaps_before_departure(self, dest_id: str) -> None:
        """새 출발 직전에 local·global costmap 을 한 번 비운다. _nav_lock 안에서 부른다.

        local 왜(2026-10-09 run81·82): 초음파 표시는 빔 밖으로 나가면 지워지지 않는다. 접근·
        도착 끝의 회전(마지막 정렬, 손잡이 돌리기)으로 빔 밖에 남은 표시가 출발 회전을
        막아, 10초 포기(FollowPath 실패 -> BT 의 ClearLocalCostmap)가 비울 때까지 섰다 —
        run81 15:48 홈, run82 18:14 사회 복지창구, 각 10 s. 그 서비스를 출발 순간에
        미리 부른다. 라이다·카메라 표시도 함께 지워지지만 local costmap 이 1초에 5번 다시
        그리고, 출발 명령 -> VCC 첫 움직임이 약 2 s(run82 18:14:02.45 -> 04.50)라
        움직이기 전에 돌아온다. 대가: 지금 센서가 못 보는 옆·뒤의 낮은 물체·상판 기억도
        사라진다(10초 포기 때와 같은 일을 10초 먼저 한다).

        global 왜(2026-10-10 run84 뒤 사용자 결정): 레일 앞 검사(IsRailAheadClear)는 global
        costmap 을 본다. 초음파 표시는 앞 빔이 1.5 m 안에서 그 자리를 다시 비춰야만 지워져,
        접근 때 그린 상자 표시가 상자를 치운 뒤에도 남았다 — 15:26:55 에 그린 (5.41, 0.25)
        표시는 빔이 다시 본 15:31:18 까지 레일을 막았고, 로봇은 그 자리를 비켜 가느라 다시
        보지 못했다(그 사이 15:29 창구 안내가 왼쪽 위 코너를 잘랐다). 출발 때 상자는 앞
        2.7~3.1 m(빔 1.5 m 밖)라, 비운 뒤에는 빔이 1.5 m 안에서 실제로 볼 때만 다시 그린다.
        같은 서비스를 BT 복구도 부른다. 비워도 금지구역은 옛 마스크를 그대로 쓰다가 0.07 s
        안에 다시 받고(10-10 16:32 planner_server 로그), 라이다 표시는 다음 갱신(1 Hz)에
        돌아온다. 레일 검사는 2번 이어져야 막힘이라 비우기 전 지도를 한 박자 읽어도 레일을
        안 버린다. 대가: 지금 안 보이는 낮은 물체(초음파만 보는 것) 기억도 사라져, 다시
        만나면 1.5 m 안에서 새로 찾는다.

        BasicNavigator.clearLocalCostmap()·clearGlobalCostmap() 은 서비스가 없으면 끝없이
        기다리고 응답도 시한 없이 기다린다 — 같은 클라이언트를 시한부로 직접 부른다. 두 요청을
        함께 보내고 시한 하나(CLEAR_BEFORE_DEPARTURE_TIMEOUT_SEC) 안에서 함께 기다린다.
        실패해도 출발은 막지 않는다(지금까지와 같은 상태이고 10초 포기는 그대로 남는다).
        """
        try:
            # 앞 task 가 아직 돌고 있으면(회전 중 앱 선점 — 취소 스레드보다 먼저 왔다)
            # 로봇이 움직이는 중이다. 움직이는 중에는 비우지 않는다(검토 2-a).
            if not self.navigator.isTaskComplete():
                self.get_logger().warn(
                    f"출발 전 costmap 비우기 건너뜀 — 앞 task 가 아직 돈다 ({dest_id})")
                return
            sent = []
            for name, client in (("local", self.navigator.clear_costmap_local_srv),
                                 ("global", self.navigator.clear_costmap_global_srv)):
                if not client.service_is_ready():
                    self.get_logger().warn(
                        f"출발 전 {name} costmap 비우기 건너뜀 — 서비스 없음 ({dest_id})")
                    continue
                sent.append((name, client, client.call_async(ClearEntireCostmap.Request())))
            deadline = time.monotonic() + CLEAR_BEFORE_DEPARTURE_TIMEOUT_SEC
            for name, client, future in sent:
                left = deadline - time.monotonic()
                if not future.done() and left > 0.0:
                    rclpy.spin_until_future_complete(self.navigator, future, timeout_sec=left)
                if not future.done():
                    client.remove_pending_request(future)
                    self.get_logger().warn(
                        f"출발 전 {name} costmap 비우기 응답 없음 "
                        f"({CLEAR_BEFORE_DEPARTURE_TIMEOUT_SEC:.1f} s) — 그대로 출발 ({dest_id})")
                    continue
                self.get_logger().info(f"출발 전 {name} costmap 비움 ({dest_id})")
        except Exception as exc:  # noqa: BLE001 - 보내기 자체는 막지 않는다
            self.get_logger().warn(f"출발 전 costmap 비우기 실패: {exc}")

    def _patrol_leg_stale(self) -> bool:
        """순찰 구간을 보내려는데 순찰이 이미 끝났나(비상정지 스레드가 그 사이 끼어든 경우)."""
        return self.logic.state != State.PATROLLING or self.logic.estop_active

    def _start_nav(self, action: Navigate) -> None:
        dest = action.destination
        # 순찰 구간(2026-11-14)은 목적지 id 로 안다 — 상태는 비상정지 스레드가 그 사이 바꿀 수 있다.
        # 순찰 구간은 goal_* 사건을 내지 않고, 순찰이 끝난 뒤에는 보내지 않는다.
        patrol_leg = dest.id.startswith(PATROL_DESTINATION_PREFIX)
        # 도크 앞(준비 위치)으로 가는 주행(2026-11-18) — 앱에 일반 주행 알림을 내지 않고, 결과는 도킹 흐름이 받는다.
        dock_nav = dest.id == DOCK_DESTINATION_ID
        if patrol_leg and self._patrol_leg_stale():
            self.get_logger().warn(f"순찰 구간 전송 안 함 — 그 사이 순찰이 끝났다 ({dest.id})")
            return
        if self._delivery_dest_id is not None and dest.id != self._delivery_dest_id:
            # 배송지가 아닌 곳으로 출발한다(음성 안내·홈·접근 등) — 배송은 끝났다.
            self._delivery_dest_id = None
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

        # Nav2 액션 서버가 없으면 보내지 않는다(2026-10-07 검토). Humble goToPose 는
        # 서버가 없으면 wait_for_server(1.0)을 끝없이 되풀이해 주 콜백 그룹을 붙잡는다 —
        # 틱·"비카야" 판정·앱 서비스가 모두 멈춘다. 관문(check_gate)을 거치지 않는
        # 재시도·재개·주행 중 바꾸기의 원래 목적지 재출발도 여기서 함께 막는다.
        if not self._nav2_ready():
            if not (patrol_leg or dock_nav or self._quiet_goal_events()):
                self._publish_goal_event("goal_rejected", dest, "Nav2 가 준비되지 않았습니다.")
            self.get_logger().error(f"NavigateToPose 전송 보류: Nav2 미준비 ({dest.id})")
            self._nav_send_failed(dock_nav)
            return
        # 앞선 취소가 아직 Nav2 응답을 기다리는 중이면 여기서 영원히 기다리지
        # 않는다. 못 보낸 것을 실패로 처리하는 편이 콜백을 막는 것보다 낫다.
        if not self._nav_lock.acquire(timeout=self._nav_lock_timeout_sec):
            if not (patrol_leg or dock_nav):   # 순찰 구간·도크 앞은 goal_* 사건을 내지 않는다
                self._publish_goal_event(
                    "goal_rejected", dest, "이전 goal 취소가 아직 처리 중입니다."
                )
            self.get_logger().error(
                f"NavigateToPose 전송 보류: 이전 취소가 "
                f"{self._nav_lock_timeout_sec:.1f}초 안에 끝나지 않았다 ({dest.id})"
            )
            self._nav_send_failed(dock_nav)
            return
        # 사람 접근 goal 은 접근 트리, 나머지는 Navigate.tree 가 고른 트리
        # (빈 문자열 = 기본 트리). 2026-10-07 안내(위치만)·대기 장소 트리 추가.
        bt = (nav_behavior_tree(dest.id, self._approach_bt)
              or self._tree_files.get(action.tree, ""))
        # 안내 트리는 레일을 부른다. 레일 서버가 없는 지도(레일 파일 없음·use_route 끔)면
        # 레일 없는 안내 트리로 바꾼다 — nav2 launch 가 기본 트리를 고르는 것과 같은 판단.
        if action.tree == NAV_TREE_GUIDED and bt and not self._route_server_up():
            bt = self._guided_no_rail_bt
        # 대기 장소로 혼자 가는 주행은 앱에 일반 주행 알림을 내지 않는다 — 앱에는
        # 미션이 따로 내는 wait_spot_blocked·wait_expired 만 뜬다(주행 실패 팝업 중복 방지).
        quiet = patrol_leg or dock_nav or self._quiet_goal_events()
        try:
            if bt != self._task_bt:
                self._finish_task_for_bt_switch()
            if self._clear_before_next_nav:
                self._clear_costmaps_before_departure(dest.id)
            accepted = self.navigator.goToPose(goal, behavior_tree=bt)
            self._nav_active = bool(accepted)
            if accepted:
                # 번호표를 올린다. 거부됐으면 올리지 않는다 — 옛 goal 이
                # 그대로 잡혀 있으므로, 대기 중인 취소가 그것을 마저 지워야 한다.
                self._nav_gen += 1
                self._task_bt = bt
                # 주행이 시작됐다 — 이 goal 이 성공으로 끝나기 전의 다음 출발(재개·재시도·
                # 주행 중 바꾸기)은 비우지 않는다. 거부됐으면 그대로 두어 다시 보낼 때 비운다.
                self._clear_before_next_nav = False
                self._nav_task_is_spin = False
        finally:
            self._nav_lock.release()
        if accepted and patrol_leg and self._patrol_leg_stale():
            # 구간을 보내는 사이(costmap 비우기·수락 대기) 비상정지·종료가 끼어들었다. 그때의 취소는
            # 아직 goal 이 없어 빈손으로 끝났다 — 방금 보낸 goal 을 거둔다. 안 거두면 래치가 풀린 뒤
            # Nav2 가 그 지점으로 혼자 간다(해제 뒤 스스로 다시 출발하지 않는다, SPEC 1절 8항).
            self.get_logger().warn(f"순찰 구간 전송 직후 취소 — 그 사이 순찰이 끝났다 ({dest.id})")
            self._cancel_nav(None)
            return
        if accepted:
            if not quiet:
                self._publish_goal_event("goal_sent", dest)
                self._publish_goal_event("goal_accepted", dest)
            self.get_logger().info(
                f"NavigateToPose 전송: {dest.id} ({dest.pose.x:.2f}, {dest.pose.y:.2f}, "
                f"{dest.pose.yaw_deg:.1f}deg) 트리={Path(bt).name if bt else '기본'}"
            )
        else:
            if not quiet:
                self._publish_goal_event("goal_rejected", dest, "Nav2 goal rejected")
            # goal 거부 → 다음 tick 에서 FAILED 처리되도록 상태를 만든다.
            self.get_logger().error(f"NavigateToPose goal 거부됨: {dest.id}")
            self._nav_send_failed(dock_nav)

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
                self._nav_task_is_spin = True
        if accepted:
            self.get_logger().info(
                f"제자리 회전 시작: {action.yaw_rad:+.2f} rad "
                f"({math.degrees(action.yaw_rad):+.0f}°) — {action.reason}"
            )
        else:
            self.get_logger().error(f"Spin 거부됨 ({action.reason}) - 회전 없이 넘어간다")
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
            if not self.navigator.isTaskComplete(timeout_sec=NAV_POLL_WAIT_SEC):
                return NavStatus.RUNNING
            self._nav_active = False
            result = self.navigator.getResult()
        if not self._nav_task_is_spin and result == TaskResult.SUCCEEDED:
            # 안내(주행)가 성공으로 끝났다 — 다음 새 출발 직전에 local·global costmap 을 비운다.
            self._clear_before_next_nav = True
        # 복귀 중이면 목적지가 홈이다. 도착 여부가 곧 "이 홈에 갈 수 있는가"의
        # 답이므로 여기서 home.yaml 의 visited_ok 를 기록한다.
        returning = self.logic.state == State.RETURNING
        if self._quiet_goal_events():
            # 대기 장소로 혼자 가는 주행 — 결과는 로직이 판단해 따로 알린다.
            if result == TaskResult.SUCCEEDED:
                return NavStatus.SUCCEEDED
            if result == TaskResult.CANCELED:
                return NavStatus.CANCELED
            return NavStatus.FAILED
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

    def _route_server_up(self) -> bool:
        """레일 서버(compute_route 액션)가 떠 있는가. 모르면 False — 레일 없는 트리는
        레일이 있어도 동작하지만(자유주행), 레일 트리는 레일이 없으면 통째로 실패한다."""
        try:
            from rclpy.action.graph import get_action_names_and_types
            names = get_action_names_and_types(self)
        except Exception:  # noqa: BLE001 - 그래프 조회 실패는 '없음'으로 본다
            return False
        return any(name.rstrip("/").endswith("compute_route") for name, _ in names)

    # ── 도킹 (2026-11-18) ────────────────────────────────────────────────
    _DOCK_REPLY = {
        "ok": "도킹을 시작합니다.",
        "estop": "비상정지 중에는 도킹할 수 없습니다.",
        "already_docked": "이미 도크에 들어가 있습니다.",
        "busy": "다른 일을 하는 중이라 지금은 도킹할 수 없습니다.",
        "no_dock": "도크 위치가 저장되지 않았습니다. 지도 설정에서 먼저 저장하세요.",
        "nav_not_ready": "Nav2 가 준비되지 않았습니다.",
        "not_docked": "도크에 들어가 있지 않습니다.",
        "idle": "멈출 도킹이 없습니다.",
    }

    def _nav_send_failed(self, dock_nav: bool) -> None:
        """goal 을 못 보냈다 — 다음 tick 이 아니라 지금 실패로 넘긴다."""
        if dock_nav:
            self._dock_apply(self._dock.on_nav_status("failed"))
        else:
            self._run_actions(self.logic.on_tick(self._now(), NavStatus.FAILED))

    def _dock_apply(self, cmds: list) -> None:
        """도킹 흐름이 낸 명령을 실행하고, 로직에 도킹 단계를 알린다(dialog_state·음성 막기)."""
        with self._dock_lock:
            before = self.logic.dock_mode
            self.logic.dock_mode = self._dock.mode
            for c in cmds:
                if c.kind == "call":
                    self._dock_call(c.arg)
                elif c.kind == "nav":
                    self._start_nav(Navigate(c.arg))
                elif c.kind == "restage":
                    # 재시도 — 도킹 노드가 고쳐 둔 도크 위치를 다시 읽는다(11-19)
                    staging = self._dock_staging()
                    in_zone, near_spin = self._dock_near(staging)
                    self.get_logger().info("도킹 재시도: 도크 위치를 다시 읽어 준비 위치로")
                    self._dock_apply(self._dock.restage(staging, near_spin, in_zone))
                elif c.kind == "spin":
                    self._start_spin(SpinInPlace(c.arg, reason="도킹: 제자리에서 도크 보기"))
                    if not self._nav_active:   # Spin 을 못 보냈다 — 다음 tick 을 기다리지 않고 실패로 넘긴다
                        self._dock_apply(self._dock.on_nav_status("failed"))
                elif c.kind == "cancel_nav":
                    self._cancel_nav(None)
                elif c.kind == "release_held" and self._estop_active:
                    # 나오기는 끝났는데 그 사이 비상정지가 걸렸다 — 보내지 않는다(비상정지 뒤 자동으로 다시 가지 않는다)
                    self.get_logger().warn("도크에서 나왔지만 비상정지 중 — 잡아 둔 주행을 취소로 끝낸다")
                    self._run_actions(self.logic.on_dock_held_dropped(self._now()))
                elif c.kind == "release_held":
                    self.get_logger().info("도크에서 나왔다 — 잡아 둔 주행을 보낸다")
                    if isinstance(c.arg, SpinInPlace):
                        self._start_spin(c.arg)
                    else:
                        self._start_nav(c.arg)
                elif c.kind == "drop_held":
                    # 도크에서 못 나왔다 — 미션의 주행은 취소로 끝낸다(재시도하지 않는다, 순찰이면 순찰을 끝낸다)
                    self.get_logger().warn("도크에서 나오지 못해 잡아 둔 주행을 취소로 끝낸다")
                    self._run_actions(self.logic.on_dock_held_dropped(self._now()))
                elif c.kind == "event":
                    self._publish_goal_event(c.arg, self._dock.staging, c.message)
                    self.get_logger().info(f"도킹 사건 {c.arg} {c.message}")
            self.logic.dock_mode = self._dock.mode
            self.logic.dock_pending = self._dock.undock_pending and not self._dock.mode
            # 도킹을 중지해 도크 바로 앞에 서 있다 — 부른 쪽으로 고개를 돌리지 않는다(돌리려면 먼저 나와야 한다)
            self.logic.dock_near = (self._dock.near_dock and not self._dock.mode
                                    and not self._dock.undock_pending)
            if self.logic.dock_mode != before:
                self.get_logger().info(f"도킹 단계: '{before}' → '{self.logic.dock_mode}'")
                self._publish_robot_state()   # 앱·손잡이 LED 가 1 Hz 를 기다리지 않게

    def _dock_hold(self, action) -> bool:
        """미션 주행이 나가려 한다 — 도킹 흐름이 가로챌지 정한다. 가로챘으면(잡아 둠·버림) True.

        도크 안·근처에서 쉬는 중이면 잡아 두고 먼저 나온다. 흐름이 도는 중이면: 나오는 중에는 잡아 두고(나온 뒤 보냄),
        가는 중·들어가는 중·멈추는 중에는 보내지 않는다(DockFlow.intercept).
        """
        if self._dock is None:
            return False
        if isinstance(action, Navigate) and action.destination.id == DOCK_DESTINATION_ID:
            return False
        is_spin = isinstance(action, SpinInPlace)
        with self._dock_lock:
            verdict, cmds = self._dock.intercept(action, is_spin)
            if verdict == "pass":
                if self._dock.undock_pending and not self._dock.busy():
                    # 나오다 멈춘 자리(도크 밖)에서 원격 주행 — 관리자가 보고 보낸 것이라 그대로 간다(11-19 사용자).
                    # 도킹 노드의 '나오다 멈춤' 표시는 접는다(쉬는 중의 cancel).
                    self.get_logger().info("나오다 멈춘 자리에서 원격 주행 — 그대로 보낸다")
                    self._dock_call(CALL_CANCEL)
                return False
            if verdict == "held":
                self.get_logger().info("도크 안·나오는 중 — 도크에서 나온 뒤 보낸다")
                self._dock_apply(cmds)
                return True
            phase = self._dock.phase or "도크 앞"
        # drop — 보내지 않는다. 로직에는 회전이면 실패(회전 없이 넘어간다), 주행이면 취소로 알린다(순찰이면 끝낸다).
        self.get_logger().warn(
            f"도킹 관련 중({phase})이라 {'제자리 회전' if is_spin else '주행'}을 보내지 않는다")
        self._run_actions(self.logic.on_tick(self._now(), NavStatus.FAILED) if is_spin
                          else self.logic.on_dock_held_dropped(self._now()))
        return True

    def _dock_holding(self) -> bool:
        """도킹 흐름이 미션 주행을 잡아 두었는가(도크에서 나온 뒤 보낸다) — 요청은 받아들인 것이다."""
        return self._dock is not None and self._dock.held is not None

    _DOCK_REFUSE_DOCKING = "도킹 중입니다. '도킹 중지'를 누른 뒤 다시 보내 주세요."
    _DOCK_REFUSE_STOPPING = "도크에서 나오다 멈추는 중입니다. 잠시 뒤 다시 보내 주세요."

    def _dock_refuse(self, response, label: str) -> bool:
        """도킹 노드가 로봇을 움직이는 중이라 원격 주행을 받을 수 없으면 거절 응답을 채우고 True.

        가는 중·들어가는 중: 먼저 '도킹 중지'를 눌러야 한다(11-19 검토 — 그대로 받으면 Nav2 와 도킹 노드가
        /cmd_vel_req 를 다툰다). 나오는 중: 받는다(나온 뒤 보낸다). 나오다 멈추는 중: 결과가 올 때까지만 거절."""
        d = self._dock
        if d is None:
            return False
        if d.phase in ("going", "entering"):
            message = self._DOCK_REFUSE_DOCKING
        elif d.phase == "leaving" and d.canceling:
            message = self._DOCK_REFUSE_STOPPING
        else:
            return False
        response.accepted = False
        response.message = message
        self.get_logger().warn(f"{label} 요청 거부: 도킹 흐름 {d.phase}")
        return True

    def _dock_call(self, name: str) -> None:
        cli = self._dock_clients[name]
        if not cli.service_is_ready():
            self.get_logger().error(f"/vica/dock/{name} 없음 — vica_docking 노드가 떠 있지 않다")
            self._dock_apply(self._dock.on_call_result(name, False, "dock_node_down"))
            return
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda f, n=name: self._on_dock_call_done(n, f))

    def _on_dock_call_done(self, name: str, fut) -> None:
        try:
            res = fut.result()
            ok, msg = bool(res.success), str(res.message)
        except Exception as exc:  # noqa: BLE001
            ok, msg = False, f"call_failed: {exc!r}"
        if not ok:
            self.get_logger().warn(f"/vica/dock/{name} 거절: {msg}")
        with self._dock_lock:   # 응답 콜백은 주 콜백 그룹 밖에서 돌 수 있다 — 흐름을 고치는 것도 잠금 안에서
            self._dock_apply(self._dock.on_call_result(name, ok, msg))

    def _on_dock_state(self, msg: String) -> None:
        st = parse_node_state(msg.data)
        if st is None:
            return
        with self._dock_lock:
            self._dock_apply(self._dock.on_node_state(st))

    def _dock_staging(self):
        spot = read_dock_spot(dock_yaml_path(self._destinations_path))
        if spot is None or spot.map_id != self._map_id:
            return None
        return staging_destination(spot)

    def _dock_near(self, staging) -> tuple:
        """(도크 구역 안인가, 준비 위치 바로 옆이면 도크를 보려고 돌 각 rad·아니면 None). 위치를 모르면 (False, None)."""
        pose = getattr(self, "_robot_pose", None)
        if staging is None or pose is None:
            return False, None
        along, lateral, turn = staging_relation(
            pose.x, pose.y, pose.yaw_deg, staging.pose.x, staging.pose.y, staging.pose.yaw_deg)
        kind = classify_start(along, lateral)
        return kind == "zone", (turn if kind == "spin" else None)

    def _dock_far_check(self) -> None:
        """도크 근처·나오다 멈춤 표시가 낡았는가(1 s 마다). 도킹 노드는 일이 끝날 때만 이 표시를 고친다 — 그 뒤 앱 수동
        주행 등으로 로봇이 도크에서 멀어져도 그대로 남아, 다음 원격 주행이 '도크에서 나오기'부터 하다 실패한다.
        로봇 위치가 도크에서 DOCK_FAR_M 넘게 떨어졌으면 표시를 내린다(위치를 모르면 그대로 둔다)."""
        d = self._dock
        if d is None or d.busy() or d.mode or not (d.near_dock or d.undock_pending):
            return
        pose = getattr(self, "_robot_pose", None)
        if pose is None or self._pose_cov_xy > DOCK_FAR_COV:
            return
        spot = read_dock_spot(dock_yaml_path(self._destinations_path))
        if spot is None or spot.map_id != self._map_id:
            return
        dist = math.hypot(pose.x - spot.apex_x, pose.y - spot.apex_y)
        if dist <= DOCK_FAR_M:
            return
        with self._dock_lock:
            if d.busy() or d.mode:
                return
            self.get_logger().info(f"도크에서 {dist:.1f} m 떨어졌다 — 도크 근처·나오다 멈춤 표시를 내린다")
            # 도킹 노드의 표시도 접는다(쉬는 중의 cancel). 안 접으면 다음 상태 발행 때 다시 올라온다(11-19 재검토).
            self._dock_call(CALL_CANCEL)
            d.undock_pending = False
            d.near_dock = False
            self._dock_apply([])

    def _on_dock_start(self, request: MissionCommand.Request,
                       response: MissionCommand.Response) -> MissionCommand.Response:
        """앱 '도킹 시작'(목업 62). 도크 앞 1 m 까지 Nav2 로 간 뒤 들어간다(바로 옆이면 제자리 회전)."""
        with self._dock_lock:
            staging = self._dock_staging()
            in_zone, near_spin = self._dock_near(staging)
            ok, reason, cmds = self._dock.request_start(
                self.logic.state == State.IDLE, self._estop_active, self._nav2_ready(),
                staging, near_spin, in_zone)
            self._dock_apply(cmds)
        if ok:
            # 도크로 가기 시작할 때 한 번 알린다(2026-11-19 사용자 지정 문구)
            self._run_actions([Say(MSG_DOCK_GO, priority="narration")])
        response.accepted = ok
        response.message = self._DOCK_REPLY.get(reason, reason)
        self.get_logger().info(f"앱 도킹 시작: {reason}")
        return response

    def _on_dock_undock(self, request: MissionCommand.Request,
                        response: MissionCommand.Response) -> MissionCommand.Response:
        """앱 '도크에서 나오기'(목업 67). 후진 → 제자리 회전 → 주행 모드(그 자리)."""
        with self._dock_lock:
            ok, reason, cmds = self._dock.request_undock(
                self.logic.state == State.IDLE, self._estop_active)
            self._dock_apply(cmds)
        response.accepted = ok
        response.message = "도크에서 나옵니다." if ok else self._DOCK_REPLY.get(reason, reason)
        self.get_logger().info(f"앱 도크에서 나오기: {reason}")
        return response

    def _on_dock_cancel(self, request: MissionCommand.Request,
                        response: MissionCommand.Response) -> MissionCommand.Response:
        """앱 '도킹 중지'·'나오기 중지'(목업 65). 그 자리에 멈추고 다시 하지 않는다."""
        with self._dock_lock:
            ok, reason, cmds = self._dock.request_cancel()
            self._dock_apply(cmds)
        response.accepted = ok
        response.message = "멈춥니다." if ok else self._DOCK_REPLY.get(reason, reason)
        self.get_logger().info(f"앱 도킹 중지: {reason}")
        return response

    def _quiet_goal_events(self) -> bool:
        """지금 주행이 앱에 일반 주행 알림을 내지 않는 주행인가(대기 장소로 혼자 가는 길·순찰 구간).

        순찰 구간(2026-11-14)은 goal_sent·goal_succeeded 등을 내지 않는다 — 손잡이가 goal_succeeded 를
        도착 표시로, 앱이 도착 팝업으로 쓴다. 순찰은 patrol_* 사건을 로직이 따로 낸다.
        """
        return self.logic.state in (
            State.MOVING_TO_WAIT_SPOT, State.MOVING_BACK_TO_DEST, State.PATROLLING)

    def _record_home_visit(self, visited_ok: bool) -> None:
        """복귀 결과를 home.yaml 에 남긴다. 실패해도 주행 판정을 막지 않는다.

        실패여도 로봇이 홈 30 cm 안에 와 있으면 다녀온 것으로 본다(방향만 못 맞춘 경우, 11-19).
        정말 실패면 '가 봄'은 그대로 두고 '접근할 수 없음'만 남긴다.
        """
        reached = visited_ok or self._robot_near_home()
        try:
            home = self._home_storage.record_reach(self._map_id, reached)
        except (ValueError, OSError) as exc:
            self.get_logger().warn(f"홈 확인 상태를 기록하지 못했습니다: {exc}")
            return
        self._home_watch = (
            None if home is None or (home.visited_ok and not home.reach_failed) else (home.x, home.y)
        )

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

    def _publish_goal_event(self, event: str, destination, reason: str = "",
                            extra: Optional[dict] = None) -> None:
        """goal 사건을 앱에 알린다. destination 이 None 이면 좌표 없이 보낸다.

        목적지 없는 사건이 하나 있다 — 취소할 주행이 없을 때의 `state_idle`.
        빈 값으로 채우는 이유는 앱이 키 부재와 빈 값을 다르게 다루지 않기
        때문이고, 키를 빼면 옛 앱에서 KeyError 가 날 수 있어서다.
        """
        # 대장(P1): 사건 → 사실. 전이표는 ledger.apply_goal_event(순수 함수,
        # 최종 리뷰 4·6절 M1·M4) — 홈·사람 접근 합성 목적지는 거기서 이름을
        # ""로 걸러 대장에 안 남긴다. 여기는 결과만 복사·필요하면 저장한다.
        dest_id = destination.id if destination else ""
        dest_name = destination.name if destination else ""
        if apply_goal_event(self._ledger, event, dest_id, dest_name, time.time()):
            self._save_ledger()

        # 배송 도착 표시. 배송지의 도착에만 싣고, 그 배송이 끝나는 사건(도착·실패·취소·거부)에서
        # 기억을 지운다. 일시정지·출발 알림은 배송이 이어지므로 남긴다.
        delivery_extra = {}
        if dest_id and dest_id == self._delivery_dest_id:
            if event == "goal_succeeded":
                delivery_extra = {"delivery": True}
            if event in ("goal_succeeded", "goal_failed", "goal_canceled", "goal_rejected"):
                self._delivery_dest_id = None

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
                # 대기 알림(wait_spot_blocked·wait_expired)만 싣는 칸. 옛 앱은 모르는 키라 무시한다.
                **(extra or {}),
                # 배송지 도착에만 "delivery": true(2026-10-10). 옛 앱은 모르는 키라 무시한다.
                **delivery_extra,
            },
            ensure_ascii=False,
        )
        self.pub_goal_event.publish(msg)

    def _save_ledger(self) -> None:
        """대장 파일 갱신. 실패는 경고만 — 파일 오류로 노드가 죽지 않는다."""
        if not self._ledger_store.write(self._ledger):
            self.get_logger().warning(f"대장 파일 쓰기 실패: {self._ledger_store.path}")


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
