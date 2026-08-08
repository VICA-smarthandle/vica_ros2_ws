"""Judge Nav2 liveness from lifecycle_manager's own diagnostic.

`<node>/get_state` 서비스가 답을 주지 못할 때 쓰는 **두 번째 근거**다. lifecycle_manager는
자기가 관리하는 노드를 bond로 감시하므로, 그것이 active라고 말하면 그 관리 그룹은 살아 있다.
서비스 응답보다 약한 근거라 서비스가 답하지 못할 때만 쓴다.

ROS 의존이 없다. 판정은 전부 여기 있고 노드 파일은 값을 넘기기만 한다
(robot_health_monitor_node 상단 주석의 "판정 로직은 전부 순수 모듈에" 규칙).

세 가지를 함께 봐야 판정이 선다. 하나라도 빠지면 **Nav2가 죽은 상태에서 앱에 "주행 준비
완료"가 뜬다.**

문구
    nav2_lifecycle_manager는 활성이면 "Nav2 is active", 아니면 "Nav2 is inactive"를 낸다
    (libnav2_lifecycle_manager_core.so에서 문자열 확인, 2026-08-09). 부분 문자열로 보면
    **in**active가 active에 걸려 죽은 순간에 정상으로 읽힌다. 그래서 정규화 후 완전일치로
    본다. 모르는 문구는 활성이 아니다 — fail-safe 방향이다.

등급
    diagnostic_aggregator는 발행이 끊긴 항목의 message는 그대로 둔 채 level만 STALE로 바꿔
    재발행한다. 문구만 보면 "Nav2 is active"가 계속 보이므로 level도 함께 본다.

신선도
    monitor의 diag_items는 만료 제거를 하지 않는다. 신선도를 보지 않으면 Nav2가 죽은 뒤에도
    마지막 항목이 dict에 남아 활성 판정이 영구히 고정된다. 같은 파일의
    _worst_diag_by_component와 같은 is_fresh_ns 계약을 쓴다.

어느 manager를 볼지도 이름으로 갈라야 한다. localization과 navigation 둘 다
nav2_lifecycle_manager라서 진단 이름이 똑같이 `Nav2 Health`이고 문구도 똑같다
(bringup_launch.py가 두 launch를 모두 포함한다). manager 노드 이름으로 갈라지 않으면 AMCL만
살아 있어도 주행이 활성으로 읽힌다.
"""

from .agg_parser import DIAG_OK
from .freshness import is_fresh_ns


# nav2_lifecycle_manager가 활성일 때 내는 문구. 정규화 후 완전일치로 본다.
ACTIVE_MESSAGE = 'nav2 is active'

# 주행 lifecycle manager를 가리는 이름 조각. 둘 다 들어 있어야 근거로 쓴다.
# 평면형(`lifecycle_manager_navigation: Nav2 Health`)과 계층형
# (`/VICA/Navigation/lifecycle_manager_navigation: Nav2 Health`) 모두 통과한다.
NAV2_MANAGER_FRAGMENT = 'lifecycle_manager_navigation'
NAV2_HEALTH_FRAGMENT = 'nav2 health'


# poll_nav2_state가 이번 tick에 할 일.
POLL_WAIT = 'wait'          # 부른 호출이 아직 살아 있다. 그대로 둔다.
POLL_ASK = 'ask'            # 서비스에 새로 묻는다.
POLL_FALLBACK = 'fallback'  # 서비스가 답을 못 준다. 진단으로 판정한다.


def decide_poll_action(in_flight, call_started_ns, service_ready, now_ns, timeout_ns):
    """Decide what poll_nav2_state should do this tick.

    fallback이 '서비스 미준비'에만 달려 있으면 안 된다. 2026-08-01 실기 사건은 서버가 떠
    있는데 응답만 오지 않은 경우였고, 그때 service_is_ready()는 계속 True다. 시한을 두지
    않으면 in_flight가 영구히 True로 남아 폴링 자체가 멈춘다.
    """
    if in_flight:
        if is_fresh_ns(call_started_ns, now_ns=now_ns, timeout_ns=timeout_ns):
            return POLL_WAIT
        return POLL_FALLBACK
    return POLL_ASK if service_ready else POLL_FALLBACK


def message_says_active(message) -> bool:
    """Report whether this diagnostic message says Nav2 is active.

    대소문자와 공백 흔들림만 흡수하고, 그 밖의 문구는 활성으로 보지 않는다.
    "Nav2 is inactive"가 여기서 걸러진다.
    """
    if not message:
        return False
    return ' '.join(str(message).split()).lower() == ACTIVE_MESSAGE


def is_nav2_manager_diag(name) -> bool:
    """Report whether this diagnostic name is the navigation lifecycle manager's."""
    if not name:
        return False
    lowered = str(name).lower()
    return NAV2_MANAGER_FRAGMENT in lowered and NAV2_HEALTH_FRAGMENT in lowered


def is_nav2_active(diag_items, now_ns: int, timeout_ns: int) -> bool:
    """Report whether the navigation lifecycle manager says Nav2 is active.

    ``diag_items``는 monitor가 들고 있는 ``{name: (DiagItem, seen_ns)}`` 그대로다.
    신선하고 level이 OK이며 활성 문구인 항목이 하나라도 있어야 True다.
    """
    for name, (item, seen_ns) in diag_items.items():
        if not is_nav2_manager_diag(name):
            continue
        if not is_fresh_ns(seen_ns, now_ns=now_ns, timeout_ns=timeout_ns):
            continue
        if item.level != DIAG_OK:
            continue
        if message_says_active(item.message):
            return True
    return False
