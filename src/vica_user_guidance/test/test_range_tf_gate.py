"""초음파 Range 발행 게이트 순수 로직 테스트.

왜 이 게이트가 필요한가(2026-09-07 실기):

    Nav2 의 `RangeSensorLayer` 는 Range 메시지를 받을 때마다 그 메시지 시각의
    `odom -> usonic_*` 변환을 조회하는데, **그 조회가 던지는 예외를 아무도
    잡지 않는다.** TF 가 몇 초라도 끊기면 `controller_server` 프로세스가
    통째로 죽는다(9/7 18:44 크래시 덤프로 확인).

    Nav2 쪽 코드는 우리가 고칠 수 없다. 대신 **메시지를 보내지 않으면 그 층이
    아예 돌지 않는다** — `RangeSensorLayer::updateCostmap()` 은 수신 버퍼를
    순회할 뿐이라 버퍼가 비면 아무 일도 하지 않는다. 그래서 발행하는 쪽에서
    막는다.

이 게이트가 하는 일은 하나다: **TF 조회가 가능한 채널만 발행을 허락한다.**
TF 조회 자체(tf2)는 노드가 하고, 여기에는 그 결과(bool)만 들어온다 —
`inference_gate.py` 와 같은 방식으로 rclpy·tf2 에 의존하지 않는다.
"""

import pytest

from vica_user_guidance.range_tf_gate import RangeTfGate


def test_blocks_when_tf_missing():
    """TF 를 못 찾으면 그 채널은 발행하지 않는다 — 이 게이트의 존재 이유."""
    gate = RangeTfGate(channels=2)
    assert gate.allow(0, tf_ok=False) is False


def test_allows_when_tf_available():
    """평시에는 그대로 통과시킨다. 게이트가 기능을 줄이면 안 된다."""
    gate = RangeTfGate(channels=2)
    assert gate.allow(0, tf_ok=True) is True


def test_resumes_immediately_after_tf_returns():
    """TF 가 돌아오면 다음 프레임에서 곧바로 재개한다.

    유예가 목적이지 차단이 목적이 아니다. 상태를 붙들고 있으면 복구가 늦는다.
    """
    gate = RangeTfGate(channels=2)
    gate.allow(0, tf_ok=False)
    gate.allow(0, tf_ok=False)
    assert gate.allow(0, tf_ok=True) is True


def test_channels_are_independent():
    """한 채널의 TF 실패가 다른 채널을 막지 않는다.

    좌우 센서는 프레임이 다르다. 한쪽 변환만 실패하는 상황이 실제로 있고,
    그때 멀쩡한 쪽까지 끊으면 장애물 감지가 통째로 사라진다.
    """
    gate = RangeTfGate(channels=2)
    assert gate.allow(0, tf_ok=False) is False
    assert gate.allow(1, tf_ok=True) is True


def test_transition_reported_once_per_state_change():
    """로그는 상태가 바뀔 때만 1회다.

    초음파는 채널당 10 Hz 다. 막힐 때마다 경고하면 로그가 폭주해서 정작
    중요한 줄이 묻힌다(9/7 controller_server 로그가 그렇게 묻혔다).
    """
    gate = RangeTfGate(channels=1)

    gate.allow(0, tf_ok=True)
    assert gate.take_transition(0) is None      # 시작 상태와 같으면 조용하다

    gate.allow(0, tf_ok=False)
    assert gate.take_transition(0) is False     # 막히기 시작 -> 1회 보고
    gate.allow(0, tf_ok=False)
    assert gate.take_transition(0) is None      # 계속 막혀 있으면 조용하다

    gate.allow(0, tf_ok=True)
    assert gate.take_transition(0) is True      # 풀림 -> 1회 보고
    gate.allow(0, tf_ok=True)
    assert gate.take_transition(0) is None


def test_blocked_count_is_kept_for_diagnosis():
    """막은 횟수를 센다. 진단에서 '조용히 안 나간 것'을 알 수 있어야 한다."""
    gate = RangeTfGate(channels=1)
    gate.allow(0, tf_ok=False)
    gate.allow(0, tf_ok=False)
    gate.allow(0, tf_ok=True)
    assert gate.blocked_count(0) == 2


def test_rejects_bad_channel_index():
    """채널 수를 벗어난 접근은 조용히 통과시키지 않는다.

    배선 실수를 런타임에 숨기면 '초음파가 안 나가는데 이유를 모르는' 상태가 된다.
    """
    gate = RangeTfGate(channels=2)
    with pytest.raises(IndexError):
        gate.allow(2, tf_ok=True)


def test_channels_must_be_positive():
    """채널 0개는 설정 실수다. 조용히 굴러가느니 기동 때 죽는 편이 낫다."""
    with pytest.raises(ValueError):
        RangeTfGate(channels=0)
