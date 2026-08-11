"""음성 조작 intent 의 계약이 세 곳에서 어긋나지 않는지 감시한다.

`VicaIntent.msg` 의 `intent` 는 자유 문자열이고 값 목록은 **주석의 약속**이다.
구조가 아니라 약속이라 한쪽만 늘어나도 컴파일이 깨지지 않는다. 실제로 그 사고가
났다 — 2026-07-27 에 cancel/pause/resume 을 늘렸는데 음성 저장소의 intent 목록은
4종에 머물러 있어, **로봇은 처리할 준비가 됐는데 말로는 쓸 수 없는 상태**가
2주 넘게 이어졌다(2026-08-10 확인).

메시지 정의와 노드 배선을 대조해 그 어긋남을 잡는다. 노드는 rclpy 가 필요해
import 하지 않고 소스를 읽는다.
"""
import re
from pathlib import Path

# 진행 중인 안내를 조작하는 intent. 목적지가 아니라 현재 안내가 대상이라
# matched_destination_id 가 필요 없고, mission_manager_node 가 일반 경로가 아닌
# _on_voice_mission_command 로 보낸다.
MISSION_COMMAND_INTENTS = ('cancel', 'cancel_keep', 'pause', 'resume')


def _pkg_dir() -> Path:
    return Path(__file__).parents[1]


def _node_source() -> str:
    path = _pkg_dir() / 'vica_mission_manager' / 'mission_manager_node.py'
    return path.read_text(encoding='utf-8')


def _msg_source() -> str:
    path = _pkg_dir().parents[0] / 'vica_interfaces' / 'msg' / 'VicaIntent.msg'
    return path.read_text(encoding='utf-8')


def _dispatch_values() -> set:
    """`_on_intent` 가 조작 요청으로 보내는 값 집합."""
    source = _node_source()
    match = re.search(
        r'if msg\.intent in \(([^)]*)\):\s*\n\s*self\._on_voice_mission_command',
        source,
    )
    assert match, (
        '_on_intent 의 조작 요청 분기를 찾지 못했다. 형태가 바뀌었으면 이 시험도 '
        '함께 고친다 — 검사 자체가 조용히 무력화되면 안 된다.'
    )
    # 노드가 작은따옴표를 쓰든 큰따옴표를 쓰든 같게 읽는다.
    return set(re.findall(r"""['"]([^'"]+)['"]""", match.group(1)))


def test_every_mission_command_intent_is_dispatched():
    """분기 목록에 값이 빠지면 그 말은 조용히 무시된다.

    일반 intent 경로로 흘러가 navigate 가 아니므로 아무 일도 일어나지 않고,
    사용자에게는 로봇이 못 알아들은 것으로 보인다.
    """
    missing = sorted(set(MISSION_COMMAND_INTENTS) - _dispatch_values())
    assert missing == [], f'_on_intent 분기에서 빠진 값: {missing}'


def test_dispatch_has_no_undocumented_value():
    """반대 방향도 본다 — 배선만 있고 계약에 없는 값이 생기지 않도록."""
    extra = sorted(_dispatch_values() - set(MISSION_COMMAND_INTENTS))
    assert extra == [], (
        f'분기에는 있으나 이 시험이 모르는 값: {extra}. '
        'VicaIntent.msg 주석과 이 목록에 함께 적는다.'
    )


def test_message_comment_documents_every_value():
    """`.msg` 주석이 정본이다. 값을 늘리고 문서화하지 않으면 다음 사람이 모른다."""
    comment = _msg_source()
    undocumented = [name for name in MISSION_COMMAND_INTENTS if name not in comment]
    assert undocumented == [], (
        f'VicaIntent.msg 주석에 설명이 없는 값: {undocumented}'
    )


def test_cancel_keep_reaches_the_negative_answer():
    """`cancel_keep` 이 부정 응답으로 이어지는지 본다.

    이 배선이 없으면 "아니요"에 즉시 응답이 없고, 확인 시한(30초)이 지나야
    안내가 이어진다. 눈으로 확인할 수 없는 사용자에게 그 30초는 침묵이다.
    """
    source = _node_source()
    branch = re.search(
        r"""elif msg\.intent == ['"]cancel_keep['"]:(.*?)"""
        r"""elif msg\.intent == ['"]pause['"]:""",
        source,
        re.S,
    )
    assert branch, 'cancel_keep 분기를 찾지 못했다'
    assert 'on_cancel_confirm_answer(False' in branch.group(1), (
        'cancel_keep 이 on_cancel_confirm_answer(False) 로 이어지지 않는다'
    )
