"""로봇이 말하는 멘트에 하드 긴급어가 섞이지 않는지 검사하는 회귀 테스트.

배경: 상시 긴급어 감시(vica-voice-llm 의 emergency_monitor)는 마이크를 계속 열어
두므로, 스피커로 나간 로봇 자기 목소리도 듣는다. 멘트에 "멈춰"/"정지" 같은 하드
긴급어가 들어 있으면 다음 경로로 자가 E-stop 이 걸린다.

    스피커 → 마이크 → /vica/emergency → emergency_estop_bridge
    → /voice_emergency_stop → emergency_stop_node

멘트를 고칠 때 이 테스트가 실패하면, 문구에서 긴급어를 빼는 쪽으로 고친다.
"""
import re

from vica_mission_manager import mission_logic
from vica_mission_manager.mission_logic import HARD_EMERGENCY_KEYWORDS


def _spoken_messages() -> dict:
    """mission_logic 의 MSG_* 상수를 모은다 (새 멘트가 추가돼도 자동 포함)."""
    return {
        name: value
        for name, value in vars(mission_logic).items()
        if name.startswith("MSG_") and isinstance(value, str)
    }


def test_messages_have_no_hard_emergency_keyword():
    offenders = []
    for name, text in sorted(_spoken_messages().items()):
        # detect_emergency 와 같은 정규화(공백 제거)를 쓴다.
        norm = re.sub(r"\s+", "", text)
        for keyword in sorted(HARD_EMERGENCY_KEYWORDS):
            if keyword in norm:
                offenders.append(f"{name}={text!r} <- '{keyword}'")

    assert offenders == [], (
        "로봇 멘트에 하드 긴급어가 있어 자가 E-stop 이 발생한다:\n  "
        + "\n  ".join(offenders)
    )


def test_message_collection_is_not_empty():
    """introspection 이 조용히 0건이 되어 위 테스트가 무력화되는 것을 막는다."""
    assert len(_spoken_messages()) >= 10
