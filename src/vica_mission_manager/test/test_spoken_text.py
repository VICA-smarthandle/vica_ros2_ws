"""로봇이 말하는 멘트에 하드 긴급어가 섞이지 않는지 검사하는 회귀 테스트."""
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


def test_no_message_leaves_josa_unresolved():
    """"(으)로" 같은 미해결 조사 표기가 멘트에 남아 있지 않은지 본다."""
    offenders = [
        f"{name}={text!r}"
        for name, text in sorted(_spoken_messages().items())
        if "(으)" in text or "(이)" in text or "(과)" in text or "(를)" in text
    ]
    assert offenders == [], (
        "멘트에 미해결 조사 표기가 남아 있다. josa_euro() 로 정해야 한다:\n  "
        + "\n  ".join(offenders)
    )


def test_josa_euro_picks_by_final_consonant():
    """받침 없음·ㄹ 받침이면 '로', 그 밖의 받침이면 '으로'."""
    assert mission_logic.josa_euro("화장실") == "로"
    assert mission_logic.josa_euro("안내센터") == "로"
    assert mission_logic.josa_euro("식당") == "으로"
    assert mission_logic.josa_euro("대강당") == "으로"
    assert mission_logic.josa_euro("") == "로"
    assert mission_logic.josa_euro("407") == "로"


def test_destination_messages_render_with_correct_josa():
    """실제 목적지 이름으로 완성된 문장을 고정한다."""
    assert (
        mission_logic.say_destination(mission_logic.MSG_START, "별빛관 1층 화장실")
        == "별빛관 1층 화장실로 안내를 시작합니다."
    )
    assert (
        mission_logic.say_destination(mission_logic.MSG_START, "메인홀 지하 1층 식당")
        == "메인홀 지하 1층 식당으로 안내를 시작합니다."
    )
    assert (
        mission_logic.say_destination(mission_logic.MSG_RESUMED, "별빛관 1층 안내센터")
        == "별빛관 1층 안내센터로 다시 출발합니다."
    )
