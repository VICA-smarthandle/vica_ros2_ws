"""장애물 안내 — 미션이 말할지 마지막으로 정한다 (2026-10-09 2단계, 설계서 5절 '대화와 겹칠 때').

대화가 먼저다: 귀가 듣는 중이거나 취소 확인의 답을 기다리면 말하지 않는다. 다른 말 중이면 TTS 가
버리도록 ambient 로 보낸다. 늦은 후보(행동 시작 2.5초 뒤)는 버린다.
"""
import pytest

from vica_mission_manager import obstacle_judge as oj
from vica_mission_manager.mission_logic import (
    EAR_GRACE_SEC, MSG_OBSTACLE_AVOID, MSG_OBSTACLE_SLOW, OBSTACLE_STALE_SEC, MissionLogic, Say, State,
)


def _driving():
    logic = MissionLogic()
    logic.state = State.NAVIGATING
    return logic


def _said(actions):
    return [(a.text, a.priority) for a in actions if isinstance(a, Say)]


def test_phrases_match_the_judge():
    assert oj.PHRASES == {"avoid": MSG_OBSTACLE_AVOID, "slow": MSG_OBSTACLE_SLOW}


@pytest.mark.parametrize("phrase,text", [("avoid", MSG_OBSTACLE_AVOID), ("slow", MSG_OBSTACLE_SLOW)])
def test_speaks_while_guiding_as_ambient(phrase, text):
    actions, why = _driving().obstacle_cue(phrase, 100.0, 100.4)
    assert why == ""
    assert _said(actions) == [(text, "ambient")]


@pytest.mark.parametrize("state", [State.IDLE, State.PAUSED, State.CONFIRMING, State.RETURNING, State.WAITING])
def test_silent_when_not_guiding(state):
    logic = MissionLogic()
    logic.state = state
    actions, why = logic.obstacle_cue("avoid", 100.0, 100.4)
    assert actions == [] and why == "not_navigating"


def test_silent_while_the_ear_is_listening():
    logic = _driving()
    logic.on_listen_state("open", 100.0)
    actions, why = logic.obstacle_cue("avoid", 100.2, 100.5)
    assert actions == [] and why == "ear_busy"


def test_silent_during_the_grace_after_the_user_spoke():
    logic = _driving()
    logic.on_listen_state("open", 100.0)
    logic.on_listen_state("closed", 101.0)          # 말이 LLM 으로 가는 중 — 대답이 곧 온다
    actions, why = logic.obstacle_cue("slow", 102.0, 102.3)
    assert actions == [] and why == "ear_busy"
    later = 101.0 + EAR_GRACE_SEC + 0.5
    actions, why = logic.obstacle_cue("slow", later, later + 0.3)
    assert why == "" and _said(actions) == [(MSG_OBSTACLE_SLOW, "ambient")]


def test_empty_listen_window_frees_the_ear_at_once():
    logic = _driving()
    logic.on_listen_state("open", 100.0)
    logic.on_listen_state("empty:ghost", 101.0)
    actions, why = logic.obstacle_cue("avoid", 101.2, 101.4)
    assert why == "" and actions


def test_silent_while_the_cancel_question_waits():
    logic = _driving()
    logic.cancel_confirm_pending = True              # "안내를 취소할까요?" 답 대기
    actions, why = logic.obstacle_cue("avoid", 100.0, 100.3)
    assert actions == [] and why == "question_pending"


def test_late_cue_is_dropped():
    actions, why = _driving().obstacle_cue("avoid", 100.0, 100.0 + OBSTACLE_STALE_SEC + 0.1)
    assert actions == [] and why == "stale"


def test_unknown_phrase_is_dropped():
    actions, why = _driving().obstacle_cue("boom", 100.0, 100.1)
    assert actions == [] and why == "unknown_phrase"
