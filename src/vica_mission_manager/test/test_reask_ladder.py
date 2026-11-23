"""되묻기 사다리 시험 — 설계 6.8, 원칙 8."""
from vica_mission_manager.reask_ladder import ReaskLadder


def test_legacy_policy_reasks_the_same_question_once():
    l = ReaskLadder()
    assert [l.next(False), l.next(False), l.next(True)] == ["same", "end", "end"]


def test_p8_three_steps_and_staff_only_while_driving():
    l = ReaskLadder(three_step=True)
    assert [l.next(True), l.next(True), l.next(True), l.next(True)] == ["retry", "louder", "help", "end"]
    l.reset()
    assert [l.next(False), l.next(False), l.next(False)] == ["retry", "louder", "end"]
    l.reset()
    assert [l.next(True, can_help=False) for _ in range(3)] == ["retry", "louder", "end"]


def test_p8_understood_utterance_resets():
    l = ReaskLadder(three_step=True)
    l.next(False)
    l.reset()
    assert l.step == 0 and l.next(False) == "retry"


def test_p8_step_counts_calls_and_stops_at_three():
    l = ReaskLadder(three_step=True)
    assert l.step == 0
    for _ in range(6):
        l.next(False)
    assert l.step == 3
    assert l.next(False) == "end"


def test_legacy_ignores_driving_and_can_help():
    l = ReaskLadder()
    assert l.next(True, can_help=True) == "same"
    assert l.next(True, can_help=True) == "end"
