"""emergency_estop_bridge 의 EstopPulse 순수 로직 테스트 (진행순서 ③)."""
from vica_mission_manager.estop_pulse import EstopPulse
from vica_mission_manager.mission_logic import HARD_EMERGENCY_KEYWORDS


class TestEstopPulse:
    def test_inactive_before_trigger(self):
        assert EstopPulse(3.0).active(0.0) is False

    def test_active_during_pulse(self):
        p = EstopPulse(3.0)
        p.trigger(10.0)
        assert p.active(10.0) is True
        assert p.active(12.9) is True

    def test_expires_after_pulse(self):
        p = EstopPulse(3.0)
        p.trigger(10.0)
        assert p.active(13.0) is False  # keyboard_knob /estop_reset 이 막히지 않게

    def test_retrigger_extends(self):
        p = EstopPulse(3.0)
        p.trigger(10.0)
        p.trigger(12.0)  # 펄스 중 재긴급 → 연장
        assert p.active(12.9) is True
        assert p.active(14.9) is True
        assert p.active(15.0) is False


class TestKeywordContract:
    def test_hard_keywords_match_plan(self):
        # 계획서 확정 목록: 멈춰/정지/스탑/스톱/안돼/위험해
        assert HARD_EMERGENCY_KEYWORDS == {"멈춰", "정지", "스탑", "스톱", "안돼", "위험해"}

    def test_soft_keywords_excluded(self):
        # "천천히/느리게/잠깐" 은 감속·유보 계열 — v2 (TODOS.md #6)
        for kw in ("천천히", "느리게", "잠깐"):
            assert kw not in HARD_EMERGENCY_KEYWORDS
