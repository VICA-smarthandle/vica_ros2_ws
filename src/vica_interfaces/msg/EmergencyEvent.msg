# 상시 긴급어 감지 이벤트 (LLM 을 거치지 않는 안전 경로, CLAUDE.md Phase 4)
# 실제 정지 판단/실행은 Safety Supervisor / State Machine 이 한다.
string keyword        # 매칭된 긴급어 (예: "멈춰")
string source_text    # STT 가 인식한 원본 텍스트
float64 detected_at   # 감지 시각 (unix time)
