# VICA LLM 의도 해석 결과 (state machine 에 전달되는 '제안', 이동 명령 아님)
# navigate / question / clarify / unknown
# + cancel / pause / resume  — 진행 중인 안내를 조작하는 요청 (2026-07-27 추가)
#   cancel  : 목적지 취소. Mission Manager가 되물어 확인한 뒤에만 실제로 취소한다.
#   pause   : 일시정지. 목적지를 기억한 채 멈춘다.
#   resume  : 다시 출발. 기억해 둔 목적지로 재요청한다.
# 이 세 값에는 matched_destination_id가 필요 없다(진행 중인 안내가 대상).
# + affirm / deny — 로봇이 직전에 던진 질문에 대한 짧은 답 "네"/"아니오" (2026-08-24 추가)
#   어느 질문의 답인지는 메시지에 담지 않는다. 질문을 던진 쪽(Mission Manager)이
#   자기 상태로 판정한다 — 음성은 대화 이력을 쌓지 않고, 상태를 가진 쪽이 시계열을
#   판정한다는 기존 원칙 그대로다. 현재 소비처는 AWAITING_USER(사람 접근 질문,
#   설계 4절) 하나뿐이며 그 외 상태에서는 로그만 남기고 무시된다.
#   취소 확인("취소할까요?")의 긍/부정은 기존 경로(cancel 재발화)가 따로 있어
#   여기에 합치지 않는다 — 두 질문이 겹칠 일은 없지만(상태가 배타적) 경로를
#   섞으면 회귀 범위가 넓어진다.
#   두 값에는 matched_destination_id 가 필요 없고, **reply 는 빈 문자열이어야 한다**
#   — 응답 발화(수락/거절 안내)는 Mission 이 하므로 음성이 함께 말하면 두 번 말한다.
string intent
string destination_candidate   # LLM 이 고른 목적지 표현 (없으면 "")
string matched_destination_id  # 코드가 확정한 목적지 id (없으면 "")
float32 confidence             # 0.0 ~ 1.0
bool need_confirm
string reply                   # 사용자에게 들려줄 한국어 답변
string safety_flag             # normal / emergency
