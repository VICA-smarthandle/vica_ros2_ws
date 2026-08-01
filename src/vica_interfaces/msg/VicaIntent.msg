# VICA LLM 의도 해석 결과 (state machine 에 전달되는 '제안', 이동 명령 아님)
# navigate / question / clarify / unknown
# + cancel / pause / resume  — 진행 중인 안내를 조작하는 요청 (2026-07-27 추가)
#   cancel  : 목적지 취소. Mission Manager가 되물어 확인한 뒤에만 실제로 취소한다.
#   pause   : 일시정지. 목적지를 기억한 채 멈춘다.
#   resume  : 다시 출발. 기억해 둔 목적지로 재요청한다.
# 이 세 값에는 matched_destination_id가 필요 없다(진행 중인 안내가 대상).
string intent
string destination_candidate   # LLM 이 고른 목적지 표현 (없으면 "")
string matched_destination_id  # 코드가 확정한 목적지 id (없으면 "")
float32 confidence             # 0.0 ~ 1.0
bool need_confirm
string reply                   # 사용자에게 들려줄 한국어 답변
string safety_flag             # normal / emergency
