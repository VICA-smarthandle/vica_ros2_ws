# VICA LLM 의도 해석 결과 (state machine 에 전달되는 '제안', 이동 명령 아님)
# navigate / question / clarify / unknown
# + cancel / pause / resume  — 진행 중인 안내를 조작하는 요청 (2026-07-27 추가)
#   cancel  : 목적지 취소. Mission Manager가 되물어 확인한 뒤에만 실제로 취소한다.
#             되묻는 중에 cancel이 한 번 더 오면 그것이 확정이다.
#   pause   : 일시정지. 목적지를 기억한 채 멈춘다.
#   resume  : 다시 출발. 기억해 둔 목적지로 재요청한다.
# + cancel_keep  — 취소 되묻기에 대한 부정 (2026-08-10 추가)
#   "안내를 취소할까요?"에 사용자가 "아니요"라고 답한 경우다. 이 값이 없던 동안에는
#   부정을 전달할 방법이 없어 Mission Manager의 확인 시한(30초)이 지나야 안내가
#   이어졌고, 눈으로 확인할 수 없는 사용자에게 그 30초가 침묵으로 남았다.
#   되묻는 중이 아니면 아무 일도 하지 않는다(무해한 no-op).
# 이 값들에는 matched_destination_id가 필요 없다(진행 중인 안내가 대상).
string intent
string destination_candidate   # LLM 이 고른 목적지 표현 (없으면 "")
string matched_destination_id  # 코드가 확정한 목적지 id (없으면 "")
float32 confidence             # 0.0 ~ 1.0
bool need_confirm
string reply                   # 사용자에게 들려줄 한국어 답변
string safety_flag             # normal / emergency
