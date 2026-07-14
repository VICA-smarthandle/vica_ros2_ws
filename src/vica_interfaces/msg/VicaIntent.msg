# VICA LLM 의도 해석 결과 (state machine 에 전달되는 '제안', 이동 명령 아님)
string intent                  # navigate / question / clarify / unknown
string destination_candidate   # LLM 이 고른 목적지 표현 (없으면 "")
string matched_destination_id  # 코드가 확정한 목적지 id (없으면 "")
float32 confidence             # 0.0 ~ 1.0
bool need_confirm
string reply                   # 사용자에게 들려줄 한국어 답변
string safety_flag             # normal / emergency
