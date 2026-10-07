# 로봇 현재 상태 (question 답변 등에 활용)
int32 current_floor    # 층. 알 수 없으면 -1
string current_building
bool is_moving
# 일시정지 상태. 목적지를 기억한 채 멈춰 있어 "다시 출발"로 재개할 수 있다.
# is_moving=false 만으로는 그냥 정지와 구분되지 않아 별도 필드로 둔다.
bool is_paused
# ---- 로봇 대장 (2026-09-21 P1, 스펙 3절). 미션이 적고 LLM 노드가 읽는 사실. 없으면 ""/-1 ----
string dialog_state           # idle / awaiting_user / confirming / seeking / navigating / paused / asking_next / asking_wait_time / waiting / returning / estopped …
string place_here             # "407호 앞" / "407호와 화장실 사이" / "" = 위치 미확인
float32 place_here_dist_m     # 가장 가까운 등록 장소까지 m. 모르면 -1
string active_destination     # 가는 중인 곳 name. 없으면 ""
string last_destination       # 직전에 도착한 곳 name. 없으면 ""
int32 last_arrived_age_sec    # 그 도착 뒤 지난 초. 없으면 -1
string aborted_destination    # 확인까지 갔다가 안 간 곳 name. 없으면 ""
int32 wait_minutes            # 대기 요청 분. 없으면 -1
int32 wait_left_sec           # 대기 남은 초. 대기 중이 아니면 -1
int32 battery_pct             # 배터리 %. 모르면 -1
# ---- 대기 장소 (2026-10-07). 뒤에 덧붙인 칸이라 옛 소비자는 그대로 동작한다 ----
string door_side              # 마지막 도착 기준 목적지 입구가 있는 쪽 "앞"/"뒤"/"오른쪽"/"왼쪽". 없으면 ""
string wait_place             # 지금 기다리는 곳 "입구 오른쪽"/"입구 왼쪽"/"입구 맞은편"/"입구 앞". 대기 장소 없는 대기·대기 아님은 ""
