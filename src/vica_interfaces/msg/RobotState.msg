# 로봇 현재 상태 (question 답변 등에 활용)
int32 current_floor    # 층. 알 수 없으면 -1
string current_building
bool is_moving
# 일시정지 상태. 목적지를 기억한 채 멈춰 있어 "다시 출발"로 재개할 수 있다.
# is_moving=false 만으로는 그냥 정지와 구분되지 않아 별도 필드로 둔다.
bool is_paused
