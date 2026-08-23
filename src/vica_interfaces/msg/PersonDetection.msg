# person_detector_node 가 5 Hz 로 발행하는 사람 탐지 결과 하나다.
#
# 이 메시지는 접근 요청의 근거이지 goal 이 아니다. 소비자는 이 값만 보고 주행하지
# 않는다. goal 권한은 Mission Manager 가 독점하며, 실제 접근은 RequestApproach.srv 로
# 요청한다(devlog/2026-08-23-사람접근-구현설계.md 3절).
#
# 발행 주기 5 Hz 는 CPU 가 병목이라 그 이상 올리지 않는다. 보행 1.2 m/s 에서 0.24 m
# 마다 갱신되므로 goal 갱신 임계 0.5 m 에는 충분하다(설계 6.2절).
#
# header.frame_id 는 "map" 이다. 노드가 컨테이너 안에서 TF 조회로 변환을 끝내고
# 발행하므로 소비자는 다시 변환하지 않는다. header.stamp 은 추론에 쓴 이미지의 취득
# 시각(SYSTEM_TIME)이며 로그·표시 전용이다. 소비자의 stale 판정은 이 값이 아니라
# 수신 시각(STEADY_TIME)으로 한다. 두 시간축을 빼는 것은 정의되지 않은 연산이다.
std_msgs/Header header

# map 좌표계에서 본 사람의 위치. orientation 은 판정하지 않으므로 항등 쿼터니언
# (w=1.0)을 넣는다. 사람이 어느 쪽을 보는지는 이번 범위가 아니다.
# 접근 goal 은 이 위치에서 로봇 쪽으로 1.1 m 물러난 지점이며, 그 계산은 Mission
# Manager 가 한다(설계 5절).
geometry_msgs/Pose pose

# 탐지 신뢰도. 0.0 ~ 1.0
# stable 판정에 쓰는 임계 0.6 은 가정값이며 모델 실측 후 확정한다 [미검증].
float32 confidence

# 같은 사람을 계속 같은 번호로 부르기 위한 추적 식별자.
# 접근을 마치거나 포기한 뒤 같은 id 로는 60초간 재접근하지 않는다(설계 4절).
# 추적기를 Ultralytics 내장으로 할지 자체 구현으로 할지는 아직 [미정] 이다(설계 12절).
uint32 TRACK_ID_NONE=0     # 추적기가 id 를 붙이지 못했다. 이때 approachable 은 false 다
uint32 track_id

# 로봇(base_link 원점)에서 사람까지의 직선 거리. 단위 m.
# 마스크 전체가 아니라 몸통 depth 중앙값을 쓴다 — 지팡이 픽셀이 섞이면 값이 왜곡된다
# (설계 12절). depth 를 읽지 못하면 NaN 이다. 0.0 은 "거리 0 m"로 오독된다.
float32 distance_m

# 오탐 판정. confidence 0.6 이상이 1.0초 연속되었는가.
# "진짜 사람인가"만 본다. 다가갈 값어치가 있는지는 보지 않는다(설계 6.1절).
bool stable

# 접근 가치 판정. 3초간 이동이 0.3 m 미만이고 distance_m 이 1.5 ~ 4.0 m 인가.
# 지나가는 사람을 거른다. 접근 속도 0.3 m/s 인 로봇은 보행 1.2 m/s 를 따라잡지
# 못하므로 멈춰 선 사람만 대상으로 한다. 거리 상한 4.0 m 는 그보다 멀면 도착까지
# 10초를 넘겨 도착 시점에 상황이 달라지기 때문이다(설계 6.1절).
#
# stable 과 approachable 은 둘 다 person_detector_node 가 판정한다. 시계열을 보는
# 판정이라 탐지 이력을 가진 쪽이 해야 하고, Mission Manager 는 이력을 쌓지 않는다.
# approachable 은 접근의 필요조건일 뿐이며 승인은 Mission Manager 가 한다.
bool approachable
