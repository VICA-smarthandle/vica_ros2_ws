# Smart Handle 회전 안내 cue. 구동 명령이 아니라 사용자 안내 신호다.
# 이 메시지는 /cmd_vel*, Nav2 goal 어디에도 영향을 주지 않는다.
#
# 1단계는 EKF /odom yaw 변화량 기반 사후 감지다. 회전이 시작된 뒤 NOW를 발행한다.
# 2단계(추후)는 Nav2 path look-ahead로 PREPARE 사전 예고를 추가한다.
std_msgs/Header header

uint8 DIRECTION_NONE=0
uint8 DIRECTION_LEFT=1
uint8 DIRECTION_RIGHT=2
uint8 direction

uint8 PHASE_IDLE=0
uint8 PHASE_PREPARE=1      # 2단계 전용. 1단계는 발행하지 않는다
uint8 PHASE_NOW=2
uint8 PHASE_COMPLETE=3     # 진입 종료 tick 1회만. 소비자는 놓쳐도 되게 설계할 것
uint8 PHASE_CANCELED=4
uint8 phase

float32 distance_m         # 2단계 전용. 1단계는 항상 NaN (0.0은 "거리 0m"로 오독됨)
float32 turn_angle_deg     # 윈도우 누적 yaw 변화량. 부호: CCW(+) = LEFT (REP-103)
uint32 sequence_id         # 회전 진입마다 증가. 이전 sequence cue는 폐기한다

# header.stamp 기준 SYSTEM_TIME. 로그·rosbag·앱 표시 전용이다.
# 소비자의 stale 판정은 이 값이 아니라 수신 시각(STEADY_TIME)으로 한다.
# 두 시간축을 빼는 것은 정의되지 않은 연산이다.
builtin_interfaces/Time valid_until

# /odom 미수신·stale 여부. true면 direction/phase는 판단 불가를 뜻하며
# turn_angle_deg는 NaN이다. "정상 직진"과 "판단 불가"를 구분하기 위한 필드다.
# 소비자는 이 값을 근거로 어떤 안전 동작도 하지 않는다. 진단 표시 전용이다.
bool source_stale
