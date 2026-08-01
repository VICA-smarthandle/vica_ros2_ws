# 전체 시스템 상태 요약. robot_health_monitor_node가 /robot/health로 발행한다.
#
# 이 메시지는 진단·표시 전용이다. Mission Manager의 Goal 권한이나 Safety의 주행
# 승인을 대체하지 않는다(초안 4.2절). 모니터가 죽어도 모터 안전 정지는 유지된다.
#
# 발행 방식: 1 Hz 상시 + 상태 변화 시 즉시. transient_local이지만 앱은 그것에
# 의존하지 않는다. Humble rosbridge의 subscribe op는 durability를 지정하지 않아
# latched 샘플을 못 받을 수 있으므로, 앱 재접속 복원은 1 Hz 상시 발행과 아래
# active_faults 배열로 한다(/app_estop_state가 쓰는 주기 브로드캐스트와 같은 방식).
std_msgs/Header header

# 전체 상태. 초안 7.2절 체계와 같다.
uint8 STATE_STARTING=0    # startup grace 중. 미수신을 아직 결함으로 보지 않는다
uint8 STATE_READY=1       # 아래 readiness 조건 전부 충족 + safety 허용 상태
uint8 STATE_DEGRADED=2    # 비필수 기능만 이상
uint8 STATE_STOPPED=3     # STOP 등급 결함으로 주행 불가
uint8 STATE_ESTOPPED=4    # 중앙 래치 활성
uint8 STATE_FAULT=5       # 원인 불명 또는 반복 복구 실패
uint8 state

# 컴포넌트별 준비 상태. 세 값을 구분하는 것이 중요하다.
#
# [중요] UNKNOWN은 "정상"이 아니다. 관측 수단이 아예 없다는 뜻이다.
# 예를 들어 Smart Handle은 아두이노 → 젯슨 상향 통신이 없어 서보·LED·진동이 실제로
# 동작했는지 확인할 방법이 없다(SmartHandleState.msg 주석 참조). 이때 READY로 보고하면
# 관리자 앱이 초록불을 띄워 잘못된 안심을 준다. UNKNOWN으로 보고해 "관측 불가"를
# 표시해야 한다.
#
# 하드웨어나 진단이 추가되면 같은 필드가 UNKNOWN에서 READY/NOT_READY로 바뀐다.
# 메시지와 앱을 고치지 않고 값만 달라지는 것이 이 3상태 설계의 목적이다.
uint8 READINESS_UNKNOWN=0     # 관측 수단이 없다. 판정하지 않았다
uint8 READINESS_NOT_READY=1   # 관측했고, 준비되지 않았다
uint8 READINESS_READY=2       # 관측했고, 준비되었다

uint8 motor_readiness
uint8 safety_readiness
uint8 localization_readiness
uint8 navigation_readiness
uint8 lidar_readiness
uint8 perception_readiness      # nvblox·카메라
uint8 guidance_readiness
uint8 voice_readiness
uint8 app_readiness

uint16 active_fault_count

# 현재 활성 결함 중 가장 높은 등급. RobotFault.SEVERITY_* 값이다.
uint8 highest_severity

# highest_severity에 해당하는 결함의 fault_code. 없으면 빈 문자열이다.
string primary_fault_code

# 현재 활성 결함 전체. 초안 7.2절에는 없는 추가 필드이며, 앱이 재접속했을 때
# 화면을 한 번에 복원하기 위해 넣었다. 이벤트 이력은 /robot/events가 담당한다.
RobotFault[] active_faults
