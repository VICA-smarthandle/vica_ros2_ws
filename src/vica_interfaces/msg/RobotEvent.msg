# 결함의 상태 변화 하나를 전달한다. robot_health_monitor_node가 /robot/events로
# 발행하고 앱·TTS·logger가 소비한다.
#
# 같은 결함이 유지되는 동안 매 진단 주기마다 발행하지 않는다. 초안 10.3절의
# 폭주 방지 규칙에 따라 전이 시점에만 발행하며, 유지 중에는 RobotHealth의
# active_faults에서 occurrence_count만 올라간다.
#
# QoS는 volatile이다. 앱이 재접속하면 지나간 이벤트는 받지 못하고, 현재 상태는
# /robot/health의 active_faults로 복원한다(초안 12절과 같은 판단).
std_msgs/Header header

RobotFault fault

# 어떤 전이인지.
uint8 TRANSITION_RAISED=0      # 정상 -> 결함. 최초 1회
uint8 TRANSITION_ESCALATED=1   # 같은 결함의 severity 상승. 즉시 발행
uint8 TRANSITION_REMINDER=2    # 장시간 유지 중 설정 간격마다 재알림
uint8 TRANSITION_CLEARED=3     # 결함 -> 정상. 1회만. fault.active는 false다
uint8 transition
