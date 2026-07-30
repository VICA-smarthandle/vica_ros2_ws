# 단일 결함(fault) 하나를 기술하는 순수 데이터다. Header가 없어서
# RobotHealth(현재 활성 목록)와 RobotEvent(상태 전이)가 같은 구조를 재사용한다.
#
# 이 메시지는 진단·표시 전용이다. 소비자는 이 값을 근거로 어떤 정지·주행 동작도
# 하지 않는다. 정지는 safety_supervisor_node와 motor driver가 직접 판정한다
# (vica_system_health_monitoring_draft.md 3.1절).
#
# 상태는 문자열이 아니라 component + fault_code로 관리한다(초안 3.5절). 앱과 TTS는
# fault_code로 필터·중복 판정을 하고, 화면 문구는 detail/suggested_action을 쓴다.

# 결함이 속한 컴포넌트. required_components.yaml의 key와 같은 값을 쓴다.
#   motor | safety | localization | navigation | lidar | perception
#   | guidance | voice | app | computer | monitor
string component

# 기계 판독 결함 코드. 예: MOTOR_CAN_TIMEOUT, LIDAR_SCAN_STALE.
# 정본은 vica_system_monitor/fault_catalog.py다.
string fault_code

# 등급. 초안 9절 체계와 같다. 값이 클수록 심각하다.
uint8 SEVERITY_OK=0
uint8 SEVERITY_WARN=1        # 성능 저하 가능성. 조건부 주행 허용
uint8 SEVERITY_DEGRADED=2    # 일부 비필수 기능 사용 불가
uint8 SEVERITY_STOP=3        # 안전 확인 전 주행 불가
uint8 SEVERITY_ESTOP=4       # 즉시 정지 필요. 자동 해제 금지
uint8 SEVERITY_FAULT=5       # 반복 복구 실패 또는 원인 불명
uint8 severity

# 현재 원인이 진행 중인지. false는 해소된 결함(CLEARED 이벤트)을 뜻한다.
bool active

# 원인이 사라져도 유지되는지. 중앙 래치를 소유한 emergency_stop_node의 상태를
# 반영하는 표시값이며, 이 필드가 래치를 만들거나 해제하지 않는다.
bool latched

# 같은 (component, fault_code)가 관측된 누적 횟수. 알림 폭주 억제용(초안 10.3절).
uint32 occurrence_count

# 표시·로그 전용 SYSTEM_TIME이다. 모니터 내부의 신선도 판정은 이 값이 아니라
# STEADY_TIME 수신 시각으로 한다. 두 시간축을 빼는 것은 정의되지 않은 연산이다.
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen

# 사람이 읽는 상세. 측정값을 포함한다. 예: "CAN 응답 0.82초 미수신"
string detail

# 관리자가 취할 조치. 예: "로봇을 확인한 뒤 앱에서 안전 초기화를 실행하세요"
string suggested_action
