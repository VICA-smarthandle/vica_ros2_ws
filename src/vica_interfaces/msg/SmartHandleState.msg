# Smart Handle 장치 상태 진단. heartbeat 프로토콜은 사용하지 않는다.
#
# [중요] 아두이노 → 젯슨 상향 통신이 없다. 따라서 servo_ok / *_led_ok는 장치를
# 관측한 값이 아니라 "전송이 성공했다"는 간접 지표일 뿐이며, 서보나 LED가 실제로
# 동작했음을 보증하지 않는다. 관리자 앱이 이를 "정상 동작 중"으로 표시하면
# 잘못된 안심을 준다.
std_msgs/Header header

bool connected             # 포트 open 성공 + 최근 write 예외 없음
bool user_contact          # 터치센서 미장착 확정. 항상 false
bool servo_ok              # 상향 통신 없음 → connected와 동일값 [미검증]
bool left_led_ok           # 동일 [미검증]
bool right_led_ok          # 동일 [미검증]
bool haptic_ok             # 진동 모터 미장착. 항상 false

uint8 FAULT_NONE=0
uint8 FAULT_PORT_OPEN=1        # 포트 open 실패 (처음부터 미연결)
uint8 FAULT_WRITE_FAIL=2       # open 후 write 예외 (수신 중 단절)
uint8 FAULT_NOT_CONFIGURED=3   # enable_serial=false (mock/개발 모드)
uint8 fault_code

uint8 last_state_code      # 아두이노로 마지막 전송에 성공한 상태코드
uint32 write_error_count   # 누적 write 실패 횟수. 간헐 단절 진단용
