# Smart Handle 장치 상태 진단. heartbeat 프로토콜은 사용하지 않는다.
#
# [중요] 상향 통신은 터치와 초음파만 올라온다. servo_ok / *_led_ok는 여전히 장치를
# 관측한 값이 아니라 "전송이 성공했다"는 간접 지표일 뿐이며, 서보나 LED가 실제로
# 동작했음을 보증하지 않는다. 관리자 앱이 이를 "정상 동작 중"으로 표시하면
# 잘못된 안심을 준다.
std_msgs/Header header

bool connected             # 포트 open 성공 + 최근 write 예외 없음

# 터치센서(D11, active-low, 2026-09-05). 그 순간 손잡이를 잡고 있는가.
#
# **판정이 아니라 사실이다.** 모드 진입 3초와 손 놓음 0.5초는 mission_manager 가
# 이 값을 보고 잰다. 여기서 재지 않는다.
#
# uplink_fresh 가 false 면 이 값은 **항상 false** 다. 마지막 값을 붙들고 있으면
# 죽은 센서가 "잡고 있다"고 말하게 된다.
bool user_contact

# 상향 프레임(20Hz)이 최근에 왔는가. 2026-09-04 신설.
#
# 소비자가 user_contact=false 를 "손을 안 잡았다"와 "센서가 죽었다" 중 무엇인지
# 가르는 유일한 근거다. 상향이 끊기면 스마트핸들 모드를 **묻지도 않는다**
# (vica-voice-llm 의 HANDLE_UNAVAILABLE). 그 분기가 이 필드에 달려 있다.
bool uplink_fresh

bool servo_ok              # 상향으로 관측하지 않는다 → connected와 동일값 [미검증]
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
