std_msgs/Header header

bool connected

bool user_contact

bool uplink_fresh

bool servo_ok
bool left_led_ok
bool right_led_ok
bool haptic_ok

uint8 FAULT_NONE=0
uint8 FAULT_PORT_OPEN=1
uint8 FAULT_WRITE_FAIL=2
uint8 FAULT_NOT_CONFIGURED=3
uint8 fault_code

uint8 last_state_code
uint32 write_error_count
