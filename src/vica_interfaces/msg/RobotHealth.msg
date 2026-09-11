std_msgs/Header header

uint8 STATE_STARTING=0
uint8 STATE_READY=1
uint8 STATE_DEGRADED=2
uint8 STATE_STOPPED=3
uint8 STATE_ESTOPPED=4
uint8 STATE_FAULT=5
uint8 state

uint8 READINESS_UNKNOWN=0
uint8 READINESS_NOT_READY=1
uint8 READINESS_READY=2

uint8 motor_readiness
uint8 safety_readiness
uint8 localization_readiness
uint8 navigation_readiness
uint8 lidar_readiness
uint8 perception_readiness
uint8 guidance_readiness
uint8 voice_readiness
uint8 app_readiness

uint16 active_fault_count

uint8 highest_severity

string primary_fault_code

RobotFault[] active_faults
