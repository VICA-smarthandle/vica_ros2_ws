std_msgs/Header header

uint8 DIRECTION_NONE=0
uint8 DIRECTION_LEFT=1
uint8 DIRECTION_RIGHT=2
uint8 direction

uint8 PHASE_IDLE=0
uint8 PHASE_PREPARE=1
uint8 PHASE_NOW=2
uint8 PHASE_COMPLETE=3
uint8 PHASE_CANCELED=4
uint8 phase

float32 distance_m
float32 turn_angle_deg
uint32 sequence_id

builtin_interfaces/Time valid_until

bool source_stale
