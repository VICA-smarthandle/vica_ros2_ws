string component

string fault_code

uint8 SEVERITY_OK=0
uint8 SEVERITY_WARN=1
uint8 SEVERITY_DEGRADED=2
uint8 SEVERITY_STOP=3
uint8 SEVERITY_FAULT=4
uint8 severity

bool active

bool latched

uint32 occurrence_count

builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen

string detail

string suggested_action
