std_msgs/Header header

uint8 DISABLED=0
uint8 PRETENSION=1
uint8 TRACTION=2
uint8 RELEASING=3

uint8 DIRECTION_CORRECTION_OFF=0
uint8 DIRECTION_CORRECTION_SHADOW=1
uint8 DIRECTION_CORRECTION_ACTIVE=2

uint8 mode
geometry_msgs/Vector3 locked_direction_base
float64 target_force_n
uint8 direction_correction_mode
geometry_msgs/Vector3 lateral_velocity_base
