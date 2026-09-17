std_msgs/Header header

uint8 IDLE=0
uint8 APPROACH=1
uint8 BRAKING=2
uint8 FINE_ADJUST=3
uint8 CORRECTING=4
uint8 SETTLING=5

uint8 phase
float64 force_rate_nps
float64 predicted_force_n
float64 estimated_stiffness_n_per_m
float64 raw_velocity_mps
float64 limited_velocity_mps
bool speed_limited
bool acceleration_limited
