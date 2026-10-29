# Raw hardware feedback. This is motor angle, NOT jaw opening in metres.
std_msgs/Header header
uint8 motor_id
bool target_reached
uint16 raw_speed
uint32 raw_angle
float64 motor_speed_rad_s
float64 motor_angle_deg
