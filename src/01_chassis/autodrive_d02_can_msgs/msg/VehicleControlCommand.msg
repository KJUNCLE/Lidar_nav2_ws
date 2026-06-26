uint8 DRIVE_MODE_AUTO=1
uint8 DRIVE_MODE_REMOTE=3

uint8 SHIFT_NONE=0
uint8 SHIFT_D=1
uint8 SHIFT_N=2
uint8 SHIFT_R=3

std_msgs/Header header

uint8 drive_mode
uint8 shift_level

# Encoded range: -400..400 rpm, factor 0.1.
float32 left_motor_speed_rpm
# Encoded range: -400..400 rpm, factor 0.1.
float32 right_motor_speed_rpm

bool brake_enable

uint8 left_turn_light_req
uint8 right_turn_light_req
uint8 position_light_req
uint8 low_beam_req
int8 steering_angle_speed_valid
uint8 brake_force_command_valid
