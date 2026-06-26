uint8 LEVEL_NONE=0
uint8 LEVEL_1=1
uint8 LEVEL_2=2
uint8 LEVEL_3=3

std_msgs/Header header
uint32 can_id

uint8 bms_soc_warning
uint8 mcu_disconnect_warning
uint8 mcu_motor_warning
uint8 mcu_overspeed_warning

uint8 turn_disconnect_warning
uint8 turn_lock_warning
uint8 turn_unstoppable_warning
uint8 sas_angle_error
uint8 brake_error_warning
