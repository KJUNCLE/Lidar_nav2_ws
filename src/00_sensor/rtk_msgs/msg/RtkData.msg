std_msgs/Header header

# WGS84 geodetic position.
float64 latitude
float64 longitude
float64 altitude

# NED velocity in m/s.
float32 north_velocity
float32 east_velocity
float32 down_velocity

# Euler attitude in degrees.
float32 roll
float32 pitch
float32 azimuth

# Angular velocity in deg/s.
float32 x_angular_velocity
float32 y_angular_velocity
float32 z_angular_velocity

# Linear acceleration in g.
float32 x_acc
float32 y_acc
float32 z_acc

# Standard deviations in the units of the related fields.
float32 latitude_std
float32 longitude_std
float32 altitude_std
float32 north_velocity_std
float32 east_velocity_std
float32 down_velocity_std
float32 roll_std
float32 pitch_std
float32 azimuth_std

float64 sec_of_week
uint32 gps_week_number

float32 temperature
float32 wheel_speed_status
uint8 ins_status
float32 position_type
float32 heading_type
float32 numsv

uint8 pdata_type
int16 pdata1
int16 pdata2
int16 pdata3

bool primary_checksum_ok
bool full_checksum_ok
uint8[] raw_frame
