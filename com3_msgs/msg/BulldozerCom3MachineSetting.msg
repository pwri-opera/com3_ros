# track lock
# false: unlock, true: locked
bool track_lock

# blade lock
# false: unlock, true: locked
bool blade_lock

# engine rpm　
# valid number is from 100 to 255. (100: temporay values)
float64 engine_rpm

# Front control mode is retained as ROS data and is not sent over CAN.
uint8 front_control_mode
