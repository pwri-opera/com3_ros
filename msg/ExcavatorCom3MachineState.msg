# lock command state by emergency remote controller
# false:Off, On:On
bool lock_cmd_state

# pilot shutoff valve state by emergency remote controller
# false:Locked, true:Unlocked
bool pilot_shutoff_valve_state

# system error from excavator system
# currentry, this value is from ZX200X7 system.
# false: normal, true: anomal
bool system_error 

# errors from other system.
# false: normal, true: anomal
bool can_error_pl
bool can_error_body
bool can_error_ict
bool lock_receiver_error
bool emergency_stop_receiver_error
bool switch_error

# control_state
# TBD
uint8 control_state

# hydraulic_oil_temp
# range is from -100 to 250 degC.
uint8 hydraulic_oil_temp

# enfine state
# 0 = off, 1 = on
bool engine_state

# alive counter is incremented by system
uint8 alive_counter


