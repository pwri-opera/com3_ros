########### State1 (base) ###########
# horn 
# true : buzzer
bool horn

# engine start
bool engine_start

# track lock
# false: unlock, true: locked
bool track_lock

# blade lock
# false: unlock, true: locked
bool blade_lock

# emg stop
# true: force stop machine
bool emg_stop

# shift up
# true: shift up
bool shift_up

# shift down
# true: shift down
bool shift_down


########### State2 (MC func) ###########

# back grading mode
# true : enable back grading
bool enable_back_grading

# enable MC mode
# true : enable MC mode
bool enable_mc

# lift_blade
# true : lifting blade on the ground 
bool lift_blade

# setting cutfill_level
bool cutfill_level_up
bool cutfill_level_down

# engine rpm　
# valid number is from 100 to 255. (100: temporay values)
float64 engine_rpm
