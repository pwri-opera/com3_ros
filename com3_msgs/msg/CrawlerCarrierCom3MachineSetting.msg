## input/output

uint8 operation_mode       # 0: remote, 1: auto
uint8 velocity_mode        # 0: vw, 1: tracked spped, 2: pilot input
bool swing_mode             # false: angle control, true: lever input
bool dump_mode             # false: angle control, true: lever input
bool eco_mode # false: power, true: eco
bool speed_mode # false: turtle, true: rabbit 

# ブレーキ解除要求
# true=解除要求あり、false=解除要求なし
bool park_brake_release
bool swing_brake_release
bool dump_brake_release

## output only
bool engine_on
uint8 state_number # control state for machine
uint16 engine_rpm