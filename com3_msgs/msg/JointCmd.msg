# joint name of control target.
string[] joint_name

# control type of joint
# 0:position, 1:velocity, 2:effor
uint8 control_type

# controlled variable for each joint name
float64[] position
float64[] velocity
float64[] effort