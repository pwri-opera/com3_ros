#!/bin/bash

# # Joint command for cmd_vel
# ros2 topic pub /mst110cr_2/cmd_vel geometry_msgs/msg/Twist "linear:
#   x: 0.0
#   y: 0.0
#   z: 0.0
# angular:
#   x: 0.0
#   y: 0.0
#   z: 0.0" -r 5

# joint command for tracks actuators
# ros2 topic pub /mst110cr_2/track_cmd com3_msgs/msg/JointCmd "{
#   joint_name: ['right_track', 'left_track'],
#   control_type: 2,
#   effort: [50.0, 50.0]
# }" -r 10

# ros2 topic pub /mst110cr_2/track_cmd com3_msgs/msg/JointCmd "{
#   joint_name: ['right_track', 'left_track'],
#   control_type: 1,
#   velocity: [0.5, 0.5]
# }" -r 10



# ros2 topic pub /mst110cr_2/rot_dump_cmd com3_msgs/msg/JointCmd "{
#   joint_name: ['rotate_joint', 'dump_joint'],
#   control_type: 2,
#   effort: [50.0, 0.0]
# }" -r 10

# ros2 topic pub /mst110cr_2/rot_dump_cmd com3_msgs/msg/JointCmd "{
#   joint_name: ['rotate_joint', 'dump_joint'],
#   control_type: 0,
#   position: [0, 0]
# }"


# # Set dump angle command
# TARGET_DEG=60  # 送りたい角度 [deg]
# TARGET_RAD=$(echo "scale=6; $TARGET_DEG * 3.1415926535 / 180" | bc -l)
# echo "Sending dump target angle: $TARGET_DEG deg = $TARGET_RAD rad"
# ros2 action send_goal /set_dump_angle com3_msgs/action/SetDumpAngle "{target_angle: $TARGET_RAD}" --feedback

# # Set swing angle command
# TARGET_DEG=60  # 送りたい角度 [deg]
# TARGET_RAD=$(echo "scale=6; $TARGET_DEG * 3.1415926535 / 180" | bc -l)
# echo "Sending swng target angle: $TARGET_DEG deg = $TARGET_RAD rad"
# ros2 action send_goal /set_swing_angle com3_msgs/action/SetSwingAngle "{target_angle: $TARGET_RAD}" --feedback



### Excavator machine setting command
# #ros2 topic pub machine_setting_cmd com3_msgs/msg/ExcavatorCom3MachineSetting "{'engine_rpm':1800, 'power_eco_mode':true, 'travel_speed_mode':true, 'working_mode_notice':false, 'yellow_led_mode':0, 'front_control_mode':0, 'tracks_control_mode':0}" -r 10

# operation_mode=1      # 0: remote, 1: auto
# velocity_mode=1        # 0: vw, 1: tracked spped, 2: pilot input
# swing_mode=true          # false: angle control, true: lever input
# dump_mode=true           # false: angle control, true: lever input
# eco_mode=false            # false: power, true: eco
# speed_mode=false          # false: turtle, true: rabbit 

# park_brake_release=true
# swing_brake_release=false
# dump_brake_release=false


# ros2 topic pub /mst110cr_2/crawler_carrier_machine_setting com3_msgs/msg/CrawlerCarrierCom3MachineSetting "{
#   operation_mode: ${operation_mode},
#   velocity_mode: ${velocity_mode},
#   swing_mode: ${swing_mode},
#   dump_mode: ${dump_mode},
#   eco_mode: ${eco_mode},
#   speed_mode: ${speed_mode},
#   park_brake_release: ${park_brake_release},
#   swing_brake_release: ${swing_brake_release},
#   dump_brake_release: ${dump_brake_release}
# }"