#!/bin/bash

### Joint command for cmd_vel
# ros2 topic pub /mst110cr2/cmd_vel geometry_msgs/msg/Twist "linear:
#   x: 0.0
#   y: 0.0
#   z: 0.0
# angular:
#   x: 0.0
#   y: 0.0
#   z: 0.0"

### joint command for tracks actuators
# ros2 topic pub /mst110cr2/track_cmd com3_msgs/msg/JointCmd "{
#   joint_name: ['right_track', 'left_track'],
#   effort: [30.0, 30.0]
# }" -r 10

# ros2 topic pub /mst110cr2/track_cmd com3_msgs/msg/JointCmd "{
#   joint_name: ['right_track', 'left_track'],
#   effort: [0.0, 0.0]
# }" -r 10


# ros2 topic pub /mst110cr2/rot_dump_cmd com3_msgs/msg/JointCmd "{
#   joint_name: ['rotate_joint', 'dump_joint'],
#   control_type: 2,
#   effort: [0.0, 10.0]
# }" -r 10


### Excavator machine setting command
#ros2 topic pub machine_setting_cmd com3_msgs/msg/ExcavatorCom3MachineSetting "{'engine_rpm':1800, 'power_eco_mode':true, 'travel_speed_mode':true, 'working_mode_notice':false, 'yellow_led_mode':0, 'front_control_mode':0, 'tracks_control_mode':0}" -r 10

# operation_mode=1      # 0: remote, 1: auto
# velocity_mode=0       # 0: vw, 1: tracked spped, 2: pilot input
# swing_mode=false          # false: angle control, true: lever input
# dump_mode=false           # false: angle control, true: lever input
# eco_mode=false            # false: power, true: eco
# speed_mode=false          # false: turtle, true: rabbit 

# ros2 topic pub --once /mst110cr2/crawler_carrier_machine_setting com3_msgs/msg/CrawlerCarrierCom3MachineSetting "{
#   operation_mode: ${operation_mode},
#   velocity_mode: ${velocity_mode},
#   swing_mode: ${swing_mode},
#   dump_mode: ${dump_mode},
#   eco_mode: ${eco_mode},
#   speed_mode: ${speed_mode}
# }"