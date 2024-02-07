#!/bin/bash

### Joint command for excavator front actuators
ros2 topic pub front_cmd com3_msgs/msg/JointCmd "{joint_name: ['boom_joint','arm_joint','bucket_joint','swing_joint'], position: [0.0,0,0,0], velocity: [0,0,0,0], effort: [0,0,0,0]}" -r 10

### Excavator machine setting command
ros2 topic pub machine_setting_cmd com3_msgs/msg/ExcavatorCom3MachineSetting "{'engine_rpm':1800, 'power_eco_mode':true, 'travel_speed_mode':true, 'working_mode_notice':false, 'yellow_led_mode':0, 'front_control_mode':0, 'tracks_control_mode':0}" -r 10