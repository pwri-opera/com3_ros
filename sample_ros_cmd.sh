#!/bin/bash

#ros2 topic pub test std_msgs/msg/Float64 "{data: 1}"

# ros2 topic pub test trajectory_msgs/msg/joint_trajectory "{joint_names: ['<your-name>'], points: {positions: [0.0], velocities: [1.0], effort: [0.0]}}"

# ros2 topic pub joint_states sensor_msgs/msg/JointState "{name: ['<your-name>'], position: [0.0], velocity: [1.0], effort: [0.0]}"


# ros2 topic pub test trajectory_msgs/msg/JointTrajectory "{joint_names: ['boom','angle'], points: [positions: [0.0], velocities: [1.0], effort: [0.0]]}"

# ros2 topic pub front_cmd com3_msgs/msg/JointCmd "{joint_name: ['boom','arm','bucket','swing'], position: [0.0,0,0,0], velocity: [0.0005,-0.0005,0.1275,-16], effort: [0,0,0,0]}" -r 10

# Sample cmd for front actuators
# ros2 topic pub /zx200/front_cmd com3_msgs/msg/JointCmd "{joint_name: ['boom','arm','bucket','swing'], position: [0,0,0,0], velocity: [0,0,0,0], effort: [0.0,0,0,0]}" -r 10

# boom cmd
# ros2 topic pub /zx200/front_cmd com3_msgs/msg/JointCmd "{joint_name: ['boom','arm','bucket','swing'], position: [0,0,0,0], velocity: [0,0,0,0], effort: [-0.24,0,0,0]}" -r 10 

# ros2 topic pub /zx200/machine_setting_cmd com3_msgs/msg/ExcavatorCom3MachineSetting "{'engine_rpm':2500, 'power_eco_mode':true, 'travel_speed_mode':true, 'working_mode_notice':false, 'yellow_led_mode':0, 'horn':false, 'front_control_mode':1, 'tracks_control_mode':1}" -r 10

ros2 topic pub /zx200/front_cmd com3_msgs/msg/JointCmd "{joint_name: ['boom_joint','arm_joint','bucket_joint','swing_joint', 'bucket_end_joint'], position: [0.0,0,0,0,0], velocity: [0,0,0,0,0], effort: [0, 0, 0, 0, 0]}" -r 10

# 非常停止
# ros2 topic pub /zx200/emg_stop std_msgs/msg/Bool "data: true" 

# ros2 topic pub /zx200/tracks_cmd com3_msgs/msg/JointCmd "{joint_name: ['left_track','right_track'], position: [0,0], velocity: [0,0], effort: [0.1,0.1]}" -r 10


# ros2 topic pub test std_msgs/msg/Int32MultiArray "{layout: {dim: [],data_offset: 0},{data: []}}"

# flow rate 2
# cansend vcan0 18FF80E8#1122334455667788

# flow rate 1
#cansend vcan0 18FF7FE8#0102000000000000

# machine state
# cansend vcan0 18FF84E8#0102000000000000

# front angle
#cansend vcan0 18FF7DE8#0000000000000000

# front angle velocity
#cansend vcan0 18FF78E8#0102000000000000
