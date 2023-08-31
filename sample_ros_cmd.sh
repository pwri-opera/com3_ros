#!/bin/bash

#ros2 topic pub test std_msgs/msg/Float64 "{data: 1}"

# ros2 topic pub test trajectory_msgs/msg/joint_trajectory "{joint_names: ['<your-name>'], points: {positions: [0.0], velocities: [1.0], effort: [0.0]}}"

# ros2 topic pub joint_states sensor_msgs/msg/JointState "{name: ['<your-name>'], position: [0.0], velocity: [1.0], effort: [0.0]}"


# ros2 topic pub test trajectory_msgs/msg/JointTrajectory "{joint_names: ['boom','angle'], points: [positions: [0.0], velocities: [1.0], effort: [0.0]]}"

# ros2 topic pub front_cmd com3_msgs/msg/JointCmd "{joint_name: ['boom','arm','bucket','swing'], position: [0.0,0,0,0], velocity: [0.0005,-0.0005,0.1275,-16], effort: [0,0,0,0]}" -r 10

ros2 topic pub front_cmd com3_msgs/msg/JointCmd "{joint_name: ['boom','arm','bucket','swing'], position: [0.0,0,0,0], velocity: [-1,1,-2,2], effort: [-1,2,-2,-1]}" -r 10

# ros2 topic pub machine_setting_cmd com3_msgs/msg/ExcavatorCom3MachineSetting "{'engine_rpm':900, 'power_eco_mode':true, 'travel_speed_mode':true, 'working_mode_notice':false, 'yellow_led_mode':0, 'horn':false, 'front_control_mode':1, 'tracks_control_mode':1}" -r 10

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
