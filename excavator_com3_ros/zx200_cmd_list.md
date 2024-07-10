| 区分 | 項目 | 種類 | メッセージの名前 | メッセージの型 | 要素 | 凡例 |
|---|---|---|---|---|---|---|
| 外部制御装置から油圧ショベルへの入力 | フロント(ブーム、アーム、バケット、旋回)への動作指令 | topic | /(excavator_name)/front_cmd | com3_ros/msg/JointCmd | Joint_name : String []<br>Effort: Float64 []<br>Velocity: Float64 []<br>Position: Float64 [] | Joint_name : String [bucket_joint, arm_joint, boom_joint, swing_joint]<br>Effort: Float64 [0,0,0,0]<br>Velocity: Float64 [0,0,0,0]<br>Position: Float64 [0,0,0,0] |
|  | 左右クローラへの動作指令 | topic | /(excavator_name)/tracks_cmd | com3_ros/msg/JointCmd | Joint_name : String []<br>Effort: Float64 []<br>Velocity: Float64 []<br>Position: Float64 [] | Joint_name: [left_track, right_track]<br>Effort: Float64[0,0]<br>Velocity: Float64[0,0]<br>Position: Float64[0,0]  |
|  | アタッチメントの動作指令 | topic | TBD | TBD | TBD | TBD |
|  | 車体中心の並進移動速度/回転各速度指令 | topic | /(excavator_name)/cmd_vel | geometry_msgs/msg/Twist | - | - |
|  | 非常停止 | topic | /(excavator_name)/emg_stop | std_msgs/msg/Bool | - | - |
|  | 機械設定指令 | topic | /(excavator_name)/machine_setting | com3_ros/msg/ ExcavatorCom3MachineSetting  | float64 engine_rpm<br>bool power_eco_mode<br>bool travel_speed_mode<br>bool working_mode_notice<br>uint8 yellow_led_mode<br>bool horn<br>uint8 front_control_mode<br>uint8 tracks_control_mode | - |
|  | 刃先位置指令1 | その他 | - | MoveGroupInterface | - | - |
|  | 刃先位置指令2 | action | /(excavator_name)/move_group | moveit_msgs/action/MoveGroup | - | - |
|  | 走行経路指令1 | topic | /(excavator_name)/goal_pose | geometry_msgs/msg/PoseStamped | - | - |
|  | 走行経路指令2 | action | /(excavator_name)/navigate_to_pose | nav2_msgs/msg/NavigateToPose | - | 　 |
| 油圧ショベルから外部装置への出力 | 各関節の角度、角速度 | topic | /(excavator_name)/joint_state | sensor_msgs/msg/JointState | - | name:[bucket_joint, arm_joint, boom_joint, swing_joint, ・・・・・]<br>TBD |
|  | ローカル座標系における車両の中心位置姿勢 | topic | /(excavator_name)/odom_pose | nav_msgs/msg/Odometry | 　 | 　 |
|  | グローバル座標(平面直角座標)系における車体の中心位置姿勢 | topic | /(excavator_name)/glocal_pose | nav_msgs/msg/Odometry | 　 | 　 |
|  | 油圧アクチュエータのメイン/パイロット圧力 | topic | TBD | 　 | 　 | 　 |
|  | 車体状態 | topic | 　 | com3_msgs/msg/ExcavatorCom3MachineState | bool lock_cmd_state<br>bool pilot_shutoff_valve_state<br>bool system_error <br>bool can_error_pl<br>bool can_error_body<br>bool can_error_ict<br>bool lock_receiver_error<br>bool emergency_stop_receiver_error<br>bool switch_error<br>uint8 control_state<br>uint8 hydraulic_oil_temp<br>bool engine_state<br>uint8 alive_counter | 　 |
