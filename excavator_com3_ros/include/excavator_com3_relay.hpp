#pragma once

#include <iostream>
#include <memory>
#include <boost/make_shared.hpp>
#include <boost/bind/bind.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <com3_msgs/msg/joint_cmd.hpp>
#include <com3_msgs/msg/excavator_com3_machine_setting.hpp>
#include <com3_msgs/msg/excavator_com3_machine_state.hpp>
#include <com3/excavator_com3_dbc.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define initial_interval 100 //[ms]
#define cmd_timeout 500      //[ms]

namespace excavator_com3_can
{
  class gateway : public excavator_com3_dbc, public rclcpp::Node
  {
  public:
    gateway(boost::asio::io_context &io, std::string can_port, std::string dbc_path, std::vector<double> dead_zone_value)
        : excavator_com3_dbc(dbc_path),
          rclcpp::Node("gateway"),
          sock(io),
          send_pilot_cmd_1_timer(io, boost::asio::chrono::milliseconds(initial_interval)),
          send_pilot_cmd_2_timer(io, boost::asio::chrono::milliseconds(initial_interval)),
          send_front_vel_cmd_timer(io, boost::asio::chrono::milliseconds(initial_interval)),
          send_tracks_vel_cmd_timer(io, boost::asio::chrono::milliseconds(initial_interval)),
          send_machine_setting_cmd_timer(io, boost::asio::chrono::milliseconds(initial_interval)),
          dead_zone(dead_zone_value)
    {
      const auto idx = canary::get_interface_index(can_port);
      auto const ep = canary::raw::endpoint{idx};
      sock.open();
      sock.bind(ep);

      is_emg_stop_enable = false;

      pilot_cmd_1 = boost::make_shared<excavator_com3::Pilot_Pressure_Cmd_1>();
      pilot_cmd_2 = boost::make_shared<excavator_com3::Pilot_Pressure_Cmd_2>();
      front_vel_cmd = boost::make_shared<excavator_com3::Velocity_Cmd_1>();
      tracks_vel_cmd = boost::make_shared<excavator_com3::Velocity_Cmd_2>();
      setting_cmd = boost::make_shared<excavator_com3::Machine_Setting_Cmd>();

      publisher_variable_init();

      start_receive();
      send_pilot_cmd_1_timer.async_wait(boost::bind(&gateway::send_pilot_cmd_1, this));
      send_pilot_cmd_2_timer.async_wait(boost::bind(&gateway::send_pilot_cmd_2, this));
      send_front_vel_cmd_timer.async_wait(boost::bind(&gateway::send_front_vel_cmd, this));
      send_tracks_vel_cmd_timer.async_wait(boost::bind(&gateway::send_tracks_vel_cmd, this));
      send_machine_setting_cmd_timer.async_wait(boost::bind(&gateway::send_machine_setting_cmd, this));

      sub_front_cmd_ = this->create_subscription<com3_msgs::msg::JointCmd>("front_cmd", 10,
                                                                           [this](const com3_msgs::msg::JointCmd::SharedPtr msg)
                                                                           { this->front_cmd_callback(msg); });
      sub_tracks_cmd_ = this->create_subscription<com3_msgs::msg::JointCmd>("tracks_cmd", 10,
                                                                            [this](const com3_msgs::msg::JointCmd::SharedPtr msg)
                                                                            { this->tracks_cmd_callback(msg); });
      sub_twist_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
                                                                            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
                                                                            { this->twist_callback(msg); });
      sub_machine_setting_cmd_ = this->create_subscription<com3_msgs::msg::ExcavatorCom3MachineSetting>("machine_setting_cmd", 10,
                                                                                                        [this](const com3_msgs::msg::ExcavatorCom3MachineSetting::SharedPtr msg)
                                                                                                        { this->setting_cmd_callback(msg); });
      // sub_attachment_cmd_ = this->create_subscription<com3_msgs::msg::ExcavatorCom3Attachment>("lever2", 10,
      //                                                                                   [this](const com3_msgs::msg::ExcavatorCom3Attachment::SharedPtr msg)
      //                                                                                   { this->attachment_cmd_callback(msg); });
      sub_emg_stop_cmd_ = this->create_subscription<std_msgs::msg::Bool>("emg_stop", 10,
                                                                        [this](const std_msgs::msg::Bool::SharedPtr msg)
                                                                         { this->emg_stop_cmd_callback(msg); });

      pub_machine_state = this->create_publisher<com3_msgs::msg::ExcavatorCom3MachineState>("machine_state", 10);
      pub_hydraulic_flow_1 = this->create_publisher<std_msgs::msg::Int32MultiArray>("hydraulics_flow_front", 10);
      pub_hydraulic_flow_2 = this->create_publisher<std_msgs::msg::Int32MultiArray>("hydraulics_flow_tracks", 10);
      pub_joint_state = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);
      pub_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
      pub_pi_pressure_1 = this->create_publisher<std_msgs::msg::Float64MultiArray>("pi_pressure_1", 10);
      pub_pi_pressure_2 = this->create_publisher<std_msgs::msg::Float64MultiArray>("pi_pressure_1", 10);
      pub_main_pressure_1 = this->create_publisher<std_msgs::msg::Float64MultiArray>("main_pressure_1", 10);
      pub_main_pressure_2 = this->create_publisher<std_msgs::msg::Float64MultiArray>("main_pressure_1", 10);

      last_front_cmd_time = last_tracks_cmd_time = last_twist_cmd_time = last_machine_setting_cmd_time = this->get_clock()->now();
    };
    ~gateway()
    {
      send_pilot_cmd_1_timer.cancel();
      send_pilot_cmd_2_timer.cancel();
      send_front_vel_cmd_timer.cancel();
      send_tracks_vel_cmd_timer.cancel();
      send_machine_setting_cmd_timer.cancel();
    }

  private:
    // CAN send/recv functions
    void send_pilot_cmd_1()
    {
      rclcpp::Duration elapsed_time = this->get_clock()->now() - last_front_cmd_time;
      frame f = {};
      if (elapsed_time.seconds() * 1000. > cmd_timeout || is_emg_stop_enable)
      {
        pilot_cmd_1->boom_down = 0;
        pilot_cmd_1->boom_up = 0;
        pilot_cmd_1->arm_crowd = 0;
        pilot_cmd_1->arm_dump = 0;
        pilot_cmd_1->bucket_crowd = 0;
        pilot_cmd_1->bucket_dump = 0;
        pilot_cmd_1->swing_left = 0;
        pilot_cmd_1->swing_right = 0;
      }
      excavator_com3_dbc::encode(*pilot_cmd_1, f);

      sock.async_send(canary::net::buffer(&f, sizeof(f)), boost::bind(&gateway::send_handle, this));
      send_pilot_cmd_1_timer.expires_at(send_pilot_cmd_1_timer.expiry() + boost::asio::chrono::milliseconds(pilot_cmd_1->Cycle_time()));
      send_pilot_cmd_1_timer.async_wait(boost::bind(&gateway::send_pilot_cmd_1, this));
    }

    void send_pilot_cmd_2()
    {
      rclcpp::Duration elapsed_time = this->get_clock()->now() - last_tracks_cmd_time;
      frame f = {};
      if (elapsed_time.seconds() * 1000. > cmd_timeout || is_emg_stop_enable)
      {
        pilot_cmd_2->left_track_backward = 0;
        pilot_cmd_2->left_track_forward = 0;
        pilot_cmd_2->right_track_backward = 0;
        pilot_cmd_2->right_track_forward = 0;
      }
      excavator_com3_dbc::encode(*pilot_cmd_2, f);

      sock.async_send(canary::net::buffer(&f, sizeof(f)), boost::bind(&gateway::send_handle, this));
      send_pilot_cmd_2_timer.expires_at(send_pilot_cmd_2_timer.expiry() + boost::asio::chrono::milliseconds(pilot_cmd_2->Cycle_time()));
      send_pilot_cmd_2_timer.async_wait(boost::bind(&gateway::send_pilot_cmd_2, this));
    }

    void send_front_vel_cmd()
    {
      rclcpp::Duration elapsed_time = this->get_clock()->now() - last_front_cmd_time;
      frame f = {};
      if (elapsed_time.seconds() * 1000. > cmd_timeout || is_emg_stop_enable)
      {
        front_vel_cmd->boom_target_anguler_velocity = 0;
        front_vel_cmd->arm_target_anguler_velocity = 0;
        front_vel_cmd->bucket_target_anguler_velocity = 0;
        front_vel_cmd->swing_target_anguler_velocity = 0;
      }
      excavator_com3_dbc::encode(*front_vel_cmd, f);

      sock.async_send(canary::net::buffer(&f, sizeof(f)), boost::bind(&gateway::send_handle, this));
      send_front_vel_cmd_timer.expires_at(send_front_vel_cmd_timer.expiry() + boost::asio::chrono::milliseconds(front_vel_cmd->Cycle_time()));
      send_front_vel_cmd_timer.async_wait(boost::bind(&gateway::send_front_vel_cmd, this));
    }

    void send_tracks_vel_cmd()
    {
      frame f = {};
      rclcpp::Duration elapsed_time = this->get_clock()->now() - last_twist_cmd_time;
      if (elapsed_time.seconds() * 1000. > cmd_timeout || is_emg_stop_enable)
      {
        tracks_vel_cmd->target_travel_center_velocity = 0;
        tracks_vel_cmd->target_travel_yaw_rate = 0;
      }
      elapsed_time = this->get_clock()->now() - last_tracks_cmd_time;
      if (elapsed_time.seconds() * 1000. > cmd_timeout || is_emg_stop_enable)
      {
        tracks_vel_cmd->target_travel_center_velocity = 0;
        tracks_vel_cmd->target_travel_yaw_rate = 0;
      }

      excavator_com3_dbc::encode(*tracks_vel_cmd, f);

      sock.async_send(canary::net::buffer(&f, sizeof(f)), boost::bind(&gateway::send_handle, this));
      send_tracks_vel_cmd_timer.expires_at(send_tracks_vel_cmd_timer.expiry() + boost::asio::chrono::milliseconds(tracks_vel_cmd->Cycle_time()));
      send_tracks_vel_cmd_timer.async_wait(boost::bind(&gateway::send_tracks_vel_cmd, this));
    }

    void send_machine_setting_cmd()
    {
      frame f = {};
      rclcpp::Duration elapsed_time = this->get_clock()->now() - last_machine_setting_cmd_time;
      if (elapsed_time.seconds() * 1000. > cmd_timeout)
      {
        // machine setting is not set timeout
      }

      setting_cmd->alive_counter++;
      excavator_com3_dbc::encode(*setting_cmd, f);

      sock.async_send(canary::net::buffer(&f, sizeof(f)), boost::bind(&gateway::send_handle, this));
      send_machine_setting_cmd_timer.expires_at(send_machine_setting_cmd_timer.expiry() + boost::asio::chrono::milliseconds(tracks_vel_cmd->Cycle_time()));
      send_machine_setting_cmd_timer.async_wait(boost::bind(&gateway::send_machine_setting_cmd, this));
    }

    void send_handle() {}

    void start_receive()
    {
      // rttr::type t = rttr::type::get<excavator_com3::Hydraulic_Flow_Rate_2>();
      // for (auto &prop : t.get_properties())
      //   std::cout << "name: " << prop.get_name();
      // std::cout << std::endl;

      sock.async_receive(canary::net::buffer(&recv_f, sizeof(recv_f)), boost::bind(&gateway::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

      if (recv_f.header.payload_length())
      {
        excavator_com3_dbc::decode(recv_f);
        switch (recv_f.header.id() | 0x80000000)
        {
        case excavator_com3::Machine_State::id:
          get_can_bus_msg(com3_machine_state);
          machine_state.lock_cmd_state = com3_machine_state.lock_cmd_state;
          machine_state.pilot_shutoff_valve_state = com3_machine_state.pilot_shutoff_valve_state;
          machine_state.system_error = com3_machine_state.system_error;
          machine_state.can_error_pl = com3_machine_state.can_error_pl;
          machine_state.can_error_ict = com3_machine_state.can_error_ict;
          machine_state.can_error_body = com3_machine_state.can_error_body;
          machine_state.lock_receiver_error = com3_machine_state.lock_receiver_error;
          machine_state.emergency_stop_receiver_error = com3_machine_state.emergency_stop_receiver_error;
          machine_state.switch_error = com3_machine_state.switch_error;
          machine_state.control_state = com3_machine_state.control_state;
          machine_state.hydraulic_oil_temp = machine_state.hydraulic_oil_temp;
          machine_state.engine_state = machine_state.engine_state;
          machine_state.alive_counter = com3_machine_state.alive_counter;
          pub_machine_state->publish(machine_state);
          break;
        case excavator_com3::Hydraulic_Flow_Rate_2::id:
          // std_msgs::msg::Int32MultiArray a;
          get_can_bus_msg(com3_h_flow_2);
          hydraulic_flow_2.data[0] = com3_h_flow_2.right_track_motor_a_flow_rate;
          hydraulic_flow_2.data[1] = com3_h_flow_2.right_track_motor_b_flow_rate;
          hydraulic_flow_2.data[2] = com3_h_flow_2.left_track_motor_a_flow_rate;
          hydraulic_flow_2.data[3] = com3_h_flow_2.left_track_motor_b_flow_rate;
          pub_hydraulic_flow_2->publish(hydraulic_flow_2);
          break;
        case excavator_com3::Hydraulic_Flow_Rate_1::id:
          get_can_bus_msg(com3_h_flow_1);
          hydraulic_flow_1.data[0] = com3_h_flow_1.boom_cylinder_bottom_flow_rate;
          hydraulic_flow_1.data[1] = com3_h_flow_1.boom_cylinder_rod_flow_rate;
          hydraulic_flow_1.data[2] = com3_h_flow_1.arm_cylinder_bottom_flow_rate;
          hydraulic_flow_1.data[3] = com3_h_flow_1.arm_cylinder_rod_flow_rate;
          hydraulic_flow_1.data[4] = com3_h_flow_1.bucket_cylinder_bottom_flow_rate;
          hydraulic_flow_1.data[5] = com3_h_flow_1.bucket_cylinder_rod_flow_rate;
          hydraulic_flow_1.data[6] = com3_h_flow_1.swing_motor_a_flow_rate;
          hydraulic_flow_1.data[7] = com3_h_flow_1.swing_motor_b_flow_rate;
          pub_hydraulic_flow_1->publish(hydraulic_flow_1);
          break;
        case excavator_com3::Pilot_Pressure_2::id:
          get_can_bus_msg(com3_pi_pressure_2);
          pi_pressure_2.data[0] = com3_pi_pressure_2.right_track_forward_pilot_prs;
          pi_pressure_2.data[1] = com3_pi_pressure_2.right_track_backward_pilot_prs;
          pi_pressure_2.data[2] = com3_pi_pressure_2.left_track_forward_pilot_prs;
          pi_pressure_2.data[3] = com3_pi_pressure_2.left_track_backward_pilot_prs;
          pi_pressure_2.data[4] = com3_pi_pressure_2.attachment_a_pilot_pressure;
          pi_pressure_2.data[5] = com3_pi_pressure_2.attachment_b_pilot_pressure;
          pi_pressure_2.data[6] = com3_pi_pressure_2.assist_a_pilot_pressure;
          pi_pressure_2.data[7] = com3_pi_pressure_2.assist_b_pilot_pressure;
          pub_pi_pressure_2->publish(pi_pressure_2);
          break;
        case excavator_com3::Pilot_Pressure_1::id:
          get_can_bus_msg(com3_pi_pressure_1);
          pi_pressure_1.data[0] = com3_pi_pressure_1.boom_up_pilot_pressure;
          pi_pressure_1.data[1] = com3_pi_pressure_1.boom_up_pilot_pressure;
          pi_pressure_1.data[2] = com3_pi_pressure_1.arm_crowd_pilot_pressure;
          pi_pressure_1.data[3] = com3_pi_pressure_1.arm_dump_pilot_pressure;
          pi_pressure_1.data[4] = com3_pi_pressure_1.bucket_crowd_pilot_pressure;
          pi_pressure_1.data[5] = com3_pi_pressure_1.bucket_dump_pilot_pressure;
          pi_pressure_1.data[6] = com3_pi_pressure_1.swing_right_pilot_pressure;
          pi_pressure_1.data[7] = com3_pi_pressure_1.swing_left_pilot_pressure;
          pub_pi_pressure_1->publish(pi_pressure_1);
          break;
        case excavator_com3::Pressure_2::id:
          get_can_bus_msg(com3_main_pressure_2);
          main_pressure_2.data[0] = com3_main_pressure_2.right_track_motor_a_pressure;
          main_pressure_2.data[1] = com3_main_pressure_2.right_track_motor_b_pressure;
          main_pressure_2.data[2] = com3_main_pressure_2.left_track_motor_a_pressure;
          main_pressure_2.data[3] = com3_main_pressure_2.left_track_motor_b_pressure;
          main_pressure_2.data[4] = com3_main_pressure_2.attachment_a_pressure;
          main_pressure_2.data[5] = com3_main_pressure_2.attachment_b_pressure;
          main_pressure_2.data[6] = com3_main_pressure_2.assist_a_pressure;
          main_pressure_2.data[7] = com3_main_pressure_2.assit_b_pressure;
          pub_main_pressure_2->publish(main_pressure_2);
          break;
        case excavator_com3::Pressure_1::id:
          get_can_bus_msg(com3_main_pressure_1);
          main_pressure_1.data[0] = com3_main_pressure_1.boom_cylinder_bottom_pressure;
          main_pressure_1.data[1] = com3_main_pressure_1.boom_cylinder_rod_pressure;
          main_pressure_1.data[2] = com3_main_pressure_1.arm_cylinder_bottom_pressure;
          main_pressure_1.data[3] = com3_main_pressure_1.arm_cylinder_rod_pressure;
          main_pressure_1.data[4] = com3_main_pressure_1.bucket_cylinder_bottom_pressure;
          main_pressure_1.data[5] = com3_main_pressure_1.bucket_cylinder_rod_pressure;
          main_pressure_1.data[6] = com3_main_pressure_1.swing_motor_a_pressure;
          main_pressure_1.data[7] = com3_main_pressure_1.swing_motor_b_pressure;
          pub_main_pressure_1->publish(main_pressure_1);
          break;
        case excavator_com3::Vehicle_Azimuth::id:
          // heading direction
          is_pose_set = is_pose_set | 0x02;
          get_can_bus_msg(com3_vehicle_azimuth);
          quat_tf.setRPY(com3_roll_pitch.roll_angle, com3_roll_pitch.pitch_angle, com3_vehicle_azimuth.track_body_azimuth * -1);
          pose.pose.orientation = tf2::toMsg(quat_tf);
          // pub_pose->publish(pose);
          break;
        case excavator_com3::Swing_Center_Position_3::id:
          is_pose_set = is_pose_set | 0x10;
          get_can_bus_msg(com3_swing_center_pos_z);
          pose.pose.position.z = com3_swing_center_pos_z.swing_center_position_z;
          break;
        case excavator_com3::Swing_Center_Position_2::id:
          is_pose_set = is_pose_set | 0x08;
          get_can_bus_msg(com3_swing_center_pos_y);
          pose.pose.position.y = com3_swing_center_pos_y.swing_center_position_y;
          break;
        case excavator_com3::Swing_Center_Position_1::id:
          is_pose_set = is_pose_set | 0x04;
          get_can_bus_msg(com3_swing_center_pos_x);
          pose.pose.position.x = com3_swing_center_pos_x.swing_center_position_x;
          break;
        case excavator_com3::Front_Pin_Position_3::id:
          break;
        case excavator_com3::Front_Pin_Position_2::id:
          break;
        case excavator_com3::Front_Pin_Position_1::id:
          break;
        case excavator_com3::Roll_Pitch_Angle::id:
          // roll and pitch angle of vehicle
          // 車体(上部旋回体)のroll, pitch
          is_pose_set = is_pose_set | 0x01;
          get_can_bus_msg(com3_roll_pitch);
          break;
        case excavator_com3::Front_Angular_Velocity::id:
          get_can_bus_msg(com3_front_ang_vel);
          joint_state.velocity[0] = com3_front_ang_vel.swing_relative_angular_velocity;
          joint_state.velocity[1] = com3_front_ang_vel.boom_relative_angular_velocity;
          joint_state.velocity[2] = com3_front_ang_vel.arm_relative_angular_velocity;
          joint_state.velocity[3] = com3_front_ang_vel.bucket_relative_angular_velocity;
          is_js_set = is_js_set | 0x1;
          break;
        case excavator_com3::Front_Angle::id:
          get_can_bus_msg(com3_front_ang);
          joint_state.position[0] = com3_front_ang.swing_relative_angle;
          joint_state.position[1] = com3_front_ang.boom_relative_angle;
          joint_state.position[2] = com3_front_ang.arm_relative_angle;
          joint_state.position[3] = com3_front_ang.bucket_relative_angle;
          is_js_set = is_js_set | 0x2;
          break;
        default:
          break;
        }
        // pose publich
        if (is_pose_set & 0x1F)
        {
          is_pose_set = 0;
          pose.header.stamp = this->get_clock()->now();
          pub_pose->publish(pose);
        }
        // front velocity angle & angle publish
        if (is_js_set & 0x3)
        {
          is_js_set = 0;
          joint_state.header.stamp = this->get_clock()->now();
          pub_joint_state->publish(joint_state);
        }
      }
    }

    void handle_receive(const boost::system::error_code &error,
                        std::size_t /*bytes_transferred*/)
    {
      if (!error)
      {
        start_receive();
      }
    }

    // Topic callback functions
    void front_cmd_callback(const com3_msgs::msg::JointCmd::SharedPtr &msg)
    {
      last_front_cmd_time = this->get_clock()->now();
      int joint_num = msg->joint_name.size();

      if ((int)msg->effort.size() == joint_num && (int)msg->velocity.size() == joint_num)
      {
        for (int i = 0; i < (int)msg->joint_name.size(); i++)
        {
          if (msg->joint_name[i] == "swing_joint")
          {
            effort2pilot_pressure(msg->effort[i], pilot_cmd_1->swing_left, pilot_cmd_1->swing_right,dead_zone[7],dead_zone[6]);
            front_vel_cmd->swing_target_anguler_velocity = msg->velocity[i];
          }
          else if (msg->joint_name[i] == "boom_joint")
          {
            effort2pilot_pressure(msg->effort[i], pilot_cmd_1->boom_down, pilot_cmd_1->boom_up,dead_zone[1],dead_zone[0]);
            front_vel_cmd->boom_target_anguler_velocity = msg->velocity[i];
          }
          else if (msg->joint_name[i] == "arm_joint")
          {
            effort2pilot_pressure(msg->effort[i], pilot_cmd_1->arm_crowd, pilot_cmd_1->arm_dump,dead_zone[2],dead_zone[3]);
            front_vel_cmd->arm_target_anguler_velocity = msg->velocity[i];
          }
          else if (msg->joint_name[i] == "bucket_joint")
          {
            effort2pilot_pressure(msg->effort[i], pilot_cmd_1->bucket_crowd, pilot_cmd_1->bucket_dump,dead_zone[4],dead_zone[5]);
            front_vel_cmd->bucket_target_anguler_velocity = msg->velocity[i];
          }
        }
      }
    }

    void tracks_cmd_callback(const com3_msgs::msg::JointCmd::SharedPtr &msg)
    {
      last_tracks_cmd_time = this->get_clock()->now();
      int joint_num = msg->joint_name.size();

      if ((int)msg->effort.size() == joint_num && (int)msg->velocity.size() == joint_num)
      {
        for (int i = 0; i < joint_num; i++)
        {
          if (msg->joint_name[i] == "left_track")
          {
            effort2pilot_pressure(msg->effort[i], pilot_cmd_2->left_track_forward, pilot_cmd_2->left_track_backward,dead_zone[10],dead_zone[11]);
            tracks_vel_cmd->target_travel_left_velocity = msg->velocity[i];
          }
          else if (msg->joint_name[i] == "right_track")
          {
            effort2pilot_pressure(msg->effort[i], pilot_cmd_2->right_track_forward, pilot_cmd_2->right_track_backward,dead_zone[9],dead_zone[10]);
            tracks_vel_cmd->target_travel_right_velocity = msg->velocity[i];
          }
        }
      }
    }

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr &msg)
    {
      last_twist_cmd_time = this->get_clock()->now();
      tracks_vel_cmd->target_travel_center_velocity = msg->linear.x;
      tracks_vel_cmd->target_travel_yaw_rate = msg->angular.z;
    }

    void setting_cmd_callback(const com3_msgs::msg::ExcavatorCom3MachineSetting::SharedPtr &msg)
    {
      last_machine_setting_cmd_time = this->get_clock()->now();
      setting_cmd->engine_rpm = msg->engine_rpm;
      setting_cmd->power_eco_mode = msg->power_eco_mode;
      setting_cmd->travel_speed_mode = msg->travel_speed_mode;
      setting_cmd->working_mode_notice = msg->working_mode_notice;
      setting_cmd->yellow_led_mode = msg->yellow_led_mode;
      setting_cmd->horn_cmd = msg->horn;
      setting_cmd->front_signal_switch_command = msg->front_control_mode;
      setting_cmd->travel_signal_switch_command = msg->tracks_control_mode;
    }

    void emg_stop_cmd_callback(const std_msgs::msg::Bool::SharedPtr &msg)
    {
      is_emg_stop_enable = msg->data;
    }

    void effort2pilot_pressure(double effort, double &out_plus, double &out_minus, const double dead_zone_plus, const double dead_zone_minus)
    {
      const double max_effort = 1;//=100[%]
      const double max_phys_value = 5;//[Mpa]
      const double zero_input_th = 0.001;
      if (std::abs(effort) > max_effort)
        effort = max_effort * std::signbit(effort);

      // ZX200では最大パイロット圧は5MPaなので以下の通りeffort[％]をout[MPa]に変換する。
      // 汎用的にこのノードを利用するためには係数を取得してinputに乗算するか、
      // ここでは物理量を扱わず、よりハードウェアに近い下位のソフトウェアで係数をかける必要がある。
      if( std::abs(effort) < zero_input_th )
      {
        out_plus = 0;
        out_minus = 0;
      }
      else if(effort > 0.)
      {
        out_plus = (std::abs(effort)+dead_zone_plus) / max_effort * max_phys_value;
        out_minus = 0;
      }
      else
      {
        out_plus = 0;
        out_minus = (std::abs(effort)+dead_zone_minus) / max_effort * max_phys_value;
      }
    }

    void publisher_variable_init()
    {
      hydraulic_flow_1.data.resize(8);
      hydraulic_flow_1.layout.dim.resize(8);
      hydraulic_flow_1.layout.dim[0].label = "boom_cylinder_bottom_flow_rate";
      hydraulic_flow_1.layout.dim[1].label = "boom_cylinder_rod_flow_rate";
      hydraulic_flow_1.layout.dim[2].label = "arm_cylinder_bottom_flow_rate";
      hydraulic_flow_1.layout.dim[3].label = "arm_cylinder_rod_flow_rate";
      hydraulic_flow_1.layout.dim[4].label = "bucket_cylinder_bottom_flow_rate";
      hydraulic_flow_1.layout.dim[5].label = "bucket_cylinder_rod_flow_rate";
      hydraulic_flow_1.layout.dim[6].label = "swing_motor_a_flow_rate";
      hydraulic_flow_1.layout.dim[7].label = "swing_motor_b_flow_rate";

      hydraulic_flow_2.data.resize(4);
      hydraulic_flow_2.layout.dim.resize(4);
      hydraulic_flow_2.layout.dim[0].label = "right_track_motor_a_flow_rate";
      hydraulic_flow_2.layout.dim[1].label = "right_track_motor_b_flow_rate";
      hydraulic_flow_2.layout.dim[2].label = "left_track_motor_a_flow_rate";
      hydraulic_flow_2.layout.dim[3].label = "left_track_motor_b_flow_rate";
      hydraulic_flow_2.data.resize(4);

      joint_state.header.frame_id = "base_link";
      joint_state.name.resize(4);
      joint_state.name[0] = "swing_joint";
      joint_state.name[1] = "boom_joint";
      joint_state.name[2] = "arm_joint";
      joint_state.name[3] = "bucket_joint";
      joint_state.position.resize(4);
      joint_state.velocity.resize(4);
      joint_state.effort.resize(4);

      pose.header.frame_id = "base_link";

      pi_pressure_1.data.resize(8);
      pi_pressure_1.layout.dim.resize(8);
      pi_pressure_1.layout.dim[0].label = "boom_up_pilot_pressure";
      pi_pressure_1.layout.dim[1].label = "boom_down_pilot_pressure";
      pi_pressure_1.layout.dim[2].label = "arm_crowd_pilot_pressure";
      pi_pressure_1.layout.dim[3].label = "arm_dump_pilot_pressure";
      pi_pressure_1.layout.dim[4].label = "bucket_crowd_pilot_pressure";
      pi_pressure_1.layout.dim[5].label = "bucket_dump_pilot_pressure";
      pi_pressure_1.layout.dim[6].label = "swing_right_pilot_pressure";
      pi_pressure_1.layout.dim[7].label = "swing_left_pilot_pressure";

      pi_pressure_2.data.resize(8);
      pi_pressure_2.layout.dim.resize(8);
      pi_pressure_2.layout.dim[0].label = "right_track_forward_pilot_prs";
      pi_pressure_2.layout.dim[1].label = "right_track_backward_pilot_prs";
      pi_pressure_2.layout.dim[2].label = "left_track_forward_pilot_prs";
      pi_pressure_2.layout.dim[3].label = "left_track_backward_pilot_prs";
      pi_pressure_2.layout.dim[4].label = "attachment_a_pilot_pressure";
      pi_pressure_2.layout.dim[5].label = "attachment_b_pilot_pressure";
      pi_pressure_2.layout.dim[6].label = "assist_a_pilot_pressure";
      pi_pressure_2.layout.dim[7].label = "assist_b_pilot_pressure";

      main_pressure_1.data.resize(8);
      main_pressure_1.layout.dim.resize(8);
      main_pressure_1.layout.dim[0].label = "boom_cylinder_bottom_pressure";
      main_pressure_1.layout.dim[1].label = "boom_cylinder_rod_pressure";
      main_pressure_1.layout.dim[2].label = "arm_cylinder_bottom_pressure";
      main_pressure_1.layout.dim[3].label = "arm_cylinder_rod_pressure";
      main_pressure_1.layout.dim[4].label = "bucket_cylinder_bottom_pressure";
      main_pressure_1.layout.dim[5].label = "bucket_cylinder_rod_pressure";
      main_pressure_1.layout.dim[6].label = "swing_motor_a_pressure";
      main_pressure_1.layout.dim[7].label = "swing_motor_b_pressure";

      main_pressure_2.data.resize(8);
      main_pressure_2.layout.dim.resize(8);
      main_pressure_2.layout.dim[0].label = "right_track_motor_a_pressure";
      main_pressure_2.layout.dim[1].label = "right_track_motor_b_pressure";
      main_pressure_2.layout.dim[2].label = "left_track_motor_a_pressure";
      main_pressure_2.layout.dim[3].label = "left_track_motor_b_pressure";
      main_pressure_2.layout.dim[4].label = "attachment_a_pressure";
      main_pressure_2.layout.dim[5].label = "attachment_b_pressure";
      main_pressure_2.layout.dim[6].label = "assist_a_pressure";
      main_pressure_2.layout.dim[7].label = "assit_b_pressure";
    }

    canary::raw::socket sock;
    boost::asio::steady_timer send_pilot_cmd_1_timer, send_pilot_cmd_2_timer, send_front_vel_cmd_timer, send_tracks_vel_cmd_timer, send_machine_setting_cmd_timer;
    frame recv_f;
    boost::shared_ptr<excavator_com3::Pilot_Pressure_Cmd_1> pilot_cmd_1;
    boost::shared_ptr<excavator_com3::Velocity_Cmd_1> front_vel_cmd;
    boost::shared_ptr<excavator_com3::Pilot_Pressure_Cmd_2> pilot_cmd_2;
    boost::shared_ptr<excavator_com3::Velocity_Cmd_2> tracks_vel_cmd;
    boost::shared_ptr<excavator_com3::Machine_Setting_Cmd> setting_cmd;

    excavator_com3::Front_Angle com3_front_ang;
    excavator_com3::Front_Angular_Velocity com3_front_ang_vel;
    excavator_com3::Roll_Pitch_Angle com3_roll_pitch;
    excavator_com3::Swing_Center_Position_1 com3_swing_center_pos_x;
    excavator_com3::Swing_Center_Position_2 com3_swing_center_pos_y;
    excavator_com3::Swing_Center_Position_3 com3_swing_center_pos_z;
    excavator_com3::Vehicle_Azimuth com3_vehicle_azimuth;
    excavator_com3::Pressure_1 com3_main_pressure_1;
    excavator_com3::Pressure_2 com3_main_pressure_2;
    excavator_com3::Pilot_Pressure_1 com3_pi_pressure_1;
    excavator_com3::Pilot_Pressure_2 com3_pi_pressure_2;
    excavator_com3::Hydraulic_Flow_Rate_1 com3_h_flow_1;
    excavator_com3::Hydraulic_Flow_Rate_2 com3_h_flow_2;
    excavator_com3::Machine_State com3_machine_state;

    rclcpp::Subscription<com3_msgs::msg::JointCmd>::SharedPtr sub_front_cmd_;
    rclcpp::Subscription<com3_msgs::msg::JointCmd>::SharedPtr sub_tracks_cmd_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_cmd_;
    rclcpp::Subscription<com3_msgs::msg::ExcavatorCom3MachineSetting>::SharedPtr sub_machine_setting_cmd_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_emg_stop_cmd_;
    bool is_emg_stop_enable;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state;
    sensor_msgs::msg::JointState joint_state;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
    geometry_msgs::msg::PoseStamped pose;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_main_pressure_1;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_main_pressure_2;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pi_pressure_1;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pi_pressure_2;
    std_msgs::msg::Float64MultiArray pi_pressure_1, pi_pressure_2, main_pressure_1, main_pressure_2;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_hydraulic_flow_1;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_hydraulic_flow_2;
    std_msgs::msg::Int32MultiArray hydraulic_flow_1, hydraulic_flow_2;
    rclcpp::Publisher<com3_msgs::msg::ExcavatorCom3MachineState>::SharedPtr pub_machine_state;
    com3_msgs::msg::ExcavatorCom3MachineState machine_state;
    tf2::Quaternion quat_tf;

    uint8_t is_pose_set, is_js_set;
    const std::vector<double> dead_zone;
    rclcpp::Time last_front_cmd_time, last_tracks_cmd_time, last_twist_cmd_time, last_machine_setting_cmd_time;
  };
}
