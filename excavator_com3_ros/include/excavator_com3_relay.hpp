#pragma once

#include <iostream>
#include <memory>
#include <boost/make_shared.hpp>
#include <boost/bind/bind.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <com3_msgs/msg/joint_cmd.hpp>
#include <com3_msgs/msg/excavator_com3_machine_setting.hpp>
#include <com3_msgs/msg/excavator_com3_machine_state.hpp>
#include <com3/excavator_com3_dbc.hpp>

#define initial_interval 100 //[ms]
#define cmd_timeout 500      //[ms]

namespace excavator_com3_can
{
  class gateway : public excavator_com3_dbc, public rclcpp::Node
  {
  public:
    gateway(boost::asio::io_context &io, std::string can_port, std::string dbc_path)
        : excavator_com3_dbc(dbc_path),
          rclcpp::Node("gateway"),
          sock(io),
          send_pilot_cmd_1_timer(io, boost::asio::chrono::milliseconds(initial_interval)),
          send_pilot_cmd_2_timer(io, boost::asio::chrono::milliseconds(initial_interval)),
          send_front_vel_cmd_timer(io, boost::asio::chrono::milliseconds(initial_interval)),
          send_tracks_vel_cmd_timer(io, boost::asio::chrono::milliseconds(initial_interval)),
          send_machine_setting_cmd_timer(io, boost::asio::chrono::milliseconds(initial_interval))
    {
      this->declare_parameter("can_port", "can");
      this->declare_parameter("dbc_path", "excavator_com3.dbc");

      const auto idx = canary::get_interface_index(can_port);
      auto const ep = canary::raw::endpoint{idx};
      sock.open();
      sock.bind(ep);

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
      pub_machine_state = this->create_publisher<com3_msgs::msg::ExcavatorCom3MachineState>("machine_state",10);
      pub_hydraulic_flow_1 = this->create_publisher<std_msgs::msg::Int32MultiArray>("hydraulics_flow_front", 10);
      pub_hydraulic_flow_2 = this->create_publisher<std_msgs::msg::Int32MultiArray>("hydraulics_flow_tracks", 10);

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
      if (elapsed_time.seconds() * 1000. > cmd_timeout)
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
      if (elapsed_time.seconds() * 1000. > cmd_timeout)
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
      if (elapsed_time.seconds() * 1000. > cmd_timeout)
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
      if (elapsed_time.seconds() * 1000. > cmd_timeout)
      {
        tracks_vel_cmd->target_travel_center_velocity = 0;
        tracks_vel_cmd->target_travel_yaw_rate = 0;
      }
      elapsed_time = this->get_clock()->now() - last_tracks_cmd_time;
      if (elapsed_time.seconds() * 1000. > cmd_timeout)
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
          get_can_bus_msg(m_state);
          machine_state.lock_cmd_state = m_state.lock_cmd_state;
          machine_state.pilot_shutoff_valve_state = m_state.pilot_shutoff_valve_state;
          machine_state.system_error = m_state.system_error;
          machine_state.can_error_pl = m_state.can_error_pl;
          machine_state.can_error_ict = m_state.can_error_ict;
          machine_state.can_error_body = m_state.can_error_body;
          machine_state.lock_receiver_error = m_state.lock_receiver_error;
          machine_state.emergency_stop_receiver_error = m_state.emergency_stop_receiver_error;
          machine_state.switch_error = m_state.switch_error;
          machine_state.control_state = m_state.control_state;
          machine_state.hydraulic_oil_temp = machine_state.hydraulic_oil_temp;
          machine_state.engine_state = machine_state.engine_state;
          machine_state.alive_counter = m_state.alive_counter;
          pub_machine_state->publish(machine_state);
          break;
        case excavator_com3::Hydraulic_Flow_Rate_2::id:
          // std_msgs::msg::Int32MultiArray a;
          get_can_bus_msg(h_flow_2);
          hydraulic_flow_2.data[0] = h_flow_2.right_track_motor_a_flow_rate;
          hydraulic_flow_2.data[1] = h_flow_2.right_track_motor_b_flow_rate;
          hydraulic_flow_2.data[2] = h_flow_2.left_track_motor_a_flow_rate;
          hydraulic_flow_2.data[3] = h_flow_2.left_track_motor_b_flow_rate;
          pub_hydraulic_flow_2->publish(hydraulic_flow_2);
          break;
        case excavator_com3::Hydraulic_Flow_Rate_1::id:
          get_can_bus_msg(h_flow_1);
          hydraulic_flow_1.data[0] = h_flow_1.boom_cylinder_bottom_flow_rate;
          hydraulic_flow_1.data[1] = h_flow_1.boom_cylinder_rod_flow_rate;
          hydraulic_flow_1.data[2] = h_flow_1.arm_cylinder_bottom_flow_rate;
          hydraulic_flow_1.data[3] = h_flow_1.arm_cylinder_rod_flow_rate;
          hydraulic_flow_1.data[4] = h_flow_1.bucket_cylinder_bottom_flow_rate;
          hydraulic_flow_1.data[5] = h_flow_1.bucket_cylinder_rod_flow_rate;
          hydraulic_flow_1.data[6] = h_flow_1.swing_motor_a_flow_rate;
          hydraulic_flow_1.data[7] = h_flow_1.swing_motor_b_flow_rate;
          pub_hydraulic_flow_1->publish(hydraulic_flow_1);
          break;
        case excavator_com3::Pilot_Pressure_2::id:
          break;
        case excavator_com3::Pilot_Pressure_1::id:
          break;
        case excavator_com3::Pressure_2::id:
          break;
        case excavator_com3::Pressure_1::id:
          break;
        case excavator_com3::Vehicle_Azimuth::id:
          break;
        case excavator_com3::Swing_Center_Position_3::id:
          break;
        case excavator_com3::Swing_Center_Position_2::id:
          break;
        case excavator_com3::Swing_Center_Position_1::id:
          break;
        case excavator_com3::Front_Pin_Position_3::id:
          break;
        case excavator_com3::Front_Pin_Position_2::id:
          break;
        case excavator_com3::Front_Pin_Position_1::id:
          break;
        case excavator_com3::Roll_Pitch_Angle::id:
          break;
        case excavator_com3::Front_Angular_Velocity::id:
          break;
        case excavator_com3::Front_Angle::id:
          break;
        default:

          break;
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
          if (msg->joint_name[i] == "swing")
          {
            effort2pilot_pressure(msg->effort[i], pilot_cmd_1->swing_left, pilot_cmd_1->swing_right);
            front_vel_cmd->swing_target_anguler_velocity = msg->velocity[i];
          }
          else if (msg->joint_name[i] == "boom")
          {
            effort2pilot_pressure(msg->effort[i], pilot_cmd_1->boom_down, pilot_cmd_1->boom_up);
            front_vel_cmd->boom_target_anguler_velocity = msg->velocity[i];
          }
          else if (msg->joint_name[i] == "arm")
          {
            effort2pilot_pressure(msg->effort[i], pilot_cmd_1->arm_crowd, pilot_cmd_1->arm_dump);
            front_vel_cmd->arm_target_anguler_velocity = msg->velocity[i];
          }
          else if (msg->joint_name[i] == "bucket")
          {
            effort2pilot_pressure(msg->effort[i], pilot_cmd_1->bucket_crowd, pilot_cmd_1->bucket_dump);
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
            effort2pilot_pressure(msg->effort[i], pilot_cmd_2->left_track_forward, pilot_cmd_2->left_track_backward);
            tracks_vel_cmd->target_travel_left_velocity = msg->velocity[i];
          }
          else if (msg->joint_name[i] == "right_track")
          {
            effort2pilot_pressure(msg->effort[i], pilot_cmd_2->right_track_forward, pilot_cmd_2->right_track_backward);
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

    void effort2pilot_pressure(double effort, double &out_plus, double &out_minus)
    {
      if (std::abs(effort) > 100.)
        effort = 100. * std::signbit(effort);

      // ZX200では最大パイロット圧は5MPaなので以下の通りeffort[％]をout[MPa]に変換する。
      // 汎用的にこのノードを利用するためには係数を取得してinputに乗算するか、
      // ここでは物理量を扱わず、よりハードウェアに近い下位のソフトウェアで係数をかける必要がある。
      if (effort > 0.)
      {
        out_plus = effort / 100. * 5;
        out_minus = 0;
      }
      else
      {
        out_plus = 0;
        out_minus = effort / 100. * 5;
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
    }

    canary::raw::socket sock;
    boost::asio::steady_timer send_pilot_cmd_1_timer, send_pilot_cmd_2_timer, send_front_vel_cmd_timer, send_tracks_vel_cmd_timer, send_machine_setting_cmd_timer;
    frame recv_f;
    boost::shared_ptr<excavator_com3::Pilot_Pressure_Cmd_1> pilot_cmd_1;
    boost::shared_ptr<excavator_com3::Velocity_Cmd_1> front_vel_cmd;
    boost::shared_ptr<excavator_com3::Pilot_Pressure_Cmd_2> pilot_cmd_2;
    boost::shared_ptr<excavator_com3::Velocity_Cmd_2> tracks_vel_cmd;
    boost::shared_ptr<excavator_com3::Machine_Setting_Cmd> setting_cmd;
    excavator_com3::Machine_State m_state;
    excavator_com3::Hydraulic_Flow_Rate_1 h_flow_1;
    excavator_com3::Hydraulic_Flow_Rate_2 h_flow_2;

    rclcpp::Subscription<com3_msgs::msg::JointCmd>::SharedPtr sub_front_cmd_;
    rclcpp::Subscription<com3_msgs::msg::JointCmd>::SharedPtr sub_tracks_cmd_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_cmd_;
    rclcpp::Subscription<com3_msgs::msg::ExcavatorCom3MachineSetting>::SharedPtr sub_machine_setting_cmd_;


    rclcpp::Publisher<com3_msgs::msg::ExcavatorCom3MachineState>::SharedPtr pub_machine_state;
    com3_msgs::msg::ExcavatorCom3MachineState machine_state;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_hydraulic_flow_1;
    std_msgs::msg::Int32MultiArray hydraulic_flow_1;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_hydraulic_flow_2;
    std_msgs::msg::Int32MultiArray hydraulic_flow_2;

    rclcpp::Time last_front_cmd_time, last_tracks_cmd_time, last_twist_cmd_time, last_machine_setting_cmd_time;
  };
}
