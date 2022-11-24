#pragma once
#include <iostream>
#include <ros/ros.h>
#include <com3/excavator_com3_dbc.hpp>

#define initial_interval 100

class excavator_com3_can : public excavator_com3_dbc
{
public:
  excavator_com3_can(boost::asio::io_context &io, std::string can_port, std::string dbc_path)
      : excavator_com3_dbc(dbc_path),
        send_timer(io, boost::asio::chrono::milliseconds(initial_interval)),
        send_timer1(io, boost::asio::chrono::milliseconds(initial_interval)),
        // send_timer2(io, boost::asio::chrono::milliseconds(initial_interval)),
        sock(io)
  {
    const auto idx = canary::get_interface_index(can_port);
    auto const ep = canary::raw::endpoint{idx};
    sock.open();
    sock.bind(ep);

    lever_cmd_1 = boost::shared_ptr<excavator_com3::lever_cmd_1>(new excavator_com3::lever_cmd_1{});
    lever_cmd_2 = boost::shared_ptr<excavator_com3::lever_cmd_2>(new excavator_com3::lever_cmd_2{});
    // setting_cmd = boost::shared_ptr<zx200::Machine_Setting_Cmd>(new zx200::Machine_Setting_Cmd{});

    alive_cnt = 0;
    start_receive();
    send_timer.async_wait(boost::bind(&excavator_com3_can::send_lever_cmd_1, this));
    send_timer1.async_wait(boost::bind(&excavator_com3_can::send_lever_cmd_2, this));
    // send_timer2.async_wait(boost::bind(&excavator_com3_can::send_machine_setting_cmd, this));

    nh = ros::NodeHandle("~");
    sub_js_cmd = nh.subscribe("js_cmd", 10, &excavator_com3_can::joint_states_cmd_callback, this);
    // ros::Subscriber sub_engine_start_cmd = nh.subscribe("engine_start", 10, engine_start_cmd_callback);
    // sub_travel_cmd = nh.subscribe("travel", 10, &excavator_com3_can::travel_cmd_callback, this);
  };
  ~excavator_com3_can()
  {
    send_timer.cancel();
    send_timer1.cancel();
    // send_timer2.cancel();
  }

private:
  void send_lever_cmd_1()
  {
    frame f;
    excavator_com3_dbc::encode(*lever_cmd_1, f);

    sock.async_send(canary::net::buffer(&f, sizeof(f)), boost::bind(&excavator_com3_can::send_handle, this));
    send_timer.expires_at(send_timer.expiry() + boost::asio::chrono::milliseconds(lever_cmd_1->Cycle_time()));
    send_timer.async_wait(boost::bind(&excavator_com3_can::send_lever_cmd_1, this));
  }

  void send_lever_cmd_2()
  {
    frame f;
    excavator_com3_dbc::encode(*lever_cmd_2, f);

    sock.async_send(canary::net::buffer(&f, sizeof(f)), boost::bind(&excavator_com3_can::send_handle, this));
    send_timer1.expires_at(send_timer1.expiry() + boost::asio::chrono::milliseconds(lever_cmd_2->Cycle_time()));
    send_timer1.async_wait(boost::bind(&excavator_com3_can::send_lever_cmd_2, this));
  }

  void send_handle() {}

  void start_receive()
  {
    sock.async_receive(canary::net::buffer(&recv_f, sizeof(recv_f)), boost::bind(&excavator_com3_can::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

    if (recv_f.header.payload_length())
    {
      excavator_com3_dbc::decode(recv_f);
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

  void travel_cmd_callback(const sensor_msgs::JointState::ConstPtr &msg)
  {

    last_cmd_time = ros::Time::now();
  }
  void joint_states_cmd_callback(const sensor_msgs::JointState::ConstPtr &msg)
  {
    // msg->name[0] = "swing_joint";
    // msg->name[1] = "boom_joint";
    // msg->name[2] = "arm_joint";
    // msg->name[3] = "backet_joint";
    lever_cmd_1->swing_cmd = std::abs(msg->effort[0]);
    lever_cmd_1->swing_status_cmd = 0;
    lever_cmd_1->swing_direction_cmd = std::signbit(msg->effort[0]);

    lever_cmd_1->boom_cmd = std::abs(msg->effort[1]);
    lever_cmd_1->boom_status_cmd = 0;
    lever_cmd_1->boom_direction_cmd = std::signbit(msg->effort[1]);

    lever_cmd_1->arm_cmd = std::abs(msg->effort[2]);
    lever_cmd_1->arm_status_cmd = 0;
    lever_cmd_1->arm_direction_cmd = std::signbit(msg->effort[2]);

    lever_cmd_1->bucket_cmd = std::abs(msg->effort[3]);
    lever_cmd_1->bucket_status_cmd = 0;
    lever_cmd_1->bucket_direction_cmd = std::signbit(msg->effort[3]);

    last_cmd_time = ros::Time::now();
  }

  boost::asio::steady_timer send_timer, send_timer1;//, send_timer2;
  canary::raw::socket sock;
  frame recv_f;
  boost::shared_ptr<excavator_com3::lever_cmd_1> lever_cmd_1;
  boost::shared_ptr<excavator_com3::lever_cmd_2> lever_cmd_2;
  // boost::shared_ptr<zx200::Machine_Setting_Cmd> setting_cmd;

  ros::NodeHandle nh;
  ros::Subscriber sub_js_cmd, sub_travel_cmd;
  ros::Time last_cmd_time;

  std::uint8_t alive_cnt;
};