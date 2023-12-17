#pragma once
#include <iostream>
#include <ros/ros.h>
#include <com3_msgs/excavator_com3_machine_setting.h>

#include <com3/excavator_com3_dbc.hpp>

#define initial_interval 100
#define cmd_timeout 500

namespace excavator_com3_can
{
  class machine_setting_cmd_relay : public excavator_com3_dbc
  {
  public:
    machine_setting_cmd_relay(boost::asio::io_context &io, std::string can_port, std::string dbc_path)
        : excavator_com3_dbc(dbc_path),
          send_timer(io, boost::asio::chrono::milliseconds(initial_interval)),
          sock(io)
    {
      const auto idx = canary::get_interface_index(can_port);
      auto const ep = canary::raw::endpoint{idx};
      sock.open();
      sock.bind(ep);

      setting_cmd = boost::make_shared<excavator_com3::machine_setting_cmd>();

      alive_cnt = 0;
      start_receive();
      send_timer.async_wait(boost::bind(&machine_setting_cmd_relay::send_machine_setting_cmd, this));

      nh = ros::NodeHandle("~");
      sub_js_cmd = nh.subscribe("machine_setting", 10, &machine_setting_cmd_relay::machine_setting_cmd_callback, this);
    }

    ~machine_setting_cmd_relay()
    {
      send_timer.cancel();
    }

  private:
    void send_machine_setting_cmd()
    {
      ros::Duration elapsed_last_cmd = ros::Time::now() - last_cmd_recv_time;
      if (elapsed_last_cmd.toSec() * 1000. > cmd_timeout)
      {

      }

      frame f = {};
      setting_cmd->alive_counter++;

      excavator_com3_dbc::encode(*setting_cmd, f);

      sock.async_send(canary::net::buffer(&f, sizeof(f)), boost::bind(&machine_setting_cmd_relay::send_handle, this));
      send_timer.expires_at(send_timer.expiry() + boost::asio::chrono::milliseconds(setting_cmd->Cycle_time()));
      send_timer.async_wait(boost::bind(&machine_setting_cmd_relay::send_machine_setting_cmd, this));
    }

    void send_handle() {}

    void start_receive()
    {
      sock.async_receive(canary::net::buffer(&recv_f, sizeof(recv_f)), boost::bind(&machine_setting_cmd_relay::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

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

    void machine_setting_cmd_callback(const com3_msgs::excavator_com3_machine_setting::ConstPtr &msg)
    {
      last_cmd_recv_time = ros::Time::now();

      setting_cmd->engine_rpm = msg->engine_rpm;
      setting_cmd->engine_onoff = msg->engine_off;
      setting_cmd->hydraulic_onoff = msg->hydraulic_off;
      setting_cmd->power_eco_mode = msg->power_eco_mode;
      setting_cmd->travel_mode = msg->travel_mode;
    }

    boost::asio::steady_timer send_timer;
    canary::raw::socket sock;
    frame recv_f;
    boost::shared_ptr<excavator_com3::machine_setting_cmd> setting_cmd;

    ros::NodeHandle nh;
    ros::Subscriber sub_js_cmd, sub_travel_cmd;
    ros::Time last_cmd_recv_time;

    std::uint8_t alive_cnt;
  };
}