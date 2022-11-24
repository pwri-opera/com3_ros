#pragma once
#include <iostream>
#include <ros/ros.h>
#include <com3/excavator_com3_dbc.hpp>
#include <com3_msgs/excavator_com3_lever_1.h>
#include <com3_msgs/excavator_com3_lever_2.h>

#define initial_interval 100
namespace excavator_com3_can
{
  class lever_cmd_relay : public excavator_com3_dbc
  {
  public:
    lever_cmd_relay(boost::asio::io_context &io, std::string can_port, std::string dbc_path)
        : excavator_com3_dbc(dbc_path),
          send_timer(io, boost::asio::chrono::milliseconds(initial_interval)),
          send_timer1(io, boost::asio::chrono::milliseconds(initial_interval)),
          sock(io)
    {
      const auto idx = canary::get_interface_index(can_port);
      auto const ep = canary::raw::endpoint{idx};
      sock.open();
      sock.bind(ep);

      lever_cmd_1 = boost::make_shared<excavator_com3::lever_cmd_1>();
      lever_cmd_2 = boost::make_shared<excavator_com3::lever_cmd_2>();

      alive_cnt = 0;
      start_receive();
      send_timer.async_wait(boost::bind(&lever_cmd_relay::send_lever_cmd_1, this));
      send_timer1.async_wait(boost::bind(&lever_cmd_relay::send_lever_cmd_2, this));

      nh = ros::NodeHandle("~");
      sub_js_cmd = nh.subscribe("lever1", 10, &lever_cmd_relay::lever1_cmd_callback, this);
      sub_travel_cmd = nh.subscribe("lever2", 10, &lever_cmd_relay::lever2_cmd_callback, this);
    };
    ~lever_cmd_relay()
    {
      send_timer.cancel();
      send_timer1.cancel();
    }

  private:
    void send_lever_cmd_1()
    {
      frame f = {};
      excavator_com3_dbc::encode(*lever_cmd_1, f);

      sock.async_send(canary::net::buffer(&f, sizeof(f)), boost::bind(&lever_cmd_relay::send_handle, this));
      send_timer.expires_at(send_timer.expiry() + boost::asio::chrono::milliseconds(lever_cmd_1->Cycle_time()));
      send_timer.async_wait(boost::bind(&lever_cmd_relay::send_lever_cmd_1, this));
    }

    void send_lever_cmd_2()
    {
      frame f = {};
      excavator_com3_dbc::encode(*lever_cmd_2, f);

      sock.async_send(canary::net::buffer(&f, sizeof(f)), boost::bind(&lever_cmd_relay::send_handle, this));
      send_timer1.expires_at(send_timer1.expiry() + boost::asio::chrono::milliseconds(lever_cmd_2->Cycle_time()));
      send_timer1.async_wait(boost::bind(&lever_cmd_relay::send_lever_cmd_2, this));
    }

    void send_handle() {}

    void start_receive()
    {
      sock.async_receive(canary::net::buffer(&recv_f, sizeof(recv_f)), boost::bind(&lever_cmd_relay::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

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

    void lever1_cmd_callback(const com3_msgs::excavator_com3_lever_1::ConstPtr &msg)
    {
      lever_cmd_1->swing_cmd = std::abs(msg->swing_cmd);
      lever_cmd_1->swing_status_cmd = 0;
      lever_cmd_1->swing_direction_cmd = std::signbit(msg->swing_cmd);

      lever_cmd_1->boom_cmd = std::abs(msg->boom_cmd);
      lever_cmd_1->boom_status_cmd = 0;
      lever_cmd_1->boom_direction_cmd = std::signbit(msg->boom_cmd);

      lever_cmd_1->arm_cmd = std::abs(msg->arm_cmd);
      lever_cmd_1->arm_status_cmd = 0;
      lever_cmd_1->arm_direction_cmd = std::signbit(msg->arm_cmd);

      lever_cmd_1->bucket_cmd = std::abs(msg->bucket_cmd);
      lever_cmd_1->bucket_status_cmd = 0;
      lever_cmd_1->bucket_direction_cmd = std::signbit(msg->bucket_cmd);

      last_cmd_time = ros::Time::now();
    }
    void lever2_cmd_callback(const com3_msgs::excavator_com3_lever_2::ConstPtr &msg)
    {
      lever_cmd_2->left_track_cmd = std::abs(msg->left_track_cmd);
      lever_cmd_2->left_track_status_cmd = 0;
      lever_cmd_2->left_track_direction_cmd = std::signbit(msg->left_track_cmd);
      lever_cmd_2->right_track_cmd = std::abs(msg->right_track_cmd);
      lever_cmd_2->right_track_status_cmd = 0;
      lever_cmd_2->right_track_direction_cmd = std::signbit(msg->right_track_cmd);

      last_cmd_time = ros::Time::now();
    }

    boost::asio::steady_timer send_timer, send_timer1;
    canary::raw::socket sock;
    frame recv_f;
    boost::shared_ptr<excavator_com3::lever_cmd_1> lever_cmd_1;
    boost::shared_ptr<excavator_com3::lever_cmd_2> lever_cmd_2;

    ros::NodeHandle nh;
    ros::Subscriber sub_js_cmd, sub_travel_cmd;
    ros::Time last_cmd_time;

    std::uint8_t alive_cnt;
  };
}