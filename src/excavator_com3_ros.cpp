#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

// #include <stdint.h>
#include<canary/frame_header.hpp>
#include <canary/interface_index.hpp>
#include <canary/raw.hpp>
// #include <canary/socket_options.hpp>
#include <iostream>
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/make_shared.hpp>
// #include <ncurses.h>

#include <com3/excavator_com3_dbc.hpp>
#include "excavator_com3_ros.hpp"

int main(int argc, char**argv){
  ros::init(argc, argv, "excavator_com3_ros");
  // ros::NodeHandle nh;

  std::string com3_can_port="vcan0";
  std::string dbc_path = "/usr/local/share/dbc/excavator_com3.dbc";
  // int channel;

  // if (nh.getParam("hitachi_ros/port_name", port_name))
  // {
  //   ROS_INFO("GET port name");
  //   cmd_handler.port_open(port_name);
  // }

  // if (nh.getParam("hitachi_ros/channel", channel))
  // {
  //   ROS_INFO("GET channel name");
  //   cmd_handler.set_channel(channel);
  // }

  // cmd_handler.set_engine_speed(Engine_speed::_30per30);
  // cmd_handler.set_engine_status(Engine_status::Engine_Drive);
  // cmd_handler.set_travel_lock(true);

  canary::net::io_context ioc;
  excavator_com3_can com3_can(ioc, com3_can_port, dbc_path);
  boost::thread t(boost::bind(&boost::asio::io_context::run, &ioc));

  ros::spin();
  return 0;
}
