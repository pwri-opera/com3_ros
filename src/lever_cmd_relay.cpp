#include <ros/ros.h>

#include<canary/frame_header.hpp>
#include <canary/interface_index.hpp>
#include <canary/raw.hpp>
#include <iostream>
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/make_shared.hpp>

#include <com3/excavator_com3_dbc.hpp>
#include "lever_cmd_relay.hpp"

int main(int argc, char**argv){
  ros::init(argc, argv, "excavator_com3_ros");

  std::string com3_can_port;
  std::string dbc_path;

  ros::param::param<std::string>("can_port", com3_can_port, "vcan0");
  ros::param::param<std::string>("dbc_path", dbc_path, "/usr/local/share/dbc/excavator_com3.dbc");

  canary::net::io_context ioc;
  excavator_com3_can::lever_cmd_relay com3_can(ioc, com3_can_port, dbc_path);
  boost::thread t(boost::bind(&boost::asio::io_context::run, &ioc));

  ros::spin();
  return 0;
}
