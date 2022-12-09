#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <canary/frame_header.hpp>
#include <canary/interface_index.hpp>
#include <canary/raw.hpp>
#include <iostream>
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/make_shared.hpp>

#include <com3/excavator_com3_dbc.hpp>
#include "lever_cmd_relay.hpp"

using namespace std::chrono_literals;

// spin()だけlever_cmd_ralayのコンストラクタに存在するのは気持ち悪いため要修正
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  canary::net::io_context ioc;
  // TODO: com3_can_port, dbc_path の ros parameter 化
  std::string com3_can_port = "vcan0";
  std::string dbc_path = "/usr/local/share/dbc/excavator_com3.dbc";
  excavator_com3_can::lever_cmd_relay com3_can(ioc, com3_can_port, dbc_path); // ここでspin()
  boost::thread t(boost::bind(&boost::asio::io_context::run, &ioc));

  rclcpp::shutdown();
  return 0;
}