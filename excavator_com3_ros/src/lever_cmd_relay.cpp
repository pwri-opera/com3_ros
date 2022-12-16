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
  std::string com3_can_port;
  std::string dbc_path;
  // created a node that gets parameters outside machine_setting_cmd_relay class
  {
    auto param_node = rclcpp::Node::make_shared("get_parameters");
    param_node->declare_parameter("can_port", "can");
    param_node->declare_parameter("dbc_path", "excavator_com3.dbc");

    com3_can_port = param_node->get_parameter("can_port").get_parameter_value().get<std::string>();
    dbc_path = param_node->get_parameter("dbc_path").get_parameter_value().get<std::string>();
  }

  auto node_ = std::make_shared<excavator_com3_can::lever_cmd_relay>(ioc, com3_can_port, dbc_path);

  boost::thread t(boost::bind(&boost::asio::io_context::run, &ioc));

  rclcpp::spin(node_);

  rclcpp::shutdown();
  return 0;
}