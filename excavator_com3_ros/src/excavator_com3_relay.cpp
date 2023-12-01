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
#include "excavator_com3_relay.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  canary::net::io_context ioc;
  std::string com3_can_port;
  std::string dbc_path;
  std::vector<double> dead_zone;

  // created a node that gets parameters outside machine_setting_cmd_relay class
  {
    auto param_node = rclcpp::Node::make_shared("get_parameters");
    param_node->declare_parameter("can_port", "can");
    param_node->declare_parameter("dbc_path", "excavator_com3.dbc");
    param_node->declare_parameter("dead_zone", std::vector<double>(0.0));

    com3_can_port = param_node->get_parameter("can_port").get_parameter_value().get<std::string>();
    dbc_path = param_node->get_parameter("dbc_path").get_parameter_value().get<std::string>();
    dead_zone = param_node->get_parameter("dead_zone").get_parameter_value().get<std::vector<double>>();
  }

  auto node_ = std::make_shared<excavator_com3_can::gateway>(ioc, com3_can_port, dbc_path, dead_zone);

  boost::thread t(boost::bind(&boost::asio::io_context::run, &ioc));

  rclcpp::spin(node_);

  rclcpp::shutdown();
  return 0;
}