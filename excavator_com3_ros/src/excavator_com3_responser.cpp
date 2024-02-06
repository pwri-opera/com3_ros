// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "com3_msgs/msg/joint_cmd.hpp"
#include "com3_msgs/msg/excavator_com3_machine_setting.hpp"
#include "com3_msgs/msg/excavator_com3_machine_state.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

class ExcavatorCmdSubscriber : public rclcpp::Node
{
public:
  ExcavatorCmdSubscriber()
  : Node("excavator_com3_responser")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "excavator/cmd", 10, std::bind(&ExcavatorCmdSubscriber::topic_callback, this, _1));
    machine_state_publisher_ = this->create_publisher<com3_msgs::msg::ExcavatorCom3MachineState>("excavator/machine_state", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&ExcavatorCmdSubscriber::timer_callback, this));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  void timer_callback()
  {
    auto machine_state_msg = com3_msgs::msg::ExcavatorCom3MachineState();

    machine_state_publisher_->publish(machine_state_msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<com3_msgs::msg::ExcavatorCom3MachineState>::SharedPtr machine_state_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExcavatorCmdSubscriber>());
  rclcpp::shutdown();
  return 0;
}
