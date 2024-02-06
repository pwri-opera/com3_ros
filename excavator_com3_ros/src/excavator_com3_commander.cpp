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
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ExcavatorCmdPublisher : public rclcpp::Node
{
public:
  ExcavatorCmdPublisher()
  : Node("excavator_com3_commander"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("excavator/cmd", 10);
    joint_cmd_publisher_ = this->create_publisher<com3_msgs::msg::JointCmd>("excavator/joint_cmd", 10);
    machine_setting_publisher_ = this->create_publisher<com3_msgs::msg::ExcavatorCom3MachineSetting>("excavator/machine_setting", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&ExcavatorCmdPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    auto joint_cmd_msg = com3_msgs::msg::JointCmd();
    auto machine_setting_msg = com3_msgs::msg::ExcavatorCom3MachineSetting();
    message.data = "Hello, world! " + std::to_string(count_++);
    joint_cmd_msg.joint_name = {"swing_joint", "boom_joint", "arm_joint", "bucket_joint"};
    joint_cmd_msg.position = {0.0, -0.5, 1.0, 0.5};
    //joint_cmd_msg.velocity = {0.0, 0.0, 0.0, 0.0};
    //joint_cmd_msg.effort = {0.0, 0.0, 0.0, 0.0};

    machine_setting_msg.engine_rpm = 1000.0;
    machine_setting_msg.power_eco_mode = true;
    machine_setting_msg.travel_speed_mode = false;
    machine_setting_msg.working_mode_notice = true;
    machine_setting_msg.yellow_led_mode = 0;
    machine_setting_msg.horn = false;
    machine_setting_msg.front_control_mode = 0;
    machine_setting_msg.tracks_control_mode = 3;

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    joint_cmd_publisher_->publish(joint_cmd_msg);
    machine_setting_publisher_->publish(machine_setting_msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<com3_msgs::msg::JointCmd>::SharedPtr joint_cmd_publisher_;
  rclcpp::Publisher<com3_msgs::msg::ExcavatorCom3MachineSetting>::SharedPtr machine_setting_publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExcavatorCmdPublisher>());
  rclcpp::shutdown();
  return 0;
}
