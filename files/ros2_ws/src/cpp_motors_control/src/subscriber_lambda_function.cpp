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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
//#include "phidgets_drivers/msg/cmd_motor_velocity.hpp"


class CmdVelSubscriber : public rclcpp::Node
{
public:
  CmdVelSubscriber()
  : Node("cmd_vel_subscriber")
  {
    auto topic_callback =
      [this](geometry_msgs::msg::Twist::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: \n"
        "linear : \n"
        "x = '%.2f';\n"
        "y = '%.2f'; \n"
        "z = '%.2f' \n"
        "angular: \n"
        "x = '%.2f'; \n"
        "y = '%.2f'; \n"
        "z = '%.2f'",
        msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
      };
    subscription_ =
      this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, topic_callback);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};


/*
class CmdMotorPublisher : public rclcpp::Node
{
public:
  CmdMotorPublisher()
  : Node("cmd_motor_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::float64>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::float64();
        message.data = (this->speed);
        RCLCPP_INFO(this->get_logge r(), "Publishing: '%.2f'", message.data);
        this->publisher_->publish(message);
      };
  }

private:
  rclcpp::Publisher<std_msgs::msg::float64>::SharedPtr publisher_;
};*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSubscriber>());
  rclcpp::shutdown();
  return 0;
}
