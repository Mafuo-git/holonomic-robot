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
  #include "std_msgs/msg/bool.hpp"

  class ShutdownNode : public rclcpp::Node
  {
  public:
    ShutdownNode()
    : Node("shutdown_manager")
    {
      // subscriber cmd_vel
      subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/shutdown_nuc", 10,
        std::bind(&ShutdownNode::ShutdownCallback, this, std::placeholders::_1)
      );
    }

  private:
    void ShutdownCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Shutdown signal received. Shutting down...");
            system("/bin/bash -c ~/shutdown.sh");
        }
    }

    void publishMotor(int index, double velocity)
    {
      std_msgs::msg::Float64 msg;
      double max_wheel_speed = 20.0; // rad/s correspondant à duty = 1.0

      double duty = velocity / max_wheel_speed;

      // saturation
      if (duty > 1.0) duty = 1.0;
      if (duty < -1.0) duty = -1.0;
      msg.data = duty;

      motor_pub_[index]->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Moving motor %d at %.2f", index, velocity);
    }

    // ROS
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_pub_[4];

    // paramètres robot
    double R_, Lx_, Ly_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelSubscriber>());
    rclcpp::shutdown();
    return 0;
  }
