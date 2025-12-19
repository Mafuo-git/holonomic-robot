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

  class CmdVelSubscriber : public rclcpp::Node
  {
  public:
    CmdVelSubscriber()
    : Node("cmd_vel_to_mecanum")
    {
      // paramètres robot
      R_  = this->declare_parameter("wheel_radius", 0.05);
      Lx_ = this->declare_parameter("lx", 0.20);
      Ly_ = this->declare_parameter("ly", 0.15);

      // publishers moteurs
      motor_pub_[0] = this->create_publisher<std_msgs::msg::Float64>(
    "/phidget_487541/set_motor_duty_cycle00", 10);

  motor_pub_[1] = this->create_publisher<std_msgs::msg::Float64>(
    "/phidget_487541/set_motor_duty_cycle01", 10);

  motor_pub_[2] = this->create_publisher<std_msgs::msg::Float64>(
    "/phidget_487736/set_motor_duty_cycle02", 10);

  motor_pub_[3] = this->create_publisher<std_msgs::msg::Float64>(
    "/phidget_487736/set_motor_duty_cycle03", 10);

      // subscriber cmd_vel
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&CmdVelSubscriber::cmdVelCallback, this, std::placeholders::_1)
      );
    }

  private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      const double vx = msg->linear.x;
      const double vy = msg->linear.y;
      const double w  = msg->angular.z;

      const double k = (Lx_ + Ly_);

      double w_fl = (1.0 / R_) * ( vx - vy - k * w );
      double w_fr = (1.0 / R_) * ( vx + vy + k * w );
      double w_rl = (1.0 / R_) * ( vx + vy - k * w );
      double w_rr = (1.0 / R_) * ( vx - vy + k * w );

      publishMotor(0, w_fl);
      publishMotor(1, w_fr);
      publishMotor(2, w_rl);
      publishMotor(3, w_rr);
      RCLCPP_INFO(this->get_logger(), "");
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
