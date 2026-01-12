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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>



#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop()
  : Node("keyboard_teleop")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), 
    "Keyboard teleoperation started\n"
    "Use the following keys to move (linear movement control):\n"
    "A Z E\n"
    "Q S D\n"
    "W X\n"
    "Use the following keys to rotate (angular movement control):\n"
    "G H\n"
    "Press Space to stop all movement\n"
    "Use U and J to increase/decrease linear speed\n"
    "Use I and K to increase/decrease angular speed\n"
    "Press O to quit\n");

    tcgetattr(STDIN_FILENO, &old_tio_);
    new_tio_ = old_tio_;
    new_tio_.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&KeyboardTeleop::keyLoop, this));
  }

  ~KeyboardTeleop()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
  }

private:
    float linear_speed_ = 0.5;
    float angular_speed_ = 1.0;
    geometry_msgs::msg::Twist twist_;
  
  void keyLoop()
  {
    char c;

    fd_set set;
    struct timeval timeout;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int rv = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
    if (rv <= 0) return;

    bool verbose = false;
    
    if (read(STDIN_FILENO, &c, 1) < 0) {
      return;
    }

    switch (c) {
      case 'a':
        this->twist_.linear.x = this->linear_speed_;
        this->twist_.linear.y = -this->linear_speed_;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Moving forward-left");}
        break;
      case 'z':
        this->twist_.linear.x = this->linear_speed_;
        this->twist_.linear.y = 0.0;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Moving forward");}
        break;
      case 'e':
        this->twist_.linear.x = this->linear_speed_;
        this->twist_.linear.y = this->linear_speed_;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Moving forward-right");}
        break;
      case 'q':
        this->twist_.linear.x = 0.0;
        this->twist_.linear.y = -this->linear_speed_;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Moving left");}
        break;
      case 's':
        this->twist_.linear.x = -this->linear_speed_;
        this->twist_.linear.y = 0.0;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Moving backward");}
        break;
      case 'd':
        this->twist_.linear.x = 0.0;
        this->twist_.linear.y = this->linear_speed_;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Moving right");}
        break;
      case 'w':
        this->twist_.linear.x = -this->linear_speed_;
        this->twist_.linear.y = -this->linear_speed_;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Moving backward-left");}
        break;
      case 'x':
        this->twist_.linear.x = -this->linear_speed_;
        this->twist_.linear.y = this->linear_speed_;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Moving backward-right");}
        break;
      case 'g':
        this->twist_.linear.x = 0.0;
        this->twist_.linear.y = 0.0;
        this->twist_.angular.z = this->angular_speed_;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Rotating left");}
        break;
      case 'h':
        this->twist_.linear.x = 0.0;
        this->twist_.linear.y = 0.0;
        this->twist_.angular.z = -this->angular_speed_;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Rotating right");}
        break;
      case ' ':
        this->twist_.linear.x = 0.0;
        this->twist_.linear.y = 0.0;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Stop");}
        break;
      case 'u':
        if (this->linear_speed_ <= 1.9)
        {
          this->linear_speed_ += 0.1;
        }
        if(verbose){RCLCPP_INFO(this->get_logger(), "linear speed increased to %.2f (max 2.0)", this->linear_speed_);}
        this->twist_.linear.x = (this->twist_.linear.x > 0 ? 1 : this->twist_.linear.x < 0 ? -1 : 0) * this->linear_speed_;
        this->twist_.linear.y = (this->twist_.linear.y > 0 ? 1 : this->twist_.linear.y < 0 ? -1 : 0) * this->linear_speed_;
        break;
      case 'j':
        if (this->linear_speed_ >= 0.1)
        {
          this->linear_speed_ -= 0.1;
        }
        if(verbose){RCLCPP_INFO(this->get_logger(), "linear speed decreased to %.2f", this->linear_speed_);}
        this->twist_.linear.x = (this->twist_.linear.x > 0 ? 1 : this->twist_.linear.x < 0 ? -1 : 0) * this->linear_speed_;
        this->twist_.linear.y = (this->twist_.linear.y > 0 ? 1 : this->twist_.linear.y < 0 ? -1 : 0) * this->linear_speed_;
        break;
      case 'i':
        if (this->angular_speed_ <= 1.4)
        {
          this->angular_speed_ += 0.1;
        }
        if(verbose){RCLCPP_INFO(this->get_logger(), "angular speed increased to %.2f (max 1.5)", this->angular_speed_);}
        this->twist_.angular.z = (this->twist_.angular.z > 0 ? 1 : this->twist_.angular.z < 0 ? -1 : 0) * this->angular_speed_;
        break;
      case 'k':
        if (this->angular_speed_ >= 0.1)
        {
          this->angular_speed_ -= 0.1;
        }
        if(verbose){RCLCPP_INFO(this->get_logger(), "angular speed decreased to %.2f", this->angular_speed_);}
        this->twist_.angular.z = (this->twist_.angular.z > 0 ? 1 : this->twist_.angular.z < 0 ? -1 : 0) * this->angular_speed_;
        break;
      case 'o':
        if(verbose){RCLCPP_INFO(this->get_logger(), "Exiting teleop");}
        this->twist_.linear.x = 0.0;
        this->twist_.linear.y = 0.0;
        this->twist_.angular.z = 0.0;
        publisher_->publish(this->twist_);
        rclcpp::shutdown();
        return;
      default:
        return;
    }

    publisher_->publish(this->twist_);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  struct termios old_tio_, new_tio_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleop>());
  rclcpp::shutdown();
  return 0;
}
