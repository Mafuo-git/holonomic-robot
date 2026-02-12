#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class CmdFromNodeRed : public rclcpp::Node
{
public:
  CmdFromNodeRed()
  : Node("cmd_from_nodered_node")
  {
    // Publisher vers cmd_vel
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    // Subscriber depuis Node-RED
    cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/cmd_from_nodered", 10,
      std::bind(&CmdFromNodeRed::cmdCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "CmdFromNodeRed node started");
  }

private:
  bool verbose = true;
  float linear_speed_ = 0.5;
  float angular_speed_ = 1.0;
  geometry_msgs::msg::Twist twist_;
  void cmdCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    

    const std::string cmd = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received command: %s", cmd.c_str());

    if (cmd == "HG") {
        this->twist_.linear.x = this->linear_speed_;
        this->twist_.linear.y = -this->linear_speed_;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Moving forward-left");}
      }
    else if (cmd == "A")
    {
        this->twist_.linear.x = this->linear_speed_;
        this->twist_.linear.y = 0.0;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Moving forward");}
      }
    else if (cmd == "HD")
    {
        this->twist_.linear.x = this->linear_speed_;
        this->twist_.linear.y = this->linear_speed_;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Moving forward-right");}
        }
    else if (cmd == "G")
    {
        this->twist_.linear.x = 0.0;
        this->twist_.linear.y = -this->linear_speed_;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Moving left");}
    }
    else if (cmd == "R")
    {
        this->twist_.linear.x = -this->linear_speed_;
        this->twist_.linear.y = 0.0;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Moving backward");}
    }
    else if (cmd == "D")
    {
        this->twist_.linear.x = 0.0;
        this->twist_.linear.y = this->linear_speed_;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Moving right");}
    }
    else if (cmd == "BG")
    {
        this->twist_.linear.x = -this->linear_speed_;
        this->twist_.linear.y = -this->linear_speed_;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Moving backward-left");}
    }
    else if (cmd == "BD")
    {
        this->twist_.linear.x = -this->linear_speed_;
        this->twist_.linear.y = this->linear_speed_;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Moving backward-right");}
    }
    else if (cmd == "RG")
    {
        this->twist_.linear.x = 0.0;
        this->twist_.linear.y = 0.0;
        this->twist_.angular.z = -this->angular_speed_;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Rotating left");}
    }
    else if (cmd == "RD")
    {
        this->twist_.linear.x = 0.0;
        this->twist_.linear.y = 0.0;
        this->twist_.angular.z = this->angular_speed_;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Rotating right");}
    }
    else if (cmd == "AR")
    {
        this->twist_.linear.x = 0.0;
        this->twist_.linear.y = 0.0;
        this->twist_.angular.z = 0.0;
        if(verbose){RCLCPP_INFO(this->get_logger(), "Stop");}
    }
    else if (cmd == "LA")
    {
        if (this->linear_speed_ <= 1.9)
        {
          this->linear_speed_ += 0.1;
        }
        if(verbose){RCLCPP_INFO(this->get_logger(), "linear speed increased to %.2f (max 2.0)", this->linear_speed_);}
        this->twist_.linear.x = (this->twist_.linear.x > 0 ? 1 : this->twist_.linear.x < 0 ? -1 : 0) * this->linear_speed_;
        this->twist_.linear.y = (this->twist_.linear.y > 0 ? 1 : this->twist_.linear.y < 0 ? -1 : 0) * this->linear_speed_;
    }
    else if (cmd == "LR")
    {
        if (this->linear_speed_ >= 0.1)
        {
          this->linear_speed_ -= 0.1;
        }
        if(verbose){RCLCPP_INFO(this->get_logger(), "linear speed decreased to %.2f", this->linear_speed_);}
        this->twist_.linear.x = (this->twist_.linear.x > 0 ? 1 : this->twist_.linear.x < 0 ? -1 : 0) * this->linear_speed_;
        this->twist_.linear.y = (this->twist_.linear.y > 0 ? 1 : this->twist_.linear.y < 0 ? -1 : 0) * this->linear_speed_;
    }
    else if (cmd == "RA")
    {
        if (this->angular_speed_ <= 1.4)
        {
          this->angular_speed_ += 0.1;
        }
        if(verbose){RCLCPP_INFO(this->get_logger(), "angular speed increased to %.2f (max 1.5)", this->angular_speed_);}
        this->twist_.angular.z = (this->twist_.angular.z > 0 ? 1 : this->twist_.angular.z < 0 ? -1 : 0) * this->angular_speed_;
    }
    else if (cmd == "RR")
    {
        if (this->angular_speed_ >= 0.1)
        {
          this->angular_speed_ -= 0.1;
        }
        if(verbose){RCLCPP_INFO(this->get_logger(), "angular speed decreased to %.2f", this->angular_speed_);}
        this->twist_.angular.z = (this->twist_.angular.z > 0 ? 1 : this->twist_.angular.z < 0 ? -1 : 0) * this->angular_speed_;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Unknown command");
        return;
    }

    cmd_vel_pub_->publish(this->twist_);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdFromNodeRed>());
  rclcpp::shutdown();
  return 0;
}
