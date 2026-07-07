#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MecanumController : public rclcpp::Node
{
public:
  MecanumController() : Node("mecanum_controller")
  {
    this->declare_parameter("wheel_base_half", 0.20);
    this->declare_parameter("track_width_half", 0.20);
    this->declare_parameter("wheel_radius", 0.05);

    L_ = this->get_parameter("wheel_base_half").as_double();
    W_ = this->get_parameter("track_width_half").as_double();
    R_ = this->get_parameter("wheel_radius").as_double();

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10,
      std::bind(&MecanumController::cmdCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Mecanum controller started");
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double wz = msg->angular.z;

    double fl = (vx - vy - (L_ + W_) * wz) / R_;
    double fr = (vx + vy + (L_ + W_) * wz) / R_;
    double rl = (vx + vy - (L_ + W_) * wz) / R_;
    double rr = (vx - vy + (L_ + W_) * wz) / R_;

    RCLCPP_INFO(
      this->get_logger(),
      "wheel rad/s -> FL: %.2f FR: %.2f RL: %.2f RR: %.2f",
      fl, fr, rl, rr
    );
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  double L_;
  double W_;
  double R_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumController>());
  rclcpp::shutdown();
  return 0;
}
