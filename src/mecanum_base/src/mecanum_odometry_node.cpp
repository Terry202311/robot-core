#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class MecanumOdometryNode : public rclcpp::Node
{
public:
  MecanumOdometryNode()
  : Node("mecanum_odometry"),
    initialized_(false),
    publish_tf_(true),
    wheel_radius_(0.04),
    wheel_base_(0.20),
    track_width_(0.18),
    counts_per_revolution_(1248.0),
    x_(0.0),
    y_(0.0),
    yaw_(0.0)
  {
    declare_parameter<double>("wheel_radius", 0.04);
    declare_parameter<double>("wheel_base", 0.20);
    declare_parameter<double>("track_width", 0.18);
    declare_parameter<double>("counts_per_revolution", 1248.0);

    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<bool>("publish_tf", true);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_base_ = get_parameter("wheel_base").as_double();
    track_width_ = get_parameter("track_width").as_double();
    counts_per_revolution_ =
      get_parameter("counts_per_revolution").as_double();

    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();

    validate_parameters();

    encoder_subscription_ =
      create_subscription<std_msgs::msg::Int64MultiArray>(
        "/wheel_encoder_counts",
        20,
        std::bind(
          &MecanumOdometryNode::encoder_callback,
          this,
          std::placeholders::_1));

    odometry_publisher_ =
      create_publisher<nav_msgs::msg::Odometry>(
        "/wheel/odometry",
        20);

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(
      get_logger(),
      "Mecanum odometry started");

    RCLCPP_INFO(
      get_logger(),
      "wheel_radius=%.4f, wheel_base=%.4f, "
      "track_width=%.4f, CPR=%.1f",
      wheel_radius_,
      wheel_base_,
      track_width_,
      counts_per_revolution_);
  }

private:
  void validate_parameters()
  {
    if (wheel_radius_ <= 0.0) {
      throw std::runtime_error(
        "wheel_radius must be greater than zero");
    }

    if (wheel_base_ <= 0.0) {
      throw std::runtime_error(
        "wheel_base must be greater than zero");
    }

    if (track_width_ <= 0.0) {
      throw std::runtime_error(
        "track_width must be greater than zero");
    }

    if (counts_per_revolution_ <= 0.0) {
      throw std::runtime_error(
        "counts_per_revolution must be greater than zero");
    }
  }

  static double normalize_angle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }

    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }

    return angle;
  }

  void encoder_callback(
    const std_msgs::msg::Int64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 4) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Expected 4 encoder values, received %zu",
        msg->data.size());
      return;
    }

    const std::array<int64_t, 4> current_counts = {
      msg->data[0],
      msg->data[1],
      msg->data[2],
      msg->data[3]
    };

    const rclcpp::Time current_time = now();

    if (!initialized_) {
      previous_counts_ = current_counts;
      previous_time_ = current_time;
      initialized_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Initial encoder counts: "
        "LF=%ld RF=%ld LB=%ld RB=%ld",
        static_cast<long>(current_counts[0]),
        static_cast<long>(current_counts[1]),
        static_cast<long>(current_counts[2]),
        static_cast<long>(current_counts[3]));
      return;
    }

    const double dt =
      (current_time - previous_time_).seconds();

    if (dt <= 0.0 || dt > 1.0) {
      previous_counts_ = current_counts;
      previous_time_ = current_time;

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Invalid encoder time interval: %.6f",
        dt);
      return;
    }

    std::array<int64_t, 4> delta_counts{};

    for (std::size_t i = 0; i < 4; ++i) {
      delta_counts[i] =
        current_counts[i] - previous_counts_[i];
    }

    const double radians_per_count =
      (2.0 * M_PI) / counts_per_revolution_;

    const double omega_lf =
      delta_counts[0] * radians_per_count / dt;

    const double omega_rf =
      delta_counts[1] * radians_per_count / dt;

    const double omega_lb =
      delta_counts[2] * radians_per_count / dt;

    const double omega_rb =
      delta_counts[3] * radians_per_count / dt;

    const double vx =
      wheel_radius_ * 0.25 *
      (omega_lf + omega_rf + omega_lb + omega_rb);

    const double vy =
      wheel_radius_ * 0.25 *
      (-omega_lf + omega_rf + omega_lb - omega_rb);

    const double rotation_radius =
      (wheel_base_ + track_width_) * 0.5;

    const double wz =
      wheel_radius_ /
      (4.0 * rotation_radius) *
      (-omega_lf + omega_rf - omega_lb + omega_rb);

    integrate_pose(vx, vy, wz, dt);

    publish_odometry(
      current_time,
      vx,
      vy,
      wz);

    previous_counts_ = current_counts;
    previous_time_ = current_time;
  }

  void integrate_pose(
    double vx,
    double vy,
    double wz,
    double dt)
  {
    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);

    const double world_vx =
      vx * cos_yaw - vy * sin_yaw;

    const double world_vy =
      vx * sin_yaw + vy * cos_yaw;

    x_ += world_vx * dt;
    y_ += world_vy * dt;
    yaw_ = normalize_angle(yaw_ + wz * dt);
  }

  void publish_odometry(
    const rclcpp::Time & stamp,
    double vx,
    double vy,
    double wz)
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw_);

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;

    odometry.pose.pose.position.x = x_;
    odometry.pose.pose.position.y = y_;
    odometry.pose.pose.position.z = 0.0;

    odometry.pose.pose.orientation.x = quaternion.x();
    odometry.pose.pose.orientation.y = quaternion.y();
    odometry.pose.pose.orientation.z = quaternion.z();
    odometry.pose.pose.orientation.w = quaternion.w();

    odometry.twist.twist.linear.x = vx;
    odometry.twist.twist.linear.y = vy;
    odometry.twist.twist.angular.z = wz;

    odometry.pose.covariance[0] = 0.05;
    odometry.pose.covariance[7] = 0.05;
    odometry.pose.covariance[35] = 0.10;

    odometry.twist.covariance[0] = 0.10;
    odometry.twist.covariance[7] = 0.10;
    odometry.twist.covariance[35] = 0.20;

    odometry_publisher_->publish(odometry);

    if (!publish_tf_) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;

    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;

    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();

    tf_broadcaster_->sendTransform(transform);
  }

  rclcpp::Subscription<
    std_msgs::msg::Int64MultiArray>::SharedPtr
    encoder_subscription_;

  rclcpp::Publisher<
    nav_msgs::msg::Odometry>::SharedPtr
    odometry_publisher_;

  std::unique_ptr<
    tf2_ros::TransformBroadcaster>
    tf_broadcaster_;

  bool initialized_;
  bool publish_tf_;

  double wheel_radius_;
  double wheel_base_;
  double track_width_;
  double counts_per_revolution_;

  double x_;
  double y_;
  double yaw_;

  std::string odom_frame_;
  std::string base_frame_;

  std::array<int64_t, 4> previous_counts_{};
  rclcpp::Time previous_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<MecanumOdometryNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("mecanum_odometry"),
      "Fatal error: %s",
      exception.what());
  }

  rclcpp::shutdown();
  return 0;
}
