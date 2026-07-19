#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <opencv2/opencv.hpp>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

class StereoCameraNode : public rclcpp::Node
{
public:
  StereoCameraNode()
  : Node("stereo_camera_node")
  {
    // Camera parameters
    device_ = declare_parameter<std::string>("device", "/dev/video0");
    frame_width_ = declare_parameter<int>("frame_width", 1280);
    frame_height_ = declare_parameter<int>("frame_height", 400);
    capture_fps_ = declare_parameter<double>("capture_fps", 60.0);
    publish_fps_ = declare_parameter<double>("publish_fps", 30.0);

    left_frame_id_ = declare_parameter<std::string>(
      "left_frame_id", "camera_left_optical_frame");

    right_frame_id_ = declare_parameter<std::string>(
      "right_frame_id", "camera_right_optical_frame");

    left_topic_ = declare_parameter<std::string>(
      "left_topic", "left/image_raw");

    right_topic_ = declare_parameter<std::string>(
      "right_topic", "right/image_raw");

    if (frame_width_ <= 0 || frame_height_ <= 0) {
      throw std::runtime_error("frame_width and frame_height must be positive");
    }

    if ((frame_width_ % 2) != 0) {
      throw std::runtime_error(
              "Side-by-side image width must be an even number");
    }

    if (publish_fps_ <= 0.0) {
      throw std::runtime_error("publish_fps must be greater than zero");
    }

    left_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(left_topic_, 10);

    right_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(right_topic_, 10);

    open_camera();

    const auto timer_period =
      std::chrono::duration<double>(1.0 / publish_fps_);

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&StereoCameraNode::capture_and_publish, this));

    RCLCPP_INFO(get_logger(), "Stereo camera node started");
    RCLCPP_INFO(get_logger(), "Device: %s", device_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Requested side-by-side mode: %dx%d @ %.1f fps",
      frame_width_,
      frame_height_,
      capture_fps_);

    RCLCPP_INFO(
      get_logger(),
      "Expected output per camera: %dx%d",
      frame_width_ / 2,
      frame_height_);

    RCLCPP_INFO(
      get_logger(),
      "Publishing: /%s and /%s",
      left_topic_.c_str(),
      right_topic_.c_str());
  }

  ~StereoCameraNode() override
  {
    if (capture_.isOpened()) {
      capture_.release();
    }
  }

private:
  void open_camera()
  {
    RCLCPP_INFO(
      get_logger(),
      "Opening camera %s with V4L2 backend...",
      device_.c_str());

    if (!capture_.open(device_, cv::CAP_V4L2)) {
      throw std::runtime_error(
              "Unable to open camera device: " + device_);
    }

    // Request MJPEG to reduce USB bandwidth.
    const int mjpg_fourcc =
      cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

    capture_.set(cv::CAP_PROP_FOURCC, mjpg_fourcc);
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
    capture_.set(cv::CAP_PROP_FPS, capture_fps_);

    // Reduce latency where supported by the V4L2/OpenCV backend.
    capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);

    const int actual_width =
      static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH));

    const int actual_height =
      static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT));

    const double actual_fps =
      capture_.get(cv::CAP_PROP_FPS);

    const int actual_fourcc =
      static_cast<int>(capture_.get(cv::CAP_PROP_FOURCC));

    const char format[] = {
      static_cast<char>(actual_fourcc & 0xff),
      static_cast<char>((actual_fourcc >> 8) & 0xff),
      static_cast<char>((actual_fourcc >> 16) & 0xff),
      static_cast<char>((actual_fourcc >> 24) & 0xff),
      '\0'
    };

    RCLCPP_INFO(
      get_logger(),
      "Actual camera mode: %dx%d @ %.2f fps, format=%s",
      actual_width,
      actual_height,
      actual_fps,
      format);

    if (actual_width != frame_width_ ||
      actual_height != frame_height_)
    {
      RCLCPP_WARN(
        get_logger(),
        "Camera did not accept requested resolution. "
        "Requested %dx%d, actual %dx%d",
        frame_width_,
        frame_height_,
        actual_width,
        actual_height);
    }
  }

  void capture_and_publish()
  {
    cv::Mat stereo_frame;

    if (!capture_.read(stereo_frame) || stereo_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Failed to capture image from %s",
        device_.c_str());
      return;
    }

    if ((stereo_frame.cols % 2) != 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Captured image width %d is not even",
        stereo_frame.cols);
      return;
    }

    const int single_width = stereo_frame.cols / 2;

    const cv::Rect left_roi(
      0,
      0,
      single_width,
      stereo_frame.rows);

    const cv::Rect right_roi(
      single_width,
      0,
      single_width,
      stereo_frame.rows);

    // clone() ensures each image owns independent memory.
    const cv::Mat left_image =
      stereo_frame(left_roi).clone();

    const cv::Mat right_image =
      stereo_frame(right_roi).clone();

    // Use exactly the same timestamp for the stereo pair.
    const rclcpp::Time timestamp = now();

    std_msgs::msg::Header left_header;
    left_header.stamp = timestamp;
    left_header.frame_id = left_frame_id_;

    std_msgs::msg::Header right_header;
    right_header.stamp = timestamp;
    right_header.frame_id = right_frame_id_;

    auto left_message =
      cv_bridge::CvImage(
      left_header,
      sensor_msgs::image_encodings::BGR8,
      left_image).toImageMsg();

    auto right_message =
      cv_bridge::CvImage(
      right_header,
      sensor_msgs::image_encodings::BGR8,
      right_image).toImageMsg();

    left_publisher_->publish(*left_message);
    right_publisher_->publish(*right_message);

    frame_counter_++;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Publishing stereo images: frame=%lu, input=%dx%d, output=%dx%d",
      static_cast<unsigned long>(frame_counter_),
      stereo_frame.cols,
      stereo_frame.rows,
      single_width,
      stereo_frame.rows);
  }

  std::string device_;
  int frame_width_;
  int frame_height_;
  double capture_fps_;
  double publish_fps_;

  std::string left_frame_id_;
  std::string right_frame_id_;
  std::string left_topic_;
  std::string right_topic_;

  cv::VideoCapture capture_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  uint64_t frame_counter_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<StereoCameraNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("stereo_camera"),
      "Stereo camera node failed: %s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
