// reference
// https://github.com/ros2/demos/blob/eloquent/image_tools/src/showimage.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include "opencv2/core/mat.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include "cpp_camera_composition/visibility_control.h"

#include <cstdio>

using Image = sensor_msgs::msg::Image;

class ShowImage : public rclcpp::Node {
private:
  rclcpp::Subscription<Image>::SharedPtr sub_;

  bool show_image_ = true;

  size_t depth_;
  rmw_qos_profile_t custom_camera_qos_profile_;

public:
  IMAGE_TOOLS_LOCAL
  ShowImage() : Node("showimage") {
    rmw_qos_reliability_policy_t reliability_policy =
        RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_qos_history_policy_t history_policy = RMW_QOS_POLICY_HISTORY_KEEP_ALL;

    /**Set QOS Config*/
    custom_camera_qos_profile_ = rmw_qos_profile_default;
    custom_camera_qos_profile_.depth = depth_;
    custom_camera_qos_profile_.reliability = reliability_policy;
    custom_camera_qos_profile_.history = history_policy;

    parse_parameters();

    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(custom_camera_qos_profile_.history,
                                  custom_camera_qos_profile_.depth),
        custom_camera_qos_profile_);

    auto callback = [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      process_image(msg, show_image_, this->get_logger());
    };

    sub_ = create_subscription<Image>("image", qos, callback);
  }

  IMAGE_TOOLS_LOCAL
  void parse_parameters() {
    depth_ = this->declare_parameter("depth", 10);
    show_image_ = this->declare_parameter("show_image", true);
  }

  /// Convert a sensor_msgs::Image encoding type (stored as a string) to an
  /// OpenCV encoding type.
  /**
   * \param[in] encoding A string representing the encoding type.
   * \return The OpenCV encoding type.
   */
  IMAGE_TOOLS_LOCAL
  int encoding2mat_type(const std::string &encoding) {
    if (encoding == "mono8") {
      return CV_8UC1;
    } else if (encoding == "bgr8") {
      return CV_8UC3;
    } else if (encoding == "mono16") {
      return CV_16SC1;
    } else if (encoding == "rgba8") {
      return CV_8UC4;
    } else if (encoding == "bgra8") {
      return CV_8UC4;
    } else if (encoding == "32FC1") {
      return CV_32FC1;
    } else if (encoding == "rgb8") {
      return CV_8UC3;
    } else {
      throw std::runtime_error("Unsupported encoding type");
    }
  }

  /// Convert the ROS Image message to an OpenCV matrix and display it to the
  /// user.
  // \param[in] msg The image message to show.
  IMAGE_TOOLS_LOCAL
  void process_image(const sensor_msgs::msg::Image::SharedPtr msg,
                     bool show_image, rclcpp::Logger logger) {
    RCLCPP_INFO(logger, "Received image #%s", msg->header.frame_id.c_str());
    std::cerr << "Received image #" << msg->header.frame_id.c_str()
              << std::endl;

    if (show_image) {
      // Convert to an OpenCV matrix by assigning the data.
      cv::Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding),
                    const_cast<unsigned char *>(msg->data.data()), msg->step);

      if (msg->encoding == "rgb8") {
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
      }

      cv::Mat cvframe = frame;

      // Show the image in a window called "showimage".
      cv::imshow("showimage", cvframe);
      // Draw the screen and wait for 1 millisecond.
      cv::waitKey(1);
    }
  }

  ~ShowImage() {}
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto show_image = std::make_shared<ShowImage>();

  rclcpp::spin(show_image);
  rclcpp::shutdown();

  return 0;
}