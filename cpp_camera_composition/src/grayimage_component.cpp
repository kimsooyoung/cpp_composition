#include "cpp_camera_composition/grayimage_component.hpp"
#include <sensor_msgs/image_encodings.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

using namespace std::chrono_literals;

namespace cpp_camera_composition {

// Create a MoveRobot "component" that subclasses the generic rclcpp::Node base
// class. Components get built into shared libraries and as such do not write
// their own main functions. The process using the component's shared library
// will instantiate the class as a ROS node.
GrayImage::GrayImage(const rclcpp::NodeOptions &options)
    : Node("gray_image", options), count_(0) {

  show_camera_ = this->declare_parameter("show_camera", false);

  rmw_qos_reliability_policy_t reliability_policy =
      RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  rmw_qos_history_policy_t history_policy = RMW_QOS_POLICY_HISTORY_KEEP_ALL;

  /**Set QOS Config*/
  custom_camera_qos_profile_ = rmw_qos_profile_default;
  custom_camera_qos_profile_.depth = 10;
  custom_camera_qos_profile_.reliability = reliability_policy;
  custom_camera_qos_profile_.history = history_policy;

  auto qos =
      rclcpp::QoS(rclcpp::QoSInitialization(custom_camera_qos_profile_.history,
                                            custom_camera_qos_profile_.depth),
                  custom_camera_qos_profile_);

  // Create a publisher of "geometry_mgs/Twist" messages on the "cmd_vel" topic.
  pub_ = create_publisher<Image>("gray_image", 10);

  sub_ = create_subscription<Image>(
      "image", qos,
      std::bind(&GrayImage::sub_callback, this, std::placeholders::_1));
}

void GrayImage::sub_callback(const sensor_msgs::msg::Image::SharedPtr img) {

  cv::Mat ros_img(img->height, img->width, CV_8UC3, (void *)(&img->data[0]));
  cv::Mat grayscale_img;

  cv::cvtColor(ros_img, grayscale_img, cv::COLOR_BGR2GRAY);

  // Create the output message and copy coverted data
  std::shared_ptr<Image> out_bgr = std::make_shared<Image>();

  out_bgr->header.stamp = img->header.stamp;
  out_bgr->header.frame_id = img->header.frame_id;
  out_bgr->height = grayscale_img.rows;
  out_bgr->width = grayscale_img.cols;

  int num = 1; // for endianness detection
  out_bgr->is_bigendian = !(*(char *)&num == 1);

  out_bgr->step = grayscale_img.step;

  size_t size = out_bgr->step * out_bgr->height;
  out_bgr->data.resize(size);

  out_bgr->encoding = "mono8";
  memcpy((char *)(&out_bgr->data[0]), &grayscale_img.data[0], size);

  // Conditionally show image
  if (show_camera_) {
    // Show the image in a window called "cam2image".
    cv::imshow("cam2image", grayscale_img);
    // Draw the image to the screen and wait 1 millisecond.
    cv::waitKey(1);
  }

  // pub_->publish(std::move(out_bgr));
}

} // namespace cpp_camera_composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(cpp_camera_composition::GrayImage)