#ifndef COMPOSITION__GRAYIMAGE_COMPONENT_HPP_
#define COMPOSITION__GRAYIMAGE_COMPONENT_HPP_

#include "cpp_camera_composition/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>

using Image = sensor_msgs::msg::Image;

namespace cpp_camera_composition {

class GrayImage : public rclcpp::Node {
public:
  IMAGE_TOOLS_PUBLIC
  explicit GrayImage(const rclcpp::NodeOptions &options);

protected:
  IMAGE_TOOLS_LOCAL
  void sub_callback(const sensor_msgs::msg::Image::SharedPtr img);

private:
  size_t count_;
  bool show_camera_;
  rmw_qos_profile_t custom_camera_qos_profile_;

  rclcpp::Publisher<Image>::SharedPtr pub_;
  rclcpp::Subscription<Image>::SharedPtr sub_;
};

} // namespace cpp_camera_composition

#endif // COMPOSITION__GRAYIMAGE_COMPONENT_HPP_