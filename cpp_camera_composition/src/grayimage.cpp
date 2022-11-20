#include "cpp_camera_composition/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;
using Image = sensor_msgs::msg::Image;

class GrayImage : public rclcpp::Node {
private:
  size_t count_;
  bool show_camera_;

  rmw_qos_profile_t custom_camera_qos_profile_;

  rclcpp::Publisher<Image>::SharedPtr pub_;
  rclcpp::Subscription<Image>::SharedPtr sub_;

public:
  GrayImage() : rclcpp::Node("gray_image_node") {

    show_camera_ = this->declare_parameter("show_camera", false);

    rmw_qos_reliability_policy_t reliability_policy =
        RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_qos_history_policy_t history_policy = RMW_QOS_POLICY_HISTORY_KEEP_ALL;

    /**Set QOS Config*/
    custom_camera_qos_profile_ = rmw_qos_profile_default;
    custom_camera_qos_profile_.depth = 10;
    custom_camera_qos_profile_.reliability = reliability_policy;
    custom_camera_qos_profile_.history = history_policy;

    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(custom_camera_qos_profile_.history,
                                  custom_camera_qos_profile_.depth),
        custom_camera_qos_profile_);

    // Create a publisher of "geometry_mgs/Twist" messages on the "cmd_vel"
    // topic.
    pub_ = create_publisher<Image>("gray_image", 10);

    sub_ = create_subscription<Image>(
        "image", qos,
        std::bind(&GrayImage::sub_callback, this, std::placeholders::_1));
  }

  void sub_callback(const sensor_msgs::msg::Image::SharedPtr img) {

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
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto show_image = std::make_shared<GrayImage>();

  rclcpp::spin(show_image);
  rclcpp::shutdown();

  return 0;
}