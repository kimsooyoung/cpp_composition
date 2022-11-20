// references
// https://snowdeer.github.io/ros2/2018/01/04/ros2-opencv-camera-image-publisher/
// https://github.com/ros2/demos/blob/foxy/image_tools/src/cam2image.cpp
// https://surfertas.github.io/ros2/2019/08/17/ros2-qos.html
// https://stackoverflow.com/questions/65935364/looking-for-cv-bridge-example-in-c-for-ros2


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include "opencv2/core/mat.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include "cpp_camera_composition/visibility_control.h"

#include <cstdio>

using namespace std;
using namespace cv;

using Image = sensor_msgs::msg::Image;

class ImagePublisher : public rclcpp::Node {

private:
  /** QOS depth size */
  size_t depth_;

  /** img freq/width/height */
  double freq_;
  size_t width_;
  size_t height_;

  /** cam number /dev/video? */
  int device_id_;

  /** run cv::imshow() or not */
  bool show_camera_;

  cv::VideoCapture cap;

  rmw_qos_profile_t custom_camera_qos_profile_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Image>::SharedPtr img_pub_;

public:
  ImagePublisher() : Node("image_publisher_node") {

    rmw_qos_reliability_policy_t reliability_policy =
        RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_qos_history_policy_t history_policy = RMW_QOS_POLICY_HISTORY_KEEP_ALL;

    /**Set QOS Config*/
    custom_camera_qos_profile_ = rmw_qos_profile_default;
    custom_camera_qos_profile_.depth = depth_;
    custom_camera_qos_profile_.reliability = reliability_policy;
    custom_camera_qos_profile_.history = history_policy;

    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(custom_camera_qos_profile_.history,
                                  custom_camera_qos_profile_.depth),
        custom_camera_qos_profile_);

    parse_parameters();

    if (!cap.open(device_id_)) {
      RCLCPP_ERROR(this->get_logger(), "Cannot Open Camera with ID : %d",
                   device_id_);
      throw std::runtime_error("Could not open video stream");
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));

    if (!cap.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
      throw std::runtime_error("Could not open video stream");
    }

    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / freq_)),
        std::bind(&ImagePublisher::timer_callback, this));

    img_pub_ = create_publisher<Image>("image", qos);
  }

  void parse_parameters() {
    depth_ = this->declare_parameter("depth", 10);
    freq_ = this->declare_parameter("freq", 30.0);
    width_ = this->declare_parameter("width", 640);
    height_ = this->declare_parameter("height", 480);
    device_id_ = static_cast<int>(this->declare_parameter("device_id", 0));
    show_camera_ = this->declare_parameter("show_camera", false);
  }

  void timer_callback() {
    cv::Mat frame;

    // Initialize a shared pointer to an Image message.
    auto msg = std::make_unique<Image>();
    msg->is_bigendian = false;

    cap >> frame;

    // If no frame was grabbed, return early
    if (frame.empty()) {
      return;
    }

    // humble에서는 이렇게 되지만,
    // std_msgs::msg::Header header;
    // header.frame_id = "camera_frame";
    // header.stamp = this->now();
    // Image container(frame, header);

    // foxy에서는 별도로 구현해주어야 함
    convert_frame_to_message(frame, *msg);

    // Conditionally show image
    if (show_camera_) {
      // Show the image in a window called "cam2image".
      cv::imshow("cam2image", frame);
      // Draw the image to the screen and wait 1 millisecond.
      cv::waitKey(1);
    }

    // Publish the image message and increment the publish_number_.
    // RCLCPP_INFO(get_logger(), "Publishing image #%zd", publish_number_++);
    img_pub_->publish(std::move(msg));
  }

  std::string mat_type2encoding(int mat_type) {
    switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
    }
  }

  void convert_frame_to_message(const cv::Mat &frame,
                                sensor_msgs::msg::Image &msg) {
    // copy cv information into ros message
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = mat_type2encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);
    msg.header.frame_id = "camera_frame";
  }

  ~ImagePublisher() {}
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto image_publisher = std::make_shared<ImagePublisher>();

  rclcpp::spin(image_publisher);
  rclcpp::shutdown();

  return 0;
}