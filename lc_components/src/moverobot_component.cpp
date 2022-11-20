// references
// https://github.com/ros2/demos/blob/foxy/composition/CMakeLists.txt

#include "lc_components/moverobot_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

namespace lc_components
{

// Create a MoveRobot "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.
MoveRobot::MoveRobot(const rclcpp::NodeOptions & options)
: Node("moverobot", options), count_(0)
{
  // Create a publisher of "geometry_mgs/Twist" messages on the "cmd_vel" topic.
  pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Use a timer to schedule periodic message publishing.
  timer_ = create_wall_timer(1s, std::bind(&MoveRobot::on_timer, this));
}

void MoveRobot::on_timer()
{
  auto msg = std::make_unique<geometry_msgs::msg::Twist>();
  msg->linear.x = 0.3;
  msg->angular.z = 0.3;
  RCLCPP_INFO(this->get_logger(), "Publishing: %f", msg->linear.x);
  std::flush(std::cout);

  // Put the message into a queue to be processed by the middleware.
  // This call is non-blocking.
  pub_->publish(std::move(msg));
}

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(lc_components::MoveRobot)