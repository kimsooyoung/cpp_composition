// reference
// https://gcc.gnu.org/wiki/Visibility

#ifndef COMPOSITION__MOVEROBOT_COMPONENT_HPP_
#define COMPOSITION__MOVEROBOT_COMPONENT_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "lc_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

namespace lc_components {

class MoveRobot : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit MoveRobot(const rclcpp::NodeOptions &options);

protected:
  void on_timer();

private:
  size_t count_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace lc_components

#endif // COMPOSITION__MOVEROBOT_COMPONENT_HPP_