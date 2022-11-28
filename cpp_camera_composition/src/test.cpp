#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;


int main(int argv, char** argc){

    rclcpp::init(argv, argc);

    auto test_node = std::make_shared<rclcpp::Node>("test_node");

    RCLCPP_INFO(test_node->get_logger(), "BF Test");
    rclcpp::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(test_node->get_logger(), "AFT Test");

    rclcpp::shutdown();

    return 0;
}