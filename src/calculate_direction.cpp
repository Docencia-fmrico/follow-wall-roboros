#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

rclcpp::Node::SharedPtr node = nullptr;

void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  RCLCPP_INFO(node->get_logger(), "DATA!!!");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = rclcpp::Node::make_shared("simple_node_sub");
  auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan_raw", 10, callback);
  
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}