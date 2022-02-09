#include "rclcpp/rclcpp.hpp"
//#include "sensor_msgs/msg/laserscan.hpp"

rclcpp::Node::SharedPtr node = nullptr;
/*
void callback(const sensor_msgs::msg::laserscan::SharedPtr msg)
{
  RCLCPP_INFO(node->get_logger(), "DATA!!!");
}
*/

enum actions{
  TURN_RIGHT,
  TURN_LEFT,
  MOVING_TURN_RIGHT,
  MOVING_TURN_LEFT,
  CONTINUE,
  STOP
};

enum sectors{
  RIGHT_NEAR,
  FRONT_NEAR,
  LEFT_NEAR,
  RIGHT_FAR,
  FRONT_FAR,
  LEFT_FAR,
}

struct laserscan_result{
  int data[6];//primero los 3 sectores cercanos en sentido antihorario, despues 3 sectores lejanos antihorario
}

enum actions decide_action(struct laserscan_result laser){
  //avoid colisions
  if(laser.data[RIGHT_NEAR]){
    return TURN_LEFT;
  }
  if(laser.data[FRONT_NEAR]){
    return TURN_LEFT;
  }
  if(laser.data[LEFT_NEAR]){
    return TURN_RIGHT;
  }
  //follow wall
  if(laser.data[RIGHT_FAR]){
    return CONTINUE;
  }
  //search wall
  return MOVING_TURN_RIGHT;
}





int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = rclcpp::Node::make_shared("simple_node_sub");
  /*
  auto subscription = node->create_subscription<sensor_msgs::msg::laserscan>(
    "/scan_raw", 10, callback);
  */
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}