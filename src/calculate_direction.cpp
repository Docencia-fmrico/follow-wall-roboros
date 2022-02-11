#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

rclcpp::Node::SharedPtr node = nullptr;


enum actions{
  TURN_RIGHT,
  TURN_LEFT,
  MOVING_TURN_RIGHT,
  MOVING_TURN_LEFT,
  CONTINUE,
  STOP
};

enum sectors{
  RIGHT_NEAR=0,
  FRONT_NEAR=1,
  LEFT_NEAR=2,
  RIGHT_FAR=3,
  FRONT_FAR=4,
  LEFT_FAR=5
};

struct laserscan_result{
  int data[6];//primero los 3 sectores cercanos en sentido antihorario, despues 3 sectores lejanos antihorario
};


const float angular_v=0.5;
const float linear_v=1.0;

class EngineNode : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lasersub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  enum actions next_action;

public:
  EngineNode()
  : Node("engine_node")
  {
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/mobile_base_controller/cmd_vel_unstamped", 10);
    //sub_ = create_subscription<std_msgs::msg::Int32>(
      //"laser_info", 10, std::bind(&EngineNode::callback, this, _1));
    //lasersub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    //  "scan_raw", 10, std::bind(&EngineNode::callback, this, _1));
  }

  void doWork()
  {
    //geometry_msgs/msg/Twist has Vector3 linear and Vector3 angular

    //directions.linear=geometry_msgs::msg::Vector3(1.4,3,0);
    //directions.angular=geometry_msgs::msg::Vector3(2.5,3,4);

    //TODO: fallo RCLCPP_INFO("Publishing direction V:[%f] direction W:[%f]" , directions.V,directions.W);

    //decision here?


    pub_->publish(generate_twist_msg(CONTINUE));
  }



private:
  geometry_msgs::msg::Twist generate_twist_msg(enum actions action){
    geometry_msgs::msg::Twist msg;
    switch (action)
    {
    case TURN_RIGHT:
      msg.linear.x = 0;
      msg.angular.z = angular_v;
      break;
    case TURN_LEFT:
      msg.linear.x = 0;
      msg.angular.z = -angular_v;
      break;
    case MOVING_TURN_RIGHT:
      msg.linear.x = linear_v;
      msg.angular.z = angular_v;
      break;
    case MOVING_TURN_LEFT:
      msg.linear.x = linear_v;
      msg.angular.z = -angular_v;
      break;
    case CONTINUE:
      msg.linear.x = linear_v;
      msg.angular.z = 0;
      break;
    case STOP:
      msg.linear.x = 0;
      msg.angular.z = 0;
      break;
    default:
      //error
      break;
    }
    return msg;
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

};






int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  //auto node = EngineNode::make_shared("simple_node_pub");
  auto node = std::make_shared<EngineNode>();

  rclcpp::Rate loop_rate(500ms);
  while(rclcpp::ok()){
    node->doWork();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  //node = rclcpp::Node::make_shared("simple_node_sub");
  /*
  auto subscription = node->create_subscription<sensor_msgs::msg::laserscan>(
    "/scan_raw", 10, callback);
  */
 /*
  struct laserscan_result laser_res;
  laser_res.data[RIGHT_NEAR]=0;
  laser_res.data[FRONT_NEAR]=0;
  laser_res.data[LEFT_NEAR]=0;
  laser_res.data[RIGHT_FAR]=0;
  laser_res.data[FRONT_FAR]=0;
  laser_res.data[LEFT_FAR]=0;

  while(true){
    RCLCPP_INFO(node->get_logger(), "DATA!!!");
    rclcpp::spin_some(node);
  }
  */

  rclcpp::shutdown();
  return 0;
}