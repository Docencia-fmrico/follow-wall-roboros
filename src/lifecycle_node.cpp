// RoboRos group
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/float64.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

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
  LEFT_FAR
};

struct laserscan_result{
  int data[6];//primero los 3 sectores cercanos en sentido antihorario, despues 3 sectores lejanos antihorario
};

const float angular_v=0.5;
const float linear_v=1.0;

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

class LCNcalc_dir : public rclcpp_lifecycle::LifecycleNode
{
public:
  LCNcalc_dir()
  : rclcpp_lifecycle::LifecycleNode("lifecycle_node_example")
  {
    declare_parameter("speed", 0.34);
    pub_ = create_publisher<std_msgs::msg::Float64>("configured_speed", 100);
    }

    using CallbackReturnT = 
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());
   
    speed_ = get_parameter("speed").get_value<double>();
   
    return CallbackReturnT::SUCCESS;
  }


  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
    
    pub_->on_activate();
    
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
    
    pub_->on_deactivate();
    
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());
    
    pub_.reset();

    return CallbackReturnT::SUCCESS;
  }

    CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
    
    pub_.reset();
    
    return CallbackReturnT::SUCCESS;
  }

    CallbackReturnT on_error(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  void do_work() 
  {
    if (pub_->is_activated()) {
      //std_msgs::msg::Float64 msg;
      //msg.data = speed_;
    struct laserscan_result laser_res;
    laser_res.data[RIGHT_NEAR]=0;
    laser_res.data[FRONT_NEAR]=0;
    laser_res.data[LEFT_NEAR]=0;
    laser_res.data[RIGHT_FAR]=0;
    laser_res.data[FRONT_FAR]=0;
    laser_res.data[LEFT_FAR]=0;
    //RCLCPP_INFO(get_logger(), "Do work");
    generate_twist_msg(TURN_LEFT);

      //pub_->publish(msg);
    }
  }

private:
  double speed_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr pub_;

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
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LCNcalc_dir>();
    RCLCPP_INFO(node->get_logger(), "weeee");

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->do_work();

    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }
  

  rclcpp::shutdown();

  return 0;
}