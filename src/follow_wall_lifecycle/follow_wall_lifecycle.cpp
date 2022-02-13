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

#include "follow_wall_lifecycle/follow_wall_lifecycle.hpp"

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

LCNcalc_dir::LCNcalc_dir() : rclcpp_lifecycle::LifecycleNode("follow_wall_node"){
    pub_ = create_publisher<std_msgs::msg::Float64>("configured_speed", 100);
    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/mobile_base_controller/cmd_vel_unstamped", 10);
    RCLCPP_INFO(get_logger(), "Creating LFC!!!");
}

CallbackReturnT LCNcalc_dir::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());
    //speed_ = get_parameter("speed").get_value<double>();
    return CallbackReturnT::SUCCESS;
}

CallbackReturnT LCNcalc_dir::on_activate(const rclcpp_lifecycle::State & state) 
{
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
    twist_pub_->on_activate();
    return CallbackReturnT::SUCCESS;
}

CallbackReturnT LCNcalc_dir::on_deactivate(const rclcpp_lifecycle::State & state) 
{
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
    pub_->on_deactivate();    
    return CallbackReturnT::SUCCESS;
}

CallbackReturnT LCNcalc_dir::on_cleanup(const rclcpp_lifecycle::State & state) 
{
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());    
    pub_.reset();
    return CallbackReturnT::SUCCESS;
}

CallbackReturnT LCNcalc_dir::on_shutdown(const rclcpp_lifecycle::State & state) 
{
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());    
    pub_.reset();
    return CallbackReturnT::SUCCESS;
}

CallbackReturnT LCNcalc_dir::on_error(const rclcpp_lifecycle::State & state) 
{
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
}

void LCNcalc_dir::do_work() 
{
    if (twist_pub_->is_activated()) {
        struct laserscan_result laser_res;
        laser_res.data[RIGHT_NEAR]=0; //-- Faltan resto de array para rellenar laser
        RCLCPP_INFO(get_logger(), "Do work");
        generate_twist_msg(TURN_LEFT);
        twist_pub_->publish(generate_twist_msg(CONTINUE));
    }
}

geometry_msgs::msg::Twist LCNcalc_dir::generate_twist_msg(enum actions action){
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

