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

#ifndef FOLLOW_WALL_LIFECYCLE_HPP_
#define FOLLOW_WALL_LIFECYCLE_HPP_

#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/float64.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"


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

enum{
  LASERPARTITION=3
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

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


class LCNcalc_dir : public rclcpp_lifecycle::LifecycleNode
{
public:
    double speed_;
    float average_side_values[LASERPARTITION][2];
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lasersub_;

    geometry_msgs::msg::Twist generate_twist_msg(enum actions action);

public:
    LCNcalc_dir();
    CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
    CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
    CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
    CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
    CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
    CallbackReturnT on_error(const rclcpp_lifecycle::State & state);
    void do_work();
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

#endif  //FOLLOW_WALL_LIFECYCLE_HPP_