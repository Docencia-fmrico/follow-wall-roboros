// Copyright 2022 RoboRos
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
#include <string>
#include "follow_wall_lifecycle/follow_wall_lifecycle.hpp"

enum actions LCNcalc_dir::decide_action(struct laserscan_result laser)
{
  int d = 5;
  if (laser.data[FRONT_NEAR] > d || laser.data[LEFT_NEAR] > d || laser.data[RIGHT_NEAR] > d) {
    RCLCPP_INFO(get_logger(), "algo cerca-alejate");
    if (laser.data[FRONT_NEAR] > d) {
      RCLCPP_INFO(get_logger(), "girando derecha");
      return TURN_RIGHT;
    }
    if (laser.data[LEFT_NEAR] > d) {
      RCLCPP_INFO(get_logger(), "girando derecha");
      return MOVING_TURN_RIGHT;
    }
    RCLCPP_INFO(get_logger(), "girando izquierda");
    return MOVING_TURN_LEFT;
  }

  if (!(laser.data[FRONT_FAR] > d) && !(laser.data[LEFT_FAR] > d) &&
    !(laser.data[RIGHT_FAR] > d))
  {
    RCLCPP_INFO(get_logger(), "nada lejos, quizas has perdido la pared gira a la izquierda");
    return MOVING_TURN_LEFT;
  }
  if (!(laser.data[FRONT_FAR] > d) && (laser.data[LEFT_FAR] > d) &&
    !(laser.data[RIGHT_FAR] > d))
  {
    RCLCPP_INFO(get_logger(), "solo a la izq, acercate a la pared");
    return MOVING_TURN_LEFT;
  }
  if ((laser.data[FRONT_FAR] > d) && !(laser.data[LEFT_FAR] > d) &&
    !(laser.data[RIGHT_FAR] > d))
  {
    RCLCPP_INFO(get_logger(), "solo de frente, avanza hacia el obstaculo");
    return CONTINUE;
  }
  if (!(laser.data[FRONT_FAR] > d) && !(laser.data[LEFT_FAR] > d) &&
    (laser.data[RIGHT_FAR] > d))
  {
    RCLCPP_INFO(get_logger(), "obstaculo a la derecha! gira para ir hacia el");
    return TURN_RIGHT;
  }

  if ((laser.data[FRONT_FAR] > d) && (laser.data[LEFT_FAR] > d) &&
    !(laser.data[RIGHT_FAR] > d))
  {
    RCLCPP_INFO(get_logger(), "de frente izquierda, sigue");
    return CONTINUE;
  }
  if ((laser.data[FRONT_FAR] > d) &&
    !(laser.data[LEFT_FAR] > d) && (laser.data[RIGHT_FAR] > d))
  {
    RCLCPP_INFO(
      get_logger(),
      "de frente derecha, como hemos llegado aqui? gira hasta poner obstaculo de frente");
    return TURN_RIGHT;
  }
  if (!(laser.data[FRONT_FAR] > d) && (laser.data[LEFT_FAR] > d) &&
    (laser.data[RIGHT_FAR] > d))
  {
    RCLCPP_INFO(get_logger(), "a ambos lados, como??? continua pero acercate a la pared de izq");
    return MOVING_TURN_LEFT;
  }
  if ((laser.data[FRONT_FAR] > d) && (laser.data[LEFT_FAR] > d) &&
    (laser.data[RIGHT_FAR] > d))
  {
    RCLCPP_INFO(
      get_logger(), "a todos lados, gira hacia la izquierda para acercarte al obstaculo");
    return MOVING_TURN_LEFT;
  }
  RCLCPP_INFO(get_logger(), "algo raro");
  return MOVING_TURN_LEFT;
}

LCNcalc_dir::LCNcalc_dir()
: rclcpp_lifecycle::LifecycleNode("follow_wall_node")
{
  pub_ = create_publisher<std_msgs::msg::Float64>("configured_speed", 100);
  twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/mobile_base_controller/cmd_vel_unstamped", 10);
  lasersub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan_raw", 10, std::bind(&LCNcalc_dir::callback, this, _1));
  RCLCPP_INFO(get_logger(), "Creating LFC!!!");
}

CallbackReturnT LCNcalc_dir::on_configure(const rclcpp_lifecycle::State & state)
{
  angular_v = get_parameter("angular_v").get_value<float>();
  linear_v = get_parameter("linear_v").get_value<float>();
  near_limit = get_parameter("near_limit").get_value<float>();
  far_limit = get_parameter("far_limit").get_value<float>();
  robot = get_parameter("robot").get_value<std::string>();

  RCLCPP_INFO(
    get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());
  RCLCPP_INFO(
    get_logger(), "Parameters, angular_v:%f linear_v:%f near_limit:%f far_limit:%f",
    angular_v, linear_v, near_limit, far_limit);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT LCNcalc_dir::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
  twist_pub_->on_activate();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT LCNcalc_dir::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
  pub_->on_deactivate();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT LCNcalc_dir::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());
  pub_.reset();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT LCNcalc_dir::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
  pub_.reset();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT LCNcalc_dir::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
  return CallbackReturnT::SUCCESS;
}

void LCNcalc_dir::do_work()
{
  if (twist_pub_->is_activated()) {
    struct laserscan_result laser_res;
    laser_res.data[RIGHT_NEAR] = average_side_values[0][1];
    laser_res.data[FRONT_NEAR] = average_side_values[1][1];
    laser_res.data[LEFT_NEAR] = average_side_values[2][1];
    laser_res.data[RIGHT_FAR] = average_side_values[0][0];
    laser_res.data[FRONT_FAR] = average_side_values[1][0];
    laser_res.data[LEFT_FAR] = average_side_values[2][0];

    RCLCPP_INFO(get_logger(), "RIGHT_NEAR %d", laser_res.data[RIGHT_NEAR]);
    RCLCPP_INFO(get_logger(), "FRONT_NEAR %d", laser_res.data[FRONT_NEAR]);
    RCLCPP_INFO(get_logger(), "LEFT_NEAR %d", laser_res.data[LEFT_NEAR]);
    RCLCPP_INFO(get_logger(), "RIGHT_FAR %d", laser_res.data[RIGHT_FAR]);
    RCLCPP_INFO(get_logger(), "FRONT_FAR %d", laser_res.data[FRONT_FAR]);
    RCLCPP_INFO(get_logger(), "LEFT_FAR %d", laser_res.data[LEFT_FAR]);

    RCLCPP_INFO(get_logger(), "Do work");
    twist_pub_->publish(generate_twist_msg(decide_action(laser_res)));
  }
}

void LCNcalc_dir::callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "RANGES 0= %f", msg->ranges[0]);
  RCLCPP_INFO(get_logger(), "RANGES min= %f", msg->angle_min);
  RCLCPP_INFO(get_logger(), "RANGES size= %d", msg->ranges.size());
  RCLCPP_INFO(get_logger(), "RANGES max= %f", msg->angle_max);

  int iterations_per_size = msg->ranges.size();

  average_side_values[0][1] = 0;
  average_side_values[1][1] = 0;
  average_side_values[2][1] = 0;
  average_side_values[0][0] = 0;
  average_side_values[1][0] = 0;
  average_side_values[2][0] = 0;

  if (robot.compare("tiago")) {
    for (int i = 0; i < LASERPARTITION; i++) {
      // intial pointer
      int first_zone_range = iterations_per_size * i;
      int counterF = 0;
      int counterN = 0;
      for (int a = 0; a < iterations_per_size; a++) {
        // dsicard out of range values
        if (msg->ranges[first_zone_range + a] >= msg->range_min &&
          msg->ranges[first_zone_range + a] <= msg->range_max)
        {
          if (msg->ranges[first_zone_range + a] < near_limit) {
            // near side
            counterN++;
          } else if (msg->ranges[first_zone_range + a] < far_limit) {
            // far side
            counterF++;
          }
        }
      }
      average_side_values[i][0] = counterF;
      average_side_values[i][1] = counterN;
    }
  } else {
    for (int i = 0; i < static_cast<int> msg->ranges.size(); i++) {
      if (i < 60 || i > 300) {
        if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <=
          msg->range_max)
        {
          if (msg->ranges[i] < near_limit) {
            // near side
            average_side_values[1][1]++;
          } else if (msg->ranges[i] < far_limit) {
            // far side
            average_side_values[1][0]++;
          }
        }
        continue;
      }
      if (i < 120) {
        if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <=
          msg->range_max)
        {
          if (msg->ranges[i] < near_limit) {
            // near side
            average_side_values[2][1]++;
          } else if (msg->ranges[i] < far_limit) {
            // far side
            average_side_values[2][0]++;
          }
        }
        continue;
      }
      if (i > 240) {
        if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <=
          msg->range_max)
        {
          if (msg->ranges[i] < near_limit) {
            // near side
            average_side_values[0][1]++;
          } else if (msg->ranges[i] < far_limit) {
            // far side
            average_side_values[0][0]++;
          }
        }
      }
    }
  }
}

geometry_msgs::msg::Twist LCNcalc_dir::generate_twist_msg(enum actions action)
{
  geometry_msgs::msg::Twist msg;
  switch (action) {
    case TURN_RIGHT:
      msg.linear.x = 0;
      msg.angular.z = -angular_v;
      break;
    case TURN_LEFT:
      msg.linear.x = 0;
      msg.angular.z = angular_v;
      break;
    case MOVING_TURN_RIGHT:
      msg.linear.x = linear_v;
      msg.angular.z = -angular_v;
      break;
    case MOVING_TURN_LEFT:
      msg.linear.x = linear_v;
      msg.angular.z = angular_v;
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
      // error
      break;
  }
  return msg;
}
