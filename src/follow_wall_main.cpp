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

#include <memory>
#include "follow_wall_lifecycle/follow_wall_lifecycle.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LCNcalc_dir>();
  node->declare_parameter("angular_v", 0.1);
  node->declare_parameter("linear_v", 0.05);
  node->declare_parameter("near_limit", 0.5);
  node->declare_parameter("far_limit", 0.7);
  node->declare_parameter("robot", "kobuki");

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->do_work();
    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }
  rclcpp::shutdown();

  return 0;
}
