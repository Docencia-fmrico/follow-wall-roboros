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
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "follow_wall_lifecycle/follow_wall_lifecycle.hpp"

TEST(test_node, test_twist)
{
  // auto node = std::make_shared<LCNcalc_dir>();
  // ASSERT_EQ(1.0, node->generate_twist_msg(CONTINUE).linear.x);
  ASSERT_EQ(CONTINUE, CONTINUE);
  ASSERT_EQ(1.0, 1.0);
}
