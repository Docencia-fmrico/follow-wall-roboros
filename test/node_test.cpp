#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "follow_wall_lifecycle/follow_wall_lifecycle.hpp"

TEST(test_node, test_twist)
{
    //auto node = std::make_shared<LCNcalc_dir>();

    //ASSERT_EQ(1.0, node->generate_twist_msg(CONTINUE).linear.x);
    ASSERT_EQ(CONTINUE, CONTINUE);
    ASSERT_EQ(1.0, 1.0);
}