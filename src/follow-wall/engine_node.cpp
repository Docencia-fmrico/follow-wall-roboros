// Copyright 2020 Intelligent Robotics Lab
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



#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class EngineNode : public rclcpp::Node
{
public:
  EngineNode()
  : Node("engine_node"),
    counter(0)
  {

    pub_ = create_publisher<geometry_msgs/msg/Twist>("mobile_base_controller/cmd_vel_unstamped", 10);
    //sub_ = create_subscription<std_msgs::msg::Int32>(
      //"laser_info", 10, std::bind(&EngineNode::callback, this, _1));
    lasersub_ = create_subscription<sensor_msgs/msg/LaserScan>(
      "scan_raw", 10, std::bind(&EngineNode::callback, this, _1));
  }

  void doWork()
  {
    geometry_msgs/msg/Twist directions;
    //geometry_msgs/msg/Twist has Vector3 linear and Vector3 angular

    directions.linear=Vector3(1.4,3,0);
    directions.angular=Vector3(2.5,3,4);

    RCLCPP_INFO("Publishing direction V:[%f] direction W:[%f]" , directions.V,directions.W);

    //decision here?

    pub_->publish(directions);
  }

private:
  rclcpp::Subscription<ensor_msgs/msg/LaserScan>::SharedPtr lasersub_;
  rclcpp::Publisher<geometry_msgs/msg/Twist>::SharedPtr pub_;
  int decision;

private:
  void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
  {
    /*std_msgs/Header header # timestamp in the header is the acquisition time of
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

    float32 angle_min            # start angle of the scan [rad]
    float32 angle_max            # end angle of the scan [rad]
    float32 angle_increment      # angular distance between measurements [rad]

    float32 range_min            # minimum range value [m]
    float32 range_max            # maximum range value [m]

    float32[] ranges             # range data [m]
                                # (Note: values < range_min or > range_max should be discarded)
    float32[] intensities        # intensity data [device-specific units].  If your
                                # device does not provide intensities, please leave
                                # the array empty.*/
    int average_side_values[5][]:

    int areasize=(msg.angle_min-msg.angle_max)/5;
    int iterations_per_size=areasize/msg.angle_increment;
    for(int i=0;i<5,i++){
      int first_zone_range=iterations_per_size*i;
      int average;
      for(int a=0;a<iterations_per_size;a++){
        average+=range[first_zone_range+a];
      }
      average_side_values[i]=average/5;

    }
                              

    RCLCPP_INFO("I heard: [%d]",msg);
  }

};

/*int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MyNodePublisher>();

  rclcpp::Rate loop_rate(500ms);
  while (rclcpp::ok()) {
    node->doWork();

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  rclcpp::shutdown();

  return 0;
}*/