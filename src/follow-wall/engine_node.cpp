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

#include "geometry_msgs/msg/Twist"
#include "geometry_msgs/msg/laser_scan"

enum{
  LASERPARTITION=3
};

using namespace std::chrono_literals;


int rad2degr(float rad){
    return float*(180/3.1416);
}

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

    RCLCPP_INFO("I heard: [%d]",msg);

    float average_side_values[LASERPARTITION][2]:

    int areasize=(rad2degr(msg.angle_min)-rad2degr(msg.angle_max))/LASERPARTITION;
    int iterations_per_size=areasize\rad2degr(msg.angle_increment);

    float semicircle_half=(msg.range_max-msg.range_min)/2;

    for(int i=0;i<LASERPARTITION,i++){

      int first_zone_range=iterations_per_size*i;
      int averageF,averageN,counterF,counterN = 0;

      for(int a=0;a<iterations_per_size;a++){
        //dsicard out of range values
        if(range[first_zone_range+a]>=msg.range_min && range[first_zone_range+a]<=msg.range_max){

          if(range[first_zone_range+a]>semicircle_half){
            //far aside part
            averageF+=averageF+range[first_zone_range+a];
            counterF++;
          }else{
            //near part
            averageN+=averageN+range[first_zone_range+a];
            counterN++;
          }
        }
      }
      average_side_values[i][0]=averageF/counterF;
      average_side_values[i][1]=averageN/counterN;

    }
                              
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