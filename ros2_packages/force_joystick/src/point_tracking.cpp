// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;



/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class PointTracking : public rclcpp::Node
{
public:

  //visualization_msgs::msg::Marker marker1

  PointTracking()
  : Node("point_tracking"), count_(0)
  {


    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&PointTracking::joy_callback, this, std::placeholders::_1));


    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("joy_marker", 10);
    timer_ = this->create_wall_timer(
      20ms, std::bind(&PointTracking::timer_callback, this));




  }



private:

    float x;
    float y;






  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) 
  {


    x = msg->axes[1];
    y = msg->axes[0];

  }

  void timer_callback()
  {
    //RCLCPP_INFO(this->get_logger(), "Publishing marker1");

    visualization_msgs::msg::Marker marker1; //= std::make_unique<visualization_msgs::msg::Marker>();
    uint32_t shape1 = visualization_msgs::msg::Marker::SPHERE;
    marker1.type = shape1;

    marker1.header.frame_id = "/marker_frame";
    //marker1.header.stamp = ros::Time::now();

    marker1.action = visualization_msgs::msg::Marker::ADD;

    marker1.pose.position.x = 0;//x*5.0; // scaling factor
    marker1.pose.position.y = y*5.0;
    marker1.pose.position.z = 0;

    marker1.pose.orientation.x = 0.0;
    marker1.pose.orientation.y = 0.0;
    marker1.pose.orientation.z = 0.0;
    marker1.pose.orientation.w = 1.0;
    marker1.scale.x = 1.0;
    marker1.scale.y = 1.0;
    marker1.scale.z = 1.0;
    marker1.color.r = 0.0f;
    marker1.color.g = 1.0f;
    marker1.color.b = 0.0f;
    marker1.color.a = 1.0;
    marker1.color.a = 1.0;

    publisher_->publish(marker1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  //visualization_msgs::msg::Marker::SharedPtr marker1_;

  //isualization_msgs::marker marker1


  //visualization_msgs::Marker marker
  //visualization_msgs::msg::Marker::SharedPtr marker1_; // = visualization_msgs::msg::Marker();





};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointTracking>());
  rclcpp::shutdown();
  return 0;
}
