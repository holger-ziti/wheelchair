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


#include<iostream>
#include<fstream>
#include<string>
#include<sstream>
using namespace std;

#define LDR_PATH "/sys/bus/iio/devices/iio:device0/in_voltage"

int readAnalog(int number){
   stringstream ss;
   ss << LDR_PATH << number << "_raw";
   fstream fs;
   fs.open(ss.str().c_str(), fstream::in);
   fs >> number;
   fs.close();
   return number;
}


#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <stdlib.h>     /* srand, rand */


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class AnalogInPublisher : public rclcpp::Node
{
public:
  AnalogInPublisher()
  : Node("analog_in_publisher"), count_(0)
  {
    this->declare_parameter<std::int32_t>("analog_input_number", -1);
    rclcpp::Parameter int_param = this->get_parameter("analog_input_number");
    int my_int = int_param.as_int();
    
    //this->declare_parameter("analog_input_number");
    //rclcpp::Parameter int_analog_input_number = this->get_parameter("analog_input_number");
    //int an_in_int = int_analog_input_number.as_int();
    
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("raw_voltage_"+ std::to_string(my_int), 1);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&AnalogInPublisher::timer_callback, this));
   


  }
  
//  void respond()
//    {
//      this->get_parameter("analog_input_number", int_analog_input_number);
//      RCLCPP_INFO(this->get_logger(), "Hello %s", int_analog_input_number.value_to_string().c_str());
//    }


private:

  //std::int analog_input_number_int_;

  void timer_callback()
  {

  
    auto message = std_msgs::msg::Int32();
    
    // get param 
    rclcpp::Parameter int_param = this->get_parameter("analog_input_number");
    int my_int = int_param.as_int();
    //int my_int = this->my_int;
    
    int iVolt;
    iVolt = readAnalog(my_int); 
    message.data = iVolt;
    
    /* use randomly number generate int between 1 and 10: */
    //int iRand; 
    //iRand = rand() % 10 + 1; 
    //message.data = iRand;
    
    
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.value_to_string().c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnalogInPublisher>());
  rclcpp::shutdown();
  return 0;
}
