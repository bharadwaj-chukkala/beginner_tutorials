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

#include "../include/beginner_tutorials/publisher_function.hpp"

MinimalPublisher::MinimalPublisher()
    : Node("minimal_publisher"), count_(0) {
       publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
       timer_ = this->create_wall_timer(500ms,
        std::bind(&MinimalPublisher::timer_callback, this));

        auto service_name = "add_two_ints";
        auto serviceCallbackPtr = std::bind (&MinimalPublisher::add, this, _1, _2);
        service = create_service<example_interfaces::srv::AddTwoInts>(service_name, serviceCallbackPtr);
        
}

void MinimalPublisher::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hey, This is Bharadwaj, ID:  "
             + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}


void MinimalPublisher::add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  

  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

