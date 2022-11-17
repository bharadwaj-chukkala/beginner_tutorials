/**
 * @file publisher_member_function.cpp
 * @author Bharadawaj Chukkala (bchukkal@umd.edu)
 * @brief Object Oriented Publisher Server Implementation
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright
 *  // Copyright 2016 Open Source Robotics Foundation, Inc.
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
 */
#include "../include/beginner_tutorials/publisher_function.hpp"

MinimalPublisher::MinimalPublisher(): Node("minimal_publisher"), count_(0) {
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = this->create_wall_timer(1s,
  std::bind(&MinimalPublisher::timer_callback, this));

  // created a service to add two integers
  auto service_name = "add_two_ints";
  auto serviceCallbackPtr = std::bind(&MinimalPublisher::add, this, _1, _2);
  service = create_service<example_interfaces::srv::AddTwoInts>(service_name, serviceCallbackPtr);
}

void MinimalPublisher::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hey, This is Bharadwaj, ID:  "
         + std::to_string(count_++);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "The Publisher is activated");
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing:" << message.data.c_str());
    publisher_->publish(message);

    // LOG LEVEL -> WARN
    if (count_ > 5) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Publisher has been running excessively");
      }

    // LOG LEVEL -> FATAL
    if (count_ > 10) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Warnings were blindsided, Fatality Occured");
    }
}


void MinimalPublisher::add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
  response->sum = request->a + request->b;
  if (response->sum == request->a + request->b) {
      // LOG LEVEL -> ERROR
      // This will ensure that a calculation mistake is caught
      RCLCPP_ERROR_STREAM(this->get_logger(), "Response is wrong" << std::to_string(invalid++));
      return;
  }

  // LOG LEVEL -> INFO
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Incoming request\na:" << request->a <<" b:" << request->b);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "sending back response:" << (long int)response->sum);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

