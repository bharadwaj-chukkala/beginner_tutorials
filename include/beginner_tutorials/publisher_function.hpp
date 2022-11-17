/**
 * @file publisher_function.hpp
 * @author Bharadwaj Chukkala (bchukkal@umd.edu)
 * @brief Object Oriented publisher Implementation for ROS2
 * @version 0.1
 * @date 2022-11-10
 * 
 * @copyright 
*   // Copyright 2016 Open Source Robotics Foundation, Inc.
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

#pragma once

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;          // for use with binding Class member
using std::placeholders::_2;          // callback function

/**
 * @brief Class (subclass of Node) and uses std::bind() to register a member function as a callback from the timer.
 * 
 */
class MinimalPublisher : public rclcpp::Node {
 public:
    /**
     * @brief Construct a new Minimal Publisher object
     * 
     */
    MinimalPublisher();

 private:
  /**
   * @brief function for timer callback
   * 
   */
  void timer_callback();

  /**
   * @brief function to add two integers
   * 
   */
  void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request>,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>);
  rclcpp::TimerBase::SharedPtr timer_;  // timer variable
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  // publisher
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service;  // service
  size_t count_;  // count variable
  size_t invalid;  // a test variable
};
