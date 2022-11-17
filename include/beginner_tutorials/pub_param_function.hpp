#ifndef INCLUDE_BEGINNER_TUTORIALS_PUB_PARAM_FUNCTION_HPP_
#define BEGINNER_TUTORIALS_PUB_PARAM_FUNCTION_HPP_
/**
 * @file publisher_param_function.hpp
 * @author Bharadwaj Chukkala (bchukkal@umd.edu)
 * @brief Object Oriented publisher parameter Implementation for ROS2
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


# pragma once
#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

/**
 * @brief Class Minimal Param
 * 
 */
class MinimalParam : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Param object
   * 
   */
  MinimalParam();

  /**
   * @brief timer callback function
   * 
   */
  void timer_callback();

 private:
  rclcpp::TimerBase::SharedPtr timer_;  // timer variable
};

#endif
