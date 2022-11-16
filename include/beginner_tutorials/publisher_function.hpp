/**
 * @file publisher_function.hpp
 * @author Bharadwaj Chukkala (bchukkal@umd.edu)
 * @brief Object Oriented publisher Implementation for ROS2
 * @version 0.1
 * @date 2022-11-10
 * 
 * @copyright Copyright (c) 2022
 * 
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
    MinimalPublisher();

 private:
  void timer_callback();
  void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request>,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>);
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service;
  size_t count_;
  size_t invalid;
};
