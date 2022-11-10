/**
 * @file subscriber_function.hpp
 * @author Bharadwaj Chukkala (bchukkal@umd.edu)
 * @brief Object Oriented Subscriber Implementation for ROS2
 * @version 0.1
 * @date 2022-11-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once




#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/**
 * @brief Class (subclass of Node) and registers a member function as a callback from the topic.
 * 
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
    MinimalSubscriber();

 private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
