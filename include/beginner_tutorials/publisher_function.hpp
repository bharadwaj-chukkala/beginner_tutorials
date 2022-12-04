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
#include <string>
#include <rclcpp/logging.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/change_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;
using REQ = const std::shared_ptr<beginner_tutorials
                ::srv::ChangeMessage::Request>;
using RESP = std::shared_ptr<beginner_tutorials
                ::srv::ChangeMessage::Response>;

/**
 * @brief Class (subclass of Node) and uses std::bind() to register a member function as a callback from the timer.
 * 
 */
class MinimalPublisher : public rclcpp::Node {
 public:
    MinimalPublisher();

 private:
    void timer_callback();
    void change_message(REQ req, RESP resp);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Service<beginner_tutorials::srv::ChangeMessage>::SharedPtr service_;
    size_t count_;
    std::string publish_message_;  // String to hold the message to be published
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};