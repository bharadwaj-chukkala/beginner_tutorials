/**
 * @file subscriber_member_function.cpp
 * @author Bharadawaj Chukkala (bchukkal@umd.edu)
 * @brief 
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

#include "../include/beginner_tutorials/subscriber_function.hpp"

MinimalSubscriber::MinimalSubscriber()
  : Node("minimal_subscriber") {
     // Set the logger level to DEBUG from INFO
     this->get_logger().set_level(rclcpp::Logger::Level::Debug);
     RCLCPP_DEBUG_STREAM(this->get_logger(),
                    "Retrieving queue size parameter value");

     // Argument/Parameter for setting queue size of the buffer.
     auto queue_size_desc = rcl_interfaces::msg::ParameterDescriptor();
     queue_size_desc.description = "Sets the size of the Queue (Default 10.0).";
     this->declare_parameter("queue_size", 10.0, queue_size_desc);
     auto queue_size = this->get_parameter("queue_size")
                    .get_parameter_value().get<std::float_t>();

     if (queue_size < 0) {
          RCLCPP_FATAL_STREAM(this->get_logger(),
                    "Invalid queue size set for the node!");
     } else if (queue_size == 0) {
          RCLCPP_ERROR_STREAM(this->get_logger(),
                    "Zero queue size set for the node!");
     }

     subscription_ = this->create_subscription<std_msgs::msg::String>(
              "chatter", queue_size,
              std::bind(&MinimalSubscriber::topic_callback, this, _1));

     // Check if there are publishers to the subscribed topic.
     if (this->count_publishers("chatter") == 0) {
        RCLCPP_WARN_STREAM(this->get_logger(),
              "No publisher found on the subscribed topic!");
     }
}

void MinimalSubscriber::topic_callback(
  const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg->data);
}


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
