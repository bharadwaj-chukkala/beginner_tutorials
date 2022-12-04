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

#include<chrono>
#include<signal.h>
#include "../include/beginner_tutorials/publisher_function.hpp"

MinimalPublisher::MinimalPublisher()
    : Node("talker"), count_(0) {
       // Set the logger level to DEBUG from INFO
       this->get_logger().set_level(rclcpp::Logger::Level::Debug);

       RCLCPP_DEBUG_STREAM(this->get_logger(),
                    "Getting parameter values [publisher frequency and queue size]");

       // Argument/Parameter for setting publisher frequency (in Hz)
       auto pub_freq_desc = rcl_interfaces::msg::ParameterDescriptor();
       pub_freq_desc.description = "Sets the Param value of frequency to Hz units.";
       this->declare_parameter("pub_freq", 2.0, pub_freq_desc);
       auto pub_freq = this->get_parameter("pub_freq")
                    .get_parameter_value().get<std::float_t>();

       // Argument/Parameter for setting queue size of the buffer.
       auto queue_size_desc = rcl_interfaces::msg::ParameterDescriptor();
       queue_size_desc.description = "Sets the size of the Queue.";
       this->declare_parameter("queue_size", 10.0, queue_size_desc);
       auto queue_size = this->get_parameter("queue_size")
                    .get_parameter_value().get<std::float_t>();

       if (pub_freq < 0) {
          RCLCPP_FATAL_STREAM(this->get_logger(),
                    "Invalid frequency set for the publisher!");
       } else if (pub_freq == 0) {
          RCLCPP_ERROR_STREAM(this->get_logger(),
                    "Zero frequency set for the publisher!");
       }

       if (queue_size < 0) {
          RCLCPP_FATAL_STREAM(this->get_logger(),
                    "Invalid queue size set for the node!");
       } else if (queue_size == 0) {
          RCLCPP_ERROR_STREAM(this->get_logger(),
                    "Zero queue size set for the node!");
       }

       publisher_ = this->create_publisher<std_msgs::msg::String>
                    ("chatter", queue_size);

       timer_ = this->create_wall_timer(std::chrono::milliseconds(
                    static_cast<int>(1000 / pub_freq)),
                    std::bind(&MinimalPublisher::timer_callback, this));

       auto serviceCallbackPtr = std::bind(&MinimalPublisher::change_message,
                    this, std::placeholders::_1, std::placeholders::_2);

       service_ = create_service<beginner_tutorials::srv::ChangeMessage>(
                    "update_message", serviceCallbackPtr);

       publish_message_ = "Custom String Message for Printing";

       tf_static_broadcaster_ = std::make_shared<
                tf2_ros::StaticTransformBroadcaster>(this);

       geometry_msgs::msg::TransformStamped t;
       t.header.stamp = this->get_clock()->now();
       t.header.frame_id = "world";
       t.child_frame_id = "talk";
       // Translation component in meters
       t.transform.translation.x = 0.5;
       t.transform.translation.y = 0.6;
       t.transform.translation.z = 0.7;

       // Quaternion corresponding to XYZ Euler Angles
       // of (0.1, 0.1, 0.1) rad
       t.transform.rotation.x = 0.0689451;
       t.transform.rotation.y = 0.0595743;
       t.transform.rotation.z = 0.0639417;
       t.transform.rotation.w = 0.9451326;

       tf_static_broadcaster_->sendTransform(t);
}

void MinimalPublisher::timer_callback() {
    // Check if the node is not shutdown.
    if (rclcpp::ok()) {
        auto message = std_msgs::msg::String();
        auto id_str = std::string(", ID: ");
        message.data = publish_message_ + id_str + std::to_string(count_++);
        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.data);
        publisher_->publish(message);
    } else {
        RCLCPP_ERROR_STREAM(this->get_logger(), "User stops the node!");
    }
}

void MinimalPublisher::change_message(REQ req, RESP resp) {
    // If empty request message is received from the Service client.
    if (req->data == std::string("None")) {
        RCLCPP_WARN_STREAM(this->get_logger(),
                    "Received an empty request string, Try inputting again!");
    } else {
        resp->new_message = req->data;
        publish_message_ = resp->new_message;
        RCLCPP_INFO_STREAM(this->get_logger(),
                    "Request Received: " << req->data);
        RCLCPP_INFO_STREAM(this->get_logger(),
                    "Response Sent: " << resp->new_message);
    }
}

/**
 * @brief Callback function to detect when Ctrl+C is pressed.
 * 
 * @param signum SIGINT Interrupt ID (2 -> Ctrl+C)
 */
void node_shutdown_cb(int signum) {
    if (signum == 2) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),
                        "Manual shutdown of the node!");
    }
}

int main(int argc, char * argv[]) {
  // To catch Ctrl+C keypress interrupt
  signal(SIGINT, node_shutdown_cb);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();

  return 0;
}

