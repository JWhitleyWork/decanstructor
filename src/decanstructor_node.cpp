// Copyright 2017-2023 Joshua Whitley
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <decanstructor/decanstructor_node.hpp>

#include <chrono>
#include <memory>

using CanEventMsgT = decanstructor::msg::CanEvent;
using CanFrameMsgT = can_msgs::msg::Frame;

using namespace std::chrono_literals;

namespace DeCANstructor
{

DCRosNode::DCRosNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("DeCANstructor", options)
{
  m_event_pub = this->create_publisher<CanEventMsgT>("events", rclcpp::QoS{20});
  m_can_sub = this->create_subscription<CanFrameMsgT>(
    "can_tx", rclcpp::QoS{100}, std::bind(&DCRosNode::OnCanMsg, this, std::placeholders::_1));

  // TODO(jwhitleywork): Get parameters and set m_options
  m_in_playback_mode = this->declare_parameter("playback", false);

  if (m_in_playback_mode) {
    RCLCPP_INFO(this->get_logger(), "Waiting for playback to begin...");

    while (this->now().nanoseconds() == 0 && rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(), "In playback mode but no playback detected. Waiting...");
      std::this_thread::sleep_for(1s);
    }

    if (this->now().nanoseconds() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Node shut down. No playback detected. Exiting...");
      return;
    } else {
      m_event_sub = this->create_subscription<CanEventMsgT>(
        "events", rclcpp::QoS{20},
        std::bind(&DCRosNode::OnEventPublished, this, std::placeholders::_1));
    }
  }
}

void DCRosNode::OnCanMsg(const CanFrameMsgT::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Got callback!");
  if (m_can_msg_cb_func) {
    (*m_can_msg_cb_func)(msg);
  }
}

void DCRosNode::OnEventPublished(const CanEventMsgT::SharedPtr msg)
{
  (void)msg;

  if (m_can_event_cb_func) {
    (*m_can_event_cb_func)();
  }
}

void DCRosNode::register_can_msg_callback(
  std::function<void(const CanFrameMsgT::SharedPtr)> cb_func)
{
  m_can_msg_cb_func = std::unique_ptr<std::function<void(const CanFrameMsgT::SharedPtr)>>(&cb_func);
}

void DCRosNode::register_can_event_callback(std::function<void()> cb_func)
{
  // TODO(jwhitleywork): Pass the received message along in the std::function in the future
  m_can_event_cb_func = std::unique_ptr<std::function<void()>>(&cb_func);
}

std::chrono::time_point<std::chrono::system_clock> DCRosNode::get_ros_time() const
{
  // This specifically ignores RCL_STEADY_TIME for simplicity
  auto ros_now = this->now().nanoseconds();
  return std::chrono::system_clock::time_point(std::chrono::nanoseconds{ros_now});
}

void DCRosNode::publish_can_event() const
{
  CanEventMsgT event_msg;
  event_msg.event_desc = "";
  event_msg.header.stamp = this->now();

  m_event_pub->publish(event_msg);
}

bool DCRosNode::in_playback_mode() const
{
  return m_in_playback_mode;
}

}  // namespace DeCANstructor
