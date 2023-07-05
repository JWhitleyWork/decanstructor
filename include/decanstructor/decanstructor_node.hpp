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

#ifndef DECANSTRUCTOR__DECANSTRUCTOR_NODE_HPP_
#define DECANSTRUCTOR__DECANSTRUCTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <decanstructor/msg/can_event.hpp>

#include <memory>

using CanEventMsgT = decanstructor::msg::CanEvent;
using CanFrameMsgT = can_msgs::msg::Frame;

namespace DeCANstructor
{

class DCRosNode : public rclcpp::Node
{
public:
  explicit DCRosNode(const rclcpp::NodeOptions & options);
  rclcpp::Logger & get_ros_logger();
  void register_can_msg_callback(
    std::function<void(const CanFrameMsgT::SharedPtr)> cb_func);
  void register_can_event_callback(std::function<void()> cb_func);
  std::chrono::time_point<std::chrono::system_clock> get_ros_time() const;
  void publish_can_event() const;
  bool in_playback_mode() const;

private:
  void OnCanMsg(const CanFrameMsgT::SharedPtr msg);
  void OnEventPublished(const CanEventMsgT::SharedPtr msg);

  rclcpp::Publisher<CanEventMsgT>::SharedPtr m_event_pub;
  rclcpp::Subscription<CanFrameMsgT>::SharedPtr m_can_sub;
  rclcpp::Subscription<CanEventMsgT>::SharedPtr m_event_sub;
  std::unique_ptr<std::function<void(const CanFrameMsgT::SharedPtr)>> m_can_msg_cb_func;
  std::unique_ptr<std::function<void()>> m_can_event_cb_func;
  bool m_in_playback_mode;
};

}  // namespace DeCANstructor

#endif  // DECANSTRUCTOR__DECANSTRUCTOR_NODE_HPP_
