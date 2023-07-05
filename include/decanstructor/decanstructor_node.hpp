// Copyright 2017-2023 Joshua Whitley
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef DECANSTRUCTOR__DECANSTRUCTOR_NODE_HPP_
#define DECANSTRUCTOR__DECANSTRUCTOR_NODE_HPP_

#include "decanstructor/common.hpp"

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <decanstructor/msg/can_event.hpp>

using CanEventMsgT = decanstructor::msg::CanEvent;
using CanFrameMsgT = can_msgs::msg::Frame;

namespace DeCANstructor
{

class DCRosNode : public rclcpp::Node
{
public:
  DCRosNode(const rclcpp::NodeOptions & options);
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

}  // namesapce DeCANstructor

#endif  // DECANSTRUCTOR__DECANSTRUCTOR_NODE_HPP_
