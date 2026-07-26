// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msgs:msg/TargetDepth.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "msgs/msg/target_depth.hpp"


#ifndef MSGS__MSG__DETAIL__TARGET_DEPTH__BUILDER_HPP_
#define MSGS__MSG__DETAIL__TARGET_DEPTH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msgs/msg/detail/target_depth__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msgs
{

namespace msg
{

namespace builder
{

class Init_TargetDepth_depth
{
public:
  Init_TargetDepth_depth()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::msgs::msg::TargetDepth depth(::msgs::msg::TargetDepth::_depth_type arg)
  {
    msg_.depth = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msgs::msg::TargetDepth msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msgs::msg::TargetDepth>()
{
  return msgs::msg::builder::Init_TargetDepth_depth();
}

}  // namespace msgs

#endif  // MSGS__MSG__DETAIL__TARGET_DEPTH__BUILDER_HPP_
