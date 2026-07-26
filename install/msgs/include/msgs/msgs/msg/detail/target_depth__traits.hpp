// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msgs:msg/TargetDepth.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "msgs/msg/target_depth.hpp"


#ifndef MSGS__MSG__DETAIL__TARGET_DEPTH__TRAITS_HPP_
#define MSGS__MSG__DETAIL__TARGET_DEPTH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msgs/msg/detail/target_depth__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TargetDepth & msg,
  std::ostream & out)
{
  out << "{";
  // member: depth
  {
    out << "depth: ";
    rosidl_generator_traits::value_to_yaml(msg.depth, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TargetDepth & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: depth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth: ";
    rosidl_generator_traits::value_to_yaml(msg.depth, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TargetDepth & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace msgs

namespace rosidl_generator_traits
{

[[deprecated("use msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const msgs::msg::TargetDepth & msg,
  std::ostream & out, size_t indentation = 0)
{
  msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const msgs::msg::TargetDepth & msg)
{
  return msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msgs::msg::TargetDepth>()
{
  return "msgs::msg::TargetDepth";
}

template<>
inline const char * name<msgs::msg::TargetDepth>()
{
  return "msgs/msg/TargetDepth";
}

template<>
struct has_fixed_size<msgs::msg::TargetDepth>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<msgs::msg::TargetDepth>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<msgs::msg::TargetDepth>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSGS__MSG__DETAIL__TARGET_DEPTH__TRAITS_HPP_
