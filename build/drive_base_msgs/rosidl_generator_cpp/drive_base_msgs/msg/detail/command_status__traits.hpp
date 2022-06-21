// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from drive_base_msgs:msg/CommandStatus.idl
// generated code does not contain a copyright notice

#ifndef DRIVE_BASE_MSGS__MSG__DETAIL__COMMAND_STATUS__TRAITS_HPP_
#define DRIVE_BASE_MSGS__MSG__DETAIL__COMMAND_STATUS__TRAITS_HPP_

#include "drive_base_msgs/msg/detail/command_status__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'cmd_header'
#include "drive_base_msgs/msg/detail/command_header__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const drive_base_msgs::msg::CommandStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_yaml(msg.stamp, out, indentation + 2);
  }

  // member: cmd_header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_header:\n";
    to_yaml(msg.cmd_header, out, indentation + 2);
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const drive_base_msgs::msg::CommandStatus & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<drive_base_msgs::msg::CommandStatus>()
{
  return "drive_base_msgs::msg::CommandStatus";
}

template<>
inline const char * name<drive_base_msgs::msg::CommandStatus>()
{
  return "drive_base_msgs/msg/CommandStatus";
}

template<>
struct has_fixed_size<drive_base_msgs::msg::CommandStatus>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value && has_fixed_size<drive_base_msgs::msg::CommandHeader>::value> {};

template<>
struct has_bounded_size<drive_base_msgs::msg::CommandStatus>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value && has_bounded_size<drive_base_msgs::msg::CommandHeader>::value> {};

template<>
struct is_message<drive_base_msgs::msg::CommandStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DRIVE_BASE_MSGS__MSG__DETAIL__COMMAND_STATUS__TRAITS_HPP_
