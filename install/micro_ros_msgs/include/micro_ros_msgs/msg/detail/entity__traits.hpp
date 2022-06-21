// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from micro_ros_msgs:msg/Entity.idl
// generated code does not contain a copyright notice

#ifndef MICRO_ROS_MSGS__MSG__DETAIL__ENTITY__TRAITS_HPP_
#define MICRO_ROS_MSGS__MSG__DETAIL__ENTITY__TRAITS_HPP_

#include "micro_ros_msgs/msg/detail/entity__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const micro_ros_msgs::msg::Entity & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: entity_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "entity_type: ";
    character_value_to_yaml(msg.entity_type, out);
    out << "\n";
  }

  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: types
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.types.size() == 0) {
      out << "types: []\n";
    } else {
      out << "types:\n";
      for (auto item : msg.types) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const micro_ros_msgs::msg::Entity & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<micro_ros_msgs::msg::Entity>()
{
  return "micro_ros_msgs::msg::Entity";
}

template<>
inline const char * name<micro_ros_msgs::msg::Entity>()
{
  return "micro_ros_msgs/msg/Entity";
}

template<>
struct has_fixed_size<micro_ros_msgs::msg::Entity>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<micro_ros_msgs::msg::Entity>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<micro_ros_msgs::msg::Entity>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MICRO_ROS_MSGS__MSG__DETAIL__ENTITY__TRAITS_HPP_
