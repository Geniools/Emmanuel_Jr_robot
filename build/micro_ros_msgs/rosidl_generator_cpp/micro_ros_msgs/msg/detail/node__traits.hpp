// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from micro_ros_msgs:msg/Node.idl
// generated code does not contain a copyright notice

#ifndef MICRO_ROS_MSGS__MSG__DETAIL__NODE__TRAITS_HPP_
#define MICRO_ROS_MSGS__MSG__DETAIL__NODE__TRAITS_HPP_

#include "micro_ros_msgs/msg/detail/node__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'entities'
#include "micro_ros_msgs/msg/detail/entity__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const micro_ros_msgs::msg::Node & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: node_namespace
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "node_namespace: ";
    value_to_yaml(msg.node_namespace, out);
    out << "\n";
  }

  // member: node_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "node_name: ";
    value_to_yaml(msg.node_name, out);
    out << "\n";
  }

  // member: entities
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.entities.size() == 0) {
      out << "entities: []\n";
    } else {
      out << "entities:\n";
      for (auto item : msg.entities) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const micro_ros_msgs::msg::Node & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<micro_ros_msgs::msg::Node>()
{
  return "micro_ros_msgs::msg::Node";
}

template<>
inline const char * name<micro_ros_msgs::msg::Node>()
{
  return "micro_ros_msgs/msg/Node";
}

template<>
struct has_fixed_size<micro_ros_msgs::msg::Node>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<micro_ros_msgs::msg::Node>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<micro_ros_msgs::msg::Node>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MICRO_ROS_MSGS__MSG__DETAIL__NODE__TRAITS_HPP_
