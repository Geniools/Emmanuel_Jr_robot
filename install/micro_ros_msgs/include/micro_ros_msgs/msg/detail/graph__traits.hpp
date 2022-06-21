// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from micro_ros_msgs:msg/Graph.idl
// generated code does not contain a copyright notice

#ifndef MICRO_ROS_MSGS__MSG__DETAIL__GRAPH__TRAITS_HPP_
#define MICRO_ROS_MSGS__MSG__DETAIL__GRAPH__TRAITS_HPP_

#include "micro_ros_msgs/msg/detail/graph__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'nodes'
#include "micro_ros_msgs/msg/detail/node__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const micro_ros_msgs::msg::Graph & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: nodes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.nodes.size() == 0) {
      out << "nodes: []\n";
    } else {
      out << "nodes:\n";
      for (auto item : msg.nodes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const micro_ros_msgs::msg::Graph & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<micro_ros_msgs::msg::Graph>()
{
  return "micro_ros_msgs::msg::Graph";
}

template<>
inline const char * name<micro_ros_msgs::msg::Graph>()
{
  return "micro_ros_msgs/msg/Graph";
}

template<>
struct has_fixed_size<micro_ros_msgs::msg::Graph>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<micro_ros_msgs::msg::Graph>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<micro_ros_msgs::msg::Graph>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MICRO_ROS_MSGS__MSG__DETAIL__GRAPH__TRAITS_HPP_
