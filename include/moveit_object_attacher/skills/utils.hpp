#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/plugins.hpp>
#include <cnr_param/cnr_param.h>
#include <rclcpp/node.hpp>

namespace bt_executer
{
namespace utils
{

template<typename T>
inline bool get_param(rclcpp::Node *node, std::string ns, std::string param_name, T& param, std::string what)
{
  if(cnr::param::has(ns + param_name, what))
  {
    if(not cnr::param::get(ns + param_name, param, what))
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Cannot load " << ns + param_name + " parameter.");
      RCLCPP_ERROR_STREAM(node->get_logger(), "what:" << what);
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), ns + param_name + " parameter not available.");
    RCLCPP_ERROR_STREAM(node->get_logger(), "what: " << what);
    return false;
  }
  return true;
}

} // namespace utils

} // namespace bt_executer
